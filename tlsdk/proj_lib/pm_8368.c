
#if(!(__TL_LIB_8368__ ))
#include "../proj/config/user_config.h"
#include "../proj/mcu/config.h"
#endif 

#if(__TL_LIB_8368__ || (MCU_CORE_TYPE && MCU_CORE_TYPE == MCU_CORE_8368))

#include "../proj/common/types.h"
#include "../proj/common/compatibility.h"
#include "../proj/common/bit.h"
#include "../proj/common/utility.h"
#include "../proj/common/static_assert.h"
#include "../proj/mcu/compiler.h"
#include "../proj/mcu/register.h"
#include "../proj/mcu/anareg.h"
#include "../proj/mcu/analog.h"
#include "../proj/mcu/clock.h"
#include "../proj/mcu/clock_i.h"

#include "../proj/mcu/gpio.h"
#include "rf_drv.h"
#include "pm.h"


/////////////////////////////////////////////////////////////////////
// Battery Detection
//////////////////////////////////////////////////////////////////////////
int battery_low_by_set ( u8 chn, u8 v_ref, u8 v_scale ) {
	analog_write(0x02, (analog_read(0x02)&0x8f) | v_ref);
	analog_write(0x03, (analog_read(0x03)&0x80) | v_scale | chn);
	analog_write(0x06, (analog_read(0x06)&0xfe));  //Power up cmp

	WaitUs (50);
	return analog_read(0x23) & 0x20;   //ana23<5>
}

////////////////////// End of Battery ////////////////////////////////////////////
#define  MSPI_IE_DISABLE	do{REG_ADDR8(0x5a1) &= 0xe1;}while(0)
#define  MSPI_IE_ENABLE		do{REG_ADDR8(0x5a1) |= 0x1e;}while(0)

_attribute_ram_code_ _attribute_no_inline_ void  sleep_start(void){
  write_reg8(0x80006f,0x81);
  volatile unsigned int i;
  for(i=0; i<0x30; i++);
}


#if 0
_attribute_ram_code_ _attribute_no_inline_ void  suspend_start(void)
{
	volatile unsigned int i;

	REG_ADDR8 (0x0d) = 0;
	REG_ADDR8 (0x0c) = 0xb9;
	for(i=0; i<2; i++); //1440ns when 32M clk
	REG_ADDR8 (0x0d) = 1;

	//MSPI_IE_DISABLE;
	write_reg8(0x80006f,0x81);
	for(i=0; i<0x30; i++);
	//MSPI_IE_ENABLE;

	REG_ADDR8 (0x0d) = 0;
	REG_ADDR8 (0x0c) = 0xab;
	for(i=0; i<2; i++); //1440ns when 32M clk
	REG_ADDR8 (0x0d) = 1;
}
#endif

const TBLCMDSET  tbl_cpu_wakeup_init[] = {
	0x0060, 0x00,		TCMD_UNDER_BOTH | TCMD_WRITE,	//open all the clk,disable all the rst
	0x0061, 0x00,		TCMD_UNDER_BOTH | TCMD_WRITE,	//open all the clk,disable all the rst
	0x0062, 0x00,		TCMD_UNDER_BOTH | TCMD_WRITE,	//open all the clk,disable all the rst
	0x0063, 0xff,		TCMD_UNDER_BOTH | TCMD_WRITE,	//open all the clk,disable all the rst
	0x0064, 0xc7,		TCMD_UNDER_BOTH | TCMD_WRITE,
	
	0x0620, 0x01,		TCMD_UNDER_BOTH | TCMD_WRITE,
#if 1
	////////////////////////////8886 deepsleep analog register recover////////////////////////////
	0x18, 0x00,		TCMD_UNDER_BOTH | TCMD_WAREG,	//restore power down control
	0x13, 0x74,		TCMD_UNDER_BOTH | TCMD_WAREG,	//wakeup reset time: 7f shortest
	0x26, 0x40,		TCMD_UNDER_BOTH | TCMD_WAREG,	//32K cap
	0x10, 0x08,		TCMD_UNDER_BOTH | TCMD_WAREG,	//8ms mode default
#if   1 //RF_FAST_MODE_1M
	0x12, 0x02,     TCMD_UNDER_BOTH | TCMD_WAREG,  		//enable 32K RC
	0x0065, 0x01,		TCMD_UNDER_BOTH | TCMD_WRITE,	//enable 32K clock to calibration
	0x0027, 0x0f,		TCMD_UNDER_BOTH | TCMD_WRITE,	//16 cycles of 32K clock
	0x0026, 0xa6,		TCMD_UNDER_BOTH | TCMD_WRITE,	//start calibration
#else
	0x24, 0x18,     TCMD_UNDER_BOTH | TCMD_WAREG,   //trk32k_en
#endif

	0x83, 0x10,  TCMD_UNDER_BOTH | TCMD_WAREG, //enable baseband PLL
	0x84, 0x20,  TCMD_UNDER_BOTH | TCMD_WAREG, //enable 192M clock to dig
#endif
};

void cpu_wakeup_init(void){
	LoadTblCmdSet (tbl_cpu_wakeup_init, sizeof (tbl_cpu_wakeup_init)/sizeof (TBLCMDSET));
}

void cpu_set_gpio_wakeup (int pin, int pol, int en) {
	///////////////////////////////////////////////////////////
	// wkup_pad_en:  ana_16-- GP[24:17] ([0]->GP17)
	//				 ana_2b-- [2]:GP31,[1]:GP27,[0]:GP26
	// pol:			 ana_14-- [7]:GP24,GP23;[6]:GP22,GP21;[5]:GP20,GP19;BIT[4]:GP18,GP17
	//               ana_2b-- [6]:GP31,[5]:GP27,[4]:GP26
	u8 en_mask;
	u8 pol_mask;
	u8 val;


	u8 bit = pin & 0xff;
	if( pin >=GPIO_GP17 &&  pin<=GPIO_GP24){
		if(pin==GPIO_GP24){
			en_mask = BIT(7);
		}
		else{
			en_mask = bit>>1;
		}

		if(bit&0x06){ 			//GP17 GP18
			pol_mask = BIT(4);
		}
		else if(bit&0x18){ 		//GP19 GP20
			pol_mask = BIT(5);
		}
		else if(bit&0x60){ 		//GP21 GP22
			pol_mask = BIT(6);
		}
		else{             		//GP23 GP24
			pol_mask = BIT(7);
		}

		///////////// polarity ////////////
		val = analog_read (0x14);
		if (pol) {
			val &= ~pol_mask;
		}
		else {
			val |= pol_mask;
		}
		analog_write (0x14, val);

		///////////// enable ////////////////
		val = analog_read (0x16);
		if (en) {
			val |= en_mask;
		}
		else {
			val &= ~en_mask;
		}
		analog_write (0x16, val);

	}
	else if(pin==GPIO_GP26 || pin==GPIO_GP27 ||pin==GPIO_GP31){
		if(pin==GPIO_GP26){
			en_mask = BIT(0);
			pol_mask = BIT(4);
		}
		else if(pin==GPIO_GP27){
			en_mask = BIT(1);
			pol_mask = BIT(5);
		}
		else{
			en_mask = BIT(2);
			pol_mask = BIT(6);
		}

		///////////// polarity  and enable////////////
		val = analog_read (0x2b);
		if (pol) {
			val &= ~pol_mask;
		}
		else {
			val |= pol_mask;
		}
		///////////// enable ////////////////
		if (en) {
			val |= en_mask;
		}
		else {
			val &= ~en_mask;
		}
		analog_write (0x2b, val);
	}
}

u32 tick_32k_bk;
int tick_1ms_adjust;

#if(__PROJECT_MOUSE__ || __PROJECT_KEYBOARD__)
_attribute_ram_code_ void cpu_adjust_system_tick (int adjust_ms)
#else
void cpu_adjust_system_tick (int adjust_ms)
#endif
{
#if (CLOCK_SYS_CLOCK_HZ == 16000000)
	tick_1ms_adjust = adjust_ms * 16000;		//assume 16Mhz clock,
#else
	tick_1ms_adjust = adjust_ms * 8000;			//assume 8Mhz clock,
#endif
#if ( CHIP_8366_A1	)
	u32 t = tick_32k_bk + adjust_ms;
	analog_write(0x10, t);
	analog_write(0x11, t>>8);
#endif
	return;
}


u32 cpu_working_tick;
u32 cpu_wakup_last_tick;



int cpu_sleep_wakeup (int deepsleep, int wakeup_src, u32 wakeup_ms)
{
	analog_write (0x13, 0x70);// reset/wait time ( 7f-70 ) * 2= 30 cycle = 1 ms
    u32 timer_tick = wakeup_ms<<5;

	/////////////////// set wakeup source /////////////////////////////////
	u8 anareg80 = analog_read(0x80);		//xtal cap
	u8 anareg81 = analog_read(0x81);		//xtal cap

	reg_wakeup_en = (wakeup_src&PM_WAKEUP_CORE) ? FLD_WAKEUP_SRC_GPIO : 0 ;

	analog_write (0x81, 0xc0);					//turn off xtal cap
	analog_write (0x80, 0xa1);					//turn off xtal 4.6p cap
	analog_write (0x18, (deepsleep ? PM_AUTO_PWDN_DEEPSLEEP : PM_AUTO_PWDN_SUSPEND) | \
				((wakeup_src & PM_WAKEUP_TIMER) ? 0 : PM_AUTO_PWDN_32K));

	u8 reg66 = REG_ADDR8(0x66);

	if (wakeup_src & PM_WAKEUP_TIMER){
		analog_write(0x12, 0x00);  //32k_timer_disable
	    analog_write(0x17, PM_32KRC_RESET);  //reset
		analog_write(0x17, 0);

		analog_write(0x0f, timer_tick&0xff);
		analog_write(0x10, (timer_tick>>8)&0xff);
		analog_write(0x11, (timer_tick>>16)&0xff);

		analog_write(0x12, 0x02);  //32k_timer_enable
	}

	analog_write (0x23, 0x0f);  //clear status
	analog_write (0x17, wakeup_src);

	REG_ADDR8(0x66) = 0x00;

	cpu_working_tick = clock_time () - cpu_wakup_last_tick;

	sleep_start();

	cpu_wakup_last_tick = clock_time ();			// system tick on wakeup

	REG_ADDR8(0x66) = reg66;

	u8 anareg23 = analog_read(0x23);


	analog_write (0x80, anareg80);			//restore xtal cap setting
	analog_write (0x81, anareg81);			//restore xtal cap setting

	return anareg23;
}


void cpu_suspend_wakeup_sim ( u32 time_us ){
    static u32 tick;
    while (!clock_time_exceed (tick, time_us));
    tick = clock_time ();
}

int blt_sleep_wakeup (int deepsleep, int wakeup_src, u32 wakeup_tick)
{
	u32 span = wakeup_tick - clock_time ();
	if (span < 2000 * CLOCK_SYS_CLOCK_1US)
	{
		u32 t = clock_time ();
		analog_write (0x23, 0x0f);			//clear all status
		u8 st;
		do {
			st = analog_read (0x23) & 0x0f;
		} while ( ((u32)clock_time () - t < span) && !st);
		return st;
	}

	u32 tick_cur;
	u32 t32k;
	u16 tick_16_rc32k;
	u8 reg66 = REG_ADDR8(0x66);

	if (wakeup_src & PM_WAKEUP_TIMER)
	// go to suspend
	{	//t32 = span * 16 / reg24
		span -= 1000 * CLOCK_SYS_CLOCK_1US;			//count wakeup-reset time
		tick_16_rc32k = REG_ADDR16(0x24);
		t32k = span * 16 / tick_16_rc32k;
		analog_write(0x0f, t32k);
		analog_write(0x10, t32k>>8);
		analog_write(0x11, t32k>>16);

		blt_get_32k_tick ();
		analog_write(0x17, PM_32KRC_RESET);  		//reset
		tick_cur = clock_time ();					//PLL clock
		analog_write (0x17, wakeup_src);			//start
	}

	REG_ADDR8(0x66) = 0x00;
	/////////////////// set wakeup source /////////////////////////////////
	///////////////// store gpio wakeup enable setting ////////////////////

	u8 anareg80 = analog_read(0x80);		//xtal cap
	u8 anareg81 = analog_read(0x81);		//xtal cap


	reg_wakeup_en = (wakeup_src&PM_WAKEUP_CORE) ? FLD_WAKEUP_SRC_GPIO : 0 ;

	//////////////////////// set deepsleep flag
	if (deepsleep) {
		analog_write (0x05, 0x0);		//keep bbpll ldo on when wakeup
	}

	analog_write (0x81, 0xc0);					//turn off xtal cap
	analog_write (0x80, 0xa1);					//turn off xtal 4.6p cap
	analog_write (0x18, (deepsleep ? PM_AUTO_PWDN_DEEPSLEEP : PM_AUTO_PWDN_SUSPEND) | \
				((wakeup_src & PM_WAKEUP_TIMER) ? 0 : PM_AUTO_PWDN_32K));

	analog_write (0x23, 0x0f);  //clear status
	analog_write (0x17, wakeup_src);

	static u32		cpu_working_tick, cpu_wakup_last_tick;

	cpu_working_tick = clock_time () - cpu_wakup_last_tick;

	sleep_start();

	cpu_wakup_last_tick = clock_time ();			// system tick on wakeup

	u8 anareg23 = analog_read(0x23);


	analog_write (0x80, anareg80);			//restore xtal cap setting
	analog_write (0x81, anareg81);			//restore xtal cap setting


	if (wakeup_src & PM_WAKEUP_TIMER)
	{
		REG_ADDR8(0x66) = reg66;
		u32 tsus = blt_get_32k_tick ();
		if (analog_read(0x23) & PM_WAKEUP_TIMER)		//wake-up from timer
		{
			tsus = blt_get_32k_tick ();					//must...
			tsus += t32k;
		}
		tsus = tsus * tick_16_rc32k >> 4;
		reg_system_tick = tick_cur + tsus;			//new system tick
	}

	REG_ADDR8(0x26) = 0xa6;				// start 32K calibration
	return anareg23;
}

#endif

