
#if(!(__TL_LIB_8366__ ))
#include "../proj/config/user_config.h"
#include "../proj/mcu/config.h"
#endif 

#if(__TL_LIB_8366__ || (MCU_CORE_TYPE && MCU_CORE_TYPE == MCU_CORE_8366))

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

#include "rf_drv.h"
#include "pm.h"

/////////////////////////////////////////////////////////////////////
// Battery Detection
//////////////////////////////////////////////////////////////////////////
#if(0)
const unsigned char tbl_battery_ref[] = {
		 V0P98_REF,
		 V1P1_REF,
		 V1P18_REF,
		 V1P25_REF,
		 V1P3_REF,
		 V1P66_REF
};

const unsigned char tbl_battery_scale[] = {
		V0P98_SCALE,
		V1P1_SCALE,
		V1P18_SCALE,
		V1P25_SCALE,
		V1P3_SCALE,
		V1P66_SCALE
};
const unsigned char tbl_ldo_set[] = {
		0x07,		//2.0 V
		0x06, 		//2.0 ~ 2.25
		0x05, 		//2.25 ~ 2.5
		0x04, 		//2.5 ~ 2.8
		0x03,		//2.8 ~ 3.0
		0x02,		//3.0 ~ 3.3
		0x01,		// > 3.3
		0x00
};

int battery_low () {
	u8 ref_avdd = 0x07;
	u8 scale = 0x03;
	analog_write(0x03, scale | VBAT_CHANNEL);
	analog_write(0x00,(analog_read(0x00) & 0x7f) | (ref_avdd <<7));
	analog_write(0x02, (ref_avdd >> 1) | 0xe0);
	WaitUs(50);
	analog_write(0x44,analog_read(0x44) | 0x01);//clear result;

	analog_write(0x03,  tbl_battery_scale[VBAT_LOW_LEVLE] | VBAT_CHANNEL);
	analog_write(0x00, (analog_read(0x00) & 0x7f) | (tbl_battery_ref[VBAT_LOW_LEVLE] <<7));
	analog_write(0x02, (tbl_battery_ref[VBAT_LOW_LEVLE] >> 1) | 0xe0);
	WaitUs (50);

	return (analog_read(0x44)& 0x01);
}

void battery_by_comp_init(){
	 analog_write(0x0a,0x3f);
	 analog_write(0x05,0x02);
	 analog_write(0x24,0x01);
	 analog_write(0x25,0xa0);
	 write_reg8(0x800026,0x07);
}
#endif




int battery_low_by_set ( u8 chn, u8 v_ref, u8 v_scale ) {
	analog_write(0x02, (analog_read(0x02)&0x8f) | v_ref);
	analog_write(0x03, (analog_read(0x03)&0x80) | v_scale | chn);
	analog_write(0x06, (analog_read(0x06)&0xfe));  //Power up cmp

	WaitUs (50);
	return analog_read(0x23) & 0x20;   //ana23<5>
}

////////////////////// End of Battery ////////////////////////////////////////////
_attribute_ram_code_ _attribute_no_inline_ void  sleep_start(void){
      write_reg8(0x80006f,0x81);
      volatile unsigned int i;
      for(i=0; i<0x30; i++);
}
const TBLCMDSET  tbl_cpu_wakeup_init[] = {
	0x0060, 0x00,		TCMD_UNDER_BOTH | TCMD_WRITE,	//open all the clk,disable all the rst
	0x0061, 0x00,		TCMD_UNDER_BOTH | TCMD_WRITE,	//open all the clk,disable all the rst
	0x0062, 0x00,		TCMD_UNDER_BOTH | TCMD_WRITE,	//open all the clk,disable all the rst
	0x0063, 0xff,		TCMD_UNDER_BOTH | TCMD_WRITE,	//open all the clk,disable all the rst
	0x0064, 0xff,		TCMD_UNDER_BOTH | TCMD_WRITE,
	
	0x0620, 0x01,		TCMD_UNDER_BOTH | TCMD_WRITE,
#if 1
	////////////////////////////8886 deepsleep analog register recover////////////////////////////
	0x18, 0x00,		TCMD_UNDER_BOTH | TCMD_WAREG,	//restore power down control
	0x13, 0x7f,		TCMD_UNDER_BOTH | TCMD_WAREG,	//wakeup reset time: 7f shortest
	0x26, 0x40,		TCMD_UNDER_BOTH | TCMD_WAREG,	//32K cap
	0x10, 0x08,		TCMD_UNDER_BOTH | TCMD_WAREG,	//8ms mode default
	0x12, 0x10,		TCMD_UNDER_BOTH | TCMD_WAREG,	//32K continuous mode
	0x23, 0x80,		TCMD_UNDER_BOTH | TCMD_WAREG,	//kick start 32k

	// 61 48 20 90 20 0c

//	0x81, 0x40,		TCMD_UNDER_BOTH | TCMD_WAREG,	//turn off cap
//	0x80, 0x61,		TCMD_UNDER_BOTH | TCMD_WAREG,	//
//	0x82, 0x20,		TCMD_UNDER_BOTH | TCMD_WAREG,	//
//	0x83, 0x90,		TCMD_UNDER_BOTH | TCMD_WAREG,	//
//	0x84, 0x20,		TCMD_UNDER_BOTH | TCMD_WAREG,	//
//	0x85, 0x0c,		TCMD_UNDER_BOTH | TCMD_WAREG,	//

#endif
};

void cpu_wakeup_init(void){
	LoadTblCmdSet (tbl_cpu_wakeup_init, sizeof (tbl_cpu_wakeup_init)/sizeof (TBLCMDSET));
}

#define AREG_PAD_WAKEUP_EN(i)				((i>>8) + 0x27)
#define AREG_PAD_WAKEUP_POL(i)				((i>>8) + 0x21)

void cpu_set_gpio_wakeup (int pin, int pol, int en) {
	///////////////////////////////////////////////////////////
	// wkup_pad_en:  0x16  {p_gpio[10:7],p_gpio[4:1]}
	// pol:			 0x14  BIT(7):GP10,GP9;BIT(6):GP8,GP7;BIT(5):GP4,GP3;BIT(4):GP2,GP1
	// 0x17[0]:	  wakeup_dig_en
	// 0x17[1]:	  wakeup_32k_en
	u8 en_mask;
	u8 pol_mask;
	u8 val;

	if(pin&0xf0){  // 1111 0000:GP10-GP7
		en_mask = pin;
	}
	else if(pin&0x1e00){ //1 1110 0000 0000  GP4-Gp1
		en_mask = pin>>9;
	}
	else{  //other gpio func return
		return;
	}

	if(pin&0xc0){   //GP10 GP9
		pol_mask = BIT(7);
	}
	else if(pin&0x30){  //GP8 GP7
		pol_mask = BIT(6);
	}
	else if(pin&0x1800){  //GP4 GP3
		pol_mask = BIT(5);
	}
	else if(pin&0x700){  //GP2 GP1
		pol_mask = BIT(4);
	}
	else{
		pol_mask = 0;
	}

	////////////////////////// polarity ////////////////////////
	val = analog_read (0x14);
	if (pol) {
		val &= ~pol_mask;
	}
	else {
		val |= pol_mask;
	}
	analog_write (0x14, val);

	/////////////////////////// enable /////////////////////
	val = analog_read (0x16);
	if (en) {
		val |= en_mask;
	}
	else {
		val &= ~en_mask;
	}
	analog_write (0x16, val);
}

u32 tick_32k_bk;
int tick_1ms_adjust;

#if(__PROJECT_MOUSE__ || __PROJECT_KEYBOARD__ || __PROJECT_8366_MIIW_MOUSE__)
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

int cpu_sleep_wakeup (int deepsleep, int wakeup_src, u32 wakeup_tick) {


	/////////////////// set wakeup source /////////////////////////////////
	///////////////// store gpio wakeup enable setting ////////////////////

	u8 anareg80 = analog_read(0x80);		//xtal cap
	u8 anareg81 = analog_read(0x81);		//xtal cap

	u8 anareg16;
	if (! (wakeup_src & PM_WAKEUP_PAD)) {	//PM_WAKEUP_PAD disabled
		anareg16 = analog_read (0x16);
		analog_write (0x16, 0);
	}

	if(wakeup_src & PM_WAKEUP_TIMER){
		if (tick_32k_bk != wakeup_tick) {
			//cfg =	ReadAnalogReg(0x17);
			analog_write(0x17, PM_32KRC_RESET);
			analog_write(0x17, 0);

			tick_32k_bk = wakeup_tick;
			analog_write(0x10,(wakeup_tick)&0xff);
			analog_write(0x11,(wakeup_tick>>8)&0xff);//
			analog_write(0x12,(wakeup_tick>>16)|0x10);

			analog_write (0x23, 0x80);		//kick start 32K
		}
	}

	analog_write (0x17, wakeup_src | PM_AUTO_PWDN_EN);

	///////////////////////// change to 32M RC clock before suspend /////////////
	u8 reg66 = read_reg8(0x800066);			//
	write_reg8 (0x800066, 0);				//change to 32M rc clock

	//////////////////////// set deepsleep flag
	if (deepsleep) {

	}

	analog_write (0x81, 0xc0);					//turn off xtal cap
	analog_write (0x80, 0xa1);					//turn off xtal 4.6p cap
	analog_write (0x18, (deepsleep ? PM_AUTO_PWDN_DEEPSLEEP : PM_AUTO_PWDN_SUSPEND) | \
				((wakeup_src & PM_WAKEUP_TIMER) ? 0 : PM_AUTO_PWDN_32K));  //0x5f


	analog_write (0x23, 0x0f);  //clear status

	sleep_start();

	u8 anareg23 = analog_read(0x23);

	if (deepsleep) {		// reboot ?

	}

	analog_write (0x80, anareg80);			//restore xtal cap setting
	analog_write (0x81, anareg81);			//restore xtal cap setting


	//////////////////// restore GPIO wake-up configuration ////////////////
	if (! (wakeup_src & PM_WAKEUP_PAD)) {
		analog_write (0x16, anareg16);
	}

	////////////////// system clock has to be on ////////////////////////////
	write_reg8 (0x800066, reg66);			//restore system clock

	return anareg23;
}

u32 cpu_working_tick;
u32  cpu_wakup_last_tick;

#ifndef SRAM_SLEEP_CODE
#define SRAM_SLEEP_CODE     1
#endif
#if SRAM_SLEEP_CODE
_attribute_ram_code_
#endif
int cpu_sleep_wakeup_rc (int deepsleep, int wakeup_src, u32 wakeup_ms)
{
	///////// assume 8M RC clock: 125 us => 1000 tick; 1ms => 8000 tick
    
#if CHIP_8366_A1
	analog_write (0x13, 0x7c);			// reset/wait time on wakeup
#define tms 	wakeup_ms
#else
	//while (1) { write_reg8 (0x808008, read_reg8(0x808008) + 1); }
#if (CLOCK_SYS_CLOCK_HZ == 16000000)
	write_reg8 (0x800066, 0x24);
	sleep_us (300);
	write_reg8 (0x800066, 0x22);
	u32 tc = (wakeup_ms * 16000) + tick_1ms_adjust + cpu_wakup_last_tick - clock_time () - 500;  //240 us*16 = 3840 tick
	int tms = tc >> 14;
	if (tms < 1) {
		tms = 1;
	}
	//int twait = 0x68 - ((tc & 0x3fff) >> 9);
	int twait = 0x68;

#elif (CLOCK_SYS_CLOCK_HZ == 24000000)   //for dongle
	int tms = wakeup_ms;  //tms < 256
	int twait =	0x20;
#else
	u32 tc = (wakeup_ms * 8000) + tick_1ms_adjust + cpu_wakup_last_tick - clock_time () - 1200;  //240 us*16 = 3840 tick
	int tms = tc >> 13;
	if (tms < 1) {
		tms = 1;
	}
	int twait = 0x60 - ((tc & 0x1fff) >> 8);
#endif

	analog_write (0x13, twait);			// reset/wait time on wakeup
#endif
	/////////////////// set wakeup source /////////////////////////////////
	///////////////// store gpio wakeup enable setting ////////////////////

#if(!__PROJECT_VACUUM__)
	u8 anareg80 = analog_read(0x80);		//xtal cap
	u8 anareg81 = analog_read(0x81);		//xtal cap
#endif

	u8 anareg16;
	if (! (wakeup_src & PM_WAKEUP_PAD)) {	//PM_WAKEUP_PAD disabled
		anareg16 = analog_read (0x16);
		analog_write (0x16, 0);
	}

	//////////////////////// set deepsleep flag
	if (deepsleep) {
		analog_write (0x05, 0x0);		//keep bbpll ldo on when wakeup
	}

#if(!__PROJECT_VACUUM__)
	analog_write (0x81, 0xc0);					//turn off xtal cap
	analog_write (0x80, 0xa1);					//turn off xtal 4.6p cap
#endif
	analog_write (0x18, (deepsleep ? PM_AUTO_PWDN_DEEPSLEEP : PM_AUTO_PWDN_SUSPEND) | \
				((wakeup_src & PM_WAKEUP_TIMER) ? 0 : PM_AUTO_PWDN_32K));  //0x5f
#if CHIP_8366_A1
	if ( (tick_32k_bk != wakeup_ms)&& (wakeup_src & PM_WAKEUP_TIMER) ){
#else
	if(wakeup_src & PM_WAKEUP_TIMER){
#endif
		analog_write(0x17, PM_32KRC_RESET);
		analog_write(0x17, 0);

		tick_32k_bk = wakeup_ms;
		analog_write(0x10, tms&0xff);
		analog_write(0x11, tms>>8);
		analog_write(0x12, 0x10);
		while (!analog_read(0x20)) {
			analog_write (0x23, 0x80);		//kick start 32K
			sleep_us (80);
		}
	}

	analog_write (0x17, wakeup_src | PM_AUTO_PWDN_EN);
	analog_write (0x23, 0x0f);  //clear status

	cpu_working_tick = clock_time () - cpu_wakup_last_tick;
	//cpu_working_tick = twait;

	u8 r = reg_irq_en;
	reg_irq_en = 0;

	sleep_start();

	reg_irq_en = r;

	cpu_wakup_last_tick = clock_time ();			// system tick on wakeup

	u8 anareg23 = analog_read(0x23);


	if (deepsleep) {		// reboot ?

	}

#if(!__PROJECT_VACUUM__)
	analog_write (0x80, anareg80);			//restore xtal cap setting
	analog_write (0x81, anareg81);			//restore xtal cap setting
#endif


	//////////////////// restore GPIO wake-up configuration ////////////////
	if (! (wakeup_src & PM_WAKEUP_PAD)) {
		analog_write (0x16, anareg16);
	}

	return anareg23;
}

void cpu_usb_suspend_wakeup(void){

}

void cpu_suspend_wakeup_sim ( u32 time_us ){
    static u32 tick;
    while (!clock_time_exceed (tick, time_us));
    tick = clock_time ();
}

#endif

