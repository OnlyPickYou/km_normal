
#if(!(__TL_LIB_8266__ ))
#include "../proj/config/user_config.h"
#include "../proj/mcu/config.h"
#endif 

#if(__TL_LIB_8266__ || (MCU_CORE_TYPE && MCU_CORE_TYPE == MCU_CORE_8266))

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

#ifndef			REVISION_8266_A2
#define			REVISION_8266_A2		0
#endif

////////////////////////////////////////////////////
const u16 wakeup_src_pin[] = {
	BIT(4),		//MSD0
	BIT(5),		//MSDI
	BIT(7),		//MCLK
	BIT(6),		//MCSN

	BIT(14),	//DO
	BIT(13),	//DI
	BIT(15),	//CK
	BIT(12),	//CN

	BIT(0),		//PWM0
	BIT(1),		//PWM1
	BIT(2),		//PWM2
	0,			//SWS
	0,			//SWM

	BIT(8),		//GP0
	BIT(9),		//GP1
	BIT(10),	//GP2
	BIT(11),	//GP3
	BIT(3),		//GP4
	0,			//GP5
	0,			//GP6
	0,			//GP7
	0,			//DM
	0,			//DP
};

/////////////////////////////////////////////////////////////////////
// Battery Detection
//////////////////////////////////////////////////////////////////////////

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

////////////////////// End of Battery ////////////////////////////////////////////
_attribute_ram_code_ _attribute_no_inline_ void  sleep_start(void){
      write_reg8(0x80006f,0x81);
      volatile unsigned int i;
      for(i=0; i<0x30; i++);
}

_attribute_ram_code_ _attribute_no_inline_ void  suspend_start(void)
{

	REG_ADDR8 (0x0d) = 0;
	REG_ADDR8 (0x0c) = 0xb9;
	REG_ADDR8 (0x0d) = 0;
	REG_ADDR8 (0x0d) = 0;
	REG_ADDR8 (0x0d) = 1;
	write_reg8(0x80006f,0x81);
	volatile unsigned int i;
	for(i=0; i<0x30; i++);
	REG_ADDR8 (0x0d) = 0;
	REG_ADDR8 (0x0c) = 0xab;
	REG_ADDR8 (0x0d) = 0;
	REG_ADDR8 (0x0d) = 0;
	REG_ADDR8 (0x0d) = 1;
}


const TBLCMDSET  tbl_cpu_wakeup_init[] = {
	0x0060, 0x00,		TCMD_UNDER_BOTH | TCMD_WRITE,	//open all the clk,disable all the rst
	0x0061, 0x40,		TCMD_UNDER_BOTH | TCMD_WRITE,	//open all the clk,disable all the rst
	0x0062, 0x00,		TCMD_UNDER_BOTH | TCMD_WRITE,	//open all the clk,disable all the rst
	0x0063, 0xff,		TCMD_UNDER_BOTH | TCMD_WRITE,	//open all the clk,disable all the rst
	0x0064, 0xc7,		TCMD_UNDER_BOTH | TCMD_WRITE,
	//0x0065, 0xff,		TCMD_UNDER_BOTH | TCMD_WRITE,	//open all the clk,disable all the rst
	

	/*Start Close Code Encryption module to avoid reset 3 times program die problem*/
	0x0074, 0x53,		TCMD_UNDER_BOTH | TCMD_WRITE,
	0x007c, 0xf7,		TCMD_UNDER_BOTH | TCMD_WRITE,
	0x0074, 0x00,		TCMD_UNDER_BOTH | TCMD_WRITE,
	/*End Close Code Encryption module to avoid reset 3 times program die problem*/



	0x0067, 0x00,		TCMD_UNDER_BOTH | TCMD_WRITE,	//open all the clk,disable all the rst
	0x0066, 0x00,		TCMD_UNDER_BOTH | TCMD_WRITE,	//open all the clk,disable all the rst
	0x0073, 0x00,		TCMD_UNDER_BOTH | TCMD_WRITE,	//low power divider disable
	0x0620, 0x01,		TCMD_UNDER_BOTH | TCMD_WRITE,
	0x074f, 0x01,		TCMD_UNDER_BOTH | TCMD_WRITE,	//enable system tick

	////////////////////////////8886 deepsleep analog register recover////////////////////////////
//	0x04, 0xe7,		TCMD_UNDER_BOTH | TCMD_WAREG,	//
//	0x25, 0xb0,		TCMD_UNDER_BOTH | TCMD_WAREG,	//
//	0x26, 0x15,		TCMD_UNDER_BOTH | TCMD_WAREG,	//
	//0x80, 0x41,		TCMD_UNDER_BOTH | TCMD_WAREG,	//turn off cap
	0x81, 0xc0,		TCMD_UNDER_BOTH | TCMD_WAREG,	//turn off cap
//	0x05, 0x6a,		TCMD_UNDER_BOTH | TCMD_WAREG,	//turn off crystal pad
#if	REVISION_8266_A2
	0x20, 0xea,		TCMD_UNDER_BOTH | TCMD_WAREG,	//wakeup reset time: 1500us
#else
	0x20, 0xd0,		TCMD_UNDER_BOTH | TCMD_WAREG,	//wakeup reset time: 1500us
#endif
	0x2d, 0x0f,		TCMD_UNDER_BOTH | TCMD_WAREG,	//quick settle: 200 us
	0x81, 0xc0,		TCMD_UNDER_BOTH | TCMD_WAREG,	//xtal cap off to quick settle osc

	/////////////////////////////////////////////////////////////////////////////////////////////
	//0x30, 0xff,		TCMD_UNDER_BOTH | TCMD_WAREG,	//cal wait len
	0x2c, 0x00,		TCMD_UNDER_BOTH | TCMD_WAREG,	//ISO, llldo default
	0x2f, 0x6c,		TCMD_UNDER_BOTH | TCMD_WAREG,	//use cal output for 32K/32M reference
	0x2f, 0xcc,		TCMD_UNDER_BOTH | TCMD_WAREG,	//cal wait len
	0x2f, 0xcd,		TCMD_UNDER_BOTH | TCMD_WAREG,	//cal wait len
	0x05, 0x62,		TCMD_UNDER_BOTH | TCMD_WAREG,	//turn on crystal pad
//	0x00, 0xe8,		TCMD_UNDER_BOTH | TCMD_WAREG,	//xtal ldo: 1.95v
};

unsigned char	cap_32m, cap_32k, cres_32k;
void cpu_rc_clock_calibration (void){
	static u32 cal_t32m, cal_t32k;
	volatile int i;
	cal_t32m = read_reg32 (0x800630);
	for (i=0; i<500; i++) {
		if (analog_read (0x44) & 0x20) {
			cap_32m = analog_read (0x47);
			analog_write (0x33, cap_32m);
			//analog_write (0x2f, 0x8e);	//
			cap_32k = i;
			break;
		}
	}
	cal_t32m = read_reg32 (0x800630) - cal_t32m;

#if 0
	//////////// cal 32k: coarse /////////////
	cal_t32k = read_reg32 (0x800630);
	analog_write (0x2f, 0xac);	//
	analog_write (0x2e, 0xf6);	//
	for (i=0; i<100; i++);
	analog_write (0x2e, 0xf7);	//
	for (i=0; i<10000; i++) {
		if (analog_read (0x44) & 0x10) {
			cap_32k = analog_read (0x45);
			analog_write (0x32, cap_32k);
			break;
		}
	}
#else
	//cal_t32k = read_reg32 (0x800630) - cal_t32k;
	///////   32K fine tune  //////////////////////
	analog_write (0x2f, 0xbc);	//
	analog_write (0x2e, 0xf6);	//
	for (i=0; i<100; i++);
	analog_write (0x2e, 0xf7);	//
	for (i=0; i<10000; i++) {
		if (analog_read (0x44) & 0x10) {
			cap_32k = analog_read (0x45);
			cres_32k = analog_read (0x46);
			analog_write (0x31, cres_32k);
			analog_write (0x32, cap_32k);
			//analog_write (0x2f, 0x8e);	//
			break;
		}
	}
#endif
	cal_t32k = read_reg32 (0x800630) - cal_t32k;
	analog_write (0x2f, 0x8c);	//
}

void cpu_wakeup_init(void){
	LoadTblCmdSet (tbl_cpu_wakeup_init, sizeof (tbl_cpu_wakeup_init)/sizeof (TBLCMDSET));
	//available user register: 30 ~ 3e, 3f[6:0]
	//cpu_rc_clock_calibration ();
}

void cpu_system_timer_test () {
	sleep_us (10000);
	static u32 at0, at1;
	at0 = clock_time ();
	at1 = read_reg32 (0x800740);
	write_reg8(0x80074f,  BIT(1));	//change system timer to 32K
	sleep_us (10000);
	write_reg8(0x80074f, BIT(0));	//change back
	while (!(read_reg8 (0x80074f) & BIT(1)));
	//sleep_us (100);
	at0 = clock_time () - at0;
	at1 = read_reg32(0x800740) - at1;
}

u32			cpu_system_tick;
int			cpu_tick_adjust_us;
void cpu_set_system_tick (u32 tick) {
	cpu_system_tick = tick;
	write_reg32 (0x800740, cpu_system_tick);
}

_attribute_ram_code_ void cpu_adjust_system_tick (int adjust_us) {
	cpu_tick_adjust_us = adjust_us;
}

#if 0
u32 a0, a1, a2, a3, a4, a5, at, an;
void cpu_suspend_wakeup_sim (int us) {
	a0 = clock_time ();
	a5 = read_reg32 (0x800750);
	at = analog_read (0x40) + (analog_read (0x41) << 8) + (analog_read (0x42) << 16);
	int tick_next = (us - cpu_tick_adjust_us) * CLOCK_SYS_CLOCK_1US;
	u32 cpu_system_tick_next = cpu_system_tick + tick_next;
	u32 diff = cpu_system_tick_next - read_reg32 (0x800740);
	if (diff > 0x80000000) {
		cpu_system_tick_next = read_reg32 (0x800740) + tick_next;
	}
	write_reg32 (0x800748, cpu_system_tick_next);
	write_reg8  (0x80074c, 0xd0);

	//analog_write (0x44, 0x0f);				//clear wakeup flag
	write_reg8 (0x80074f, BIT(1));			//change system tick to 32K RC tick
	while (read_reg8 (0x80074f) & BIT(1));

	a4 = read_reg32 (0x800740);

	u32 t = clock_time ();
	//while (!(analog_read (0x44) & BIT(1)) && !clock_time_exceed (t, 100000));
	while (!(read_reg8 (0x80064a) & BIT(3)) && !clock_time_exceed (t, 100000));

	write_reg8(0x80074f, BIT(0));			//change back to system tick
	while (!(read_reg8 (0x80074f) & BIT(1)));

	a1 = clock_time () - a0;
	a0 = analog_read (0x44);
	a2 = read_reg32 (0x800740);
	a3 = read_reg32 (0x800748);
	an ++;
	while (!(read_reg8 (0x80064a) & BIT(3)));
	a1 = analog_read (0x40) + (analog_read (0x41) << 8) + (analog_read (0x42) << 16);
	if (an == 1)
		while (1);

	cpu_system_tick = read_reg32 (0x800740);
	analog_write (0x44, 0x0f);				//clear wakeup flag
	write_reg8 (0x80064a, BIT(3));
	sleep_us (500);
}

#else

void cpu_suspend_wakeup_sim (int us) {
	static u32 cpu_wakeup_tick;

	cpu_wakeup_tick += cpu_tick_adjust_us * CLOCK_SYS_CLOCK_1US;

	while (!clock_time_exceed (cpu_wakeup_tick, us));
	cpu_wakeup_tick = clock_time ();
}

#endif

void cpu_config_wakeup (u32 tick_us, u32 gpio, u32 wakeup_polarity) {
	//while (!clock_time_exceed (cpu_32k_tick, ms));
	//cpu_32k_tick = clock_time ();
	//sleep_us (500);
}

#define AREG_PAD_WAKEUP_EN(i)				((i>>8) + 0x27)
#define AREG_PAD_WAKEUP_POL(i)				((i>>8) + 0x21)

void cpu_set_gpio_wakeup (int pin, int pol, int en) {
	///////////////////////////////////////////////////////////
	// wakeup en: 	0x27 - 0x2b
	// pol:			0x21 - 0x25
	// 0x26[1:0]: pf_pol
	// 0x26[3:2]: pf_wakeup_en
	// 0x26[4]:	  reserved
	// 0x26[5]:	  wakeup_dig_en
	// 0x26[6]:	  wakeup_32k_en
	// 0x26[7]:	  wakeup_comp_en
	////////////////////////// polarity ////////////////////////
	u8 mask = pin & 0xff;
	if(!mask)
		return;

	u8 areg = AREG_PAD_WAKEUP_POL (pin);
	u8 val = analog_read (areg);
	if (pol) {
		val &= ~mask;
	}
	else {
		val |= mask;
	}
	analog_write (areg, val);		//polarity

	/////////////////////////// enable /////////////////////
	areg = AREG_PAD_WAKEUP_EN (pin);
	if (areg == 0x2c) {
		areg = 0x26;
		mask <<= 2;
	}
	val = analog_read (areg);
	if (en) {
		val |= mask;
	}
	else {
		val &= ~mask;
	}
	analog_write (areg, val);
}


#define		SYSTEM_TICK_ALWAYS_ON		1

u32 TICK_BEFORE_SUSPAND;
u32 TICK_AFTER_SUSPAND;
u32 TICK_AFTER_SUSPAND2;
u32 TICK_AFTER_SUSPAND3;
u32 TICK_AFTER_SUSPAND4;


u32 cpu_wakup_last_tick;
u32 cpu_working_tick;
_attribute_ram_code_ int cpu_sleep_wakeup (int deepsleep, int wakeup_src, u32 wakeup_tick) {

	u8 r = reg_irq_en;
	reg_irq_en = 0;
	int system_tick_enable = SYSTEM_TICK_ALWAYS_ON || wakeup_src & PM_WAKEUP_TIMER;

    analog_write(0x2c, 0x00);

	/////////////////// set wakeup source /////////////////////////////////
	///////////////// store gpio wakeup enable setting ////////////////////
	u8 i;
	u8 ana_gpio[6];
	ana_gpio[0] = analog_read (0x26);
	if (! (wakeup_src & PM_WAKEUP_PAD)) {	//PM_WAKEUP_PAD disabled
		for (i=1; i<6; i++) {
			ana_gpio[i] = analog_read (0x26 + i);
			analog_write (0x26 + i, 0);
		}
		analog_write (0x26, wakeup_src);
	}
	else {
		analog_write (0x26, (ana_gpio[0] & 0x0f) | wakeup_src);
	}

	u8 anareg01 = analog_read(0x01);		//
//	u8 anareg05 = analog_read(0x05);		//
	u8 anareg2c = analog_read(0x2c);		//power down control
	u8 anareg80 = analog_read(0x80);		//xtal cap
	u8 anareg81 = analog_read(0x81);		//xtal cap

    write_reg8(0x80006e,0x00);
    if (wakeup_src & PM_WAKEUP_CORE) {
        write_reg8(0x80006e,0x08);
    }
	analog_write (0x44, 0x0f);				//clear all flag
	
	////////////////////////////// set wakeup tick ////////////////////////////
	if (system_tick_enable) {
		anareg2c &= 0xfe;						//32K RC need be enabled
		reg_system_wakeup_tick = wakeup_tick;
		reg_system_tick_ctrl = FLD_SYSTEM_TICK_STOP;
		while (reg_system_tick_ctrl & FLD_SYSTEM_TICK_RUNNING);
	}
	else {
		write_reg8(0x80074c,0x20);
		anareg2c |= BIT(0);
	}

	///////////////////////// change to 32M RC clock before suspend /////////////
	u8 reg66 = read_reg8(0x800066);			//
	//write_reg8 (0x800066, 0);				//change to 32M rc clock  ////////remove

	//////////////////////// set deepsleep flag
	if (deepsleep) {
		analog_write(rega_deepsleep_flag, analog_read(rega_deepsleep_flag) | 0x01);
	} else {
	    analog_write(rega_deepsleep_flag, analog_read(rega_deepsleep_flag) & 0xfe);
    }

	analog_write(0x2c, (deepsleep ? 0xfe : 0x5e) | anareg2c);
	analog_write(0x01, anareg01 | BIT(3));		//floating R to reduce DVDD leakage
	analog_write (0x81, 0xc0);					//turn off xtal cap
	// 0x80: bit7-aac off; bit4-pd_dig; bit3-pd_rfpll
	analog_write (0x80, 0xa1);					//turn off xtal 4.6p cap

    //TICK_BEFORE_SUSPAND = reg_system_tick;
    
    //while((analog_read(0x44) & 0x0f)==0);

	if(deepsleep){
		sleep_start();
	}
	else{
		suspend_start();
	}

    //TICK_AFTER_SUSPAND = reg_system_tick;

//	volatile int k;
//	while (1) {write_reg8 (0x800001, k++);}
//	analog_write (0x05, anareg05 | BIT(3));
//	analog_write (0x05, anareg05 & ~BIT(3));

	u8 anareg44 = analog_read(0x44);

	/*suspend recover setting*/
	analog_write(0x01, anareg01);
	analog_write(0x2c, anareg2c);

	if (deepsleep) {		// reboot ?

	}
	cpu_wakup_last_tick = clock_time ();			// system tick on wakeup

	analog_write (0x80, anareg80);			//restore xtal cap setting
	analog_write (0x81, anareg81);			//restore xtal cap setting
	//////////////////// restore GPIO wake-up configuration ////////////////
	if (! (wakeup_src & PM_WAKEUP_PAD)) {
		for (i=0; i<6; i++) {
			analog_write (0x26 + i, ana_gpio[i]);
		}
	}

    //TICK_AFTER_SUSPAND2 = reg_system_tick;
	////////////////// system clock has to be on ////////////////////////////
	write_reg8 (0x800066, reg66);			//restore system clock

	if (system_tick_enable) {
		reg_system_tick_ctrl = FLD_SYSTEM_TICK_START;
		while (!(reg_system_tick_ctrl & FLD_SYSTEM_TICK_RUNNING));
        //TICK_AFTER_SUSPAND3 = reg_system_tick;
	}

	write_reg8(0x80074c, 0x90);					//auto mode
	write_reg8(0x80074f, 0x01);					//enable system timer
	//WaitUs(1);
	//TICK_AFTER_SUSPAND4 = reg_system_tick;

    //WaitUs(1600);

	reg_irq_en = r;

	return anareg44;
}

_attribute_ram_code_ void kb_cpu_sleep_wakeup (int deepsleep, int wakeup_src, u32 wakeup_tick) {

	u8 r = reg_irq_en;
	reg_irq_en = 0;
	int system_tick_enable = SYSTEM_TICK_ALWAYS_ON || wakeup_src & PM_WAKEUP_TIMER;

	/////////////////// set wakeup source /////////////////////////////////
	///////////////// store gpio wakeup enable setting ////////////////////
	if (wakeup_src & PM_WAKEUP_PAD) {
		analog_write (0x26, (analog_read (0x26) & 0x0f) | wakeup_src);
	}
	else {
		analog_write (0x26, wakeup_src);
	}

	u8 anareg2c;
	u8 anareg01 = analog_read(0x01);
	u8 anareg80 = analog_read(0x80);		//xtal cap
	u8 anareg81 = analog_read(0x81);		//xtal cap

    write_reg8(0x80006e,wakeup_src & PM_WAKEUP_CORE ? 0x08 : 0x00);

	analog_write (0x44, 0x0f);				//clear all flag

	////////////////////////////// set wakeup tick ////////////////////////////
	if (system_tick_enable) {
		anareg2c = 0x00;						//32K RC need be enabled
		reg_system_wakeup_tick = wakeup_tick;
		reg_system_tick_ctrl = FLD_SYSTEM_TICK_STOP;
		while (reg_system_tick_ctrl & FLD_SYSTEM_TICK_RUNNING);
	}
	else {
		write_reg8(0x80074c,0x20);
		anareg2c = 0x01;
	}

	//////////////////////// set deepsleep flag
	if (deepsleep) {
		analog_write(rega_deepsleep_flag, analog_read(rega_deepsleep_flag) | 0x01);
	} else {
	    analog_write(rega_deepsleep_flag, analog_read(rega_deepsleep_flag) & 0xfe);
    }

	analog_write(0x2c, (deepsleep ? 0xfe : 0x5e) | anareg2c);
	analog_write(0x01, anareg01 | BIT(3));		//floating R to reduce DVDD leakage
	analog_write (0x81, 0xc0);					//turn off xtal cap
	// 0x80: bit7-aac off; bit4-pd_dig; bit3-pd_rfpll
	analog_write (0x80, 0xa1);					//turn off xtal 4.6p cap

	reg_gpio_pa_ie &= 0xf3;
	reg_gpio_pb_ie &= 0xf3;
	//dbg_led_low;
	sleep_start();
	//dbg_led_high;
	reg_gpio_pa_ie |= 0x0c;
	reg_gpio_pb_ie |= 0x0c;

	/*suspend recover setting*/
	analog_write(0x01, anareg01);
	analog_write(0x2c, anareg2c);

	if (deepsleep) {		// reboot ?

	}
	cpu_wakup_last_tick = clock_time ();			// system tick on wakeup

	analog_write (0x80, anareg80);			//restore xtal cap setting
	analog_write (0x81, anareg81);			//restore xtal cap setting

    //TICK_AFTER_SUSPAND2 = reg_system_tick;
	////////////////// system clock has to be on ////////////////////////////
	//write_reg8 (0x800066, reg66);			//restore system clock

	if (system_tick_enable) {
		reg_system_tick_ctrl = FLD_SYSTEM_TICK_START;
		while (!(reg_system_tick_ctrl & FLD_SYSTEM_TICK_RUNNING));
        //TICK_AFTER_SUSPAND3 = reg_system_tick;
	}

	write_reg8(0x80074c, 0x90);					//auto mode
	write_reg8(0x80074f, 0x01);					//enable system timer

	reg_irq_en = r;
}

void cpu_usb_suspend_wakeup(void){

}

int cpu_sleep_wakeup_rc (int deepsleep, int wakeup_src, u32 wakeup_ms)
{

}

#endif

