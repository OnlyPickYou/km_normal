
#if(!(__TL_LIB_8267__ ))
#include "../proj/config/user_config.h"
#include "../proj/mcu/config.h"
#endif

#ifndef			OTA_PROGRAM_SIZE
#define			OTA_PROGRAM_SIZE			(64*1024)
#endif


#if(__TL_LIB_8267__ || (MCU_CORE_TYPE && MCU_CORE_TYPE == MCU_CORE_8267))
#include "../proj/tl_common.h"
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

#define 		POWER_SAVING_16MPAD_DIGITAL		1   //16MHz clock to digital


#define RESET_WAKEUP_TIME_2000 		1



#if (RESET_WAKEUP_TIME_2000)
	#define RESET_TIME_US	    	  1970
	#define EARLYWAKEUP_TIME_US       2100
	#define EMPTYRUN_TIME_US       	  2400
#elif(RESET_WAKEUP_TIME_2200)
	#define RESET_TIME_US	    	  2170
	#define EARLYWAKEUP_TIME_US       2300
	#define EMPTYRUN_TIME_US       	  2600
#elif(RESET_WAKEUP_TIME_2400)
	#define RESET_TIME_US	    	  2370
	#define EARLYWAKEUP_TIME_US       2500
	#define EMPTYRUN_TIME_US       	  2800
#elif(RESET_WAKEUP_TIME_2600)
	#define RESET_TIME_US	    	  2570
	#define EARLYWAKEUP_TIME_US       2700
	#define EMPTYRUN_TIME_US       	  3000
#elif(RESET_WAKEUP_TIME_2800)
	#define RESET_TIME_US	    	  2770
	#define EARLYWAKEUP_TIME_US       2900
	#define EMPTYRUN_TIME_US       	  3200
#else
#endif



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

int			cpu_tick_adjust_us;
void cpu_adjust_system_tick (int adjust_us) {
	cpu_tick_adjust_us = adjust_us;
}

////////////////////// End of Battery ////////////////////////////////////////////
_attribute_ram_code_ _attribute_no_inline_ void  sleep_start(void){
	volatile unsigned int i;

	REG_ADDR8 (0x0d) = 0;
	REG_ADDR8 (0x0c) = 0xb9;
	for(i=0; i<2; i++); //1440ns when 32M clk
	REG_ADDR8 (0x0d) = 1;

	REG_ADDR8(0x58b) &= 0xfe; //SWS dataO -> 0 , close digital pullup
	REG_ADDR8(0x5a1) &= 0x0f; //MSPI ie disable
	write_reg8(0x80006f,0x81);
	for(i=0; i<0x30; i++);
	REG_ADDR8(0x5a1) |= 0xf0; //MSPI ie enable
	REG_ADDR8(0x58b) |= 0x01; //SWS dataO -> 1 , open digital pullup


	REG_ADDR8 (0x0d) = 0;
	REG_ADDR8 (0x0c) = 0xab;
	for(i=0; i<2; i++); //1440ns when 32M clk
	REG_ADDR8 (0x0d) = 1;
}


_attribute_no_inline_ void  suspend_start(void)
{

	volatile unsigned int i;

	REG_ADDR8 (0x0d) = 0;
	REG_ADDR8 (0x0c) = 0xb9;
	for(i=0; i<2; i++); //1440ns when 32M clk
	REG_ADDR8 (0x0d) = 1;

	write_reg8(0x80006f,0x81);
	for(i=0; i<0x30; i++);

	REG_ADDR8 (0x0d) = 0;
	REG_ADDR8 (0x0c) = 0xab;
	for(i=0; i<2; i++); //1440ns when 32M clk
	REG_ADDR8 (0x0d) = 1;
}

const TBLCMDSET  tbl_cpu_wakeup_init[] = {
	0x0060, 0x00,		TCMD_UNDER_BOTH | TCMD_WRITE,	//open all the clk,disable all the rst
	0x0061, 0x00,		TCMD_UNDER_BOTH | TCMD_WRITE,	//open all the clk,disable all the rst
	0x0062, 0x00,		TCMD_UNDER_BOTH | TCMD_WRITE,	//open all the clk,disable all the rst
	0x0063, 0xff,		TCMD_UNDER_BOTH | TCMD_WRITE,	//open all the clk,disable all the rst
	0x0064, 0xff,		TCMD_UNDER_BOTH | TCMD_WRITE,
	//0x0065, 0xff,		TCMD_UNDER_BOTH | TCMD_WRITE,	//open all the clk,disable all the rst
//	0x063d, 0x1f,		TCMD_UNDER_BOTH | TCMD_WRITE,	//fast SRB speed, not working with USB MCU mode

//	0x0067, 0x00,		TCMD_UNDER_BOTH | TCMD_WRITE,	//open all the clk,disable all the rst
//	0x0066, 0x00,		TCMD_UNDER_BOTH | TCMD_WRITE,	//open all the clk,disable all the rst
	0x0073, 0x04,		TCMD_UNDER_BOTH | TCMD_WRITE,	//low power divider disable
	0x0620, 0x01,		TCMD_UNDER_BOTH | TCMD_WRITE,
	0x074f, 0x01,		TCMD_UNDER_BOTH | TCMD_WRITE,	//enable system tick


#if(POWER_SAVING_16MPAD_DIGITAL)
	0x80, 0x71,  TCMD_UNDER_BOTH | TCMD_WAREG,  //16MHz clock to digital power down
#else
	0x80, 0x61,  TCMD_UNDER_BOTH | TCMD_WAREG,
#endif

	0x20, 0xc1,		TCMD_UNDER_BOTH | TCMD_WAREG,

	0x2d, 0x0f,		TCMD_UNDER_BOTH | TCMD_WAREG,	//quick settle: 200 us

	//ana_2f<0>, 32M RC calib enable will leads to IC die in high temp, fixed by sihui 20160419
	//0x2f, 0xcc,		TCMD_UNDER_BOTH | TCMD_WAREG,	//cal wait len
	//0x2f, 0xcd,		TCMD_UNDER_BOTH | TCMD_WAREG,	//cal wait len

	0x05, 0x62,		TCMD_UNDER_BOTH | TCMD_WAREG,	//turn off crystal pad,		bit7: bbpll		(8267)
	0x88, 0x0f,		TCMD_UNDER_BOTH | TCMD_WAREG,	//bit[1:0]: 192M CLOCK to digital			(8267)
};


u32		ota_program_offset;
void cpu_wakeup_init(void){
	LoadTblCmdSet (tbl_cpu_wakeup_init, sizeof (tbl_cpu_wakeup_init)/sizeof (TBLCMDSET));
	if (REG_ADDR8(0x63e))
	{
		REG_ADDR8(0x63e) = (REG_ADDR8(0x63e) & 0x3) | ((OTA_PROGRAM_SIZE>>12)<<2);
		ota_program_offset = 0;
	}
	else
	{
		ota_program_offset = 0x20000;
	}

	u8 buf[4] = {0};
	flash_read_page(ota_program_offset, 4, buf);
	u32 tmp = buf[0] | (buf[1]<<8) |(buf[2]<<16) | (buf[3]<<24);
	if(tmp != ONES_32){
		foreach(i, 16){
			flash_erase_sector(ota_program_offset+i*0x1000);
		}
	}

	//cpu_rc_clock_calibration ();
}



#define AREG_PAD_WAKEUP_EN(i)				((i>>8) + 0x27)
#define AREG_PAD_WAKEUP_POL(i)				((i>>8) + 0x21)

void cpu_set_gpio_wakeup (int pin, int pol, int en) {
	///////////////////////////////////////////////////////////
	// 		  PA[7:0]	    PB[7:0]		PC[7:0]		PD[7:0]		 PE[3:0]
	// en: 	ana_21<7:0>	 ana_22<7:0>  ana_23<7:0>  ana_24<7:0>	ana_25<3:0>
	// pol:	ana_27<7:0>	 ana_28<7:0>  ana_29<7:0>  ana_2a<7:0>	ana_2b<3:0>
    unsigned char mask = pin & 0xff;
	unsigned char areg;
	unsigned char val;

	////////////////////////// polarity ////////////////////////
	areg = AREG_PAD_WAKEUP_POL(pin);
	val = ReadAnalogReg(areg);
	if (pol) {
		val &= ~mask;
	}
	else {
		val |= mask;
	}
	WriteAnalogReg (areg, val);

	/////////////////////////// enable /////////////////////
	areg = AREG_PAD_WAKEUP_EN(pin);
	val = ReadAnalogReg(areg);
	if (en) {
		val |= mask;
	}
	else {
		val &= ~mask;
	}
	WriteAnalogReg (areg, val);
}


#define		SYSTEM_TICK_ALWAYS_ON		0


u32 cpu_wakup_last_tick;
_attribute_no_inline_ u32		cpu_get_32k_tick ()
{
	u32		t0 = 0, t1 = 0, n;

	n = 0;
	//REG_ADDR8(0x74c) = 0x90;							//system timer auto mode enable
	REG_ADDR8(0x74c) = 0x28;							//system timer manual mode, interrupt disable
	while (1)
	{
		REG_ADDR8(0x74f) = BIT(3);							//start read
		while (REG_ADDR8(0x74f) & BIT(3));
		t0 = t1;
		t1 = REG_ADDR32(0x754);
		if (n)
		{
			if ((u32)(t1 - t0) < 2)
			{
				return t1;
			}
			else if ( (t0^t1) == 1 )	// -1
			{
				return t0;
			}
		}
		n++;
	}
	return t1;
}


#define SWAP_BIT0_BIT6(x)     ( ((x)&0xbe) | ( ((x)&0x01)<<6 ) | ( ((x)&0x40)>>6 )  )


_attribute_ram_code_ int cpu_sleep_wakeup (int deepsleep, int wakeup_src, u32 wakeup_tick)
{
	u16 tick_32k_calib = REG_ADDR16(0x750);
	u16 tick_32k_halfCalib = tick_32k_calib>>1;


	u8 long_suspend = 0;
	u32 span = (u32)(wakeup_tick - clock_time ());

	if(wakeup_src & PM_WAKEUP_TIMER){
		if (span > 0xc0000000)  //BIT(31)+BIT(30)   3/4 cylce of 32bit
		{
			return  analog_read (0x44) & 0x0f;
		}
		else if (span < EMPTYRUN_TIME_US * CLOCK_SYS_CLOCK_1US) // 0 us base
		{
			u32 t = clock_time ();
			analog_write (0x44, 0x0f);			//clear all status
			u8 st;
			do {
				st = analog_read (0x44) & 0x0f;
			} while ( ((u32)clock_time () - t < span) && !st);
			return st;
		}
		else
		{
			if( span > 0x0ff00000 ){  //BIT(28) = 0x10000000   16M:16S; 32M:8S  48M: 5.5S
				long_suspend = 1;
			}
		}
	}


	////////// disable IRQ //////////////////////////////////////////
	u8 r = irq_disable ();
	u32 tick_cur = clock_time ();
	u32 tick_32k_cur = cpu_get_32k_tick ();
	u32 tick_wakeup_reset = wakeup_tick - EARLYWAKEUP_TIME_US * CLOCK_SYS_CLOCK_1US;


	/////////////////// set wakeup source /////////////////////////////////
	///////////////// store gpio wakeup enable setting ////////////////////
	analog_write (0x26, wakeup_src);

	write_reg8(0x6e, wakeup_src & PM_WAKEUP_CORE ? 0x08 : 0x00);
	analog_write (0x44, 0x0f);				//clear all flag

	////////////////////////////// set wakeup tick ////////////////////////////
	u8 rc32k_pd = 0;
	if (wakeup_src & PM_WAKEUP_TIMER ) {
		rc32k_pd = 0x00;						//32K RC need be enabled
	}
	else {
		write_reg8(0x74c,0x20);
		rc32k_pd = 0x01;
	}

	///////////////////////// change to 32M RC clock before suspend /////////////
	u8 reg66 = read_reg8(0x066);			//
	write_reg8 (0x066, 0);				//change to 32M rc clock

	analog_write(0x2c, (deepsleep ? 0xfe : 0x5e) | rc32k_pd);
	analog_write(0x2d, analog_read(0x2d) & 0xdf);


	span = (RESET_TIME_US * CLOCK_SYS_CLOCK_1US * 16 + tick_32k_halfCalib)/ tick_32k_calib;

	u32 tick_reset;
	if(long_suspend){
		tick_reset = tick_32k_cur + (u32)(tick_wakeup_reset - tick_cur)/ tick_32k_calib * 16;
	}
	else{
		tick_reset = tick_32k_cur + ((u32)(tick_wakeup_reset - tick_cur) * 16 + tick_32k_halfCalib) / tick_32k_calib;
	}


	u8 rst_cycle =  0xff - span;
	analog_write (0x20, SWAP_BIT0_BIT6(rst_cycle));				// quick wake-up, short reset time
	REG_ADDR8(0x74c) = 0x2c;
	REG_ADDR32(0x754) = tick_reset;
	REG_ADDR8(0x74f) = BIT(3);									//start write

	while (REG_ADDR8(0x74f) & BIT(3));
	analog_write (0x44, 0x0f);								//clear all flag

	//CHN0_LOW;   //GPIO debug
	if(analog_read(0x44) & 0x0f){

	}
	else{
		sleep_start();
	}
	//CHN0_HIGH;  //GPIO debug

	if(deepsleep){
		write_reg8 (0x6f, 0x20);  //reboot
	}

	/*suspend recover setting*/
	analog_write (0x2c, 0x00);


	if(long_suspend){
		tick_cur += (u32)(cpu_get_32k_tick () - tick_32k_cur) / 16 * tick_32k_calib;
	}
	else{
		tick_cur += (u32)(cpu_get_32k_tick () - tick_32k_cur) * tick_32k_calib / 16;		// current clock
	}


	write_reg8 (0x066, reg66);			//restore system clock
	reg_system_tick = tick_cur;
	REG_ADDR8(0x74c) = 0x00;
	REG_ADDR8(0x74c) = 0x90;
	REG_ADDR8(0x74f) = BIT(0);


	u8 anareg44 = analog_read(0x44);

	if (anareg44 == 0x02)	//BIT(1) wakeup from timer only
	{
		while ((u32)(clock_time () -  wakeup_tick) > BIT(30));
	}

	irq_restore(r);    //////// irq

	return anareg44;
}




#endif
