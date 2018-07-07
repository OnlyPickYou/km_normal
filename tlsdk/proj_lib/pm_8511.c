
#if(!(__TL_LIB_8511__ ))
#include "../proj/config/user_config.h"
#include "../proj/mcu/config.h"
#endif 

#if(__TL_LIB_8511__ || (MCU_CORE_TYPE && MCU_CORE_TYPE == MCU_CORE_8511))

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
	0x13, 0x5f,		TCMD_UNDER_BOTH | TCMD_WAREG,	//wakeup reset time: (7f - 5f) * 2 = 64 tick = 2ms
	0x26, 0x40,		TCMD_UNDER_BOTH | TCMD_WAREG,	//32K cap
	0x10, 0x08,		TCMD_UNDER_BOTH | TCMD_WAREG,	//8ms mode default
	//0x12, 0x10,		TCMD_UNDER_BOTH | TCMD_WAREG,	//32K continuous mode
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

u32 tick_32k_bk;

/*
 * cavy dongle without deep sleep function
 */

_attribute_ram_code_ void cpu_adjust_system_tick (int adjust_ms)
{
	u32 t = tick_32k_bk + adjust_ms;
	analog_write(0x10, t);
	//analog_write(0x11, t>>8);
}

/*
 * suspend之后必须进行32k clock的tracking
 * calculate reset time: analog_write(0x13, x), reset_time = (7f-x)*2 cycle, 32 cycle = 1ms
 *
 * Trcaking time and Reset value Test
 * (1)Tracking_time = 50us, reset value = 0x7c(analog_write(0x13, 0x7c))
 * 5ms -> 5.06ms, 8ms -> 8.08ms, 10ms -> 10.04ms, 20ms -> 20.02ms, 100ms -> 100.5ms
 *
 * (2)Tracking_time = 150us, reset value = 0x7c(analog_write(0x13, 0x7c))
 * 5ms -> 5.06ms, 8ms -> 8.06ms, 10ms -> 10.06ms, 20ms -> 20.08ms, 100ms -> 100ms
 *
 * (3)Tracking_time = 50us, reset value = 0x77(analog_write(0x13, 0x77))
 * 5ms -> 5.09ms, 8ms -> 8.64ms, 10ms -> 10.04ms, 20ms -> 20.08ms, 100ms -> 100.96ms
 *
 * (4)Tracking_time = 150us, reset value = 0x77(analog_write(0x13, 0x77))
 * 5ms -> 5.08ms, 8ms -> 8.04ms, 10ms -> 10.04ms, 20ms -> 19.98ms, 100ms -> 100.2ms
 *
 * (5)Tracking_time = 50us, reset value = 0x70(analog_write(0x13, 0x70))
 * 5ms -> 5.06ms, 8ms -> 8.06ms, 10ms -> 10.05ms, 20ms -> 20ms, 100ms -> 100.4ms
 *
 */
u32  cpu_wakup_last_tick;
_attribute_ram_code_ int cpu_sleep_wakeup (int deepsleep, int wakeup_src, u32 wakeup_ms) {


	/////////////////// set wakeup source /////////////////////////////////
	///////////////// store gpio wakeup enable setting ////////////////////

	//analog_write(0x13, 0x5f);		// reset/wait time ( 7f-7a ) * 2= 10 cycle, r_dly

	u8 anareg00 = analog_read(0x00);
	u8 anareg02 = analog_read(0x02);

	u8 anareg80 = analog_read(0x80);
	u8 anareg81 = analog_read(0x81);

	//reg_wakeup_en = (wakeup_src & PM_WAKEUP_CORE) ? FLD_WAKEUP_SRC_GPIO : 0;	// GPIO WAKEUP


	if(wakeup_src & PM_WAKEUP_TIMER){
		if (tick_32k_bk != wakeup_ms) {
			//cfg =	ReadAnalogReg(0x17);
			analog_write(0x17, PM_32KRC_RESET);
			analog_write(0x17, 0);

			tick_32k_bk = wakeup_ms;						//  save suspend time, step is 1ms
			analog_write(0x10,(wakeup_ms)&0xff);			//	[7:0]
			analog_write(0x11,(wakeup_ms>>8)&0x07);			//	[10:8]


			analog_write (0x23, 0x80);		//kick start 32K
		}
	}


	analog_write (0x17, wakeup_src | PM_AUTO_PWDN_EN);


	///////////////////////// change to 32M RC clock before suspend /////////////
//	u8 reg66 = REG_ADDR8(0x66);
//	write_reg8 (0x800066, 2);				// change to 32M RC clock, but cavy dongle has no clock source select


	//////////////////////// set deepsleep flag ////////////////////
	if(!deepsleep){
		//analog_write (0x01, anareg01 | 0x08);  //set BIT(3)

		analog_write (0x00, 0x08);				//prevent digital LDO resistor leakage
		analog_write (0x02, 0x07);  			//prevent bbpll LDO resister leakage
	}

	//analog_write (0x18, (deepsleep ? PM_AUTO_PWDN_DEEPSLEEP : PM_AUTO_PWDN_SUSPEND) | \
				((wakeup_src & PM_WAKEUP_TIMER) ? 0 : PM_AUTO_PWDN_32K));

	analog_write (0x18, (deepsleep ? PM_AUTO_PWDN_DEEPSLEEP : PM_AUTO_PWDN_SUSPEND) | \
				((wakeup_src & PM_WAKEUP_TIMER) ? PM_AUTO_PWDN_32K : 0));


	analog_write (0x80, 0x6a);
	analog_write (0x81, 0x08);


	analog_write (0x23, 0x0f);  //clear status

	sleep_start();

	cpu_wakup_last_tick = clock_time();


//	analog_write (0x80, anareg80);			//restore xtal cap setting
//	analog_write (0x81, anareg81);			//restore xtal cap setting

	analog_write (0x00, anareg00);
	analog_write (0x02, anareg02);

	analog_write (0x80, anareg80);
	analog_write (0x81, anareg81);

	////////////////// system clock has to be on ////////////////////////////
//	write_reg8 (0x800066, reg66);			//restore system clock

	return analog_read(0x23);

}

#endif

