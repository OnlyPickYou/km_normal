
#if(!(__TL_LIB_8510__ ))
#include "../proj/config/user_config.h"
#include "../proj/mcu/config.h"
#endif 

#if(__TL_LIB_8510__ || (MCU_CORE_TYPE && MCU_CORE_TYPE == MCU_CORE_8510))

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



cavym_p_event_callback_t cavym_event_func[2] = {0};		//最多支持2个底层事件


void cavym_event_callback_void(u8 e, u8 *p) {
	return;
}

cavym_p_event_callback_t cavym_p_event_callback = &cavym_event_callback_void;	//没有调用，函数为空

_attribute_ram_code_ void cavym_event_callback_func(u8 e, u8 *p)
{
	if(cavym_event_func[e]){
		cavym_event_func[e](e, p);
	}
}

void cavym_register_event_callback(u8 e, cavym_p_event_callback_t p)
{
	cavym_p_event_callback = cavym_event_callback_func;
	cavym_event_func[e] = p;
}



#if(!CAVYM_ROM_CODE_ENABLE)
void cpu_set_gpio_wakeup (int pin, int pol, int en) {
	///////////////////////////////////////////////////////////

	/*
	 * wkup_pad_en:	0x16  wakeup_pad_en, BIT(0):GP5, BIT(1):GP6, BIT(2):GP7, BIT(3):MSDI
	 * pol:			0x14  BIT(4):GP5, BIT(5):GP6, BIT(6),GP7, BIT(7):MSDI;
	 * 				0x17  0:wakeup_dig_en, 1:wakeup_32k_en, 2:wakeup_pad_en
	 */

	u8 en_mask;
	u8 pol_mask;
	u8 val;

	u8 bit = pin & 0xff;

	/*************************** wakeup_pad_enable ***************************/

	if(pin & 0x70){		//  0111 0000:  GP7-GP5,   0001 0000 1000 //MSDI   //102 MCLK, 100 MSCN
		en_mask = (bit >> 4);
	}

	else if(pin & 0x108){
		en_mask = BIT(3);
	}
	else{  									//other gpio func return
		return;
	}
	/*************************** pad polarity ****************************/

	if(pin & 0x10){							//GP5
		pol_mask = BIT(4);
	}
	else if(pin & 0x20){					//GP6
		pol_mask = BIT(5);
	}
	else if(pin & 0x40){					//GP7
		pol_mask = BIT(6);
	}
	else if(pin & 0x108){					//MSDI,MSCN
		pol_mask = BIT(7);
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
		//val &= ~pol_mask;
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
#else
void cpu_set_gpio_wakeup (int pin, int pol, int en) {
	///////////////////////////////////////////////////////////

	/*
	 * wkup_pad_en:	0x16  wakeup_pad_en, BIT(0):GP5, BIT(1):GP6, BIT(2):GP7, BIT(3):MCLK
	 * pol:			0x14  BIT(4):GP5, BIT(5):GP6, BIT(6),GP7, BIT(7):MCLK;
	 * 				0x17  0:wakeup_dig_en, 1:wakeup_32k_en, 2:wakeup_pad_en
	 */

	u8 en_mask;
	u8 pol_mask;
	u8 val;

	u8 bit = pin & 0xff;

	/*************************** wakeup_pad_enable ***************************/

	if(pin & 0x70){		//  0111 0000:  GP7-GP5,   0001 0000 0010 : MCLK
		en_mask = (bit >> 4);
	}

	else if(pin & 0x102){
		en_mask = BIT(3);
	}
	else{  									//other gpio func return
		return;
	}
	/*************************** pad polarity ****************************/

	if(pin & 0x10){							//GP5
		pol_mask = BIT(4);
	}
	else if(pin & 0x20){					//GP6
		pol_mask = BIT(5);
	}
	else if(pin & 0x40){					//GP7
		pol_mask = BIT(6);
	}
	else if(pin & 0x102){					//MSCN
		pol_mask = BIT(7);
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
		//val &= ~pol_mask;
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


#endif



u32 tick_32k_bk;


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

	//analog_write(0x13, 0x5f);		// reset/wait time ( 7f-7a ) * 2= 10 cycle

//	u8 anareg80 = analog_read(0x80);		//xtal cap
//	u8 anareg81 = analog_read(0x81);		//xtal cap

	//u8 anareg01 = analog_read(0x01);
	u8 anareg02 = analog_read(0x02);

	reg_wakeup_en = (wakeup_src & PM_WAKEUP_CORE) ? FLD_WAKEUP_SRC_GPIO : 0;	// GPIO WAKEUP


	if(wakeup_src & PM_WAKEUP_TIMER){
		if (tick_32k_bk != wakeup_ms) {
			//cfg =	ReadAnalogReg(0x17);
			analog_write(0x17, PM_32KRC_RESET);
			analog_write(0x17, 0);

			tick_32k_bk = wakeup_ms;
			analog_write(0x10,(wakeup_ms)&0xff);
			analog_write(0x11,(wakeup_ms>>8)&0xff);//
			analog_write(0x12,(wakeup_ms>>16)|0x10);

			analog_write (0x23, 0x80);		//kick start 32K
		}
	}


	analog_write (0x17, wakeup_src | PM_AUTO_PWDN_EN);

	///////////////////////// change to 32M RC clock before suspend /////////////
	u8 reg66 = REG_ADDR8(0x66);
	write_reg8 (0x800066, 0);				//change to 32M rc clock


	//////////////////////// set deepsleep flag ////////////////////
	if(!deepsleep){
		//analog_write (0x01, anareg01 | 0x08);  //set BIT(3)
		analog_write (0x01, 0x5c);
		analog_write (0x02, anareg02 & 0xfb);  //clear BIT(2)
	}

	analog_write (0x18, (deepsleep ? PM_AUTO_PWDN_DEEPSLEEP : PM_AUTO_PWDN_SUSPEND) | \
				((wakeup_src & PM_WAKEUP_TIMER) ? 0 : PM_AUTO_PWDN_32K));

//	analog_write (0x81, 0xe0);					//turn off xtal cap
//	analog_write (0x80, 0x20);					//turn off xtal 4.6p cap



	analog_write (0x23, 0x0f);  //clear status

	sleep_start();

	cavym_p_event_callback(EVENT_POWERUP_SENSOR_PIN, 0);

	cpu_wakup_last_tick = clock_time();


//	analog_write (0x80, anareg80);			//restore xtal cap setting
//	analog_write (0x81, anareg81);			//restore xtal cap setting
	//analog_write (0x01, anareg01);			//
	analog_write (0x01, 0x54);
	analog_write (0x02, anareg02);			//

	////////////////// system clock has to be on ////////////////////////////
	write_reg8 (0x800066, reg66);			//restore system clock

	return analog_read(0x23);

}

#endif

