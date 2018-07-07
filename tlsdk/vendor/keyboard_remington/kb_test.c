/*
 * kb_test.c
 *
 *  Created on: 2015-1-23
 *      Author: Administrator
 */

#include "../../proj/tl_common.h"
#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"
#include "../common/rf_frame.h"

#include "kb_rf.h"


void debug_sys_tick(void)
{
	static u32 tick = 0;
	if(clock_time_exceed(tick,1000000)){
		tick = clock_time();
		pkt_km.per++;
	}
}


void adc_test(void)
{
	rf_power_enable(1);

	analog_write(0x08,0);  //float
	analog_write(0x09,0);  //float

	gpio_set_input_en(GPIO_GP17,0);
	gpio_set_input_en(GPIO_GP18,0);
	gpio_set_input_en(GPIO_GP22,0);
	gpio_set_input_en(GPIO_GP23,0);

	//AD clk = src_clk * step/mod = 32M(OSC) * 4/32 = 4M
	//ad time = sample_time + bit_store_time = (cycle + res_len) * T =
	REG_ADDR8(0x69) = 4;  //step
	REG_ADDR8(0x6a) = CLK_FHS_MZ;  //mod
	REG_ADDR8(0x6b) = 0x80;  //adc clk en

	REG_ADDR8(0x33) = 0x00;  //no auto
	REG_ADDR8(0x2b) = 0x00;  //ref:Vbg 1.224V
	//REG_ADDR8(0x2c) = 0x01;  //GP17
	//REG_ADDR8(0x2c) = 0x02;  //GP18
	//REG_ADDR8(0x2c) = 0x03;  //GP22
	REG_ADDR8(0x2c) = 0x04;  //GP23
	REG_ADDR8(0x3c) = 0x18;  //10 bit 3cycle  13*0.25us = 3.25us

	pkt_pairing.did = 0x77777777;
	rf_set_power_level_index (RF_POWER_8dBm);
	rf_set_channel (6, RF_CHN_TABLE);  //2430

	u32 result;
	u32 data_on_sniffer;
	u32 data_temp;
	while(1){
		sleep_us(500000);
		REG_ADDR8(0x35) = 0x80; //start
		sleep_us(6);
		result = (REG_ADDR16(0x38) & 0x3ff);

		data_temp = ((result*1224)/1023);  //mV
		data_on_sniffer = ((data_temp/1000%10)<<12) | ((data_temp/100%10)<<8) | ((data_temp/10%10)<<4) | (data_temp%10);

		pkt_pairing.flow = (result>>8) &0xff;
		pkt_pairing.type = result&0xff;
		pkt_pairing.rssi = (data_temp>>8) &0xff;
		pkt_pairing.per = data_temp&0xff;


		pkt_pairing.rsvd = (data_on_sniffer>>8) &0xff;
		pkt_pairing.seq_no = data_on_sniffer&0xff;

		rf_send_packet((u8*)&pkt_pairing, 300, 0);
	}
}

void pm_test(void)
{
	rf_power_enable(0);


	sleep_us(2000000);
	REG_ADDR8(0x599) &= 0xf7; //GP27  BAT_DET ie down
	REG_ADDR8(0x599) &= 0x01;  //52uA -> 22uA
	kb_pm_init();
	//gpio_shutdown();
	while(1){
		kb_cpu_sleep_wakeup(0,PM_WAKEUP_CORE,0);
		sleep_us(2000000);
	}
}

void clk_32k_rc_test(void)
{

	REG_ADDR8(0x65) = 0xff;
	REG_ADDR8(0x58e) &= 0xbf;
	analog_write(0x24,analog_read(0x24) | BIT(3));
	while(1);
}


extern kb_data_t	kb_event;
int simu_key_data(void)
{
	static u32 tick;
	if(kb_status.mode_link ){
		if(clock_time_exceed(tick,4000000)){
			tick = clock_time();
			kb_event.cnt = 1;
			kb_event.ctrl_key = 0;
			kb_event.keycode[0] = VK_A;
			return 1;
		}
	}

	return 0;
}

/***********************************************************************************
 *
 * 						          RF TEST
 *
 ***********************************************************************************/
volatile u32 a_dbg_main_loop;
void tx_test(void)
{
	rf_power_enable(1);
	ll_device_init ();
	rf_receiving_pipe_enble(0x3f);	// channel mask

	rf_set_power_level_index (RF_POWER_8dBm);
	kb_rf_pkt = (u8*)&pkt_km;
	rf_set_tx_pipe (PIPE_MOUSE);
	rf_set_channel (10, RF_CHN_TABLE);  //2450

	while(1){
		a_dbg_main_loop++;
		pkt_km.type++;
		rf_send_packet (kb_rf_pkt, 300, 0);
		sleep_us(50000);
	}
}


void rx_test(void)
{
	irq_enable();
	rf_power_enable(1);
	ll_device_init ();
	rf_receiving_pipe_enble(0x3f);	// channel mask

	//rf_set_power_level_index (RF_POWER_8dBm);
	kb_rf_pkt = (u8*)&pkt_km;
	rf_set_channel (10, RF_CHN_TABLE);  //2450
	rf_set_rxmode();

	while(1){
		a_dbg_main_loop++;
		if(a_dbg_main_loop == 0x800){
			//write_reg8(0x808000,0x67);
		}
	}
}
/***********************************************************************************
 *
 * 						          PM TEST
 *
 ***********************************************************************************/
#if 1
_attribute_ram_code_ int cpu_sleep_wakeup_8368 (int deepsleep, int wakeup_src, u32 wakeup_ms)
{
	return 1;
}
#else
_attribute_ram_code_ int cpu_sleep_wakeup_8368 (int deepsleep, int wakeup_src, u32 wakeup_ms)
{
	write_reg32(0x808008,0x12345678);

	analog_write (0x13, 0x77);// reset/wait time ( 7f-77 ) * 2= 16 cycle = 0.5ms
    u32 timer_tick = wakeup_ms<<5;


	u8 anareg80 = analog_read(0x80);		//xtal cap
	u8 anareg81 = analog_read(0x81);		//xtal cap

//	if (wakeup_src & PM_WAKEUP_PAD) {
//		analog_write (0x16, pad_wkp_en);
//	}

	//////////////////////// set deepsleep flag
	if (deepsleep) {
		analog_write (0x05, 0x0);		//keep bbpll ldo on when wakeup
	}

	analog_write (0x81, 0xc0);					//turn off xtal cap
	analog_write (0x80, 0xa1);					//turn off xtal 4.6p cap
	analog_write (0x18, (deepsleep ? PM_AUTO_PWDN_DEEPSLEEP : PM_AUTO_PWDN_SUSPEND) | \
				((wakeup_src & 0x02) ? 0 : PM_AUTO_PWDN_32K));  //0x02 PM_WAKEUP_TIMER


	if (wakeup_src & 0x02){ //0x02 PM_WAKEUP_TIMER

		analog_write(0x12, 0x00);  //disable

	    analog_write(0x17, 0x08);  //reset
		analog_write(0x17, 0);

		analog_write(0x0f, timer_tick&0xff);
		analog_write(0x10, (timer_tick>>8)&0xff);
		analog_write(0x11, (timer_tick>>16)&0xff);

		analog_write(0x12, 0x02);  //enable
	}

	//analog_write (0x17, wakeup_src | PM_AUTO_PWDN_EN); //PM_AUTO_PWDN_EN移到 ana_18[4]
	analog_write (0x17, wakeup_src);
	analog_write (0x23, 0x0f);  //clear status


	u8 r = reg_irq_en;
	reg_irq_en = 0;

	sleep_start();
	reg_irq_en = r;

	u8 anareg23 = analog_read(0x23);


	if (deepsleep) {		// reboot ?

	}

	analog_write (0x80, anareg80);			//restore xtal cap setting
	analog_write (0x81, anareg81);			//restore xtal cap setting


//	if (wakeup_src & PM_WAKEUP_PAD) {
//		analog_write (0x16, 0);
//	}

	return anareg23;
}
#endif

void gpio_shutdown(void)
{
	//disable ie
	write_reg8(0x800581,0x00);
	write_reg8(0x800589,0x00);
	write_reg8(0x800591,0x00);
	write_reg8(0x800599,0x00);
	write_reg8(0x8005a1,0x3e); //SWS MSPI


	//disable oen
	write_reg8(0x800582,0xff);
	write_reg8(0x80058a,0xff);
	write_reg8(0x800592,0xff);
	write_reg8(0x80059a,0xff);
	write_reg8(0x8005a2,0x3f);

	//disable dataO
	write_reg8(0x800583,0x00);
	write_reg8(0x80058b,0x00);
	write_reg8(0x800593,0x00);
	write_reg8(0x80059b,0x00);
	write_reg8(0x8005a3,0x00);

	//ds enable
//	write_reg8(0x800585,0xff);
//	write_reg8(0x80058d,0xff);
//	write_reg8(0x800595,0xff);
//	write_reg8(0x80059d,0xff);
//	write_reg8(0x8005a5,0x3f);
}


#define TEST_SPECIAL_CORE  0  //test GP32 GP26 GP27 GP31

u32 spec_pin;
void suspend_test(void)
{
	sleep_us(2000000);
	gpio_shutdown();

#if  TEST_SPECIAL_CORE
	spec_pin =  BIT(7); //GP7
#else
	spec_pin = (0x400 | BIT(0)); //GP32
#endif
	gpio_set_func(spec_pin,AS_GPIO);
	gpio_set_output_en(spec_pin,1);
	gpio_set_input_en(spec_pin,0);


#if	 1  //timer wakeup
	while(1){

			gpio_write(spec_pin,0);
			cpu_sleep_wakeup_8368(0,0x02,7);  //7ms
			gpio_write(spec_pin,1);

			sleep_us(100000);

			#if 0
			for(int i=0;i<4;i++){
				gpio_write(spec_pin,1);
				sleep_us(300000);
				gpio_write(spec_pin,0);
				sleep_us(300000);
			}
			#endif
		}
#elif  0  //core high wakeup (except GP32 GP31 GP25  GP28-GP30)
	//100K pulldown
	analog_write(0x08,0xff);//  GP17-GP20
	analog_write(0x09,0xff);//  GP21-GP24
	analog_write(0x28,0xfe);//  GP32  GP0-GP6    not GP32
	analog_write(0x29,0xff);//  GP7-GP14
	analog_write(0x2a,0xff);//  GP15 GP16     GP26 GP27 GP31

	//pol=0  high wakeup
	write_reg8(0x800584,0x00);
	write_reg8(0x80058c,0x00);
	write_reg8(0x800594,0x00);
	write_reg8(0x80059c,0x00);
	write_reg8(0x8005a4,0x00);

	//enable
	write_reg8(0x800587,0xff); //
	write_reg8(0x80058f,0xff); //
	write_reg8(0x800597,0xff); //
	write_reg8(0x80059f,0x0d); //GP31 没法测 GP25 GP28-GP30无内部下拉  暂时测GP24 GP26  GP27
	//write_reg8(0x8005a7,0x01); //GP32闪灯 暂时不测

	REG_ADDR8(0x6e) |= BIT(3); // FLD_WAKEUP_SRC_GPIO

	while(1){

		cpu_sleep_wakeup_8368(0,BIT(0),0);

		for(int i=0;i<4;i++){
			gpio_write(spec_pin,1);
			sleep_us(300000);
			gpio_write(spec_pin,0);
			sleep_us(300000);
		}
	}
#elif  0 //core low wakeup (except GP32 GP31 GP25  GP28-GP30)
	//1M pullup
	analog_write(0x08,0x55);//  GP17-GP20
	analog_write(0x09,0x55);//  GP21-GP24
	analog_write(0x28,0x00);//  GP32  GP0-GP6  floating
	analog_write(0x29,0x00);//  GP7-GP14       floating
	analog_write(0x2a,0x54);//  GP15 GP16  floating    GP26 GP27 GP31 1M pullup

	 //pol=1  low wakeup
	write_reg8(0x800584,0xff);
	write_reg8(0x80058c,0xff);
	write_reg8(0x800594,0xff);
	write_reg8(0x80059c,0xff);
	write_reg8(0x8005a4,0x3f);

	//enable
	write_reg8(0x800587,0xff); //
	write_reg8(0x80058f,0xff); //
	write_reg8(0x800597,0xff); //
	write_reg8(0x80059f,0x0d); //GP31 没法测 GP25 GP28-GP30无内部下拉  暂时测GP24 GP26  GP27
	//write_reg8(0x8005a7,0x01); //GP32闪灯 暂时不测

	REG_ADDR8(0x6e) |= BIT(3); // FLD_WAKEUP_SRC_GPIO

	while(1){

		cpu_sleep_wakeup_8368(0,BIT(0),0);

		for(int i=0;i<4;i++){
			gpio_write(spec_pin,1);
			sleep_us(300000);
			gpio_write(spec_pin,0);
			sleep_us(300000);
		}
	}

#elif 0 //pad high wakeup (10 IO OK,GP31 can not test)

	//100K pulldown
	analog_write(0x08,0xff);//  GP17-GP20
	analog_write(0x09,0xff);//  GP21-GP24
	analog_write(0x28,0xfe);//  GP32  GP0-GP6  not GP32
	analog_write(0x29,0xff);//  GP7-GP14
	analog_write(0x2a,0xff);//  GP15 GP16     GP26 GP27 GP31

	analog_write(0x14 , ( analog_read(0x14) & 0x0f)); //gpio[24:17]  high wakeup
	analog_write(0x2b , ( analog_read(0x14) & 0x8f)); //GP26 GP27 GP31  high wakeup

	analog_write(0x16 , 0xff); //gpio[24:17] enable
	analog_write(0x2b , ( analog_read(0x14) | 0x07)); //GP26 GP27 GP31  enable

	while(1){

		cpu_sleep_wakeup_8368(0,BIT(2),0);  //PM_WAKEUP_PAD

		for(int i=0;i<4;i++){
			gpio_write(spec_pin,1);
			sleep_us(300000);
			gpio_write(spec_pin,0);
			sleep_us(300000);
		}
	}

#elif 0 //pad low wakeup  (10 IO OK,GP31 can not test)
	//1M pullup
	analog_write(0x08,0x55);//  GP17-GP20
	analog_write(0x09,0x55);//  GP21-GP24
	analog_write(0x28,0x00);//  GP32  GP0-GP6  floating
	analog_write(0x29,0x00);//  GP7-GP14       floating
	analog_write(0x2a,0x54);//  GP15 GP16  floating    GP26 GP27 GP31 1M pullup

	analog_write(0x14 , ( analog_read(0x14) | 0xf0)); //gpio[24:17]  low wakeup
	analog_write(0x2b , ( analog_read(0x14) | 0x70)); //GP26 GP27 GP31  low wakeup

	analog_write(0x16 , 0xff); //gpio[24:17] enable
	analog_write(0x2b , ( analog_read(0x14) | 0x07)); //GP26 GP27 GP31  enable


	while(1){
		cpu_sleep_wakeup_8368(0,BIT(2),0);  //PM_WAKEUP_PAD
		for(int i=0;i<4;i++){
			gpio_write(spec_pin,1);
			sleep_us(300000);
			gpio_write(spec_pin,0);
			sleep_us(300000);
		}
	}
#elif TEST_SPECIAL_CORE  //GP32 GP25  GP28-GP30  high low all OK

#if 0//high up
	//100K pulldown
	analog_write(0x08,0xff);//  GP17-GP20
	analog_write(0x09,0xff);//  GP21-GP24

	//pol=0  high wakeup
	write_reg8(0x800584,0x00);
	write_reg8(0x80058c,0x00);
	write_reg8(0x800594,0x00);
	write_reg8(0x80059c,0x00);
	write_reg8(0x8005a4,0x00);
#else //low up
	//100K pulldown
	analog_write(0x08,0x55);//  GP17-GP20
	analog_write(0x09,0x55);//  GP21-GP24

	//pol=0  high wakeup
	write_reg8(0x800584,0xff);
	write_reg8(0x80058c,0xff);
	write_reg8(0x800594,0xff);
	write_reg8(0x80059c,0xff);
	write_reg8(0x8005a4,0xff);
#endif
	//enable
	write_reg8(0x80059f,0x72); //GP25 GP28-GP30
	write_reg8(0x8005a7,0x01); //GP32

	REG_ADDR8(0x6e) |= BIT(3); // FLD_WAKEUP_SRC_GPIO

	while(1){

		cpu_sleep_wakeup_8368(0,BIT(0),0);

		for(int i=0;i<4;i++){
			gpio_write(spec_pin,1);
			sleep_us(300000);
			gpio_write(spec_pin,0);
			sleep_us(300000);
		}
	}
#elif 0 //pad high wakeup deepsleep (all OK ,GP31 can not test)

#if 0
	//100K pulldown
	analog_write(0x08,0xff);//  GP17-GP20
	analog_write(0x09,0xff);//  GP21-GP24
	analog_write(0x2a,0xff);//  GP15 GP16     GP26 GP27 GP31
	analog_write(0x14 , ( analog_read(0x14) & 0x0f)); //gpio[24:17]  high wakeup
	analog_write(0x2b , ( analog_read(0x14) & 0x8f)); //GP26 GP27 GP31  high wakeup
#else
	//1M pullup
	analog_write(0x08,0x55);//  GP17-GP20
	analog_write(0x09,0x55);//  GP21-GP24
	analog_write(0x2a,0x54);//  GP15 GP16  floating    GP26 GP27 GP31 1M pullup
	analog_write(0x14 , ( analog_read(0x14) | 0xf0)); //gpio[24:17]  low wakeup
	analog_write(0x2b , ( analog_read(0x14) | 0x70)); //GP26 GP27 GP31  low wakeup
#endif

	analog_write(0x16 , 0xff); //gpio[24:17] enable
	analog_write(0x2b , ( analog_read(0x14) | 0x07)); //GP26 GP27 GP31  enable

	for(int i=0;i<4;i++){
		gpio_write(spec_pin,1);
		sleep_us(300000);
		gpio_write(spec_pin,0);
		sleep_us(300000);
	}

	cpu_sleep_wakeup_8368(1,BIT(2),0);  //PM_WAKEUP_PAD
	while(1);

#endif

}


/***********************************************************************************
 *
 * 						          GPIO TEST
 *
 ***********************************************************************************/

void gpio_test(void)
{
	write_reg8(0x808000,0x88);
	//spec_pin = GPIO_GP32;
	spec_pin = (0x400 | BIT(0));

	gpio_set_output_en(spec_pin,1);

	write_reg8(0x808000,0x99);
	gpio_set_input_en(spec_pin,0);

	write_reg8(0x808000,0xaa);

	gpio_write(spec_pin,0);

	write_reg8(0x808000,0xbb);



	while(1){
		gpio_write(spec_pin,1);

		write_reg8(0x808000,0xcc);

		sleep_us(500000);

		write_reg8(0x808000,0xdd);

		gpio_write(spec_pin,0);

		sleep_us(500000);
	}

}
