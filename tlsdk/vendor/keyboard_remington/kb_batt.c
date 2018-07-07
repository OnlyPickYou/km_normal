/*
 * kb_batt.c
 *
 *  Created on: 2015-1-21
 *      Author: Administrator
 */

#include "../../proj/tl_common.h"
#include "../../proj_lib/pm.h"
#include "../../proj_lib/rf_drv.h"
#include "../common/rf_frame.h"

#include "kb_batt.h"
#include "kb_led.h"
#include "kb_custom.h"
#include "kb_pm.h"

u32 batt_det_count = 0;


#define ADC_SAMPLE_NUM   			11
u16 adc_sample[ADC_SAMPLE_NUM+1];
u16 adc_result;

void kb_batt_det_init(void)
{
#if 0
	REG_ADDR8(0x599) &= 0xf7; //GP27  BAT_DET ie down
#else
	gpio_set_input_en(GPIO_GP23,0);
	//AD clk = src_clk * step/mod = 32M(OSC) * 4/32 = 4M    T = 0.25us
	//ad time = sample_time + bit_store_time = (cycle + res_len) * T =
	REG_ADDR8(0x69) = 4;  //step
	REG_ADDR8(0x6a) = CLK_FHS_MZ;  //mod
	REG_ADDR8(0x6b) = 0x80;  //adc clk en

	REG_ADDR8(0x33) = 0x00;  //no auto
	REG_ADDR8(0x2b) = 0x00;  //ref:Vbg 1.224V
	REG_ADDR8(0x2c) = 0x04;  //GP23
	REG_ADDR8(0x3c) = 0x18;  //10 bit 3cycle  13*0.25us = 3.25us
#endif
}

extern u32	scan_pin_need;
_attribute_ram_code_ void kb_batt_det_process(void)
{

	static u8 batt_detect_cnt = 0;
	u8 batt_detect_flag = 0;
	if(!scan_pin_need && !kb_status.rf_sending){
		batt_detect_cnt = 0;
		batt_detect_flag = 1;
	}
	else{
		batt_detect_cnt++;
		if(batt_detect_cnt >= 40 && !kb_status.rf_sending){
			batt_detect_cnt = 0;
			batt_detect_flag = 1;
		}
	}

	if(batt_detect_flag && (kb_sleep.mode == SLEEP_MODE_BASIC_SUSPEND)){
		kb_batt_det_adc_sample();
	}
}

void kb_batt_det_adc_sample(void)
{
	analog_write(0x06,0xfe);  //ana_06<0> : 0 ->Power on SAR ADC
	sleep_us(10);

	static int first_flg = 0;
	u16 result;
	int i,j;


	u16 temp;
	for(i=0;i<ADC_SAMPLE_NUM;i++){
		REG_ADDR8(0x35) = 0x80; //start
		sleep_us(5);
		adc_sample[i] = REG_ADDR16(0x38) & 0x3ff;

		if(i){
			if(adc_sample[i] < adc_sample[i-1]){
				temp = adc_sample[i];
				adc_sample[i] = adc_sample[i-1];
				for(j=i-1;j>=0 && adc_sample[j] > temp;j--){
					adc_sample[j+1] = adc_sample[j];
				}
				adc_sample[j+1] = temp;
			}
		}
	}

	analog_write(0x06,0xff);   //ana_06<0> : 1 ->Power down SAR ADC
	adc_sample[ADC_SAMPLE_NUM] = (adc_sample[6]+adc_sample[7]+adc_sample[8])/3;

	if(!first_flg){
		adc_result = adc_sample[ADC_SAMPLE_NUM];
	}
	else{
		adc_result = ( (adc_result*7) + adc_sample[ADC_SAMPLE_NUM] + 4 )>>3;
	}
	if(batt_det_count >= 400){
		batt_det_count = 0;
		result = (adc_result*1224)/1023;  //mV
		//single batt: 0 - 0.9V   double batt:1.9- 2.0 V
		if( (IS_SINGLE_BATT() && result < 900) || (!IS_SINGLE_BATT() && result< 1000)){
			kb_device_led_setup( kb_led_cfg[KB_LED_BAT_LOW] );
		}

#if DBG_ADC_DATA  //debug
		extern rf_packet_keyboard_t	pkt_km;
		result = ((result/1000%10)<<12) | ((result/100%10)<<8) | ((result/10%10)<<4) | (result%10);
		pkt_km.per = result&0xff;
		pkt_km.rssi = (result>>8)&0xff;
#endif
	}
}
