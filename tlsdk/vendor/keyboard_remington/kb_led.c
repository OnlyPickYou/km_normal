/*
 * kb_led.c
 *
 *  Created on: 2015-1-21
 *      Author: Administrator
 */
#include "../../proj/tl_common.h"

#include "kb_led.h"


kb_led_t    kb_device_led;

//on_time, off_time, repeat_count, over_wrt
kb_led_cfg_t kb_led_cfg[] = {
    {32,     1,      1,      0x00},    //power-on, 2s on
    {4,		 4,		 20,	 0x40},    //pairing auto, 2Hz, slow blink
    {2,      2,      255,    0x40},    //pairing manual, 4Hz, fast blink
    {0,      8,      1,      0x80},    //pairing end
    {8,      8,      3,      0xc0},    //pairing ok   1Hz  3s
    {4,      4,      3,      0xc0},     //battery low  2Hz
    //{4,      4,      3,      0x40},     //6
};


void kb_device_led_on_off(u8 on_off)
{
	gpio_write( kb_device_led.gpio, kb_device_led.level_on^on_off );
	kb_device_led.is_on = on_off;
	kb_device_led.clock = 0;
}

//led_level = 1 , kb_device_led.level_on = 0,  gpio_write(  ,0^on_off) = gpio_write(  ,on_off),high valid
//led_level = 0 , kb_device_led.level_on = 1,  gpio_write(  ,1^on_off) = gpio_write(  ,!on_off).low valid
void kb_device_led_init(u32 led_pin, u8 led_level, u8 cnt_rate){
	kb_device_led.gpio = led_pin;
	kb_device_led.level_on = !led_level;
	kb_device_led.cnt_rate = cnt_rate;
    gpio_set_func(led_pin,AS_GPIO);
	gpio_set_input_en( led_pin, 0 ); //input disable
    gpio_set_output_en( led_pin, 1 );//output enable
    kb_device_led_on_off(0);

#if DBG_GPIO
	gpio_set_input_en(GPIO_DBG,0);
	gpio_set_output_en(GPIO_DBG,1);
#endif
}

void kbLed_proc(void)
{
	kb_device_led.clock ++;
	if( kb_device_led.is_on || kb_device_led.on_time==0 ){
		if( kb_device_led.clock >= kb_device_led.on_time ){
			if( kb_device_led.off_time ){
				kb_device_led_on_off(0);
			}
			kb_device_led.repeat_count--;
		}
	}
	else{
		if( kb_device_led.clock >= kb_device_led.off_time ){
			if( kb_device_led.on_time )
				kb_device_led_on_off(1);
		}
	}
}


int kb_device_led_setup(kb_led_cfg_t led_cfg)
{
	u8 led_over_wrt = (led_cfg.over_wrt & 0xc0) > (kb_device_led.over_wrt & 0xc0);
	if( (kb_device_led.repeat_count !=0) && !led_over_wrt )
		return 0;  //can't update led setting now
	else{
		kb_device_led.on_time = led_cfg.on_time * kb_device_led.cnt_rate; //time_per_cycle * device_led.cnt_rate = 8ms * 8 = 64ms
		kb_device_led.off_time = led_cfg.off_time * kb_device_led.cnt_rate;
		kb_device_led.repeat_count = led_cfg.repeat_count | ((led_cfg.over_wrt&0x3f)<<8);
		kb_device_led.over_wrt = led_cfg.over_wrt;
		kb_device_led.clock = 0;
        if( kb_device_led.on_time)
        	kb_device_led_on_off(1);
		return 1;
	}
}
