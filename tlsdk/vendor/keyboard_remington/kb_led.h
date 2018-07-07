/*
 * kb_led.h
 *
 *  Created on: 2015-1-21
 *      Author: Administrator
 */

#ifndef KB_LED_H_
#define KB_LED_H_

#include "..\..\proj\common\types.h"


#define   DBG_GPIO   0

#if DBG_GPIO
#define DBG_GPIO_HIGH  do{gpio_write(GPIO_DBG,1);}while(0)
#define DBG_GPIO_LOW   do{gpio_write(GPIO_DBG,0);}while(0)
#else
#define DBG_GPIO_HIGH
#define DBG_GPIO_LOW
#endif


typedef struct{
    u8  on_time;        //led on time: *64ms
    u8  off_time;       //led off time: *64ms
    u8  repeat_count;   //led on-off repeat count bit7-~bit0
    u8  over_wrt;       //BIT[5:0]led on-off repeat count bit13-~bit8                       //BIT[7:6]-over_wrt priority: over-write last led event (11>10>01>00)
} kb_led_cfg_t;

enum{
	KB_LED_POWER_ON = 0,
	KB_LED_AUTO_PAIRING,	//1
	KB_LED_MANUAL_PAIRING,	//2
	KB_LED_PAIRING_END,		//3
	KB_LED_PAIRING_OK,		//4
	KB_LED_BAT_LOW,			//5
};


typedef struct {
	u32 gpio;

	u8  level_on;
	u8  is_on;
	u8  over_wrt;
	u8  cnt_rate;

	u8 on_time;
	u8 off_time;
	u8 repeat_count;
	u8 clock;

}kb_led_t;



extern void kb_device_led_init(u32 led_pin, u8 led_level, u8 cnt_rate);
extern int kb_device_led_setup(kb_led_cfg_t led_cfg);
//extern void kb_device_led_process(void);

extern kb_led_t    kb_device_led;
extern kb_led_cfg_t kb_led_cfg[];

#define KB_LED_BUSY  (kb_device_led.repeat_count)

//1  main:134c-1414 200bytes  kb_device_led_process:dc8-e18  80
//2  main:1368-142c 196       led_proc: df0-e34:68
extern void kbLed_proc();
static inline void kb_device_led_process(void)
{
	if(kb_device_led.repeat_count){
		kbLed_proc();
	}
}

#endif /* KB_LED_H_ */
