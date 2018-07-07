/*
 * device_led.h
 *
 *  Created on: Feb 11, 2014
 *      Author: xuzhen
 */

#pragma once
#include "user_config.h"
/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

#ifndef DEVICE_LED_MODULE_EN
#define DEVICE_LED_MODULE_EN    1
#endif


#define dbg_led_high_low    0
#if dbg_led_high_low
#if(__PROJECT_MOUSE__ || __PROJECT_MOUSE_8510__ || __PROJECT_8366_MIIW_MOUSE__)  //mouse
#define dbg_led_init do{ \
    gpio_set_func(GPIO_DM, AS_GPIO);\
    gpio_set_input_en(GPIO_DM, 0);\
    gpio_set_output_en(GPIO_DM, 1);\
}while(0)
#define dbg_led_high do{ (*((volatile u32*)0x800584)) |= 0x00200000;} while(0)
#define dbg_led_low  do{ (*((volatile u32*)0x800584)) &= 0xffdfffff; } while(0)
#else  //keyboard
#define dbg_led_init do{ \
    gpio_set_func(GPIO_PC7, AS_GPIO);\
    gpio_set_input_en(GPIO_PC7, 0);\
    gpio_set_output_en(GPIO_PC7, 1);\
}while(0)
#define dbg_led_high do{ (*((volatile u8*)0x800593)) |= 0x80;} while(0)
#define dbg_led_low  do{ (*((volatile u8*)0x800593)) &= 0x7f; } while(0)
#endif
#else
#define dbg_led_init
#define dbg_led_high
#define dbg_led_low
#endif

typedef struct{
    u8  on_time;        //led on time: *64ms
    u8  off_time;       //led off time: *64ms 
    u8  repeat_count;   //led on-off repeat count bit7-~bit0
    u8  over_wrt;       //BIT[5:0]led on-off repeat count bit13-~bit8
                        //BIT[7:6]-over_wrt priority: over-write last led event (11>10>01>00)
} led_cfg_t;


typedef struct {
	u32 gpio;
    
	u8  level_on;
	u8  is_on;
	u8  over_wrt;    
	u8  cnt_rate;
    
	u32 repeat_count;

	u32 on_time;
	u32 off_time;

	u32 clock;

}device_led_t;

extern device_led_t      device_led;

#if DEVICE_LED_MODULE_EN
void device_led_init(u32 led_pin, u32 led_level, u8 cnt_rate);
void device_led_process(device_led_t * led);
int device_led_setup(led_cfg_t led_cfg);
#else
static inline void device_led_init(u32 led_pin, u32 led_level, u8 cnt_rate) {}
static inline void device_led_process(device_led_t * led) {}
static inline void device_led_setup(led_cfg_t led_cfg) {}
#endif

#define LED_EVENT_BUSY  (device_led.repeat_count)

#define mouse_led_setup    device_led_setup
#define mouse_led_process  device_led_process
#define mouse_led_init     device_led_init

#define kb_led_setup		device_led_setup
#define kb_led_process		device_led_process
#define kb_led_init		    device_led_init
/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif
