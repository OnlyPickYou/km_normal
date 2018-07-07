/*
 * kb_pm.h
 *
 *  Created on: 2015-1-21
 *      Author: Administrator
 */

#ifndef KB_PM_H_
#define KB_PM_H_


#define SLEEP_MODE_BASIC_SUSPEND 		0
#define SLEEP_MODE_LONG_SUSPEND 		1
#define SLEEP_MODE_WAIT_DEEP    		2
#define SLEEP_MODE_DEEPSLEEP     		3

typedef struct{
    u8   mode;
    u8   device_busy;
    u8   quick_sleep;
    u8   wakeup_src;

    u8   cnt_basic_suspend;
    u8   thresh_basic_suspend;
    u8   cnt_long_suspend;
    u8   thresh_long_suspend;

    u32  wakeup_tick;
    u32  next_wakeup_tick;
} kb_slp_cfg_t;



#define	ADC_SAMPLE_TIME_TEST	0

#if ADC_SAMPLE_TIME_TEST


#define gpio_gp26_init			do{gpio_set_output_en(GPIO_GP26,1);\
								   gpio_set_input_en(GPIO_GP26,0);\
								   gpio_write(GPIO_GP26,0);}while(0)
#define gpio_gp26_output_high	do{(*(volatile u8*) (0x800000 + 0x59b)) |= BIT(2);}while(0)
#define gpio_gp26_output_low	do{(*(volatile u8*) (0x800000 + 0x59b)) &= (~BIT(2));}while(0)
#define gpio_gp26_turn			do{(*(volatile u8*) (0x800000 + 0x59b)) ^= (BIT(2));}while(0)

#endif


extern void kb_pm_init(void);
extern void kb_pm_proc(void);

extern kb_slp_cfg_t kb_sleep;

#endif /* KB_PM_H_ */
