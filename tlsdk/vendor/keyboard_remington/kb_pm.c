/*
 * kb_pm.c
 *
 *  Created on: 2015-1-21
 *      Author: Administrator
 */



#include "../../proj/tl_common.h"
#include "../../proj_lib/pm.h"
#include "../../proj_lib/rf_drv.h"
#include "../common/rf_frame.h"

#include "kb_info.h"
#include "kb_led.h"
#include "kb_rf.h"
#include "kb_pm.h"

extern u32	scan_pin_need;

kb_slp_cfg_t kb_sleep = {
	SLEEP_MODE_BASIC_SUSPEND,  //mode
	0,                  //device_busy
	0,					//quick_sleep
	0,                  //wakeup_src

    0,
    166,  //12ms unit
    0,
    19,  //100 ms unit

    0,
    0,
};

//_attribute_ram_code_
inline void kb_slp_mode_machine(kb_slp_cfg_t *s_cfg)
{
    if ( s_cfg->device_busy ){
        s_cfg->mode = SLEEP_MODE_BASIC_SUSPEND;
        s_cfg->cnt_basic_suspend = 0;
        s_cfg->cnt_long_suspend = 0;
    }
    else if (s_cfg->mode == SLEEP_MODE_BASIC_SUSPEND){
        s_cfg->cnt_basic_suspend ++;
        if ( s_cfg->cnt_basic_suspend >= s_cfg->thresh_basic_suspend ){
            s_cfg->mode = SLEEP_MODE_LONG_SUSPEND;
            s_cfg->cnt_long_suspend = 0;
        }
    }


    if ( s_cfg->mode == SLEEP_MODE_LONG_SUSPEND ) {
		s_cfg->cnt_long_suspend ++;
		if ( s_cfg->cnt_long_suspend > s_cfg->thresh_long_suspend )	{
			s_cfg->mode = SLEEP_MODE_DEEPSLEEP;
		}
	}

    if ( s_cfg->quick_sleep ) {
        s_cfg->mode = SLEEP_MODE_DEEPSLEEP;
	}
}


void kb_pm_init(void)
{
	for(int i=0;i<8;i++){
		gpio_enable_wakeup_pin(drive_pins[i], 0, 1); //core wakeup suspend: low active
		cpu_set_gpio_wakeup (drive_pins[i], 0, 1);   //pad  wakeup deep   : low active
	}
	kb_sleep.wakeup_tick = clock_time();
}


#define DEBUG_KB_NO_SUSPEND  0

extern int kb_is_lock_pressed;

_attribute_ram_code_ void kb_pm_proc(void)
{
#if(DEBUG_KB_NO_SUSPEND)
    cpu_suspend_wakeup_sim (KB_MAIN_LOOP_TIME_MS*1000);
#else
	kb_sleep.device_busy = ( kb_status.rf_sending || KB_LED_BUSY || (kb_is_lock_pressed && scan_pin_need));
	kb_sleep.quick_sleep = HOST_NO_LINK;
	if ( kb_status.kb_mode <= STATE_PAIRING && kb_status.loop_cnt < KB_NO_QUICK_SLEEP_CNT){
		kb_sleep.quick_sleep = 0;
	}

	kb_slp_mode_machine( &kb_sleep );


	kb_sleep.wakeup_src = PM_WAKEUP_TIMER;
	kb_sleep.next_wakeup_tick = kb_sleep.wakeup_tick + 11400*16;
	if ( kb_sleep.mode ==  SLEEP_MODE_LONG_SUSPEND){
    	kb_sleep.wakeup_src = PM_WAKEUP_TIMER | PM_WAKEUP_CORE;
    	kb_sleep.next_wakeup_tick = kb_sleep.wakeup_tick + 100*CLOCK_SYS_CLOCK_1MS;
	}
	else if ( kb_sleep.mode == SLEEP_MODE_DEEPSLEEP){
		if(scan_pin_need){  //有按键按着  wait_deep状态，按键释放后进入deep
			kb_status.kb_mode = STATE_WAIT_DEEP;
			kb_sleep.mode = SLEEP_MODE_WAIT_DEEP;
			kb_sleep.next_wakeup_tick = kb_sleep.wakeup_tick + 100*CLOCK_SYS_CLOCK_1MS;
		}
		else{
			kb_info_save();
			kb_sleep.wakeup_src = PM_WAKEUP_PAD;
		}
	}

    while ( !clock_time_exceed (kb_sleep.wakeup_tick, 500) );
    kb_cpu_sleep_wakeup (kb_sleep.mode == SLEEP_MODE_DEEPSLEEP, kb_sleep.wakeup_src, kb_sleep.next_wakeup_tick);
	kb_sleep.wakeup_tick = clock_time();
#endif
}


_attribute_ram_code_ u32 blt_get_32k_tick ()
{
	u8 t0, t1;
	t0 = analog_read (0x20);					//PLL clock
	do {
		t1 = analog_read(0x20);
	} while (t1 == t0);
	return t1 | (analog_read(0x21)<<8) | (analog_read(0x22)<<16);
}


_attribute_ram_code_ int kb_cpu_sleep_wakeup (int deepsleep, int wakeup_src, u32 wakeup_tick)
{
	analog_write (0x13, 0x70);// reset/wait time ( 7f-70 ) * 2= 30 cycle = 1 ms

	u32 span = wakeup_tick - clock_time ();
	if (span < 2000 * CLOCK_SYS_CLOCK_1US)
	{
		u32 t = clock_time ();
		analog_write (0x23, 0x0f);			//clear all status
		u8 st;
		do {
			st = analog_read (0x23) & 0x0f;
		} while ( ((u32)clock_time () - t < span) && !st);
		return st;
	}

	u32 tick_cur;
	u32 t32k;
	u16 tick_16_rc32k;
	u8 reg66 = REG_ADDR8(0x66);

	if (wakeup_src & PM_WAKEUP_TIMER)
	// go to suspend
	{	//t32 = span * 16 / reg24
		span -= 1000 * CLOCK_SYS_CLOCK_1US;			//count wakeup-reset time
		tick_16_rc32k = REG_ADDR16(0x24);
		t32k = span * 16 / tick_16_rc32k;
		analog_write(0x0f, t32k);
		analog_write(0x10, t32k>>8);
		analog_write(0x11, t32k>>16);

		blt_get_32k_tick ();
		analog_write(0x17, PM_32KRC_RESET);  		//reset
		tick_cur = clock_time ();					//PLL clock
		analog_write (0x17, wakeup_src);			//start
	}

	REG_ADDR8(0x66) = 0x00;
	/////////////////// set wakeup source /////////////////////////////////
	///////////////// store gpio wakeup enable setting ////////////////////

	u8 anareg80 = analog_read(0x80);		//xtal cap
	u8 anareg81 = analog_read(0x81);		//xtal cap


	reg_wakeup_en = (wakeup_src&PM_WAKEUP_CORE) ? FLD_WAKEUP_SRC_GPIO : 0 ;

	//////////////////////// set deepsleep flag
	if (deepsleep) {
		analog_write (0x05, 0x0);		//keep bbpll ldo on when wakeup
	}

	analog_write (0x81, 0xc0);					//turn off xtal cap
	analog_write (0x80, 0xa1);					//turn off xtal 4.6p cap
	analog_write (0x18, (deepsleep ? PM_AUTO_PWDN_DEEPSLEEP : PM_AUTO_PWDN_SUSPEND) | \
				((wakeup_src & PM_WAKEUP_TIMER) ? 0 : PM_AUTO_PWDN_32K));

	analog_write (0x23, 0x0f);  //clear status
	analog_write (0x17, wakeup_src);

	//static u32		cpu_working_tick, cpu_wakup_last_tick;
	//cpu_working_tick = clock_time () - cpu_wakup_last_tick;

	sleep_start();

	//cpu_wakup_last_tick = clock_time ();			// system tick on wakeup

	u8 anareg23 = analog_read(0x23);


	analog_write (0x80, anareg80);			//restore xtal cap setting
	analog_write (0x81, anareg81);			//restore xtal cap setting


	if (wakeup_src & PM_WAKEUP_TIMER)
	{
		REG_ADDR8(0x66) = reg66;
		u32 tsus = blt_get_32k_tick ();
		if (analog_read(0x23) & PM_WAKEUP_TIMER)		//wake-up from timer
		{
			tsus = blt_get_32k_tick ();					//must...
			tsus += t32k;
		}
		tsus = tsus * tick_16_rc32k >> 4;
		reg_system_tick = tick_cur + tsus;			//new system tick
	}

	REG_ADDR8(0x26) = 0xa6;				// start 32K calibration
	return anareg23;
}
