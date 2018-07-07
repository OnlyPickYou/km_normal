/*
 * mouse_power.c
 *
 *  Created on: Feb 12, 2014
 *      Author: xuzhen
 */

#include "../../proj/tl_common.h"
#include "../../proj_lib/pm.h"
#include "device_power.h"
#include "device_led.h"

sleep_cfg_t device_sleep = {
    0,				//wakeup_src
    100,			//wakeup_time
    
    0,				//cnt_8ms
    0xff,           //thresh_8ms    

    0,				//cnt_100ms
#if DEVICE_DEEPSLEEP_EN
    600,
#else
    0xffff,            //thresh_100ms * x
#endif
    
    M_SUSPEND_8MS,  //mode, mcu 8ms
    0,              //mcu_sleep_en
    0,              //not sensor sleep 
    0,              //sensor_sleep_en
    
    0,              //quick_sleep disable
    1,              //device_busy, busy
};


/*
 * Based on input parameters, and current condition
 * to determine mouse power saving mode, which can be
 * suspend, deep sleep, or just cpu stall
 */

#if(__PROJECT_KEYBOARD__)
_attribute_ram_code_
#endif
void device_sleep_mode_machine(sleep_cfg_t *s_cfg){
    if ( s_cfg->device_busy ){
        s_cfg->mode = M_SUSPEND_8MS;
        s_cfg->cnt_8ms = 0;
    }
    else if (s_cfg->mode == M_SUSPEND_8MS){
        s_cfg->cnt_8ms ++;
        if ( s_cfg->cnt_8ms >= s_cfg->thresh_8ms ){          
            s_cfg->mode = M_SUSPEND_100MS;
            s_cfg->cnt_100ms = 0;
        }
    }
    
    if ( s_cfg->mode & M_SUSPEND_100MS ) {
		s_cfg->cnt_100ms ++;
		if ( s_cfg->cnt_100ms > s_cfg->thresh_100ms )	{	
			s_cfg->mode = M_SUSPEND_DEEP_SLP;            
		}
	}
#if DEVICE_DEEPSLEEP_EN
	if ( (s_cfg->mode & M_SUSPEND_DEEP_SLP) || s_cfg->quick_sleep ) {     
        s_cfg->mode = M_SUSPEND_DEEP_SLP;
        if ( !s_cfg->mcu_sleep_en ){
			s_cfg->mode = M_SUSPEND_100MS;
            s_cfg->cnt_100ms = 0;
        }
        if ( s_cfg->sensor_sleep_en ){
			s_cfg->sensor_sleep = M_SUSPEND_SNSR_SLP;
        }
	}
#endif	
}

