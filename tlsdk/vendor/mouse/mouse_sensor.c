/*
 * mouse_sensor.c
 *
 *  Created on: Feb 12, 2014
 *      Author: xuzhen
 */

#include "../../proj/tl_common.h"
#include "mouse_sensor.h"
#include "mouse_sensor_pix.h"
#include "mouse_sensor_a3000.h"
#include "mouse_custom.h"
#include "trace.h"

int (*pf_sensor_init) (unsigned int mode);
int (*pf_sensor_motion_report) ( signed char *pBuf, u32 no_overflow );
void (*pf_sensor_shutdown) (void);
int (*pf_sensor_set_cpi) (u8 *cpi_idx);
int (*pf_sensor_wakeup) (u32 sensor);
char no_motion_rd;
char sensor_no_ov_rd;

void mouse_sensor_set_cpi( u8 *cpi_idx ){
    *cpi_idx = (*cpi_idx < mouse_cpi.sns_cpi_sgmt) ? *cpi_idx : 0;
     (*pf_sensor_set_cpi) ( mouse_cpi.sns_cpi_tbl[*cpi_idx] );
}

int mouse_sensor_hw_init( mouse_hw_t *pHW, u8 *p_sensor, int poweron ){
    int ret;
    OPTSensor_hardware_init (&pHW->sensor_data);
    int sensor_is_3204s= 0;
    if ( poweron ){
        *p_sensor = OPTSensor_Init( poweron );
    }

    if ( !(*p_sensor & SENSOR_MODE_A3000) ){
        pf_sensor_init = OPTSensor_Init;
        pf_sensor_motion_report = OPTSensor_motion_report;
        pf_sensor_set_cpi = OPTSensor_dpi_update;
        pf_sensor_shutdown = OPTSensor_Shutdown;
        pf_sensor_wakeup = Sensor3204_Wakeup;        
        *p_sensor &= ~SENSOR_MODE_A3000;
		ret = 1;
    }
#if ( MOUSE_SENSOR_A3000_EN )  
    else {
        a3000_spi_init(&pHW->sensor_data);
        pf_sensor_init = A3000_OPTSensor_Init;
        pf_sensor_motion_report = A3000_OPTSensor_motion_report;
        pf_sensor_set_cpi = A3000_OPTSensor_dpi_update;
        pf_sensor_shutdown = A3000_OPTSensor_Shutdown;
        pf_sensor_wakeup = A3000_OPTSensor_Wakeup;
        *p_sensor |= SENSOR_MODE_A3000;
        ret = 0;
    }
#endif

    //senosr motion pin init    
#if MOUSE_SENSOR_MOTION
    u32 motion_pin = pHW->sensor_int;
    gpio_write( motion_pin, 1 );    
    gpio_setup_up_down_resistor(motion_pin, 1 );
#endif
#if MOUSE_GPIO_FULL_RE_DEF    
    gpio_set_func( motion_pin, AS_GPIO ); 
    gpio_set_output_en( motion_pin, 0 );        
    gpio_set_input_en( motion_pin, 1 );
#endif
    return ret;
}

int mouse_sensor_init( u8 *p_sensor, u8 *cpi_idx ){    
	(*pf_sensor_wakeup) ( 0 );        
    *p_sensor |= (*pf_sensor_init) ( 0 );
    mouse_sensor_set_cpi ( cpi_idx );
    *p_sensor &= ~(SENSOR_MODE_POWERUP|SENSOR_MODE_POWERDOWN);
    *p_sensor |= SENSOR_MODE_WORKING;
#if MOUSE_SENSOR_MOTION
    no_motion_rd = !sensor_motion_detct || SENSOR_IS_8589(*p_sensor);
#endif
    sensor_no_ov_rd = sensor_no_overflow_rd;
}

u32 mouse_sensor_sleep_wakeup( u8 *p_sensor, u8 *sleep, u32 wakeup ){
    u32 ret = 0;
    if( wakeup && (*p_sensor & SENSOR_MODE_POWERDOWN) ){
        //3204 sensor must wakeup after it has been shutdown in deepsleep
        //but A3000 sensor can not do wakeup after power on initial  
        if ( (*pf_sensor_wakeup) ( 0 ) ){
    		*p_sensor &= ~(SENSOR_MODE_POWERUP|SENSOR_MODE_POWERDOWN);
    		*p_sensor |= SENSOR_MODE_WORKING;
    		*sleep = 0;
    		ret = SENSOR_MODE_WORKING;
        }
    }
    else if( *sleep ){
         if( !(*p_sensor & SENSOR_MODE_POWERDOWN) ){
             (*pf_sensor_shutdown) ();
            *p_sensor |= SENSOR_MODE_POWERDOWN;            
            *p_sensor &= ~SENSOR_MODE_WORKING;
            ret = SENSOR_MODE_POWERDOWN;
         }
    }
    return ret;
}
