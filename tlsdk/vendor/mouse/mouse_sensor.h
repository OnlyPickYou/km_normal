/*
 * mouse_sensor.h
 *
 *  Created on: Feb 12, 2014
 *      Author: xuzhen
 */

#ifndef MOUSE_SENSOR_H_
#define MOUSE_SENSOR_H_


#include "../common/mouse_type.h"
#include "mouse_sensor_pix.h"
#include "mouse_sensor_a3000.h"

#ifndef MOUSE_SENSOR_CUS
#define MOUSE_SENSOR_CUS      1
#endif

#ifndef MOUSE_SENSOR_MOTION
#define MOUSE_SENSOR_MOTION      1
#endif

#ifndef MOUSE_SENSOR_A3000_EN
#define MOUSE_SENSOR_A3000_EN   0//((MCU_CORE_TYPE == MCU_CORE_8366) || 0)
#endif

#ifndef MOUSE_SENSOR_YS8008_EN
#define MOUSE_SENSOR_YS8008_EN	1
#endif

#define SENSOR_X_REVERSE      0x80
#define SENSOR_Y_REVERSE      0x40
#define SENSOR_XY_SWITCH      0x20

#define SENSOR_DIRECTION_CLOCK_12	SENSOR_Y_REVERSE
#define SENSOR_DIRECTION_CLOCK_3	(SENSOR_XY_SWITCH)
#define SENSOR_DIRECTION_CLOCK_9	(SENSOR_X_REVERSE|SENSOR_XY_SWITCH|SENSOR_Y_REVERSE)
#define SENSOR_DIRECTION_CLOCK_6	(SENSOR_X_REVERSE)

#define SENSOR_MODE_POWERDOWN  0x40
#define SENSOR_MODE_WORKING    0x20
#define SENSOR_MODE_POWERUP    0x10
#define SENSOR_MODE_A3000      0x80

#define SENSOR_STATUS_CTRL  0xf0
#define SENSOR_TYPE_CTRL    0x0f

//---------------------------------------------
// Sensor type
//---------------------------------------------
#define SENSOR_3204       0x00
#define SENSOR_3204LL     0x01
#define SENSOR_3204UL     0x02
#define SENSOR_3205		  0x03

#define SENSOR_AN3205	  0x04

#define SENSOR_SIGMA_8630 0x05

#define SENSOR_PAW3207	  0x06

#define SENSOR_VT108	  0x07
#define SENSOR_M8589	  0x08
#define SENSOR_KA9		  0x09

#define SENSOR_PAW3212	  0x0a
#define SENSOR_OM16		  0x0b

#if(!MOUSE_SENSOR_YS8008_EN)
#define SENSOR_A3000	  0x0b
#else
#define SENSOR_YS8006	  0x0c
/*
 * IDENTIFY SENSOR YS8006
 *
 * (1)Product_id2 == 0x50
 * (2)REG17 == 0x5C && REG18 == 0x40 && REG19 == 0x74
 *
 */
#define SENSOR_YS8008	  0x0d

/*
 * IDENTIFY SENSOR YS8008
 * (1)Product_id2 == 0xD0
 * (2)REG16 == 0xBD && REG19 == 0x55 && REG1B == 0x1D
 *
 */

#define SENSOR_A3000	  0x0e
#endif

#define LAST_SUPPORT_SENSOR SENSOR_A3000   //Must keep this as the last sensor type!!!!!

#define	SENSOR_IS_8589(sensor)     ( (sensor&SENSOR_TYPE_CTRL) == SENSOR_M8589)
#define SENSOR_IS_SIGMA_8640(sensor) ( (sensor&SENSOR_TYPE_CTRL) == SENSOR_SIGMA_8630)
#define SENSOR_IS_SUNPLUS(sensor)   ( (sensor&SENSOR_TYPE_CTRL) == SENSOR_M8589)
#define SENSOR_IS_XIWANG(sensor)    ( (sensor&SENSOR_TYPE_CTRL) == SENSOR_VT108)

void mouse_sensor_set_cpi( u8 *cpi_idx );
int mouse_sensor_hw_init(mouse_hw_t *pHW, u8 *p_sensor, int poweon);
int mouse_sensor_init( u8 *p_sensor, u8 *cpi_idx );

extern int (*pf_sensor_motion_report) ( signed char *pBuf, u32 no_overflow );

u32 mouse_sensor_sleep_wakeup( u8 *p_sensor, u8 *sleep, u32 wakeup );

//sim sensor blinky when mouse is in sensor sleep mode
static inline u32 mouse_sensor_blinky_wkup( u32 blink_en ){
    static u8 sns_blnky_cnt  = 0;
    if( blink_en )
        sns_blnky_cnt++;
    else
        sns_blnky_cnt = 0;
    return sns_blnky_cnt & 0x08;
}

static inline void mouse_sensor_dir_adjust( signed char *px, signed char *py, unsigned char sensor_dir ){
	if(sensor_dir & SENSOR_X_REVERSE)
    	*px = - *px;

    if(sensor_dir & SENSOR_Y_REVERSE)
    	*py = - *py;

    if(sensor_dir & SENSOR_XY_SWITCH){
    	s8 tmp = *py;
    	*py = *px;
    	*px = tmp;
    }
}

extern char no_motion_rd;
extern char sensor_no_ov_rd;
static inline void mouse_sensor_data(mouse_status_t *mouse_status){    
    if( mouse_status->mouse_sensor & SENSOR_MODE_WORKING ){
    	mouse_data_t* mouse_data = mouse_status->data;
#if MOUSE_SENSOR_MOTION
    	int motion_level = gpio_read(mouse_status->hw_define->sensor_int);
        int motion_rd = no_motion_rd || !motion_level;
        if( motion_rd )
#endif
        {
            (*pf_sensor_motion_report) ( &mouse_data->x, sensor_no_ov_rd || SENSOR_IS_8589(mouse_status->mouse_sensor) );
			mouse_sensor_dir_adjust( &mouse_data->x, &mouse_data->y, mouse_status->sensor_dir );
            }
        }
}


#endif /* MOUSE_SENSOR_H_ */
