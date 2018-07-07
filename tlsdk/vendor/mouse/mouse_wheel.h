/*
 * mouse_wheel.h
 *
 *  Created on: Feb 12, 2014
 *      Author: xuzhen
 */

#ifndef MOUSE_WHEEL_H_
#define MOUSE_WHEEL_H_

#ifndef MOUSE_SENSOR_IO_FIXED
#define MOUSE_SENSOR_IO_FIXED	0
#endif

#include "../../proj/config/user_config.h"

void mouse_wheel_init(mouse_hw_t *mouse_hw);

#if(CHIP_8366_A1)

#define WHEEL_TWO_STEP_PROC   1

static inline u32 mouse_wheel_prepare_tick( void ){
	write_reg8(0x8000d8,0x01);
	return clock_time();
}

static inline void mouse_wheel_process(mouse_status_t  *mouse_status, u32 wheel_prepare_tick)
{
	while(read_reg8(0x8000d8) != 0){
		if(clock_time_exceed(wheel_prepare_tick, 150)){  //4 cylce is enough: 4*1/32k = 1/8 ms
			write_reg8(0x8000d6,0x01); //reset  d6[0]
			write_reg8(0x8000d6,0x00);
			break;
		}
	}

#if(WHEEL_TWO_STEP_PROC)
	static signed char accumulate_wheel_cnt;
	signed char wheel_cnt = read_reg8(0x8000d0);

	wheel_cnt += accumulate_wheel_cnt;
	if(wheel_cnt&1){  //Ææ
		accumulate_wheel_cnt = wheel_cnt > 0 ? 1 : -1;
	}
	else{ //Å¼
		accumulate_wheel_cnt = 0;
	}

	mouse_status->data->wheel = (wheel_cnt/2) * mouse_status->wheel_dir;
#else
	signed char wheel_cnt = read_reg8(0x8000d0);
	mouse_status->data->wheel = wheel_cnt * mouse_status->wheel_dir;
#endif
}

#else

static inline u32 mouse_wheel_prepare_tick( void ){
    return 0;
}

extern s8 wheel_cnt;
static inline void mouse_wheel_process(mouse_status_t  * mouse_status, u32 wheel_prepare_tick){
	mouse_status->data->wheel = wheel_cnt * mouse_status->wheel_dir;
	wheel_cnt = 0;
}

#endif

void mouse_wheel_detect(mouse_status_t  * mouse_status);

#endif /* MOUSE_WHEEL_H_ */
