/*
 * kb_info.c
 *
 *  Created on: 2015-1-21
 *      Author: Administrator
 */

#include "../../proj/tl_common.h"
#include "../../proj_lib/pm.h"
#include "../../proj_lib/rf_drv.h"
#include "../common/rf_frame.h"

#include "kb_rf.h"
#include "kb_info.h"


//u8 deep_count;
extern int kb_is_lock_pressed;
void kb_info_load(void)
{
#if 1
	kb_status.mode_link = analog_read(PM_REG_MODE_LINK);
	u8 check_read = analog_read(PM_REG_CHECK);
	//deep_count = analog_read(PM_REG_COUNT);
	kb_is_lock_pressed = analog_read(PM_REG_LOCK);
	u8 check_cal = 0;
	u8 * pd = (u8 *) (&kb_status.dongle_id);
	for (u8 i = PM_REG_DONGLE_ID_START; i <= PM_REG_DONGLE_ID_END; i++) {
		*pd  = analog_read (i);
		check_cal += *pd;
		pd++;
	}

	if((kb_status.mode_link&LINK_PIPE_CODE_OK) && check_cal==check_read && kb_status.dongle_id!=U32_MAX){
		kb_status.kb_mode = STATE_NORMAL;
	}
	else{
		kb_status.mode_link = 0;
		kb_status.dongle_id = 0;
	}
#endif
}

void kb_info_save(void)
{
#if 1
	u8 * pd;
	u8 check = 0;

	kb_status.dongle_id = rf_get_access_code1();
	pd = (u8 *) &kb_status.dongle_id;
	for (u8 i = PM_REG_DONGLE_ID_START; i <= PM_REG_DONGLE_ID_END; i++) {
		check += *pd;
		analog_write (i, *pd ++);
	}

	analog_write(PM_REG_MODE_LINK,kb_status.mode_link&LINK_PIPE_CODE_OK);
	analog_write(PM_REG_CHECK,check);
	//analog_write(PM_REG_COUNT,deep_count+1);
	analog_write(PM_REG_LOCK,kb_is_lock_pressed);

#endif
}
