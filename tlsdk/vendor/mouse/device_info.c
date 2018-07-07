#include "../../proj/tl_common.h"
#include "../../proj/mcu/watchdog_i.h"
#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"
#include "../common/rf_frame.h"
#include "device_info.h"
#include "mouse.h"

#define			PM_REG_START		0x19
#define			PM_REG_END			0x1f

#ifndef PM_POWERON_DETECTION_ENABLE
#define PM_POWERON_DETECTION_ENABLE 0
#endif

static device_info_t device_info;

/*
 *  Base on from power on or deep sleep back
 *  Load customizable information from the 3.3V Analog register
 *  or Load from OTP
 *
 */
void device_info_load(mouse_status_t *mouse_status)
{
#if DEVICE_INFO_STORE
    u8 * pd = (u8 *) &device_info;
    int i;
    for (i=PM_REG_START; i<=PM_REG_END; i++) {
        *pd ++ = analog_read (i);
    }
#if PM_POWERON_DETECTION_ENABLE
    if (device_info.mode != 0xe5) {
        device_info.mode = 0;
    }
    device_info.poweron = 0xe0;
    analog_write (PM_REG_END, device_info.poweron);    
#else
    device_info.poweron = ((device_info.mode^0x80) + 2) | 1;
    analog_write (PM_REG_END, device_info.poweron);
#endif
    mouse_status->mouse_mode = device_info.mode ? STATE_NORMAL : STATE_POWERON;

//   Need get poweron, cpi, etc back first
    if ( mouse_status->mouse_mode != STATE_POWERON ){
    	mouse_status->cpi = device_info.sensor & INFO_SENSOR_CPI_CTRL;
        mouse_status->mouse_sensor = device_info.sensor & INFO_SENSOR_STATUS_CTRL;
    	mouse_status->dongle_id = device_info.dongle_id;
        rf_set_access_code1 (mouse_status->dongle_id);
        }
#else
    mouse_status->mouse_mode = device_info.mode ? STATE_NORMAL : STATE_POWERON;
#endif
}

#if DEVICE_INFO_STORE
/*
 * Save the information need from the deep sleep back
 *
 */
void device_info_save(mouse_status_t *mouse_status, u32 sleep_save)
{
    u8 * pd = (u8 *) &device_info;
    int i;
    //if watchdog trigger, this flag will increase by 2
#if PM_POWERON_DETECTION_ENABLE
    device_info.mode = 0xe5;
#else
    if ( sleep_save )
        device_info.mode = device_info.poweron - 2;
#endif
    device_info.dongle_id = rf_get_access_code1();
    device_info.sensor = (mouse_status->mouse_sensor & INFO_SENSOR_STATUS_CTRL) | (mouse_status->cpi & INFO_SENSOR_CPI_CTRL) ;
    for (i=PM_REG_START; i<=PM_REG_END; i++) {
        analog_write (i, *pd ++);
    }
}
#endif

