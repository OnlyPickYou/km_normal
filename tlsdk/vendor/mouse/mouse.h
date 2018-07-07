#pragma once
#include "../common/mouse_type.h"
/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

#ifndef MOUSE_CAVY_RAM
#define MOUSE_CAVY_RAM				0
#endif

#ifndef MOUSE_DEEPSLEEP_EN
#define MOUSE_DEEPSLEEP_EN			1
#endif

#ifndef         MOUSE_EMI_4_FCC
#define         MOUSE_EMI_4_FCC     0
#endif

#if !MOUSE_EMI_4_FCC
#define         TP_GAIN_NOT_FIXED   1
#define         CUST_TP_GAIN_INFO_ADDR   (0x3f00 - 8)
#define         CUST_TP_GAIN0      ( ((*(unsigned char*) (CUST_TP_GAIN_INFO_ADDR + 2)) != 0xff) ? (*(unsigned char*) (CUST_TP_GAIN_INFO_ADDR + 2)) : (*(unsigned char*) (CUST_TP_GAIN_INFO_ADDR + 0)) ) 
#define         CUST_TP_GAIN1      ( ((*(unsigned char*) (CUST_TP_GAIN_INFO_ADDR + 3)) != 0xff) ? (*(unsigned char*) (CUST_TP_GAIN_INFO_ADDR + 3)) : (*(unsigned char*) (CUST_TP_GAIN_INFO_ADDR + 1)) ) 
#endif


#ifndef MOUSE_GPIO_FULL_RE_DEF
#define MOUSE_GPIO_FULL_RE_DEF  0
#endif


#define SWS_DATA_OUT 			1   //sws pullup: output high, output disable

#define _USER_CONFIG_DEFINED_	1	// must define this macro to make others known 

#define MAX_MOUSE_BUTTON        6

#define		PKT_BUFF_SIZE		48

typedef struct {

	u32 button[MAX_MOUSE_BUTTON];    //the sequence is left, right, middle, BB, FB, DPI
	u32 led_cntr;//tiger.yang

	//以下三项凑成8 bytes  后面定制信息位置偏移保持不变
	//vbat_channel定制时直接采用pm_8366.h中的COMP_CHANNALE，省去程序中对gpio进行comp_chn转换的代码
	u8 gpio_level_button[MAX_MOUSE_BUTTON];  // 0-5 : button pull up/down(1 for pullup)
	u8 gpio_level_led;   //led on level(1 for high valid)
	u8 vbat_channel;

	u32 cfg_1_r[2];
	u32 cfg_2_r[2];

	u32 wheel[2];
	u32 sensor_data;
	u32 sensor_sclk;
	u32 sensor_int;

}mouse_hw_t;


typedef struct {

	u8  mouse_mode;                //EMI mode, power on, pairing auto/manual, normal mode etc
	u8  dbg_mode;
	u8  high_end;                  //high-end mouse or low-end
	u8  no_ack;                    //indicate whether get the ACK for previous package send

	u8  mouse_sensor;
	u8  cpi;
	u8  sensor_dir;
	s8  wheel_dir;

	u32 dongle_id;
	u32 device_id;
	
    u8  rf_mode;					//rf data /pairing / idle, also make mouse_data_t aligned to 4
	u8  tx_power;
	u8  tx_retry;
    u8  rcv_rssi;
    
    u32 pkt_addr;    
    u32 loop_cnt;
    
    device_led_t  *led_define;
	mouse_hw_t    *hw_define;
	mouse_data_t  *data;

} mouse_status_t;


typedef enum{
	STATE_POWERON = 0,        
	STATE_SYNCING,
	STATE_PAIRING ,
	STATE_NORMAL,
	STATE_SUSPEND,
	STATE_DEEPSLEEP,
	STATE_EMI,

}MOUSE_MODE;

#define STATE_TEST_0_BIT    0x80
#define STATE_TEST_EMI_BIT  0x40
#define STATE_TEST_V_BIT    0x20

#include "mouse_default_config.h"
/////////////////// set default   ////////////////

#include "../common/default_config.h"

/////////////////// main loop, event loop  ////////////////
enum {
	EV_FIRED_EVENT_MAX = 8
};

typedef enum {
	EV_SUSPEND_NOTIFY,
	EV_WAKEUP_NOTIFY,
	EV_KEY_PRESS,
#if(MOUSE_USE_RAW_DATA)
	EV_MOUSE_RAW_DATA,
#endif	
	EV_RF_PKT_RECV,
	EV_PAIRING_START,
	EV_PAIRING_STOP,
	EV_MOUSE_EVENT,
	EV_KEYBOARD_EVENT,
#if(MODULE_SOMATIC_ENABLE)	
	EV_SOMATIC_EVENT,
#endif
	EV_EVENT_MAX,
} ev_event_e;

typedef enum {
	EV_POLL_MOUSE_EVENT, EV_POLL_KEYBOARD_EVENT,
#if(MODULE_SOMATIC_ENABLE)	
	EV_POLL_SOMATIC_EVENT,
#endif	
	EV_POLL_RF_RECV, EV_POLL_DEVICE_PKT, EV_POLL_RF_CHN_HOPPING, EV_POLL_IDLE, //  Must be the last item in ev_poll_e
	EV_POLL_MAX,
} ev_poll_e;

typedef enum {
    MS_LOW_END,                  //large current and short distance
	MS_HIGHEND_250_REPORTRATE,   //250 report rate, get sensor data per 4ms
	MS_HIGHEND_ULTRA_LOW_POWER,  //send pkt per 8ms * 3	
	MS_HIGHEND_DEFAULT,          //
} MOUSE_HIGHEND_RATE;

extern mouse_status_t   mouse_status;
void mouse_task_when_rf ( void );

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif

