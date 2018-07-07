#include "../../proj/tl_common.h"
#include "../../proj/mcu/watchdog_i.h"
#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"
#include "../common/rf_frame.h"

#include "../common/device_power.h"

#include "../../proj/drivers/keyboard.h"

#include "kb_emi.h"
#include "kb_rf.h"
#include "kb_custom.h"
#include "kb_info.h"
#include "kb_batt.h"
#include "kb_pm.h"
#include "kb_ota.h"
#include "kb_led.h"
#include "kb_test.h"


kb_lock_status_t kb_lock_status = {
		0,
		0,

		0,
		0,
};

void kb_platform_init(void);
extern  void kb_keyScan_init(void);

kb_status_t  kb_status;

extern kb_data_t	kb_event;

void  user_init(void)
{
	//memset(&kb_status,0,sizeof(kb_status));
	kb_status.no_ack = 1;
	//kb_keyScan_init();
	kb_custom_init();
	kb_batt_det_init();
	kb_info_load();
	kb_rf_init();

	int status = kb_status.host_keyboard_status | (kb_status.kb_mode == STATE_POWERON ? KB_NUMLOCK_STATUS_POWERON : 0);
	kb_scan_key (status, 1);
	kb_status.host_keyboard_status = KB_NUMLOCK_STATUS_INVALID;

	kb_platform_init();
	kb_pm_init();
	//gpio_gp26_init;

}


void kb_platform_init(void)
{
	kb_device_led_init(kb_status.led_gpio_lvd, kb_status.led_level_lvd, 5);

	if(kb_status.led_gpio_scr){
		gpio_set_input_en( kb_status.led_gpio_scr, 0 );
	    gpio_set_output_en( kb_status.led_gpio_scr, 1 );
	    gpio_write(kb_status.led_gpio_scr,0^kb_status.led_level_scr);
	}
	if(kb_status.led_gpio_cap){
		gpio_set_input_en( kb_status.led_gpio_cap, 0 );
	    gpio_set_output_en( kb_status.led_gpio_cap, 1 );
	    gpio_write(kb_status.led_gpio_cap,0^kb_status.led_level_cap);
	}
	if(kb_status.led_gpio_num){
		gpio_set_input_en( kb_status.led_gpio_num, 0 );
	    gpio_set_output_en( kb_status.led_gpio_num, 1 );
	    gpio_write(kb_status.led_gpio_num,0^kb_status.led_level_num);
	}

	if(!gpio_read(GPIO_GP31)){
		SET_SINGLE_BATT();
	}
	analog_write (0x2a, analog_read(0x2a)&0x3f );  //disable GP31 1M pullup


	//emi paring±ØÐëÔÚSTATE_POWERON?
	if(kb_status.kb_mode == STATE_POWERON){
		if( (kb_event.keycode[0] == VK_ESC) && (kb_event.cnt == 1)){
			kb_status.kb_mode  = STATE_EMI;
		}
		else{
			kb_device_led_setup( kb_led_cfg[KB_LED_POWER_ON] );
			kb_status.kb_mode = STATE_SYNCING;
			memset(&kb_event,0,sizeof(kb_event));  //clear the numlock key when power on
		}
	}
}

_attribute_ram_code_ void irq_handler(void)
{
	u16  src_rf = reg_rf_irq_status;
	if(src_rf & FLD_RF_IRQ_RX){
		irq_device_rx();
	}

	if(src_rf & FLD_RF_IRQ_TX){
		//irq_device_tx();
		reg_rf_irq_status = FLD_RF_IRQ_TX;
	}
}

extern int kb_is_lock_pressed;



u32 key_scaned;

void kb_lock_func()
{
	//static u8 kb_lock_cnt = 0;
	kb_lock_status.cur_kb_cnt = kb_event.cnt;
	kb_lock_status.cur_keycode = kb_event.keycode[0];

	if(kb_lock_status.cur_kb_cnt == 1 && kb_lock_status.cur_keycode == VK_LOCK && !kb_lock_status.last_kb_cnt){
		//kb_lock_cnt++;
		kb_is_lock_pressed = !kb_is_lock_pressed? KB_STATUS_LOCK : 0; //lock <-> unlock

		 //lock -> unlock
		if(!kb_is_lock_pressed && kb_status.host_keyboard_status == KB_NUMLOCK_STATUS_INVALID){
			kb_event.cnt = 0;  //send 0 to dongle, get ack to have numlcom status
		}
	}

	if(kb_is_lock_pressed){
		key_scaned = 0;
	}

	kb_lock_status.last_kb_cnt = kb_event.cnt;
	kb_lock_status.last_keycode = kb_event.keycode[0];
}
void main_loop(void)
{
	cpu_rc_tracking_en(RC_TRACKING_32M_ENABLE);
	batt_det_count++;
	key_scaned = kb_scan_key (kb_status.host_keyboard_status | kb_is_lock_pressed, !km_dat_sending);

	if(key_scaned){
		kb_lock_func();
	}

	if(kb_status.kb_mode == STATE_EMI){
		kb_emi_process();
	}
	else{
		if(kb_status.kb_mode <= STATE_PAIRING){
			kb_paring_and_syncing_proc();
		}

		kb_rf_proc(key_scaned);
		kb_device_led_process();
		kb_batt_det_process();
		kb_pm_proc();
	}
	cpu_rc_tracking_disable;
	kb_status.loop_cnt++;

}



