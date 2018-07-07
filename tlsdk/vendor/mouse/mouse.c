#include "../../proj/tl_common.h"
#include "../../proj/mcu/watchdog_i.h"
#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"
#include "../common/rf_frame.h"
#include "../common/emi.h"
#include "../common/device_power.h"

#include "../link_layer/rf_ll.h"

#include "device_info.h"
#include "mouse.h"
#include "mouse_rf.h"
#include "mouse_button.h"
#include "mouse_wheel.h"
#include "mouse_twheel.h"
#include "mouse_sensor_pix.h"
#include "mouse_sensor.h"
#include "mouse_batt.h"
#include "mouse_rf.h"
#include "mouse_custom.h"
#include "trace.h"

#ifndef MOUSE_GOLDEN_VPATTERN
#define MOUSE_GOLDEN_VPATTERN   0
#endif

#ifndef MOUSE_NO_HW_SIM
#define MOUSE_NO_HW_SIM         (MOUSE_GOLDEN_VPATTERN || MOUSE_EMI_4_FCC || 0)
#endif

#define	MOUSE_V_PATERN_MODE	    (MOUSE_NO_HW_SIM || 0)

mouse_status_t   mouse_status;
#if MOUSE_CAVY_RAM

#else
mouse_hw_t       mouse_hw;
#endif
mouse_data_t     mouse_data;

void mouse_sleep_wakeup_init( mouse_hw_t *pHW, u8 sensor ){
#ifdef reg_gpio_wakeup_en    
	reg_gpio_wakeup_en |= FLD_GPIO_WAKEUP_EN;	//gpio wakeup initial
#endif	
	reg_wakeup_en = FLD_WAKEUP_SRC_GPIO;        //core wakeup gpio enbable

	int i;

#if(MOUSE_WHEEL_WAKEUP_DEEP_EN)

	u32 suspend_wakeup_pin = pHW->button[5];
	u32 suspend_wakeup_level = (pHW->gpio_level_button[5] == U8_MAX);
	cpu_set_gpio_wakeup(suspend_wakeup_pin, !suspend_wakeup_level, 1);  //PAD wakeup

//	suspend_wakeup_level = !gpio_read(pHW->wheel[0]);
	//sleep_us(2);
	//cpu_set_gpio_wakeup(pHW->wheel[0], 0, 1);	//wheel wakeup sleep, high level vaild
	//cpu_set_gpio_wakeup(pHW->wheel[1], 0, 0);
	for ( i = MAX_MOUSE_BUTTON - 4; i>=0; i-- ){  //gpio¸´ÓÃÊ± ×óÖÐÓÒ»½ÐÑ
		u32 suspend_wakeuppin = pHW->button[i];
		u32 suspend_wakeupin_level = (pHW->gpio_level_button[i] == U8_MAX);
		cpu_set_gpio_wakeup(suspend_wakeuppin, !suspend_wakeupin_level, 1);  //PAD wakeup
	}
#else
	for ( i = MAX_MOUSE_BUTTON - 1; i>=0; i-- ){  //gpio¸´ÓÃÊ± ×óÖÐÓÒ»½ÐÑ
			u32 suspend_wakeuppin = pHW->button[i];
			u32 suspend_wakeupin_level = (pHW->gpio_level_button[i] == U8_MAX);
			cpu_set_gpio_wakeup(suspend_wakeuppin, !suspend_wakeupin_level, 1);  //PAD wakeup
		}
#endif
#if MOUSE_DEEPSLEEP_EN
    if ( SENSOR_IS_SIGMA_8640(mouse_status.mouse_sensor) )
        device_sleep.mcu_sleep_en = 0;
    else
        device_sleep.mcu_sleep_en = CUST_MCU_SLEEP_EN;
    
    device_sleep.sensor_sleep_en = CUST_SENSOR_SLEEP_EN;
    u32 tick = ((p_custom_cfg->slp_tick << 1) + p_custom_cfg->slp_tick) >> 6; //tick = 200ms * 256 * 256 * 3 / 64 = 614s
    device_sleep.thresh_100ms = sensor_motion_detct ? tick : (tick << 1);
#else
	device_sleep.thresh_100ms = U16_MAX;
#endif	
}

//button_init,Wheel init, Titl wheel init, Sensor init, led_init
void platform_init( mouse_status_t *pStatus )
{
	mouse_hw_t *pHW = pStatus->hw_define;
#if MOUSE_HW_CUS    
    u32 board_pix_2p4 = CUST_MOUSE_BOARD_2P4 || !gpio_read( GPIO_DM ); //pStatus->sensor_dir == SENSOR_DIRECTION_CLOCK_12;   //dir-0 & dir-1 pull-down
    u32 board_cob_sop_4p1 = CUST_MOUSE_BOARD_4P1 || !gpio_read( GPIO_DP );   //10k pull-down
    u32 board_sop_2p0_tl = CUST_MOUSE_BOARD_2P0 || CUST_MOUSE_BOARD_2P1;    
    u32 board_sop_2p0s = board_sop_2p0_tl || (!board_pix_2p4 && !board_cob_sop_4p1 && CUST_MOUSE_BOARD_AUTO);

#if 0
    //debug board 2.2/2.3
    board_sop_2p0s = 0;
    pHW->gpio_level_led = 0;
#endif
    if( board_sop_2p0s ){
        mouse_board_sop_2p0_cust( pHW );
    }
    if( board_sop_2p0_tl ){
        u8 bat_chn = CUST_MOUSE_BOARD_2P0 ? COMP_GP3 : (COMP_GP8 | MOUSE_BATT_CHN_REUSE_FLAG | MOUSE_BATT_CHN_REUSE_BTN_M);
        u32 motion_pin = CUST_MOUSE_BOARD_2P0 ? 0 : GPIO_GP3;
        pHW->button[3] = GPIO_GP8;    //bb - middle -GP8
        pHW->button[4] = GPIO_GP7;    //fb - left    -GP7
        pHW->gpio_level_led = 1;        //sop_v2.0 board ouput high to led-on
        pHW->led_cntr = GPIO_SWS;        
        pHW->vbat_channel = bat_chn;        
        pHW->sensor_int = motion_pin;
        if ( !motion_pin )
            no_motion_rd = 1;
    }
    u32 board_pix_sensor = mouse_sensor_hw_init(pHW, &pStatus->mouse_sensor, (pStatus->mouse_mode == STATE_POWERON));

 	if( board_sop_2p0s ){
        ;
    }
 	else if ( board_pix_sensor && board_cob_sop_4p1 ){ 
        mouse_board_cob_sop_4p1_cust( pHW, COMP_GP7 );       
        //mouse_board_cob_sop_4p1_cust( pHW, COMP_GP9 | MOUSE_BATT_CHN_REUSE_FLAG | MOUSE_BATT_CHN_REUSE_BTN_M );
    }
    else if ( board_pix_sensor && board_pix_2p4 ){
#if(MOUSE_WHEEL_WAKEUP_DEEP_EN)
        pHW->button[3]= GPIO_GP5;
        pHW->button[4]= GPIO_GP0;
        pHW->wheel[0]= GPIO_GP7;
        pHW->wheel[1]= GPIO_GP4;

#else
        pHW->button[3]= GPIO_GP7;
        pHW->wheel[0]= GPIO_GP5;
        pHW->wheel[1]= GPIO_GP0;
#endif
        if ( !CUST_MOUSE_BOARD_2P4 )
            pStatus->sensor_dir = (pStatus->sensor_dir == SENSOR_DIRECTION_CLOCK_12) ? SENSOR_DIRECTION_CLOCK_3 : pStatus->sensor_dir;
    }
    
    if ( mouse_cust_batter_reuse_middle_btn != U8_MAX ){
        pStatus->hw_define->vbat_channel |= mouse_cust_batter_reuse_middle_btn;
    }
#else
	mouse_sensor_hw_init(pHW, &pStatus->mouse_sensor, (pStatus->mouse_mode == STATE_POWERON));	
#endif    
	mouse_button_init(pHW);
    mouse_wheel_init(pHW);
	//mouse_twheel_init(pHW);
#if MOUSE_SW_CUS
#define led_cnt_rate    (8 << (pStatus->high_end == MS_HIGHEND_250_REPORTRATE))
	mouse_led_init(pHW->led_cntr, pHW->gpio_level_led, led_cnt_rate); 
#endif
#if MOUSE_DEEPSLEEP_EN
	mouse_sleep_wakeup_init(pStatus->hw_define, pStatus->mouse_sensor);
#endif
}

void  user_init(void)
{
	swire2usb_init();
#if MOUSE_CAVY_RAM

#else 		
	mouse_status.hw_define = &mouse_hw;
	mouse_status.led_define = &device_led;
#endif		
	mouse_status.data = &mouse_data;
	mouse_status.no_ack = 1;    //no_ack == 0, wakeup sensor from deep-sleep every time
#if ( MOUSE_NO_HW_SIM )
    p_custom_cfg = (custom_cfg_t *) (DEVICE_ID_ADDRESS);
#if MOUSE_GOLDEN_VPATTERN    
    extern rf_packet_pairing_t   pkt_pairing;
    pkt_pairing.did = p_custom_cfg->did;   //device-id init

#if(MOUSE_PIPE1_DATA_WITH_DID)
    extern rf_packet_mouse_t pkt_km;
    pkt_km.did = p_custom_cfg->did;   //device-id init
#endif

    u16 vendor_id = p_custom_cfg->vid;
    if(vendor_id != U16_MAX){
        rf_set_access_code0 (rf_access_code_16to32(vendor_id));
    }
#endif
#else
    gpio_pullup_dpdm_internal( FLD_GPIO_DM_PULLUP | FLD_GPIO_DP_PULLUP );
	mouse_custom_init( &mouse_status );
#endif

	device_info_load(&mouse_status);
#if ( !MOUSE_NO_HW_SIM )
	platform_init(&mouse_status);

#if(MOUSE_WHEEL_WAKEUP_DEEP_EN )
	cpu_set_gpio_wakeup(mouse_status.hw_define->wheel[1], 0, 0);
#endif

    mouse_sensor_init(&mouse_status.mouse_sensor, &mouse_status.cpi);
    if ( mouse_status.mouse_sensor & SENSOR_MODE_WORKING )
        cpu_set_gpio_wakeup(mouse_status.hw_define->sensor_int, 0, 1);  //PAD wakeup enable when wakeup sensor
	mouse_button_pull_and_detect(&mouse_status);
	mouse_button_process(&mouse_status);
    
    if ( mouse_status.mouse_mode == STATE_POWERON )
        mouse_led_setup( mouse_led_cfg[E_LED_POWER_ON] );
#endif
    mouse_rf_init(&mouse_status);
    rf_set_power_level_index (mouse_cust_tx_power_paring);

}

_attribute_ram_code_ void irq_handler(void)
{

	u32 src = reg_irq_src;
#if(!CHIP_8366_A1)
	if(src & FLD_IRQ_GPIO_RISC2_EN){
		gpio_user_irq_handler();
		reg_irq_src = FLD_IRQ_GPIO_RISC2_EN;
	}
#endif
	u16  src_rf = reg_rf_irq_status;
	if(src_rf & FLD_RF_IRQ_RX){
		irq_device_rx();
		//printf("It is rx interrupt\n");
	}

	if(src_rf & FLD_RF_IRQ_TX){
		irq_device_tx();
	}
}

void mouse_sync_process(mouse_status_t * mouse_status){
    static u16 pairing_time = 0;
    u32 pairing_end = 0;
#if MOUSE_GOLDEN_VPATTERN
    if ( HOST_NO_LINK ){
        mouse_status->mouse_mode = STATE_POWERON;
    }
#endif
    static u8 led_linked_host = 1;      //mouse, wakeup from deep-sleep, can blinky 3 times when linked with dongle
    if ( mouse_status->mouse_mode == STATE_POWERON ) {        
        //debug mouse get access code-1 from flash/otp
        if ( GET_HOST_ACCESS_CODE_FLASH_OTP != U16_MAX ){
		    rf_set_access_code1 ( rf_access_code_16to32( GET_HOST_ACCESS_CODE_FLASH_OTP ) );            
        }
        else{            
            rf_set_access_code1 ( U32_MAX );            //powe-on pkt link fail
        }

        mouse_sync_status_update( mouse_status );
	}
#if DEVICE_LED_MODULE_EN
    else{
        //if linked with dongle, led blinky 3 times
        if( led_linked_host && (mouse_status->no_ack == 0) ){
            mouse_led_setup( mouse_led_cfg[E_LED_RSVD] );
            led_linked_host = 0;
        }
    }
#endif
    if( mouse_status->mouse_mode == STATE_PAIRING ){
    	if( mouse_status->no_ack == 0 ){
    	    pairing_end = U8_MAX;        //pairing OK
    	}
        if( pairing_time >= 1024 ){
            pairing_end = 1;             //pairing fail, time out
        }
        else{            
            pairing_time += 1;
        }
        
        if( pairing_end ){
            pairing_time = 0;           //pairing-any-time could re-try many time
            mouse_led_setup( mouse_led_pairing_end_cfg_cust(pairing_end) );
            mouse_sync_status_update( mouse_status );
        }
        else{
            mouse_led_setup( mouse_led_cfg[E_LED_PAIRING] );
        }
    }
    else if ( mouse_status->mouse_mode != STATE_EMI ){
        mouse_sync_status_update( mouse_status );
    }
}
//u32 debug_last_wakeup_level;

#define   DEBUG_NO_SUSPEND      0
static inline void mouse_power_saving_process( mouse_status_t *mouse_status ){
    log_event( TR_T_MS_MACHINE);
#if MOUSE_DEEPSLEEP_EN
    device_sleep.quick_sleep = HOST_NO_LINK && QUICK_SLEEP_EN;

    if ( mouse_status->mouse_mode <= STATE_PAIRING ){
        if ( mouse_status->loop_cnt < 4096 )       //sync dongle time 4096 *8 ms most 
            device_sleep.quick_sleep = 0;        
    }
#endif
    device_sleep.device_busy = DEVICE_PKT_ASK || LED_EVENT_BUSY;
    if ( DEBUG_NO_SUSPEND || (mouse_status->dbg_mode & STATE_TEST_0_BIT) ){	    
        device_sleep.mode = M_SUSPEND_0;
    }
    else{
        if ( device_sleep.mode == M_SUSPEND_0 ){
#if(!MOUSE_GOLDEN_VPATTERN)					//use as golden
            while(1);     //watch dog reset
#endif
        }
        device_sleep_mode_machine( &device_sleep );
    }

#if MOUSE_WKUP_SENSOR_SIM
    u32 wkup_sesnor_sim = mouse_sensor_blinky_wkup( (device_sleep.device_busy && device_sleep.quick_sleep && !device_sleep.mcu_sleep_en) );
    if ( wkup_sesnor_sim ){
        mouse_status->no_ack = 0;
    }
#endif
    u32 wkup_sensor = ( (mouse_status->no_ack == 0) && DEVICE_PKT_ASK );
    u32 sensor_st = mouse_sensor_sleep_wakeup( &mouse_status->mouse_sensor, &device_sleep.sensor_sleep, wkup_sensor ); 
    if ( sensor_st & SENSOR_MODE_WORKING ){
        mouse_sensor_set_cpi( &mouse_status->cpi );
        cpu_set_gpio_wakeup(mouse_status->hw_define->sensor_int, 0, 1);  //PAD wakeup enable when wakeup sensor
    }
    else if ( sensor_st & SENSOR_MODE_POWERDOWN ){
        cpu_set_gpio_wakeup(mouse_status->hw_define->sensor_int, 0, 0);  //PAD wakeup disable when shut-down sensor
    }
	if ( M_SUSPEND_MCU_SLP & device_sleep.mode ){
    	device_info_save(mouse_status, 1);    	        //Save related information to 3.3V analog register
	}
#if	MOUSE_SW_CUS
	if ( mouse_status->high_end == MS_HIGHEND_250_REPORTRATE )
    	device_sleep.wakeup_time = 4;
	else
    	device_sleep.wakeup_time = 8;
#else
	device_sleep.wakeup_time = 8;
#endif
#if(CHIP_8366_A1)
    device_sleep.wakeup_src = PM_WAKEUP_TIMER;  //A1 wheel no need wkup 8ms suspend
#else
	device_sleep.wakeup_src = PM_WAKEUP_CORE | PM_WAKEUP_TIMER;  //wheel and timer can wakeup 8_ms sleep
#endif
	//Suspend, deep sleep GPIO, 32K setting
	if ( device_sleep.mode & M_SUSPEND_100MS ){
#if	MOUSE_SENSOR_MOTION
        device_sleep.wakeup_time = sensor_motion_detct ? 200 : 100;
#else
        device_sleep.wakeup_time = 100;
#endif
        device_sleep.wakeup_src = PM_WAKEUP_CORE | PM_WAKEUP_PAD | PM_WAKEUP_TIMER;
        device_sync = 0;        //ll_channel_alternate_mode ();
	}

#if MOUSE_DEEPSLEEP_EN
	else if ( M_SUSPEND_MCU_SLP & device_sleep.mode ){
		device_sleep.wakeup_time = 0;		
		device_sleep.wakeup_src = PM_WAKEUP_PAD;
        device_sync = 0;        //ll_channel_alternate_mode ();
        extern unsigned int m_sif_clk;
        extern unsigned int m_sif_data;

#if(MOUSE_WHEEL_WAKEUP_DEEP_EN )
        if(device_sleep.mode == M_SUSPEND_DEEP_SLP){		//deep sleep state, set wheel wakeup
        	u32 debug_cur_wakeup_level = gpio_read(mouse_status->hw_define->wheel[1]) ? 1 : 0;
        	cpu_set_gpio_wakeup(mouse_status->hw_define->wheel[1], !debug_cur_wakeup_level, 1);
        }
#endif


		if(!IIC_1M_PULLUP_EN)
		{
			gpio_setup_up_down_resistor(m_sif_clk, PM_PIN_PULLUP_1M);
			gpio_setup_up_down_resistor(m_sif_data, PM_PIN_PULLUP_1M);			//modify by zjs, use 1M pull up to prevent m_sif_data leakage
		}
	}
#endif
    log_event( TR_T_MS_SLEEP );
#if MOUSE_NO_HW_SIM
    device_sleep.mode = M_SUSPEND_0;
#endif
}

//low-end mouse higher current, active for longer time
#if MOUSE_SW_CUS
static inline void mouse_power_consume_process (void){
    if ( M_SUSPEND_8MS & device_sleep.mode ){
        WaitUs(mouse_cust_low_end_delay_time);
    }
}
#endif

void mouse_task_when_rf ( void ){
    dbg_led_high;
	//clear mouse_event_data	
	*((u32 *)(mouse_status.data)) = 0;
    u32 wheel_prepare_tick = mouse_wheel_prepare_tick();
    if ( MOUSE_V_PATERN_MODE || (mouse_status.dbg_mode & STATE_TEST_V_BIT) ){
        static u8 fno;
        mouse_status.data->x = fno & BIT(6) ? 6 : -6;
        mouse_status.data->y = fno & BIT(5) ? -6 : 6;
        fno++;
    }
#if MOUSE_NO_HW_SIM
    mouse_rf_post_proc(&mouse_status);
    mouse_sync_process(&mouse_status);
    mouse_power_saving_process(&mouse_status);
#else
    mouse_led_process(mouse_status.led_define);
    mouse_rf_post_proc(&mouse_status);
    mouse_sync_process(&mouse_status);

    log_event( TR_T_MS_BTN );
    mouse_power_saving_process(&mouse_status);
    mouse_wheel_process(&mouse_status, wheel_prepare_tick);
    
    mouse_button_detect(&mouse_status, MOUSE_BTN_LOW);     
    if ( (mouse_status.high_end != MS_HIGHEND_250_REPORTRATE) || (mouse_status.loop_cnt & 1) ){
        mouse_button_process(&mouse_status);
    }
#if    MOSUE_BATTERY_LOW_DETECT
    static u16 batt_det_count = 0;    
    if( (mouse_status.rf_mode != RF_MODE_IDLE) && (device_sleep.mode == M_SUSPEND_8MS) ){        
        if ( ++batt_det_count >= mouse_batt_detect_time ){
            mouse_batt_det_process(&mouse_status);
            batt_det_count = 0;
        }
    }
    else{
        batt_det_count = mouse_batt_detect_time;
    }
#endif
#endif
    mouse_status.loop_cnt ++;
    dbg_led_low;
}

_attribute_ram_code_ void mouse_task_in_ram( void ){
    p_task_when_rf = mouse_task_when_rf;
    cpu_rc_tracking_en (RC_TRACKING_32K_ENABLE);
    mouse_rf_process(&mouse_status);
    cpu_rc_tracking_disable;
    if (p_task_when_rf != NULL) {
       (*p_task_when_rf) ();
    }
    dbg_led_high;
#if MOUSE_SW_CUS
	if ( mouse_status.high_end == MS_HIGHEND_250_REPORTRATE )
	    ll_add_clock_time (4000);
    else        
        ll_add_clock_time (8000);
#else
	ll_add_clock_time (8000);
#endif

    device_sleep_wakeup( &device_sleep );
    dbg_led_low;
}

void main_loop(void)
{
	if( MOUSE_EMI_4_FCC || (mouse_status.mouse_mode == STATE_EMI) \
        || (mouse_status.dbg_mode & STATE_TEST_EMI_BIT) ){
		mouse_emi_process(&mouse_status);
	}
	else{
        log_event( TR_T_MS_EVENT_START );
#if MOUSE_SW_CUS
        if ( mouse_status.high_end == MS_LOW_END )
            mouse_power_consume_process ();
#endif
#if (!MOUSE_NO_HW_SIM)       
        mouse_button_pull(&mouse_status, MOUSE_BTN_LOW);
        mouse_sensor_data( &mouse_status );
        mouse_button_detect(&mouse_status, MOUSE_BTN_HIGH);        
        mouse_button_pull(&mouse_status, MOUSE_BTN_HIGH);
#endif
        mouse_task_in_ram();
	}
}
