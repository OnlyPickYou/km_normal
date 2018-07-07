/*
 * mouse_emi.c
 *
 *  Created on: Feb 14, 2014
 *      Author: xuzhen
 */

#include "../../proj/tl_common.h"
#include "../../proj/mcu/watchdog_i.h"
#include "../../proj_lib/rf_drv.h"
#include "../common/emi.h"
#include "mouse.h"
#include "mouse_button.h"
#include "mouse_wheel.h"
#include "mouse_twheel.h"
#include "mouse_emi.h"
#include "mouse_custom.h"

#if MOUSE_EMI_4_FCC
void mouse_emi_process(mouse_status_t *mouse_status)
{
    static s8   test_chn_idx = 1;    
    static u8   test_mode_sel = 0;
    static u8   flg_emi_init = 0;
    u32 cmd;
    static u8 *op_emi_mode  = 0x808001;
    static u8 *op_emi_chn =  0x808002;
    static u8 op_emi_init = 0;
    if ( flg_emi_init == 0 ){
        *op_emi_chn = 40;
        *op_emi_mode = 0;
        flg_emi_init = 1;
    }
    cmd = (test_chn_idx != *op_emi_chn) || (test_mode_sel != *op_emi_mode);
    test_chn_idx = *op_emi_chn;
    test_mode_sel = *op_emi_mode;
    
    emi_process( cmd , test_chn_idx, test_mode_sel, mouse_status->pkt_addr, mouse_cust_tx_power_emi );
}
#else

const led_cfg_t mouse_led_emi_cfg[] = {
    1,      2,      2,      0x80,    //Carrier
    4,      8,      2,      0x80,    //Carrier & Data
    0,      1,      2,      0x80,    //RX
    2,      0,      2,      0x80,    //TX
};

void mouse_sop16_2p0_emi_rx_ini( void ){
    write_reg8 ( 0x800066, read_reg8(0x800066) & ~(BIT(5)) | BIT(6) );  //switch 32M RC to pad-16M
    WriteAnalogReg( 0x05, 0x04);  //close 32M RC
}

void mouse_sop16_2p0_emi_rx_recover( void ){
    WriteAnalogReg( 0x05, 0x00);  //enable 32M RC
    write_reg8 ( 0x800066, read_reg8(0x800066) & ~(BIT(6)) | BIT(5) );  //switch to 32M RC from pad-16M
}

void mouse_emi_process(mouse_status_t *mouse_status)
{
    static s8   test_chn_idx = 1;    
    static u8   test_mode_sel = 0;
    static u8   flg_emi_init = 0;
    u32 cmd;    
    u32 btn_last;
	btn_last = mouse_button_pull_and_detect( mouse_status );
    if( mouse_status->dbg_mode & STATE_TEST_EMI_BIT ){        
        cmd = mouse_button_process_emi( &test_chn_idx, &test_mode_sel, 0 );
        mouse_button_process(mouse_status);
        if( !(mouse_status->dbg_mode & STATE_TEST_EMI_BIT) ){    //recover to emi rx mode
            cmd = 1;
            test_mode_sel = 2;
        }
    }
    else{
       cmd = mouse_button_process_emi( &test_chn_idx, &test_mode_sel, 1 );
    }
    cmd |= !flg_emi_init;
    if( !flg_emi_init ){
        flg_emi_init = 1;
#if MOUSE_HW_CUS
        if ( CUST_MOUSE_BOARD_2P0 || CUST_MOUSE_BOARD_2P1 ){
            pf_emi_rx_init = mouse_sop16_2p0_emi_rx_ini;            
            pf_emi_rx_recover = mouse_sop16_2p0_emi_rx_recover;
        }
#endif
    }    
    
    if ( (device_led.gpio == GPIO_SWS) && (btn_last & FLAG_BUTTON_BACKWARD) ){
        gpio_set_output_en( GPIO_SWS, 0 );        
        gpio_set_input_en( GPIO_SWS, 1 );        
		gpio_set_func( GPIO_SWS, AS_SWIRE);      //emi mode and btn_bb set sws in debug mode		
    }
    else{        
        mouse_led_setup( mouse_led_emi_cfg[test_mode_sel] );
	    mouse_led_process( mouse_status->led_define );
    }
	emi_process( cmd , test_chn_idx, test_mode_sel, mouse_status->pkt_addr, mouse_cust_tx_power_emi );
}
#endif
