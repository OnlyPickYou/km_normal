/*
 * kb_emi.c
 *
 *  Created on: 2015-1-21
 *      Author: Administrator
 */


#include "../../proj/tl_common.h"
#include "../../proj_lib/rf_drv.h"
#include "../common/rf_frame.h"
#include "../common/emi.h"

#include "kb_custom.h"
#include "kb_rf.h"
#include "kb_led.h"
#include "kb_emi.h"

extern kb_data_t	kb_event;


#define  EMI_CMD_KEY_MATCH   (kb_event.keycode[0]>=VK_F1 && kb_event.keycode[0]<=VK_F12)

kb_led_cfg_t kb_led_emi_cfg[] = {
    {1,      2,      2,      0x80},    //Carrier
    {4,      8,      2,      0x80},    //Carrier & Data
    {0,      1,      2,      0x80},    //RX
    {2,      0,      2,      0x80}     //TX
};


typedef struct{
	u8 emi_start;
	u8 key_last;
	u8 key_pre;
	u8 rsv1;
	u8 test_chn_idx;  //dafault 2430  carrier
    u8 test_mode_sel;
    u8 flg_emi_init;
    u8 rsv2;

}kb_emi_data_t;


kb_emi_data_t  kb_emi_data ={
		VK_ESC,
		0,
		0,
		0,
		1,  //dafault 2430  carrier
		0,
		0,
		0
};



static void kb_proc_key (void)
{
	if(kb_event.cnt == 1 && EMI_CMD_KEY_MATCH){
		kb_emi_data.key_last = kb_event.keycode[0];
	}
}

u32 kb_key_process_emi(u8 *chn_idx, u8 *test_mode_sel, u32 btn_pro_end)  //zjs need debug
{
	u32 cmd = 0;
	if(kb_emi_data.key_last != kb_emi_data.key_pre && EMI_CMD_KEY_MATCH)		//new event
	{
		cmd = 0x80;
		kb_emi_data.key_last -= VK_F1;
		*test_mode_sel = (kb_emi_data.key_last) & 3;
		*chn_idx = (kb_emi_data.key_last) >> 2;
	}

	if(btn_pro_end)
		kb_emi_data.key_pre = kb_emi_data.key_last;
	return cmd;
}


/*****************************************************************************
test_chn_idx:
host_cmd_chn_l  = 0,   2405
host_cmd_chn_m  = 6,   2430
host_cmd_chn_h  = 14,  2470
test_mode_sel:
0 :	host_cmd_carrier
1 :	host_cmd_cd
2 :	host_cmd_rx
3 :	host_cmd_tx
*****************************************************************************/


void kb_emi_process(void)
{
    kb_proc_key();
    u32 cmd = 0;

    cmd = kb_key_process_emi( &kb_emi_data.test_chn_idx, &kb_emi_data.test_mode_sel, 1 );
    cmd |= !kb_emi_data.flg_emi_init;
    if( !kb_emi_data.flg_emi_init ){
    	kb_emi_data.flg_emi_init = 1;
    }
    kb_device_led_setup( kb_led_emi_cfg[kb_emi_data.test_mode_sel] );
    kb_device_led_process();
	emi_process( cmd , kb_emi_data.test_chn_idx, kb_emi_data.test_mode_sel, (u8 *)&pkt_km, kb_cust_tx_power_emi );
}
