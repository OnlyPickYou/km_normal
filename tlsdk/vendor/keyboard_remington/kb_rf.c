/*
 * kb_rf.c
 *
 *  Created on: 2015-1-21
 *      Author: Administrator
 */
#if(__PROJECT_REMINGTON_KEYBOARD__)

#include "../../proj/tl_common.h"
#include "../../proj_lib/pm.h"
#include "../../proj_lib/rf_drv.h"
#include "../common/rf_frame.h"

#include "kb_custom.h"
#include "kb_led.h"
#include "kb_rf.h"
#include "kb_pm.h"


#define		CHANNEL_SLOT_TIME				8000

#define		LL_CHANNEL_SEARCH_TH			60
#define		LL_CHANNEL_SEARCH_FLAG			BIT(16)
#define		LL_NEXT_CHANNEL(c)				((c + 6) & 14)

#define     PKT_BUFF_SIZE   				48

u8  	rf_rx_buff[PKT_BUFF_SIZE*2] __attribute__((aligned(4)));
int		rf_rx_wptr;

u8		device_channel;
u16		ll_chn_tick;

u32		ll_chn_mask = LL_CHANNEL_SEARCH_FLAG;
u32		ll_clock_time;
u32		tick_last_tx;
int		device_packet_received;
int		km_dat_sending = 0;


#if(!KEYBOARD_PIPE1_DATA_WITH_DID)
u8 pipe1_send_id_flg = 0;
#endif
volatile int		device_ack_received = 0;


u8* kb_rf_pkt = (u8*)&pkt_pairing;



extern kb_data_t	kb_event;

//02(50/10)510b
rf_packet_pairing_t	pkt_pairing = {
		sizeof (rf_packet_pairing_t) - 4,	// 0x0c=16-4,dma_len
#if RF_FAST_MODE_1M
		RF_PROTO_BYTE,
		sizeof (rf_packet_pairing_t) - 6,	// rf_len
#else
		sizeof (rf_packet_pairing_t) - 5,	// 0x0b=16-5,rf_len
		RF_PROTO_BYTE,						// 0x51,proto
#endif
		PKT_FLOW_DIR,						// flow, pairing type: auto(0x50) or manual(0x10)
		FRAME_TYPE_KEYBOARD,				// 0x02,type

//		PIPE0_CODE,			// gid0

		0,					// rssi
		0,					// per
		0,					// seq_no
		0,					// reserved
		0xdeadbeef,			// device id
};




//0280510f
rf_packet_keyboard_t	pkt_km = {
		sizeof (rf_packet_keyboard_t) - 4,	//0x10=20-4,dma_len

		sizeof (rf_packet_keyboard_t) - 5,	//0x0f=20-5,rf_len
		RF_PROTO_BYTE,						// 0x51, proto
		PKT_FLOW_DIR,						// 0x80, kb data flow
		FRAME_TYPE_KEYBOARD,				// 0x02, type

//		U32_MAX,			// gid0

		0,					// rssi
		0,					// per
		0,					// seq_no
		1,					// pno
};

int kb_pairing_mode_detect(void)
{
	//if power on detect combined key is ESC+Q
	if (   (VK_ESC == kb_event.keycode[0]&& VK_Q   == kb_event.keycode[1] && 2 == kb_event.cnt) \
		|| (VK_Q   == kb_event.keycode[0]&& VK_ESC == kb_event.keycode[1] && 2 == kb_event.cnt) ){
		return 1;
	}
	return 0;
}



_attribute_ram_code_ u8	get_next_channel_with_mask(u32 mask, u8 chn)
{
	int chn_high = (mask >> 4) & 0x0f;

	if (mask & LL_CHANNEL_SEARCH_FLAG) {
		return LL_NEXT_CHANNEL (chn);
	}
	else if (chn_high != chn) {
		return chn_high;
	}
	else {
		return mask & 0x0f;
	}
}


/////////////////////////////////////////////////////////////////////////
// device side
/////////////////////////////////////////////////////////////////////////
void ll_device_init (void)
{
	reg_dma_rf_rx_addr = (u16)(u32) (rf_rx_buff);
	reg_dma2_ctrl = FLD_DMA_WR_MEM | (PKT_BUFF_SIZE>>4);   // rf rx buffer enable & size
	reg_dma_chn_irq_msk = 0;
	reg_irq_mask |= FLD_IRQ_ZB_RT_EN;    //enable RF & timer1 interrupt
	reg_rf_irq_mask = FLD_RF_IRQ_RX | FLD_RF_IRQ_TX;
}



_attribute_ram_code_ void irq_device_rx(void)
{
	u8 * raw_pkt = (u8 *) (rf_rx_buff + rf_rx_wptr * PKT_BUFF_SIZE);
	rf_rx_wptr = (rf_rx_wptr + 1) & 1;
	reg_dma_rf_rx_addr = (u16)(u32) (rf_rx_buff + rf_rx_wptr * PKT_BUFF_SIZE); //set next buffer

	reg_rf_irq_status = FLD_RF_IRQ_RX;

	if	(	raw_pkt[0] >= 15 &&
			RF_PACKET_LENGTH_OK(raw_pkt) &&
			RF_PACKET_CRC_OK(raw_pkt) )	{
		rf_packet_ack_pairing_t *p = (rf_packet_ack_pairing_t *)(raw_pkt + 8);
        rf_power_enable (0);
		extern int  rf_rx_process(u8 *);
		if (rf_rx_process (raw_pkt) && ll_chn_tick != p->tick) {
			ll_chn_tick = p->tick;			//sync time
			device_ack_received = 1;
			ll_chn_mask = p->chn;			//update channel
		}
		rf_set_channel (device_channel, RF_CHN_TABLE);
		raw_pkt[0] = 1;
	}
}


extern rf_packet_pairing_t	pkt_pairing;
task_when_rf_func p_task_when_rf = NULL;

_attribute_ram_code_ int	device_send_packet (u8 * p, u32 timeout, int retry, int pairing_link)
{
    while ( !clock_time_exceed (kb_sleep.wakeup_tick, 500) );    //delay to get stable pll clock
	rf_power_enable (1);

	static	u32 ack_miss_no;
	device_ack_received = 0;
	int i;

	for (i=0; i<=retry; i += 1) {
		rf_set_channel (device_channel, RF_CHN_TABLE);
		u32 t = clock_time ();
		rf_send_packet (p, 300, 0);
		reg_rf_irq_status = 0xffff;
        /*if ( DO_TASK_WHEN_RF_EN && p_task_when_rf != NULL) {
           (*p_task_when_rf) ();
           p_task_when_rf = NULL;
        }*/
		while (	!device_ack_received &&
				!clock_time_exceed (t, timeout) &&
				!(reg_rf_irq_status & (FLD_RF_IRX_RETRY_HIT | FLD_RF_IRX_CMD_DONE)) );

		if (device_ack_received) {
			ack_miss_no = 0;
			break;
		}
		ack_miss_no ++;
		if (ack_miss_no >= LL_CHANNEL_SEARCH_TH) {
			ll_chn_mask = LL_CHANNEL_SEARCH_FLAG;
		}

		device_channel = get_next_channel_with_mask (ll_chn_mask, device_channel);
	}

	rf_power_enable (0);

	if (i <= retry) {
		return 1;
	}
	else{
		return 0;
	}
}



void kb_rf_init(void)
{
	ll_device_init ();
	rf_receiving_pipe_enble(0x3f);	// channel mask
	kb_status.tx_retry = 5;

    if(kb_status.kb_mode == STATE_NORMAL){  //link OK deep back
    	rf_set_access_code1 (kb_status.dongle_id);
    	kb_rf_pkt = (u8*)&pkt_km;
    	rf_set_tx_pipe (PIPE_KEYBOARD);
    	rf_set_power_level_index (kb_cust_tx_power);
    }
    else{ //poweron or link ERR deepback
    	rf_set_access_code1 ( U32_MAX );
    	rf_set_tx_pipe (PIPE_PARING);
    	rf_set_power_level_index (kb_cust_tx_power_paring);
    }
}


extern u32 key_scaned;
void kb_paring_and_syncing_proc(void)
{
    if( (kb_status.mode_link&LINK_PIPE_CODE_OK) && kb_rf_pkt != (u8*)&pkt_km ){ //if link on,change to KB data pipe   kb_mode to STATE_NORMAL
    	kb_rf_pkt = (u8*)&pkt_km;
    	rf_set_tx_pipe (PIPE_KEYBOARD);
    	kb_status.tx_retry = 2;
    	rf_set_power_level_index (kb_cust_tx_power);
    	if(kb_status.kb_mode == STATE_PAIRING){
    		kb_device_led_setup( kb_led_cfg[KB_LED_PAIRING_OK] );
    	}

    	kb_status.kb_mode = STATE_NORMAL;
    	memset(&kb_event,0,sizeof(kb_event));
    	key_scaned = 1;
    }

	if( kb_status.kb_mode == STATE_PAIRING ){  //manual paring
    	//pkt_pairing.flow = PKT_FLOW_PARING;
        if( kb_status.loop_cnt >= KB_MANUAL_PARING_MOST ){ //pairing timeout,change to syncing mode
            kb_status.kb_mode = STATE_SYNCING;
            pkt_pairing.flow = PKT_FLOW_TOKEN;
            kb_device_led_setup( kb_led_cfg[KB_LED_PAIRING_END] );
            rf_set_power_level_index (kb_cust_tx_power);
        }
    }
	else if ( kb_status.kb_mode == STATE_SYNCING){
		if(kb_status.loop_cnt < KB_PARING_POWER_ON_CNT){
			pkt_pairing.flow = PKT_FLOW_TOKEN | PKT_FLOW_PARING;
		}
		else if(kb_status.loop_cnt == KB_PARING_POWER_ON_CNT){
			pkt_pairing.flow = PKT_FLOW_TOKEN;
			rf_set_power_level_index (kb_cust_tx_power);
		}

		if( kb_pairing_mode_detect() ){
			//SET_PARING_ENTERED();
			pkt_pairing.flow = PKT_FLOW_PARING;
			kb_device_led_setup(kb_led_cfg[KB_LED_MANUAL_PAIRING]);	//8Hz,fast blink
			kb_status.kb_mode  = STATE_PAIRING;
			rf_set_power_level_index (kb_cust_tx_power_paring);
			kb_status.loop_cnt = KB_PARING_POWER_ON_CNT;
		}
	}
}

_attribute_ram_code_ int  rf_rx_process(u8 * p)
{
	rf_packet_ack_pairing_t *p_pkt = (rf_packet_ack_pairing_t *) (p + 8);
	if (p_pkt->proto == RF_PROTO_BYTE) {
		pkt_pairing.rssi = p[4];
		///////////////  Paring/Link ACK //////////////////////////
		if ( p_pkt->type == FRAME_TYPE_ACK && (p_pkt->did == pkt_pairing.did) ) {	//paring/link request
			//change to pip2 STATE_NORMAL
			rf_set_access_code1 (p_pkt->gid1);//need change to pipe2 that is for kb's data
			kb_status.mode_link = LINK_WITH_DONGLE_OK;
			return 1;
		}
		////////// end of PIPE1 /////////////////////////////////////
		///////////// PIPE1: ACK /////////////////////////////
		else if (p_pkt->type == FRAME_TYPE_ACK_KEYBOARD) {
			kb_status.kb_pipe_rssi = p[4];
			kb_status.host_keyboard_status =((rf_packet_ack_keyboard_t*)p_pkt)->status;

			//BIT(2):SCR  BIT(1):CAP  BIT(0):NUM
			if(kb_status.host_keyboard_status != kb_status.pre_host_status){
				if(kb_status.led_gpio_scr){
					gpio_write(kb_status.led_gpio_scr,((kb_status.host_keyboard_status>>2)&0x01)^kb_status.led_level_scr);
				}
				if(kb_status.led_gpio_cap){
					gpio_write(kb_status.led_gpio_cap,((kb_status.host_keyboard_status>>1)&0x01)^kb_status.led_level_scr);
				}
				if(kb_status.led_gpio_num){
					gpio_write(kb_status.led_gpio_num,(kb_status.host_keyboard_status&0x01)^kb_status.led_level_scr);
				}
				kb_status.pre_host_status = kb_status.host_keyboard_status;
			}

#if(!KEYBOARD_PIPE1_DATA_WITH_DID)
			pipe1_send_id_flg = 0;
#endif

			return 1;
		}
#if(!KEYBOARD_PIPE1_DATA_WITH_DID)
		else if(p_pkt->type == FRAME_AUTO_ACK_KB_ASK_ID){
			pipe1_send_id_flg = 1;//fix auto bug
			return 1;
		}
#endif
		////////// end of PIPE1 /////////////////////////////////////
	}

	return 0;
}

////////////////////////////////////////////////////////////////////////
//	keyboard/mouse data management
////////////////////////////////////////////////////////////////////////



u32 rx_rssi = 0;
u8 	rssi_last;
inline void cal_pkt_rssi(void)
{
#if(DBG_ADC_DATA)
	kb_status.tx_retry = 2;
#else
	if(kb_status.kb_pipe_rssi){
		if(!rx_rssi){
			rx_rssi = kb_status.kb_pipe_rssi;
		}
		else{
			if ( abs(rssi_last - kb_status.kb_pipe_rssi) < 8 ){
				rx_rssi = (rx_rssi*3 + kb_status.kb_pipe_rssi ) >> 2;
			}
		}
		rssi_last = kb_status.kb_pipe_rssi;
		pkt_km.rssi = rx_rssi;

		kb_status.kb_pipe_rssi = 0;  //clear


		//0x32: -60  0x28: -70    0x23: -75   0x1d: -80
		if(pkt_km.rssi < 0x1d){   // < -80
			kb_status.tx_retry = 7;
		}
		else if(pkt_km.rssi < 0x28){  //-70
			kb_status.tx_retry = 5;
		}
		else if(pkt_km.rssi < 0x32){  //-60
			kb_status.tx_retry = 3;
		}
		else{
			kb_status.tx_retry = 2;
		}

#if(LOW_TX_POWER_WHEN_SHORT_DISTANCE)
		if(pkt_km.rssi < 0x35){  //-57   power low
			if(kb_status.tx_power == RF_POWER_2dBm){
				rf_set_power_level_index (kb_status.cust_tx_power);
				kb_status.tx_power = kb_status.cust_tx_power;
			}
		}
		else if(kb_status.tx_power == kb_status.cust_tx_power){  //power high
			rf_set_power_level_index (RF_POWER_2dBm);
			kb_status.tx_power = RF_POWER_2dBm;
		}
#endif
	}
#endif
}


void kb_rf_proc( u32 key_scaned )
{
	static u32 kb_tx_retry_thresh = 0;

	if (kb_status.mode_link ) {
		if (clock_time_exceed (tick_last_tx, 1000000)) {//1s
			kb_status.host_keyboard_status = KB_NUMLOCK_STATUS_INVALID;
		}
        if ( key_scaned ){
			memcpy ((void *) &pkt_km.data[0], (void *) &kb_event, sizeof(kb_data_t));
			pkt_km.seq_no++;
			km_dat_sending = 1;
			kb_tx_retry_thresh = 0x400;
		}

#if(!KEYBOARD_PIPE1_DATA_WITH_DID)
    	//fix auto paring bug, if dongle ACK ask for  id,send it in on pipe1
        int allow_did_in_kb_data = 0;
		if(pipe1_send_id_flg){
			if(key_scaned){ //kb data with did in last 4 byte
				if(kb_event.cnt < 3){
	        		allow_did_in_kb_data = 1;
				}
			}
			else{  //no kb data, only did in last 4 byte; seq_no keep same, so dongle reject this invalid data
				allow_did_in_kb_data = 1;
			}

			if(allow_did_in_kb_data){
				*(u32 *) (&pkt_km.data[4]) = pkt_pairing.did;  //did in last 4 byte
				pkt_km.type = FRAME_TYPE_KB_SEND_ID;
				km_dat_sending = 1;
				kb_tx_retry_thresh = 0x400;
			}
			else{
				pkt_km.type = FRAME_TYPE_KEYBOARD;
			}
		}
		else{
			 pkt_km.type = FRAME_TYPE_KEYBOARD;
		}
#endif

		if (km_dat_sending) {
            if ( kb_tx_retry_thresh-- == 0 ){
				km_dat_sending = 0;
            }
		}
	}
	else{
		pkt_pairing.seq_no++;
	}

	kb_status.rf_sending = ((km_dat_sending || !kb_status.mode_link) && (kb_status.kb_mode != STATE_WAIT_DEEP));
	if(kb_status.rf_sending){
		if(HOST_LINK_LOST && kb_status.mode_link){
			kb_status.tx_retry = 5;
		}
		if(device_send_packet ( kb_rf_pkt, 550, kb_status.tx_retry, 0) ){
			km_dat_sending = 0;
			tick_last_tx = clock_time();
			kb_status.no_ack = 0;
		}
		else{
			kb_status.no_ack ++;
#if(!DBG_ADC_DATA)
			pkt_km.per ++;
#endif
		}

		cal_pkt_rssi();
	}
}


#endif
