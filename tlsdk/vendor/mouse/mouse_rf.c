/*
 * mouse_rf.c
 *
 *  Created on: Feb 12, 2014
 *      Author: xuzhen
 */

#include "../../proj/tl_common.h"
#include "../../proj/mcu/watchdog_i.h"
#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"
#include "../common/rf_frame.h"
#include "device_info.h"
#include "mouse.h"
#include "mouse_rf.h"
#include "trace.h"

rf_packet_pairing_t	pkt_pairing = {
		sizeof (rf_packet_pairing_t) - 4,	// dma_len
#if RF_FAST_MODE_1M
		RF_PROTO_BYTE,
		sizeof (rf_packet_pairing_t) - 6,	// rf_len
#else
		sizeof (rf_packet_pairing_t) - 5,	// rf_len
		RF_PROTO_BYTE,						// proto
#endif
		PKT_FLOW_DIR,	                    // flow
		FRAME_TYPE_MOUSE,					// type

//		PIPE0_CODE,			// gid0

		0,					// rssi
		0,					// per
		0,					// seq_no
		0,					// reserved
		0x0c0c0c0c			// device id
};


rf_packet_mouse_t	pkt_km = {
		sizeof (rf_packet_mouse_t) - 4,	// dma_len

		sizeof (rf_packet_mouse_t) - 5,	// rf_len,	28-5 = 0x17
		RF_PROTO_BYTE,					// proto,	0x51
		PKT_FLOW_DIR,					// flow,	0x80
		FRAME_TYPE_MOUSE,				// type,	0x01

//		U32_MAX,			// gid0

		0,					// rssi
		0,					// per
		0,					// seq_no
		0,					// number of frame
};

#if MOUSE_RF_CUS	//cavy mouse tx power fixed
#define MOUSE_DATA_TX_RETRY 	(mouse_status->tx_retry)
#define MOUSE_DATA_TX_POWER 	(mouse_status->tx_power)
#else
#define MOUSE_DATA_TX_RETRY 	5
#define MOUSE_DATA_TX_POWER 	RF_POWER_8dBm
#endif

void mouse_rf_init(mouse_status_t *mouse_status)
{
	// pkt_mouse buffer init
	ll_device_init ();    
	rf_receiving_pipe_enble( 0x3f);	// channel mask

    mouse_status->pkt_addr = &pkt_km;
#if MOUSE_RF_CUS
    u32 tx_power_dft = RF_POWER_8dBm;
    if ( mouse_status->high_end == MS_HIGHEND_250_REPORTRATE ){
        mouse_status->tx_retry = 2;
    }
    else if ( mouse_status->high_end == MS_LOW_END ){
        mouse_status->tx_retry = 4;
        tx_power_dft = RF_POWER_2dBm;
    }
    else{
        mouse_status->tx_retry = 5;
    }
    mouse_status->tx_power = (mouse_cust_tx_power == U8_MAX) ? tx_power_dft : mouse_cust_tx_power;
#endif
}

#define			KM_DATA_NUM				8

mouse_data_t     km_data[KM_DATA_NUM];

u8	km_wptr = 0;
u8	km_rptr = 0;
u8 km_dat_sending = 0;

#if(!MOUSE_PIPE1_DATA_WITH_DID)
u8 pipe1_send_id_flg = 0;
#endif

static inline void km_data_add (u32 * s, int len)
{
	//memcpy4 ((u32 *)&km_data[km_wptr&(KM_DATA_NUM-1)], s, len);
	*(u32 *)&km_data[km_wptr&(KM_DATA_NUM-1)] = *s;
	km_wptr = (km_wptr + 1) & (KM_DATA_NUM*2-1);
	if ( ((km_wptr - km_rptr) & (KM_DATA_NUM*2-1)) > KM_DATA_NUM ) {	//overwrite older data
		km_rptr = (km_wptr - KM_DATA_NUM) & (KM_DATA_NUM*2-1);
	}
}

static inline int km_data_get ()
{
	static u16 km_dat_tx_cnt = 0;
	if (km_dat_sending && km_dat_tx_cnt ) {
        if ( (km_rptr != km_wptr) && (km_dat_tx_cnt > 3) )
            ;//km_dat_tx_cnt = 3;
		km_dat_tx_cnt --;
		return 1;
	}
	pkt_km.pno = 0;
	for (int i=0; km_rptr != km_wptr && i<4; i++) {
	//while (km_rptr != km_wptr) {
#if 0	
		memcpy4 ((u32 *) &pkt_km.data[sizeof(mouse_data_t) * pkt_km.pno++],
		(u32 *) &km_data[km_rptr & (KM_DATA_NUM-1)],
				sizeof(mouse_data_t));
#else
        *(u32 *) &pkt_km.data[sizeof(mouse_data_t) * pkt_km.pno++] = *(u32 *) &km_data[km_rptr & (KM_DATA_NUM-1)];
#endif
        km_rptr = (km_rptr + 1) & (KM_DATA_NUM*2-1);

	}

#if(!MOUSE_PIPE1_DATA_WITH_DID)
	//fix auto paring bug, if dongle ACK ask for  id,send it in on pipe1
	if(pipe1_send_id_flg && pkt_km.pno < MOUSE_FRAME_DATA_NUM){
		 *(u32 *) &pkt_km.data[sizeof(mouse_data_t) * (MOUSE_FRAME_DATA_NUM-1)] = pkt_pairing.did;
		 pkt_km.type = FRAME_TYPE_MOUSE_SEND_ID;
	}
	else{
		 pkt_km.type = FRAME_TYPE_MOUSE;
	}
#endif

	if (pkt_km.pno) {		//new frame
		pkt_km.seq_no++;
		km_dat_sending = 1;		
		km_dat_tx_cnt = RF_SYNC_PKT_TX_NUM;
	}
	return pkt_km.pno;
}

u8* mouse_rf_pkt = (u8*)&pkt_pairing;
//Prepare the TX data, include package type, RSSI, sequence number, mouse data
static inline void mouse_rf_prepare(mouse_status_t *mouse_status)
{
	mouse_data_t* ms_dat_t = mouse_status->data;
	static u8 ms_dat_release = 1;
	mouse_status->rf_mode = RF_MODE_IDLE;

	if( *((u32 *)(mouse_status->data)) || ms_dat_release ){
		km_data_add ((u32 *)ms_dat_t, sizeof (mouse_data_t));	// add data to buffer
		if( *((u32 *)(mouse_status->data)) ){
		    ms_dat_release = MOUSE_BUTTON_DEBOUNCE;
        }
        else{
		    ms_dat_release --;
        }
	}

	if( (mouse_rf_pkt != (u8*)&pkt_pairing) && km_data_get() ){	// get data from buffer		
		mouse_status->rf_mode |= RF_MODE_DATA;
        static u8 tx_power;
        //set tx power
        if ( !CHIP_8366_A1 || MOUSE_RX_RSSI_LOW ){ 
            rf_set_power_level_index (mouse_status->tx_power);
            tx_power = MOUSE_DATA_TX_POWER;
        }
        else if ( tx_power != RF_POWER_2dBm ){
            rf_set_power_level_index ( RF_POWER_2dBm );   
            tx_power = RF_POWER_2dBm;
        }
	}
#if 1
    else{
        if ( mouse_status->mouse_mode == STATE_PAIRING ){
            rf_set_power_level_index ( mouse_cust_tx_power_paring);
        }
        else if ( (mouse_status->mouse_mode <= STATE_PAIRING) && (mouse_status->loop_cnt >= PARING_POWER_ON_CNT) ){
            rf_set_power_level_index (mouse_cust_tx_power_sync);
        }
    }
#endif
    extern u8 dbg_sensor_cpi;
    pkt_km.flow = (mouse_status->loop_cnt & 1) ? (cpu_working_tick >> 8) : ((mouse_status->mouse_sensor & 0x0f) << 4) | dbg_sensor_cpi;
}

//extern u32 cpu_working_tick;
//Send package at select channel, and check ACK result
u8 device_pkt_ack;
u8 mouse_rf_send;



_attribute_ram_code_ void mouse_rf_process(mouse_status_t *mouse_status)
{
	mouse_rf_prepare(mouse_status);
    static s8 tx_skip_ctrl = 2;
    u8 tx_rssi_low = MOUSE_RX_RSSI_LOW;
#if MOUSE_SW_CUS
    if ( mouse_status->high_end == MS_HIGHEND_250_REPORTRATE ){
        tx_skip_ctrl = !tx_skip_ctrl;
        tx_rssi_low = 0;
    }
    else if (mouse_status->high_end == MS_HIGHEND_ULTRA_LOW_POWER){
        tx_skip_ctrl = ( (tx_skip_ctrl > 0) ? (tx_skip_ctrl-1): 2 );
    }
    else{
        tx_skip_ctrl = 1;      //default nevel skip tx to get outstanding performence
    }
#else
    tx_skip_ctrl = 1;
#endif
    u8 tx_not_skip = tx_skip_ctrl;
#if MOUSE_SW_CUS
    if (mouse_status->high_end == MS_HIGHEND_ULTRA_LOW_POWER){
        tx_not_skip = (tx_skip_ctrl == 2) || mouse_status->no_ack;
    }
#endif
    mouse_rf_send = ( ( tx_rssi_low || tx_not_skip) && DEVICE_PKT_ASK ) || ( mouse_status->mouse_mode <= STATE_PAIRING );
	if ( mouse_rf_send ) {



		if( device_send_packet(mouse_rf_pkt, 550, MOUSE_DATA_TX_RETRY, 0) )


		{	//Send package re-try and Check ACK result
			km_dat_sending = 0;						//km_data_send_ok
			mouse_status->no_ack = 0;
			device_pkt_ack ++;
		}
        else if (mouse_status->no_ack < RF_SYNC_PKT_TH_NUM){
    		mouse_status->no_ack++;
    		device_pkt_ack = 0;
    	}
        else{
            km_dat_sending = 0;	    //get data buff per cycle when no-link: mouse_status->no_ack  == RF_SYNC_PKT_TH_NUM
        }
	}
    log_event( TR_T_MS_RX );
}

u8	mode_link = 0;
_attribute_ram_code_ int  rf_rx_process(u8 * p)
{
	rf_packet_ack_pairing_t *p_pkt = (rf_packet_ack_pairing_t *) (p + 8);
	static u8 rf_rx_mouse;
	if (p_pkt->proto == RF_PROTO_BYTE) {
		pkt_pairing.rssi = p[4];
		pkt_km.rssi = p[4];        
        //pkt_km.per ^= 0x80;
		///////////////  Paring/Link ACK //////////////////////////
		if (p_pkt->type == FRAME_TYPE_ACK && (p_pkt->did == pkt_pairing.did) ) {	//paring/link request
            rf_set_access_code1 (p_pkt->gid1);          //access_code1 = p_pkt->gid1;
			mode_link = 1;			
			return 1;
		}
		////////// end of PIPE1 /////////////////////////////////////
		///////////// PIPE1: ACK /////////////////////////////
		else if (p_pkt->type == FRAME_TYPE_ACK_MOUSE) {
			rf_rx_mouse++;
#if(!MOUSE_PIPE1_DATA_WITH_DID)
			pipe1_send_id_flg = 0;
#endif
			return 1;
		}
#if(!MOUSE_PIPE1_DATA_WITH_DID)
		else if(p_pkt->type == FRAME_AUTO_ACK_MOUSE_ASK_ID){ //fix auto bug
			pipe1_send_id_flg = 1;
			return 1;
		}
#endif

		////////// end of PIPE1 /////////////////////////////////////
	}
	return 0;
}


