/*
 * mouse_rf.h
 *
 *  Created on: Feb 12, 2014
 *      Author: xuzhen
 */
#ifndef MOUSE_RF_H_
#define MOUSE_RF_H_

#ifndef	MOUSE_RF_CUS
#define MOUSE_RF_CUS			1
#endif

#define RF_MODE_IDLE		0
#define RF_MODE_SYNC		1
#define RF_MODE_PAIRING		2
#define RF_MODE_DATA		4
//#define RF_MODE_LOSE_SYNC	0x10

#define RF_SYNC_PKT_TH_NUM  0xff
#define RF_SYNC_PKT_TX_NUM  ( (RF_SYNC_PKT_TH_NUM << 1) + RF_SYNC_PKT_TH_NUM )

extern u8* mouse_rf_pkt;
extern rf_packet_mouse_t    pkt_km;
extern rf_packet_pairing_t	pkt_pairing;
extern u32 cpu_working_tick;

#define device_never_linked (rf_get_access_code1() == U32_MAX)

#define HOST_NO_LINK        (mouse_status->no_ack >= RF_SYNC_PKT_TH_NUM)   //host has gone or not
#define	DEVICE_PKT_ASK	    (mouse_status->rf_mode != RF_MODE_IDLE)

void mouse_rf_channel(mouse_status_t *mouse_status);

void mouse_rf_init(mouse_status_t *mouse_status);

void mouse_rf_process(mouse_status_t *mouse_status);

#include "../../proj_lib/rf_drv.h"
#include "mouse_custom.h"

#define MOUSE_RX_RSSI_LOW       (pkt_km.rssi < 0x35)
#define	FILTER_RX_RSSI( bak, input )	( ( (bak) * 3 + (input) ) >> 2 )
static inline void mouse_rf_statistic_analysis( void ){
    static u32 rx_rssi;
    static u8  rssi_last;
    if ( abs(pkt_km.rssi - rssi_last) < 8 )
        rx_rssi = FILTER_RX_RSSI( rx_rssi, pkt_km.rssi );
    rssi_last = pkt_km.rssi;
    pkt_km.rssi = rx_rssi;
}

extern u8 mouse_rf_send;
#define PARING_POWER_ON_CNT     64
static inline void mouse_rf_post_proc(mouse_status_t *mouse_status){
	if( mouse_status->mouse_mode <= STATE_PAIRING ){
		rf_set_tx_pipe (PIPE_PARING);
        
        if ( mouse_status->mouse_mode == STATE_PAIRING ){
            pkt_pairing.flow = PKT_FLOW_PARING;
        }
        else if ( mouse_status->loop_cnt < PARING_POWER_ON_CNT ){
		    pkt_pairing.flow = PKT_FLOW_TOKEN | PKT_FLOW_PARING;	
        }
        else{
            pkt_pairing.flow = PKT_FLOW_TOKEN;           
        }
        mouse_rf_pkt = (u8*)&pkt_pairing;
		mouse_status->rf_mode |= RF_MODE_SYNC;
	}
    
    if ( mouse_rf_send ){
        mouse_rf_statistic_analysis();
        if( mouse_status->no_ack ){           
            pkt_km.per++;
            pkt_km.rssi = 0;
        }
    }    
}

static inline void  mouse_sync_status_update( mouse_status_t * mouse_status ){
    mouse_status->mouse_mode = (rf_get_access_code1() != U32_MAX) ? STATE_NORMAL : STATE_SYNCING;
    if ( (mouse_rf_pkt != (u8*)&pkt_km) && (mouse_status->mouse_mode == STATE_NORMAL) ){
        rf_set_tx_pipe (PIPE_MOUSE);
        rf_set_power_level_index (mouse_status->tx_power);
        mouse_rf_pkt = (u8*)&pkt_km;
        device_info_save (mouse_status, 0);
    }
}

#endif /* MOUSE_RF_H_ */
