/*
 * dongle_rf.h
 *
 *  Created on: Feb 13, 2014
 *      Author: xuzhen
 */

#ifndef _RF_LINK_LAYER_H_
#define _RF_LINK_LAYER_H_

typedef void (*callback_rx_func) (u8 *);
typedef void (*task_when_rf_func) (void);

#if(CLOCK_SYS_CLOCK_HZ == 32000000 || CLOCK_SYS_CLOCK_HZ == 24000000)
	#define			SHIFT_US		5
#elif(CLOCK_SYS_CLOCK_HZ == 16000000 || CLOCK_SYS_CLOCK_HZ == 12000000)
	#define			SHIFT_US		4
#elif(CLOCK_SYS_CLOCK_HZ == 8000000)
	#define			SHIFT_US		3
#else
	#error clock not set properly
#endif

void	proc_debug (void);
u8	get_next_channel_with_mask(u32 mask, u8 chn);
u8	update_channel_mask(u32 mask, u8 chn, u8* per);

void ll_host_init (u8 * pkt);
void ll_device_init (void);

void irq_host_timer1 (void);
void irq_host_rx(void);
void irq_host_tx(void);

void irq_device_rx(void);
void irq_device_tx(void);
int	device_send_packet (u8 * p, u32 timeout, int retry, int pairing_link);
void ll_add_clock_time (u32 ms);
void ll_channel_alternate_mode (void);

extern int device_sync;
extern task_when_rf_func p_task_when_rf;

#define DO_TASK_WHEN_RF_EN      1




#define  FRQ_HOPPING_MODE_SWITCH_ENABLE    	1

#define  FRQ_HOPPING_MODE_NORMAL   		0  //normal hopping
#define  FRQ_HOPPING_MODE_SLOW     		1  //slow hopping for kb only

#define  PKT_RCVD_FLG_MOUSE_LOCAL     	0x01
#define  PKT_RCVD_FLG_MOUSE_GLOBAL    	0x10
#define  PKT_RCVD_FLG_MOUSE				(PKT_RCVD_FLG_MOUSE_LOCAL | PKT_RCVD_FLG_MOUSE_GLOBAL)
#define  PKT_RCVD_FLG_KB_LOCAL       	0x02
#define  PKT_RCVD_FLG_KB_GLOBAL      	0x20
#define  PKT_RCVD_FLG_KB                (PKT_RCVD_FLG_KB_LOCAL | PKT_RCVD_FLG_KB_GLOBAL)

#define  PKT_RCVD_FLG_MOUSE_KB_LOCAL	(PKT_RCVD_FLG_MOUSE_LOCAL | PKT_RCVD_FLG_KB_LOCAL)

#define  MOUSE_PKT_NEVER_COME   (!(frq_hopping_data.device_pktRcv_flg & PKT_RCVD_FLG_MOUSE_GLOBAL))
#define  MOUSE_PKT_COME           (frq_hopping_data.device_pktRcv_flg & PKT_RCVD_FLG_MOUSE_LOCAL)
#define  KB_PKT_COME           	  (frq_hopping_data.device_pktRcv_flg & PKT_RCVD_FLG_KB_LOCAL)
#define  MOUSE_PKT_5S_NO_COME	  (clock_time_exceed(frq_hopping_data.mouse_pktRcv_sysTick,5000000))

typedef struct{
	u8 frq_hp_mode;
	u8 frq_hp_chn_pkt_rcvd_max;
	u8 frq_hp_hit_diff_num;
	u8 device_pktRcv_flg;

	u32 fre_hp_always_time_us;
	u32 mouse_pktRcv_sysTick;
}frqHopping_data_t;

extern frqHopping_data_t  frq_hopping_data;


static inline void frq_hopping_mode_switch(void)
{
#if(FRQ_HOPPING_MODE_SWITCH_ENABLE)
	if(frq_hopping_data.frq_hp_mode == FRQ_HOPPING_MODE_NORMAL){  //normal hopping
		if(KB_PKT_COME){
			if(MOUSE_PKT_NEVER_COME || MOUSE_PKT_5S_NO_COME){
				frq_hopping_data.frq_hp_mode = FRQ_HOPPING_MODE_SLOW;
				frq_hopping_data.frq_hp_chn_pkt_rcvd_max = 10;
				frq_hopping_data.frq_hp_hit_diff_num = 8;
				frq_hopping_data.fre_hp_always_time_us = 3000000;
			}
		}
	}
	else{  //slow hopping
		if(MOUSE_PKT_COME){
			frq_hopping_data.frq_hp_mode = FRQ_HOPPING_MODE_NORMAL;
			frq_hopping_data.frq_hp_chn_pkt_rcvd_max = 8;
			frq_hopping_data.frq_hp_hit_diff_num = 2;
			frq_hopping_data.fre_hp_always_time_us = 120000;
		}
	}
	frq_hopping_data.device_pktRcv_flg &= (~PKT_RCVD_FLG_MOUSE_KB_LOCAL);
#endif
}


#endif /* _RF_LINK_LAYER_H_ */
