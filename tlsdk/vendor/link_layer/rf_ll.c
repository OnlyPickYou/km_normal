/*
 * rf_ll.c: link layer
 *
 *  Created on: Mar 5, 2014
 *      Author: MZ
 */
#include "../../proj/tl_common.h"
#include "../../proj/mcu/watchdog_i.h"
#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"
#include "../common/rf_frame.h"
#include "../link_layer/rf_ll.h"
#include "../dongle/trace.h"

#if 1
u8		chn_mask_fix = 0;
#else
u8		chn_mask_fix = 0x80;
#endif

u8		chn_mask = 0x80;

#define	CHANNEL_HOPPING_ALWAYS				1

#ifndef	LL_HOST_RX_MULTI_RECEIVING_EN
	#ifdef __PROJECT_DONGLE_8366__
		#define	LL_HOST_RX_MULTI_RECEIVING_EN		0
	#else
		#define	LL_HOST_RX_MULTI_RECEIVING_EN		0
	#endif
#endif

#if (__PROJECT_DONGLE_ENC_8366__)
#ifndef			CHANNEL_SLOT_TIME
#define			CHANNEL_SLOT_TIME			11000
#endif
#else
#ifndef			CHANNEL_SLOT_TIME
#define			CHANNEL_SLOT_TIME			8000
#endif
#endif


#ifndef		PKT_BUFF_SIZE
#if	(MCU_CORE_TYPE && MCU_CORE_TYPE == MCU_CORE_8266)
#define		PKT_BUFF_SIZE		256
#else
#define		PKT_BUFF_SIZE		80
#endif
#endif

#define		FH_CHANNEL_PKT_RCVD_MAX			8
#define		LL_CHANNEL_SYNC_TH				2
#define		LL_CHANNEL_SEARCH_TH			60
#define		LL_CHANNEL_SEARCH_FLAG			BIT(16)


#if((MOUSE_FOR_MINGJIAN_DONGLE || DONGLE_FOR_MINGJIAN_MOUSE) \
		&& (MOUSE_USE_4_CHN_FOR_MINGJIAN_DONGLE || DONGLE_USE_4_CHN_FOR_MINGJIAN_MOUSE))
#define		LL_NEXT_CHANNEL(c)				((c + 4) & 12)
#else
#define		LL_NEXT_CHANNEL(c)				((c + 6) & 14)
#endif


frqHopping_data_t  frq_hopping_data = {
	FRQ_HOPPING_MODE_NORMAL,
	8,
	2,
	0,
	120000,
	0
};

unsigned char  		rf_rx_buff[PKT_BUFF_SIZE*2] __attribute__((aligned(4)));
int		rf_rx_wptr;
u8		host_channel = 0;

u8		ll_rssi;
u16		ll_chn_tick;
u32		ll_chn_rx_tick;
u32		ll_chn_mask = LL_CHANNEL_SEARCH_FLAG;

u8		ll_chn_sel;
u8		ll_chn_pkt[16] = {0};
u32		ll_clock_time;
int		device_packet_received;

#if(MOUSE_FOR_MINGJIAN_DONGLE)
u8 		mouse_is_pairing;
#endif

/////////////////////////////////////////////////////////////////////
//	link management functions
/////////////////////////////////////////////////////////////////////
#define		reg_debug_cmd		REG_ADDR16(0x8008)
void	proc_debug () {
	u16	udat = reg_debug_cmd;
	u8	cmd = udat >> 8;
	u8  adr = udat;
	u8	dat;
	if (cmd == 0xff) {			//read command
		dat = analog_read (adr);
		reg_debug_cmd = dat | 0x5500;
	}
	else if (cmd <= 0x20 || (cmd>=0x80 && cmd <=0xc0)) {	//write command
		analog_write (cmd, adr);
		dat = analog_read (cmd);
		reg_debug_cmd = dat | 0x6600;
	}
	else if (cmd == 0xfe) {	//set channel mask
		chn_mask_fix = adr;
		reg_debug_cmd = adr | 0x6700;
	}
	else if (cmd == 0xfd) {	//set tx power
		rf_set_power_level_index (adr & 15);
		reg_debug_cmd = adr | 0x6800;
	}

}

u8	get_next_channel_with_mask(u32 mask, u8 chn)
{
	int chn_high = (mask >> 4) & 0x0f;

	if (mask & LL_CHANNEL_SEARCH_FLAG) {
		return LL_NEXT_CHANNEL (chn);
	}
	else if (chn_high != chn) {
		ll_chn_sel = 1;
		return chn_high;
	}
	else {
		ll_chn_sel = 0;
		return mask & 0x0f;
	}
}

u32	tick_fh = 0;
u32	fh_num = 0;
u8	update_channel_mask(u32 mask, u8 chn, u8* chn_pkt)
{
	static int ll_chn_sel_chg, ll_chn_hold;

#if CHANNEL_HOPPING_ALWAYS
	if (device_packet_received == 1) {
		tick_fh = clock_time ();
	}
	else if (clock_time_exceed(tick_fh, frq_hopping_data.fre_hp_always_time_us))  {
		chn_pkt[ll_chn_sel_chg] = 0;
		chn_pkt[!ll_chn_sel_chg] = frq_hopping_data.frq_hp_chn_pkt_rcvd_max;
	}
#endif

	if (ll_chn_hold) {
		ll_chn_hold--;
		chn_pkt[0] = chn_pkt[1] = 0;
	}
	int diff = chn_pkt[ll_chn_sel] - chn_pkt[!ll_chn_sel];
	int hit_th = diff > frq_hopping_data.frq_hp_hit_diff_num;
	if (chn_pkt[ll_chn_sel] >= frq_hopping_data.frq_hp_chn_pkt_rcvd_max || hit_th) {
		int dual_chn[2];
		dual_chn[0] = mask & 0x0f;
		dual_chn[1] = mask >> 4;
		if (hit_th) { //change channel
			ll_chn_hold = 4;
			chn = dual_chn[!ll_chn_sel];
			for (int i=0; i<8; i++) {
				chn = LL_NEXT_CHANNEL (chn);
				if ((ll_chn_sel && chn != dual_chn[1])) {
					mask = (mask & 0xf0) | chn;
					break;
				}
				else if (!ll_chn_sel && chn != dual_chn[0]) {
					mask = (mask & 0x0f) | (chn << 4);
					break;
				}
			}
			tick_fh = clock_time ();
			fh_num++;
			ll_chn_sel_chg = !ll_chn_sel;		//remember latest channel change
		}
		chn_pkt[0] = chn_pkt[1] = 0;
	}

	return mask;
}

/////////////////////////////////////////////////////////////////////
//	timer1: beacon alignment
//	rx interrupt: buffer management->preprocess->postprocess
//	tx interrupt: post process
/////////////////////////////////////////////////////////////////////
u8 *	p_debug_pkt = 0;

void ll_rx_init (u8 * pkt)
{
	p_debug_pkt = pkt;
	reg_dma_rf_rx_addr = (u16)(u32) (rf_rx_buff);
	reg_dma2_ctrl = FLD_DMA_WR_MEM | (PKT_BUFF_SIZE>>4);   // rf rx buffer enable & size
	reg_dma_chn_irq_msk = 0;

	reg_irq_mask |= FLD_IRQ_ZB_RT_EN;    //enable RF & timer1 interrupt
	reg_rf_irq_mask = FLD_RF_IRQ_RX | FLD_RF_IRQ_TX;
	// timer1 interrupt enable
	//reg_tmr1_tick = 0;
	//reg_tmr1_capt = CLOCK_SYS_CLOCK_1US * CHANNEL_SLOT_TIME;
	//reg_tmr_ctrl |= FLD_TMR1_EN;
}

#define		reg8_dbg_crc_ignore		REG_ADDR8(5)
void irq_ll_rx(void)
{
	static u32 irq_host_rx_no = 0;
	//log_event (TR_T_rf_irq_rx);

	u8 * raw_pkt = (u8 *) (rf_rx_buff + rf_rx_wptr * PKT_BUFF_SIZE);
	rf_rx_wptr = (rf_rx_wptr + 1) & 1;
	reg_dma_rf_rx_addr = (u16)(u32) (rf_rx_buff + rf_rx_wptr * PKT_BUFF_SIZE); //set next buffer
	reg_rf_irq_status = FLD_RF_IRQ_RX;

	//rf_packe_header_t *p = (rf_packet_header_t*)(((u8*)raw_pkt) + 8);
	if	(	raw_pkt[0] >= 15 &&
			RF_PACKET_LENGTH_OK(raw_pkt) &&
			(reg8_dbg_crc_ignore || RF_PACKET_CRC_OK(raw_pkt)) )	{

		//void (*p_post) (u8 *)  = NULL;
		callback_rx_func p_post = NULL;
		extern u8 *  rf_rx_response(u8 * p, void *);
		u8 * p_ack = rf_rx_response (raw_pkt, &p_post);
		if (p_ack) {
			//SetTxMode (host_channel, RF_CHN_TABLE);
			//RF_TX_PA_POWER_LEVEL (1);
			// assume running @ 32MHz, 32M/1024: 32 us resolution
			//sleep_us (10);
			//TxPkt (p_ack);
		}
		if (p_post) {
			(*p_post) (raw_pkt);
		}
	}
	//raw_pkt[0] = 1;

	irq_host_rx_no++;
}

void irq_ll_tx(void)
{
	static u32 tick_last_tx;
	tick_last_tx = clock_time ();
	//SetRxMode (host_channel, RF_CHN_TABLE);
	reg_rf_irq_status = FLD_RF_IRQ_TX;
}

///////////////////////////////////////////////////////////////////////////////
static u32 tick_last_tx;
static u32 tick_timer1_irq;

///////////////////////////////////////////////////////////////////////////////
//	host
///////////////////////////////////////////////////////////////////////////////
#if (     __PROJECT_DONGLE__  || __PROJECT_DONGLE_REMOTE__ || __PROJECT_OTA_MASTER__  ||   \
		  __PROJECT_DONGLE_RC__ || __PROJECT_VACUUM_RECEIVER__ || __PROJECT_DONGLE_8510__  || \
		__PROJECT_DONGLE_ENC_8366__ || __PROJECT_XJGD_DONGLE__  )
void ll_host_init (u8 * pkt)
{
	rf_multi_receiving_init (BIT(0) | BIT(1) | BIT(2));
	p_debug_pkt = pkt;
	reg_dma_rf_tx_addr = (u16)(u32) pkt;
	reg_dma_rf_rx_addr = (u16)(u32) (rf_rx_buff);
	reg_dma2_ctrl = FLD_DMA_WR_MEM | (PKT_BUFF_SIZE>>4);   // rf rx buffer enable & size
	reg_dma_chn_irq_msk = 0;

	reg_rf_irq_mask = FLD_RF_IRQ_RX | FLD_RF_IRQ_TX;

	// timer1 interrupt enable
	reg_tmr1_tick = 0;
	reg_tmr1_capt = CLOCK_SYS_CLOCK_1US * CHANNEL_SLOT_TIME;
	reg_tmr_ctrl |= FLD_TMR1_EN;

	//reg_irq_mask = FLD_IRQ_TMR1_EN;    //enable RF & timer1 interrupt
	//reg_irq_mask =  FLD_IRQ_ZB_RT_EN;
	reg_irq_mask |= FLD_IRQ_TMR1_EN | FLD_IRQ_ZB_RT_EN;    //enable RF & timer1 interrupt

#if (!__PROJECT_DONGLE_ENC_8366__ && !__PROJECT_DONGLE_8510__)
	extern u8   channel_mask_custom;
	if(channel_mask_custom != U8_MAX){
		chn_mask = channel_mask_custom;
	}
#endif
}


#if (__PROJECT_DONGLE_ENC_8366__)

_attribute_ram_code_ void irq_host_timer1 (void)
{
	tick_timer1_irq++;
	if (device_packet_received == 1) {
		ll_chn_pkt[ll_chn_sel]++;
	}

	frq_hopping_mode_switch();

	host_channel = get_next_channel_with_mask (chn_mask, host_channel);

	chn_mask = update_channel_mask(chn_mask, host_channel, ll_chn_pkt);

	device_packet_received = 0;

	rf_set_channel (host_channel, RF_CHN_TABLE);
	rf_set_rxmode ();

}


_attribute_ram_code_ void irq_host_rx(void)
{
	//log_event (TR_T_rf_irq_rx);

	u8 * raw_pkt = (u8 *) (rf_rx_buff + rf_rx_wptr * PKT_BUFF_SIZE);
	rf_rx_wptr = (rf_rx_wptr + 1) & 1;
	reg_dma_rf_rx_addr = (u16)(u32) (rf_rx_buff + rf_rx_wptr * PKT_BUFF_SIZE); //set next buffer

	reg_rf_irq_status = FLD_RF_IRQ_RX;

	if	(	raw_pkt[0] >= 15 &&
			RF_PACKET_LENGTH_OK(raw_pkt) &&
			RF_PACKET_CRC_OK(raw_pkt) )	{

		rf_set_tx_rx_off ();
		u32 t = *((u32 *) (raw_pkt + 8));
#if 0
		#ifdef __PROJECT_DONGLE_8366__
				t += ((raw_pkt[12] + 8) * 4 + 0) * CLOCK_SYS_CLOCK_1US;
		#else
				t += ((raw_pkt[12] + 8) * 4 + 20) * CLOCK_SYS_CLOCK_1US;
		#endif
#endif
		t += ((raw_pkt[12] + 8) * 4 + 0) * CLOCK_SYS_CLOCK_1US;

		u32 diff = t - clock_time ();
		if (diff > (300 * CLOCK_SYS_CLOCK_1US))
		{
			t = clock_time () + 5 * CLOCK_SYS_CLOCK_1US;
		}
		rf_start_stx (p_debug_pkt, t);

		callback_rx_func p_post = NULL;
		extern u8 *  rf_rx_response(u8 * p, void *);
		u8 * p_ack = rf_rx_response (raw_pkt, &p_post);
		if (p_ack) {

			//rf_set_power_level_index (RF_POWER_LEVEL_MAX);
			rf_set_ack_packet (p_ack);


			// assume running @ 32MHz, 32M/1024: 32 us resolution
			((rf_packet_ack_pairing_t *)p_ack)->rssi = raw_pkt[4];
			((rf_packet_ack_pairing_t *)p_ack)->chn = chn_mask;
			((rf_packet_ack_pairing_t *)p_ack)->per =
					(ll_chn_pkt[0] & 0xf) | ((ll_chn_pkt[1] & 0xf) << 4) ;
			//((rf_packet_ack_mouse_t *)p_ack)->info = fh_num;
		#if(CLOCK_SYS_CLOCK_HZ == 32000000)
			((rf_packet_ack_pairing_t *)p_ack)->tick = ((reg_tmr1_tick >> 10) & 0xff) |
													((tick_timer1_irq & 0xff) << 8);
		#elif(CLOCK_SYS_CLOCK_HZ == 24000000)
//			((rf_packet_ack_pairing_t *)p_ack)->tick = ((reg_tmr1_tick * 170 >> 17) & 0xff) |
//													((tick_timer1_irq & 0xff) << 8);				//8ms
//			((rf_packet_ack_pairing_t *)p_ack)->tick = ((reg_tmr1_tick * 136 >> 17) & 0xff) |
//													((tick_timer1_irq & 0xff) << 8);				//10ms
			((rf_packet_ack_pairing_t *)p_ack)->tick = ((reg_tmr1_tick * 127 >> 17) & 0xff) |
													((tick_timer1_irq & 0xff) << 8);				//11ms
		#else				// 16 MHz
			((rf_packet_ack_pairing_t *)p_ack)->tick = ((reg_tmr1_tick >> 9) & 0xff) |
										((tick_timer1_irq & 0xff) << 8);
		#endif

		}
		else {		//cancel TX
			rf_stop_trx ();
		}

		if (p_post) {
			(*p_post) (raw_pkt);
		}
		raw_pkt[0] = 1;
		device_packet_received++;
	}
}


//_attribute_ram_code_
void irq_host_tx(void)
{
	tick_last_tx = clock_time ();
	rf_set_channel (host_channel, RF_CHN_TABLE);
	rf_set_rxmode ();
	reg_rf_irq_status = FLD_RF_IRQ_TX;
}


#else  //else of __PROJECT_DONGLE_ENC_8366__
_attribute_ram_code_ void irq_host_timer1 (void)
{
	//log_event (TR_T_irq_timer1);
	tick_timer1_irq++;
	if (device_packet_received == 1) {
		ll_chn_pkt[ll_chn_sel]++;
	}
#if(!__PROJECT_DONGLE_8510__)
	extern int     dongle_support_keyboard_enable;
	if(dongle_support_keyboard_enable){
		frq_hopping_mode_switch();
	}
#endif
	host_channel = get_next_channel_with_mask (chn_mask, host_channel);

	if (chn_mask_fix) {
		chn_mask = chn_mask_fix;
	}
	else {
#if(!__PROJECT_OTA_MASTER__)
		chn_mask = update_channel_mask(chn_mask, host_channel, ll_chn_pkt);
#endif
	}
	device_packet_received = 0;
	//log_data (TR_24_rf_channel, host_channel);
	if (p_debug_pkt && clock_time_exceed (tick_last_tx, 500000)) {
		//rf_stop_trx ();
		rf_set_channel (15, RF_CHN_TABLE);
		//rf_set_tx_pipe (PIPE_MOUSE);
		//RF_TX_PA_POWER_LEVEL (0);
		((rf_packet_ack_pairing_t *)p_debug_pkt)->tick = reg_tmr1_tick;
		((rf_packet_ack_pairing_t *)p_debug_pkt)->chn = chn_mask;
		((rf_packet_ack_pairing_t *)p_debug_pkt)->info1++;
		//rf_multi_receiving_send_packet (p_debug_pkt);
		rf_set_power_level_index (RF_POWER_LEVEL_MIN);
		rf_send_single_packet (p_debug_pkt);
		//TxPkt (p_debug_pkt);
		tick_last_tx = clock_time ();
	}
	else {
#if LL_HOST_RX_MULTI_RECEIVING_EN
		rf_multi_receiving_start (host_channel, RF_CHN_TABLE);
#else
		rf_set_channel (host_channel, RF_CHN_TABLE);
		rf_set_rxmode ();
#endif
	}
}


_attribute_ram_code_ void irq_host_rx(void)
{
	static u32 irq_host_rx = 0;
	//log_event (TR_T_rf_irq_rx);

	u8 * raw_pkt = (u8 *) (rf_rx_buff + rf_rx_wptr * PKT_BUFF_SIZE);
	rf_rx_wptr = (rf_rx_wptr + 1) & 1;
	reg_dma_rf_rx_addr = (u16)(u32) (rf_rx_buff + rf_rx_wptr * PKT_BUFF_SIZE); //set next buffer

	reg_rf_irq_status = FLD_RF_IRQ_RX;

	//rf_packe_header_t *p = (rf_packet_header_t*)(((u8*)raw_pkt) + 8);
	if	(	raw_pkt[0] >= 15 &&
			RF_PACKET_LENGTH_OK(raw_pkt) &&
			RF_PACKET_CRC_OK(raw_pkt) )	{
#if (!LL_HOST_RX_MULTI_RECEIVING_EN)
		// scheduling TX, 150 us interval
		rf_set_tx_rx_off ();
		u32 t = *((u32 *) (raw_pkt + 8));
		#ifdef __PROJECT_DONGLE_8366__
				t += ((raw_pkt[12] + 8) * 4 + 0) * CLOCK_SYS_CLOCK_1US;
		#elif (__PROJECT_DONGLE_8510__)  //jiusong use only
				t += ((raw_pkt[12] + 8) * 4 + 35) * CLOCK_SYS_CLOCK_1US;
		#else
				t += ((raw_pkt[12] + 8) * 4 + 20) * CLOCK_SYS_CLOCK_1US;
		#endif
		u32 diff = t - clock_time ();
		if (diff > (300 * CLOCK_SYS_CLOCK_1US))
		{
			t = clock_time () + 5 * CLOCK_SYS_CLOCK_1US;
		}
		rf_start_stx (p_debug_pkt, t);
#endif
		callback_rx_func p_post = NULL;
		extern u8 *  rf_rx_response(u8 * p, void *);
		u8 * p_ack = rf_rx_response (raw_pkt, &p_post);
		if (p_ack) {

			rf_set_power_level_index (RF_POWER_LEVEL_MAX);
			rf_set_ack_packet (p_ack);
#if(__PROJECT_OTA_MASTER__)  //
			if( ((rf_packet_ack_pairing_t *)p_ack)->proto == RF_PROTO_BYTE ){
#endif

			// assume running @ 32MHz, 32M/1024: 32 us resolution
			((rf_packet_ack_pairing_t *)p_ack)->rssi = raw_pkt[4];
			((rf_packet_ack_pairing_t *)p_ack)->chn = chn_mask;
			((rf_packet_ack_pairing_t *)p_ack)->per =
					(ll_chn_pkt[0] & 0xf) | ((ll_chn_pkt[1] & 0xf) << 4) ;
			//((rf_packet_ack_mouse_t *)p_ack)->info = fh_num;
		#if(CLOCK_SYS_CLOCK_HZ == 32000000)
			((rf_packet_ack_pairing_t *)p_ack)->tick = ((reg_tmr1_tick >> 10) & 0xff) |
													((tick_timer1_irq & 0xff) << 8);
		#elif(CLOCK_SYS_CLOCK_HZ == 24000000)
			((rf_packet_ack_pairing_t *)p_ack)->tick = ((reg_tmr1_tick * 170 >> 17) & 0xff) |
													((tick_timer1_irq & 0xff) << 8);
		#else				// 16 MHz
			((rf_packet_ack_pairing_t *)p_ack)->tick = ((reg_tmr1_tick >> 9) & 0xff) |
										((tick_timer1_irq & 0xff) << 8);
		#endif
#if(__PROJECT_OTA_MASTER__)
			}
#endif
		}
		else {		//cancel TX
			rf_stop_trx ();
		}

		if (p_post) {
			(*p_post) (raw_pkt);
		}
		raw_pkt[0] = 1;
		device_packet_received++;
	}

	irq_host_rx++;
}


_attribute_ram_code_ void irq_host_tx(void)
{
	static irq_host_tx_no;
	irq_host_tx_no++;
	tick_last_tx = clock_time ();
#if LL_HOST_RX_MULTI_RECEIVING_EN
	rf_multi_receiving_start (host_channel, RF_CHN_TABLE);
#else
	rf_set_channel (host_channel, RF_CHN_TABLE);
	rf_set_rxmode ();
#endif

	reg_rf_irq_status = FLD_RF_IRQ_TX;
}

#endif  //end of __PROJECT_DONGLE_ENC_8366__





#else

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


u8		device_channel;
volatile int		device_ack_received = 0;
int		device_sync = 0;
_attribute_ram_code_ void irq_device_rx(void)
{
	static u32 irq_device_rx_no = 0;
	//log_event (TR_T_rf_irq_rx);

	u8 * raw_pkt = (u8 *) (rf_rx_buff + rf_rx_wptr * PKT_BUFF_SIZE);
	rf_rx_wptr = (rf_rx_wptr + 1) & 1;
	reg_dma_rf_rx_addr = (u16)(u32) (rf_rx_buff + rf_rx_wptr * PKT_BUFF_SIZE); //set next buffer

	reg_rf_irq_status = FLD_RF_IRQ_RX;

	if	(	raw_pkt[0] >= 15 &&
			RF_PACKET_LENGTH_OK(raw_pkt) &&
			RF_PACKET_CRC_OK(raw_pkt) )	{
		rf_packet_ack_pairing_t *p = (rf_packet_ack_pairing_t *)(raw_pkt + 8);
#if (CLOCK_SYS_TYPE == CLOCK_TYPE_OSC)
        rf_power_enable (0);
#endif
		extern int  rf_rx_process(u8 *);
		if (rf_rx_process (raw_pkt) && ll_chn_tick != p->tick) {
			ll_chn_tick = p->tick;			//sync time
#if(__PROJECT_KEYBOARD__ || __PROJECT_REMINGTON_KEYBOARD__ || __PROJECT_KEYBOARD_ENC_8368__ || __PROJECT_REMINGTON_KEYBOARD_TW__)
#else
			device_sync = 1;
#endif
			device_ack_received = 1;
			ll_chn_mask = p->chn;			//update channel
			ll_chn_rx_tick = clock_time ();
			ll_rssi = raw_pkt[4];
			irq_device_rx_no++;
		}
		rf_set_channel (device_channel, RF_CHN_TABLE);
		raw_pkt[0] = 1;
	}
}


_attribute_ram_code_ void irq_device_tx(void)
{
	tick_last_tx = clock_time ();
	//SetRxMode (device_channel, RF_CHN_TABLE);
	//device_ack_received = 0;

	reg_rf_irq_status = FLD_RF_IRQ_TX;
}

extern rf_packet_pairing_t	pkt_pairing;
task_when_rf_func p_task_when_rf = NULL;

#ifndef SEND_PKT_IN_RAM
#define SEND_PKT_IN_RAM 	1
#endif

#if (SEND_PKT_IN_RAM)
_attribute_ram_code_
#endif



int	device_send_packet (u8 * p, u32 timeout, int retry, int pairing_link)
{
	extern u32  cpu_wakup_last_tick;
#if(__PROJECT_MOUSE_ENC_8366__)  //only enc mouse use this value (by Tim)
	while ( !clock_time_exceed (cpu_wakup_last_tick, 1000) );    //delay to get stable pll clock	// 500 -> 1000
#else
    while ( !clock_time_exceed (cpu_wakup_last_tick, 500) );    //delay to get stable pll clock	// 500 -> 1000
#endif
    
	rf_power_enable (1);

	static	u32 ack_miss_no;

	device_ack_received = 0;
	int i;
	int step = 1;
	if (device_sync)
		step = retry + 1;

	u32 tick = clock_time ();
	for (i=0; i<=retry; i += step) {
		//device_channel = 0;
		rf_set_channel (device_channel, RF_CHN_TABLE);
		u32 t = clock_time ();
#if(__PROJECT_MOUSE_ENC_8366__)
		rf_send_packet (p, 350, step - 1);  //only enc mouse use this value (by Tim)
#else
		rf_send_packet (p, 300, step - 1);
#endif
		reg_rf_irq_status = 0xffff;
        if ( DO_TASK_WHEN_RF_EN && p_task_when_rf != NULL) {
           (*p_task_when_rf) ();
           p_task_when_rf = NULL;
        }
		while (	!device_ack_received &&
				!clock_time_exceed (t, timeout*step) &&
				//!(reg_rf_irq_status & (FLD_RF_IRX_RETRY_HIT | FLD_RF_IRX_RX_TIMEOUT | FLD_RF_IRX_CMD_DONE)) );
				!(reg_rf_irq_status & (FLD_RF_IRX_RETRY_HIT | FLD_RF_IRX_CMD_DONE)) );

		if (device_ack_received) {
			ack_miss_no = 0;
			break;
		}
		ack_miss_no ++;
#if(MOUSE_FOR_MINGJIAN_DONGLE)
		// When power on , mouse could search all channel
		static u8 debug_chn_no;
		if(mouse_is_pairing){
			device_sync = 0;
			ll_chn_mask = LL_CHANNEL_SEARCH_FLAG;
		}
		else{
			if (ack_miss_no >= LL_CHANNEL_SEARCH_TH) {
				device_sync = 0;
				//ll_chn_mask = 1; //0xff;
				ll_chn_mask = LL_CHANNEL_SEARCH_FLAG;
			}
			else if (ack_miss_no >= LL_CHANNEL_SYNC_TH) {
				device_sync = 0;
			}
		}

#else
		if (ack_miss_no >= LL_CHANNEL_SEARCH_TH) {
			device_sync = 0;
			//ll_chn_mask = 1; //0xff;
			ll_chn_mask = LL_CHANNEL_SEARCH_FLAG;
		}
		else if (ack_miss_no >= LL_CHANNEL_SYNC_TH) {
			device_sync = 0;
		}
#endif

		if (!device_sync) {	//alternate channel while device not in sync mode
			device_channel = get_next_channel_with_mask (ll_chn_mask, device_channel);
		}
	}
#if (CLOCK_SYS_TYPE == CLOCK_TYPE_OSC)
	rf_power_enable (0);
#endif
	//rf_set_channel (device_channel, RF_CHN_TABLE);
	static int adjust = 0;
	int ret = 0;
	if (i <= retry) {
		//sync timing
		u32 t0 = (ll_chn_tick & 0xff) * 32;			//time in host side
		u32 t1 = (ll_chn_rx_tick  - tick) >> SHIFT_US; 	//timeout * (i - 1);

		if (adjust) {
			adjust = 0;
		}
		else if (t0 < t1) {
			adjust = MCU_CORE_TYPE == MCU_CORE_8266 ? 3000 : 3;
		}
		else {
			t1 = t0 - t1;
			if (t1 > 3000) {			//clock slow, wait to next slot
				adjust = MCU_CORE_TYPE == MCU_CORE_8266 ? -1000 : -1;
			}
			else if (t1 < 1000){
				adjust = MCU_CORE_TYPE == MCU_CORE_8266 ? 1000 : 1;
			}
		}
		ret = 1;
	}
	else {
		adjust = 0;
	}

	cpu_adjust_system_tick (adjust);		//adjust 1ms
	return ret;
}

void ll_add_clock_time (u32 ms)
{
	if (ms > CHANNEL_SLOT_TIME * 8) {
		device_sync = 0;
		ll_clock_time = 0;
	}
	else {
		ll_clock_time += ms;
		while (ll_clock_time >= CHANNEL_SLOT_TIME) {
			ll_clock_time -= CHANNEL_SLOT_TIME;
			device_channel = get_next_channel_with_mask (ll_chn_mask, device_channel);
		}
	}

}


void ll_channel_alternate_mode ()
{
	device_sync = 0;
}

#endif

