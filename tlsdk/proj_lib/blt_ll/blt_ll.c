#if __PROJECT_BLUELIGHT__

#include "../../proj/tl_common.h"
#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"
#include "blt_ll.h"

#define		MAC_MATCH(m)	(blt_p_mac[0]==m[0] && blt_p_mac[1]==m[1] && blt_p_mac[2]==m[2])

#define		BLT_BRX2RCVD				700

unsigned char  		blt_rx_buffer[64] __attribute__((aligned(4)));
u8					blt_tx_wptr;
u8					blt_tx_rptr;
u32					blt_tx_fifo[4][8];
u32					blt_tx_buffer[12];
u32					blt_tx_crc[4][4];
u8					blt_tx_empty_packet[5] = {1, 0};
u32					blt_tx_crc_empty[4] = {};
u8					blt_tx_sending = 0;

u32				blt_pn_table[10] = {};
u32				blt_pn_table_next[10] = {};
u32				blt_crc_init = 0;
u8 *			blt_p_adv = 0;
u8 *			blt_p_rsp = 0;
u16 *			blt_p_mac = 0;
u8				blt_state = 0;

u32				blt_adv_interval;

u32				blt_next_event_tick;

u8				blt_conn_sn_slave;
u8				blt_conn_sn_master;
u8				blt_conn_chn;

u32				blt_conn_timeout;
u32				blt_conn_tick;
u32				blt_conn_interval;
u16				blt_conn_inst;
u32				blt_conn_duration;
u8				blt_conn_latency;
u8				blt_conn_chn_map[5] = {0, 0, 0, 0, 0};
u8				blt_conn_chn_hop;

u8				blt_conn_update = 0;
u32				blt_conn_timeout_next;
u32				blt_conn_offset_next = 0;
u32				blt_conn_interval_next;
u16				blt_conn_inst_next;
u16				blt_conn_latency_next;
u8				blt_conn_chn_map_next[5] = {0, 0, 0, 0, 0};

/////////////////////////////////////////////////////////////////////////////////////
u8				blt_ll_channelTable[40];
u8				blt_ll_chn_idx;

void blt_ll_channelTable_calc(u8* chm, u8 hop){
	u8 tableTemp[37], num = 0;
	foreach(k, 37){
		if(chm[k>>3] & BIT(k & 0x07)){
			tableTemp[num++] = k;
		}
	}
	u8 k = 0, l = 0;
	foreach(i, 37){
		k += hop;
		if(k >= 37){
			k -= 37;
		}
		if(chm[k>>3] & BIT(k & 0x07)){
			blt_ll_channelTable[l] = k;
		}else{
			u8 m = k;
			while(m >= num){
				m -= num;
			}
			blt_ll_channelTable[l] = tableTemp[m];
		}
		++l;
	}
}

u8 blt_next_channel (u32 tick_tolerance)
{
	if (++blt_ll_chn_idx >= 37)
	{
		blt_ll_chn_idx -= 37;
	}
	return blt_ll_channelTable[blt_ll_chn_idx];
}
//////////////////////////////////////////////////////////////////////////////////////////


volatile	u8	blt_tx_send = 0;
void irq_blt_tx(void)
{
	static u32 tick_last_tx;
	tick_last_tx = clock_time ();
	//SetRxMode (host_channel, RF_CHN_TABLE);
	reg_rf_irq_status = FLD_RF_IRQ_TX;
	blt_tx_send = 1;
}


void	blt_set_adv (u8 *p, u8 *pr)
{
	memcpy (p + 2, (u8 *)blt_p_mac, 6);
	memcpy (pr + 2, (u8 *)blt_p_mac, 6);
	pr[0] = 4;
	blt_p_adv = p;
	blt_packet_tx_crc (p, p);

	blt_p_rsp = pr;
	blt_packet_tx_crc (pr, pr);
}

void	blt_adv_init ()
{
	blt_state = BLT_LINK_STATE_ADV;
	blt_crc_init = 0xaaaaaa;
	REG_ADDR32(0x408) =  0xd6be898e;
	blt_pn_init (37, 0);
	//blt_pn_init (37, 1);
	blt_adv_interval = 200000 * CLOCK_SYS_CLOCK_1US;
	blt_next_event_tick = clock_time () + blt_adv_interval;
}

void blt_init (u8 *p_mac, u8 *p_adv, u8 *p_rsp)
{
	reg_dma_rf_rx_addr = (u16)(u32) (blt_rx_buffer);
	reg_dma2_ctrl = FLD_DMA_WR_MEM | (64>>4);   // rf rx buffer enable & size
	reg_dma_chn_irq_msk = 0;

	reg_irq_mask |= FLD_IRQ_ZB_RT_EN;    //enable RF & timer1 interrupt
	reg_rf_irq_mask = 0; //FLD_RF_IRQ_TX;
	// timer1 interrupt enable
	//reg_tmr1_tick = 0;
	//reg_tmr1_capt = CLOCK_SYS_CLOCK_1US * CHANNEL_SLOT_TIME;
	//reg_tmr_ctrl |= FLD_TMR1_EN;

	REG_ADDR16(0x50c) = (u16)((u32)blt_tx_buffer);
	write_reg32(0x800f18, clock_time() + 32);
	REG_ADDR8(0xf16) |= BIT(4);
	REG_ADDR16(0x50c) = (u16)((u32)blt_tx_buffer);
	blt_adv_init ();
	blt_p_mac = (u16 *)p_mac;
	blt_set_adv (p_adv, p_rsp);
}


//////////////////////////////////////////////////////////////////////////
//		PN & CRC24
//////////////////////////////////////////////////////////////////////////

u32		dbg_crc_init;
int crc_init_reverse (u8 * p)
{
	int init = p[0] + (p[1]<<8) + (p[2]<<16);
	dbg_crc_init = init;
	int r = 0;
	for (int i=0; i<24; i++) {
		r = ( r << 1) | (init & 1);
		init >>= 1;
	}
	return r;
}


void	blt_pn_init (int pn_init, int next)
{
	pn_init |= BIT(6);
	int		poly[2]={0, 0x44};              //0x8005 <==> 0xa001
	u32 *pt = next ? blt_pn_table_next : blt_pn_table;
	for (int i=0; i<10; i++) {
		*pt = 0;
		for (int j=0; j<32; j++) {
			*pt |= (pn_init & 1) << j;
			pn_init = (pn_init >> 1) ^ poly[pn_init & 1];
//			if (cond) {
//				pn_init ^= 0x44;
//			}
		}
		pt++;
	}

}

void	blt_pn_packet (u32 *p, int n)
{
	u32 *px = blt_pn_table;

	for (int i=0; i<n; i+=4) {
		*p++ ^= *px ++;
	}
}

_attribute_ram_code_ int	blt_packet_crc24 (unsigned char *p, int n, int crc)
{
	//crc16£º G(X) = X16 + X15 + X2 + 1

	//crc16:  x24 + x10 + x9 + x6 + x4 + x3 + x + 1 ==> 0x00065b
	//               13   14   17   19   20  22  23 ==> 0xda6000
   // static unsigned short poly[2]={0, 0xa001};              //0x8005 <==> 0xa001

	//int		poly[2]={0, 0xda6000};              //0x8005 <==> 0xa001
    for (int j=0; j<n; j++) {
    	u8 ds = p[j];
    	for (int i=0; i<8; i++) {

        	int cond = (crc ^ ds ) & 1;
        	crc >>= 1;
        	if (cond) {
        		 crc ^= 0xda6000;
        	}
            //crc = (crc >> 1) ^ poly[(crc ^ ds ) & 1];
            ds = ds >> 1;
    	}
    }
     return crc;
}

void	blt_get_crc_table (u8 *p, u32 *pt)
{
	int n = p[1];
	for (int i=0; i<4; i++) {
		p[0] = (p[0] & 0xf3) | (i<<2);
		pt[i] = blt_packet_crc24 (p, n + 2, blt_crc_init);
	}
}


int	blt_packet_tx_crc (u8 *pd, u8 *ps)
{
	int n = ps[1] + 2;
	if (pd != ps) {
		memcpy (pd, ps, n);
	}
	int crc = blt_packet_crc24 (ps, n, blt_crc_init);
	u8 *pcrc = pd + n;
	*pcrc++ = crc;
	*pcrc++ = crc >> 8;
	*pcrc++ = crc >> 16;
	return n;
}

_attribute_ram_code_ void	blt_packet_tx_pn (u32 *pd, u32 *ps)
{
	int n = ((u8 *)ps)[1] + 5;
	*pd++ = n;
	u32 *px = blt_pn_table;
	for (int i=0; i<n; i+=4){
		*pd ++ = *ps++ ^ *px++;
	}
}

const	u8 tbl_lsb2msb[16] = {0x0,0x8,0x4,0xc, 0x2,0xa,0x6,0xe, 0x1,0x9,0x5,0xd, 0x3,0xb,0x7,0xf};

#define	BYTE_LSB2MSB(a) ((tbl_lsb2msb[a&15]<<4) | (tbl_lsb2msb[a>>4]))

u32		dbg_tt;
u32		dbg_crc;
//u8		dbg_buff[40];
_attribute_ram_code_ u8	blt_packet_rx (u32 timeout, int send_tx)
{

	REG_ADDR8(0x404) = 0xff;		//enable shock-burst mode, crc: 1 byte
	REG_ADDR8(0x406) = 0x2c;		//default packet length 44
	REG_ADDR8(0x428) = 0x81;		//enable RX
	REG_ADDR8(0xf02) = 0x64;		//enable manual mode
	//return 0;
	volatile u32 *ph = (u32 *)(blt_rx_buffer + 12);
	ph[0] = ph[1] = 0;
	u8 *px = (u8 *)blt_pn_table;
	u32	t0 = clock_time ();
	while ((u32)(clock_time () - t0) < timeout) {
		// get first 4-byte
		if (*ph) {
			u8 len = (BYTE_LSB2MSB (blt_rx_buffer[13]) ^ px[1]) & 0x3f;
			if (len > 48) {
				len = 8;
			}
			REG_ADDR8(0x406) = len + 4;
			ph += 1 + (len>>2);
			*ph = 0;								//reset memory location of last byte
			u32 td = clock_time ();
			u8 *plm = blt_rx_buffer + 12;
			int offset = 0;
			int crc = blt_crc_init;
			//////////////////////////////////////////////////////////////////////
			int l = 0;
			if (len >= 13) {
				l = ((len + 8)>>3)<<2;
				u32 tf = (l - 2) * 8 * CLOCK_SYS_CLOCK_1US;
				while ((clock_time() - td) < tf);

				for (; offset<l; offset++) {
					plm[offset] = BYTE_LSB2MSB (plm[offset]) ^ px[offset];
				}
				crc = blt_packet_crc24 (plm, l, crc);
			}

			/////////////////////////////////////////////////////////////////////
			REG_ADDR8(0xf04) = 0x5c + (len & 3) * 8;
			u32 tp = (len + 5 - 1) * 8 * CLOCK_SYS_CLOCK_1US;
			while (!(*ph) && clock_time() - td < tp);

			REG_ADDR8(0x428) = 0x80;						// RX disable
			REG_ADDR8(0xf02) = 0x44;						// disable mannual mode
			REG_ADDR8(0x404) = 0xf5;						// disable shock-burst mode
			// start TX immediately
			if (send_tx)
			{
				REG_ADDR8(0xf00) = 0x85;						// single TX
				reg_rf_irq_status = FLD_RF_IRQ_TX;
				dbg_tt = clock_time ();
			}

			// LSB to MSB, PN
			for (; offset<len + 5; offset++) {
				plm[offset] = BYTE_LSB2MSB (plm[offset]) ^ px[offset];
			}

			// CRC
			crc = blt_packet_crc24 (plm + l, len + 2 - l, crc);		//

			u8 *pcrc = blt_rx_buffer + 14 + blt_rx_buffer[13];
			int crc_rx = (pcrc[0] | (pcrc[1]<<8) | (pcrc[2]<<16));


			if (crc == crc_rx) {		// CRC OK
				return BLT_BRX_CRC_OK;
			}
			else {
				REG_ADDR8(0xf00) = 0x80;		// stop SM
				dbg_crc++;
				return BLT_BRX_CRC_ERROR;
			}
		}
	}
	return BLT_BRX_TIMEOUT;
}

void blt_set_channel (signed char chn)
{
	if (chn < 11)
    	chn += 2;
    else if (chn < 37)
    	chn += 3;
    else if (chn == 37)
    	chn = 1;
    else if (chn == 38)
    	chn = 13;
    else
    	chn = 40;

    chn = chn << 1;

	/////////////////// turn on LDO and baseband PLL ////////////////
	write_reg8 (0x800f16, 0x29);

	write_reg8 (0x800428, 0x80);	// rx disable
	write_reg8 (0x800f02, 0x44);	// reset tx/rx state machine

	u32 fre = 2400 + chn;
	write_reg16 (0x8004d6, fre);	// {intg_N}
	write_reg32 (0x8004d0, (fre - 2) * 58254 + 1125);	// {intg_N, frac}

	rf_set_tp_gain (chn);
}

static inline void	blt_accesscode_init (u8 * p)
{
	REG_ADDR32(0x408) =  p[3] | (p[2]<<8) | (p[1]<<16) | (p[0]<<24);
}

u32		blt_adv_interval;
void	blt_set_adv_interval (u32 t_us)
{
	blt_adv_interval = t_us * CLOCK_SYS_CLOCK_1US;
}

u8*		blt_send_adv (int mask)
{

	static s8 chn = 37;
	u32  t_us = (blt_p_adv[1] + 10) * 8 + 180;
	for (int i=0; i<3; i++)
	{
		if (mask & BIT(i)) {
			REG_ADDR8(0x428) = 0x80;						// RX disable
			REG_ADDR8(0xf00) = 0x80;						// stop SM
			REG_ADDR8(0x404) = 0xf5;						// disable shock-burst mode
			blt_set_channel (chn);

			////////////// start TX //////////////////////////////////
			//REG_ADDR32(0xf18) =  clock_time() + 32;
			REG_ADDR8(0xf00) = 0x85;							// single TX
			u32 t = clock_time ();
			reg_rf_irq_status = FLD_RF_IRQ_TX;

			/////////////// generate Next PN: to save time ///////////////////////
			for (int k=0; k<10; k++) {
				blt_pn_table[k] = blt_pn_table_next[k];
			}
			blt_packet_tx_pn (blt_tx_buffer, (u32*)blt_p_adv);
			//blt_packet_tx_pn (blt_tx_buffer, (u32*)blt_p_rsp);
			/////////////// should be ready before tx settle //////////

			///////////// wait for TX
			chn = 37;
			//chn++;
			if (chn > 39) {
				chn = 37;
			}
			blt_pn_init (chn, 1);
			while (!(reg_rf_irq_status & FLD_RF_IRQ_TX) && (clock_time() - t) < t_us*CLOCK_SYS_CLOCK_1US);

			//////////// start RX
			REG_ADDR8(0xf00) = 0x80;
			////////////////////////////
			u8 r = blt_packet_rx (255 * CLOCK_SYS_CLOCK_1US, 1);
			if (r == BLT_BRX_CRC_OK) {
				u16 *pmac = (u16*)(blt_rx_buffer + 20);
				if (MAC_MATCH (pmac)) {				// MAC match
					if ((blt_rx_buffer[12] & 0x3f) == 0x03) {	//scan request
						static int dbg_rsp;
						dbg_rsp++;
						blt_packet_tx_pn (blt_tx_buffer, (u32*)blt_p_rsp);
						t_us = (blt_p_rsp[1] + 10) * 8 + 200;
						t = clock_time ();
						while (!(reg_rf_irq_status & FLD_RF_IRQ_TX) && (clock_time() - t) < t_us*CLOCK_SYS_CLOCK_1US);
						break;
					}
					else if ((blt_rx_buffer[12] & 0x3f) == 0x05) {	//connect request
						blt_state = blt_connect ();
						return blt_rx_buffer;
					}
				}
			}
		}
	}
	REG_ADDR8(0xf00) = 0x80;		// stop SM
	blt_next_event_tick += blt_adv_interval;
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////
#define		CLOCK_SYS_CLOCK_1250US			(1250 * CLOCK_SYS_CLOCK_1US)

u8 blt_connect ()
{
	rf_packet_connect_t *p = (rf_packet_connect_t *)(blt_rx_buffer + 8);

	blt_state = BLT_LINK_STATE_CONN;
	blt_conn_inst = 0;
	blt_conn_sn_slave = 0x10;
	blt_conn_sn_master = 0x10;
	blt_tx_wptr = blt_tx_rptr = 0;
	blt_tx_sending = 0;

	blt_accesscode_init (p->accessCode);
	blt_conn_interval = p->interval * CLOCK_SYS_CLOCK_1250US;
	blt_next_event_tick = p->winOffset * CLOCK_SYS_CLOCK_1250US - 500 * CLOCK_SYS_CLOCK_1US + clock_time ();
	blt_conn_duration = (p->winSize + 2) * CLOCK_SYS_CLOCK_1250US;
	blt_conn_timeout = p->timeout * 10000 * CLOCK_SYS_CLOCK_1US;
	blt_conn_latency = p->latency;
	blt_conn_tick = clock_time ();

	blt_conn_chn_hop = p->hop & 0x1f;
	memcpy (blt_conn_chn_map, p->chm, 5);
	blt_ll_channelTable_calc (p->chm, blt_conn_chn_hop);

	blt_ll_chn_idx = -1;
	blt_conn_chn = blt_next_channel (0);
	blt_pn_init (blt_conn_chn, 0);

	blt_crc_init = crc_init_reverse (p->crcinit);

	blt_get_crc_table (blt_tx_empty_packet, blt_tx_crc_empty);

	return 1;
}


_attribute_ram_code_ u8 blt_brx_sn_nesn ()
{
	u8 data_in = 0;
	u8 r_nesn = (blt_rx_buffer[12] >> 2) & 1;
	u8 r_sn =  (blt_rx_buffer[12] >> 3) & 1;
	if (r_nesn != blt_conn_sn_slave)						// master ACK
	{
		blt_conn_sn_slave = r_nesn;
		if (blt_tx_sending && blt_tx_rptr != blt_tx_wptr)	// skip to next packet
		{
			blt_tx_rptr++;
		}
	}

	if (r_sn != blt_conn_sn_master)						//new packet
	{
		blt_conn_sn_master = r_sn;

		data_in = blt_rx_buffer[13];
	}
	u8 idx = (!blt_conn_sn_master) | (blt_conn_sn_slave<<1);
	u32 crc;
	u8 *ptx;
	blt_tx_sending = blt_tx_rptr != blt_tx_wptr;
	if (blt_tx_sending)
	{
		 crc = blt_tx_crc[blt_tx_rptr & 3][idx];
		 ptx = (u8*) blt_tx_fifo[blt_tx_rptr & 3];
	}
	else
	{
		 crc = blt_tx_crc_empty[idx];
		 ptx = blt_tx_empty_packet;
	}
	ptx[0] = (ptx[0] & 0xf3) | (idx << 2);			//generate LLID_NESN_SN
	u8 *pcrc = ptx + ptx[1] + 2;		//get pointer to CRC
	*pcrc++ = crc;
	*pcrc++ = crc >> 8;
	*pcrc = crc >> 16;

	blt_packet_tx_pn (blt_tx_buffer, (u32*)ptx);
	dbg_tt = clock_time () - dbg_tt;
	return data_in;
}

u8 blt_push_fifo (u8 *p)
{
	if (((blt_tx_wptr - blt_tx_rptr) & 7) >= 4 ) {
		return 0;
	}
	u8 *pd = (u8 *)blt_tx_fifo[blt_tx_wptr & 3];
	memcpy (pd, p, p[1] + 2);
	blt_get_crc_table (pd, blt_tx_crc[blt_tx_wptr & 3]);
	blt_tx_wptr++;
	return 1;
}


u32		dbg_brx_ok, dbg_brx_all, dbg_chn_cmd, dgb_chn_up;

u8	blt_brx ()
{
	u8 conn_terminate = 0;

	/////////////////////////// start BRX //////////////////////////////
	blt_set_channel (blt_conn_chn);
	u8 c = blt_packet_rx (blt_conn_duration, 1);
	//while (1);
	if (c)
	{
		dbg_brx_all++;
		if (dbg_brx_all > 3) {
			//while (1);
		}
		if (c == BLT_BRX_CRC_OK)
		{
			blt_conn_tick = clock_time ();
			u8 len = blt_brx_sn_nesn ();		// handle BRX response

			u32 t = *(u32 *)(blt_rx_buffer + 8);
			static u32 dbg_sync, dbg_diff;
			dbg_sync = blt_conn_tick - t;
			if (dbg_sync < 1000 * CLOCK_SYS_CLOCK_1US)
			{
				dbg_diff = t - blt_next_event_tick;
				blt_next_event_tick = t - BLT_BRX2RCVD * CLOCK_SYS_CLOCK_1US;
			}

			dbg_brx_ok++;
			if (len)								//get new command
			{
				if ((blt_rx_buffer[12]&3) == 3 && blt_rx_buffer[14] == 1)	//update channel map
				{
					dbg_chn_cmd ++;
					blt_conn_update = 1;
					blt_conn_inst_next = *(u16 *)(blt_rx_buffer + 20);
					memcpy (blt_conn_chn_map_next, blt_rx_buffer + 15, 5);
				}
				else if ((blt_rx_buffer[12]&3) == 3 && blt_rx_buffer[14] == 0)
				{
					u16 *ps = (u16*) (blt_rx_buffer + 16);
					blt_conn_offset_next   = ps[0] * 1250 * CLOCK_SYS_CLOCK_1US;
					blt_conn_interval_next = ps[1] * 1250 * CLOCK_SYS_CLOCK_1US;
					blt_conn_latency_next = ps[2];
					blt_conn_timeout_next = ps[3] * 10000 * CLOCK_SYS_CLOCK_1US;
					blt_conn_inst_next = ps[4];
				}
				else {
					extern u8 * l2cap_att_handler(u8 * p);
					u8 *pr = l2cap_att_handler (blt_rx_buffer + 8);
					if (pr)
					{
						if ((u32)pr == 0xffffffff)		// termination
						{
							static u32 dbg_ter;
							dbg_ter++;
							conn_terminate = 0;
						}
						else
						{
							blt_push_fifo (pr + 4);
						}

					}
				}
			}
		}
		else						// CRC error
		{

		}
		/////////////// calculate next anchor ////////////////////
	}
	else						// time out
	{

	}

	////////////////////////////////////////////////////////////////
	if (conn_terminate || (u32)(clock_time() - blt_conn_tick) > blt_conn_timeout)
	{
		blt_adv_init ();
		return 0;
	}
	else
	{
		if (blt_conn_update == 1 && blt_conn_inst == blt_conn_inst_next)
		{
			dgb_chn_up++;
			blt_conn_update = 0;
			memcpy (blt_conn_chn_map, blt_conn_chn_map_next, 5);
			blt_ll_channelTable_calc (blt_conn_chn_map, blt_conn_chn_hop);
		}
		else if (blt_conn_update == 2 && (u16)(blt_conn_inst+1) == blt_conn_inst_next)
		{
			blt_conn_update = 0;
			blt_conn_interval = blt_conn_interval_next;
			blt_conn_timeout = blt_conn_timeout_next;
			blt_conn_latency = blt_conn_latency_next;
			blt_next_event_tick += blt_conn_offset_next - blt_conn_interval;
		}
		blt_conn_inst++;
		blt_conn_chn = blt_next_channel (0);
		blt_pn_init (blt_conn_chn, 0);
		blt_next_event_tick += blt_conn_interval;
	}
	return 1;
}

/////////////////////////////////////////////
//	REG_ADDR8(0x26) = 0xa6 to start
//  REG_ADDR8(0x27) = 0x0f: 16 cycles of 32K
//  REG_ADDR16(0x24): number of clock cycle of 32M
/////////////////////////////////////////////
_attribute_ram_code_ u32 blt_get_32k_tick ()
{
	u8 t0, t1;
	t0 = analog_read (0x20);					//PLL clock
	do {
		t1 = analog_read(0x20);
	} while (t1 == t0);
	return t1 | (analog_read(0x21)<<8) | (analog_read(0x22)<<16);
}

void blt_rc_calib ()
{

}
#endif
