/*
 * emi.h
 *
 *  Created on: Feb 14, 2014
 *      Author: xuzhen
 */

#ifndef EMI_H_
#define EMI_H_

#if (__PROJECT_VACUUM__  || __PROJECT_VACUUM_RECEIVER__ )

enum {
	host_cmd_chn_l  = 0,
	host_cmd_chn_m  = 10,
	host_cmd_chn_h  = 15,
	host_cmd_mask	= 0xf0,
	//host_cmd_paring = BIT(8),
	host_cmd_carrier= BIT(4),
	host_cmd_cd		= BIT(5),
	host_cmd_rx		= BIT(6),
	host_cmd_tx		= BIT(7),
};

enum{
	host_cmd_chn_0 = 0,
	host_cmd_chn_1,
	host_cmd_chn_2,
	host_cmd_chn_3,
	host_cmd_chn_4,
	host_cmd_chn_5,
	host_cmd_chn_6,
	host_cmd_chn_7,
	host_cmd_chn_8,
	host_cmd_chn_9,
	host_cmd_chn_a,
	host_cmd_chn_b = 11,
	host_cmd_chn_c,
	host_cmd_chn_d,
	host_cmd_chn_e = 14,
	host_cmd_chn_f = 15,
};

#else
enum {
	host_cmd_chn_l  = 0,
	host_cmd_chn_m  = 6,
	host_cmd_chn_h  = 14,
	host_cmd_mask	= 0xf0,
	//host_cmd_paring = BIT(8),
	host_cmd_carrier= BIT(4),
	host_cmd_cd		= BIT(5),
	host_cmd_rx		= BIT(6),
	host_cmd_tx		= BIT(7),
};
#endif

#define		TEST_MODE_CARRIER	(test_mode & host_cmd_carrier)
#define		TEST_MODE_CD		(test_mode & host_cmd_cd)
#define		TEST_MODE_RX		(test_mode & host_cmd_rx)
#define		TEST_MODE_TX		(test_mode & host_cmd_tx)


void emi_process(u8 cmd, u8 chn_idx, u8 test_mode_sel, u8 *tx_pkt, u8 tx_power_emi);
extern void (*pf_emi_rx_init) ( void );
extern void (*pf_emi_rx_recover) ( void );
extern void (*pf_emi_cust_init) ( void );

#endif /* EMI_H_ */
