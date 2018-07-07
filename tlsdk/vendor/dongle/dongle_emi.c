/*
 * dongle_emi.c
 *
 *  Created on: 2014-5-21
 *      Author: hp
 */

#include "../../proj/tl_common.h"
#include "../../proj/mcu/watchdog_i.h"
#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"
#include "../common/rf_frame.h"
#include "../common/emi.h"
#include "dongle_emi.h"
#include "dongle_custom.h"


u8      dongle_emi_cd_mode;
u8		dongle_host_cmd1;

extern u8      rf_paring_enable;
extern u32     rf_paring_tick;
extern int     soft_paring_enable;


/******************************************************************************************
 * old
 *  host_cmd[1]
 *	0  1  2   no use
 *	3                        软件配对时间允许
 *	4  5  6   carrier          test_mode_sel = 0
 *	7  8  9   cd               test_mode_sel = 1
 *	10 11 12  rx               test_mode_sel = 2
 *
 *	4  7  10  fre_7     2440   chn_idx = 1   host_cmd_chn_m
 *	5  8  11  fre_15    2476   chn_idx = 2   host_cmd_chn_h
 *	6  9  12  fre_1     2409   chn_idx = 0   host_cmd_chn_l
 *****************************************************************************************/
#if(0)
const u8	dongle_chn_mode[13] = {
		0, 0, 0,
		0,
		chn_idx_m | test_mode_carrier<<2,
		chn_idx_h | test_mode_carrier<<2,
		chn_idx_l | test_mode_carrier<<2,

		chn_idx_m | test_mode_cd<<2,
		chn_idx_h | test_mode_cd<<2,
		chn_idx_l | test_mode_cd<<2,

		chn_idx_m | test_mode_rx<<2,
		chn_idx_h | test_mode_rx<<2,
		chn_idx_l | test_mode_rx<<2,
		};
#endif



#define  PARING_ENABLE_CMD_VALUE  0x88
/******************************************************************************************
 * 	new
 *  host_cmd[1]    chn	     mode
 *  01			   2409		carrier
 *  02			   2409		cd
 *  03			   2409		rx
 *  04			   2409		tx
 *  05			   2435		carrier
 *  06			   2435		cd
 *  07			   2435		rx
 *  08			   2435		tx
 *  09			   2476		carrier
 *  0a			   2476		cd
 *  0b			   2476		rx
 *  0c			   2476		tx
 *****************************************************************************************/
enum {
	chn_idx_l          = 0,   //2409
	chn_idx_m          = 1,   //2435
	chn_idx_h          = 2,	  //2476
	test_mode_carrier  = 0,
	test_mode_cd	   = 1,
	test_mode_rx	   = 2,
	test_mode_tx	   = 3,
};

#define  CARRIER_MODE   0


void usb_host_cmd_proc(u8 *pkt)
{
	extern u8		host_cmd[8];
	extern u8		host_cmd_paring_ok;

	u8   chn_idx;
	u8   test_mode_sel;
	u8 	 cmd = 0;
	static emi_flg;


	if((host_cmd[0]==0x5) && (host_cmd[2]==0x3) )
	{
		host_cmd[0] = 0;
		dongle_host_cmd1 = host_cmd[1];

		if (dongle_host_cmd1 > 12 && dongle_host_cmd1 < 16){  //soft paring
			host_cmd_paring_ok = 0;
			rf_paring_tick = clock_time();  //update paring time

			if(dongle_host_cmd1 == 13){     //kb and mouse tolgether
				mouse_paring_enable = 1;
				keyboard_paring_enable = 1;
			}
			else if(dongle_host_cmd1 == 14){ //mouse only
				mouse_paring_enable = 1;
			}
			else if(dongle_host_cmd1 == 15){  //keyboard only
				keyboard_paring_enable = 1;
			}
		}
		else if(dongle_host_cmd1 > 0 && dongle_host_cmd1 < 13)  //1-12:进入EMI
		{
			emi_flg = 1;
			cmd = 1;

			irq_disable();
			reg_tmr_ctrl &= ~FLD_TMR1_EN;
			//rf_stop_trx ();

			chn_idx = (dongle_host_cmd1-1)/4;
			test_mode_sel = (dongle_host_cmd1-1)%4;
		}
	}

	if(emi_flg){
		emi_process(cmd, chn_idx,test_mode_sel, pkt, dongle_cust_tx_power_emi);
	}
}


