/*
 * dongle_custom.h
 *
 *  Created on: 2014-3-10
 *      Author: hp
 */

#ifndef DONGLE_CUSTOM_H_
#define DONGLE_CUSTOM_H_

#ifndef DONGLE_CUS_EN
#define DONGLE_CUS_EN		1
#endif

#ifndef	DONGLE_ID_IN_FW
#define DONGLE_ID_IN_FW		1
#endif

#include "../common/user_config.h"

void      custom_init (void);
void 	  paring_time_check(void);

#define 	RECV_PKT_RSSI(p)                (((u8 *)p)[4])


/********************************************************
paring_type customization
addr :  0x3f05
default  0xff:  auto			   1111 1111	->0xaf
         0xcf:  soft        	   1100 1111
         0xa7:  auto_m2			   1010 0111
		 0xfb:	manual             1111 1011
         0x6f:  golden dongle	   0110 1111

 *******************************************************/
#define		CUSTOM_DONGLE_GOLDEN			BIT(7)     // 0 for golden dongle
#define		CUSTOM_DONGLE_AUTO_PARING		BIT(6)     // 0 for  auto paring
#define		CUSTOM_DONGLE_SOFT_PARING		BIT(5)     // 0 for software paring

#define		CUSTOM_DONGLE_AUTO_PARING_M2	BIT(3)     // 0 for  munaul  paring
#define		CUSTOM_DONGLE_MANNUAL_PARING	BIT(2)     // 0 for  munaul  paring

/********************************************************
support_type customization
addr  :  0x3f07
default  0xff  :  mouse only
         other :  mouse/keyboard kit
 *******************************************************/


/***********************************************************************************
vendor  string : addr 0x3de0   max_len:22 characters   default : Telink
product string : addr 0x3e40   max_len:22 characters   default : Wireless Receiver
serial  string : addr 0x3ea0   max_len:22 characters   default : TLSR8366
 **********************************************************************************/
#define     VENDOR_STRING_ADDR              0x3de0
#define     PRODCT_STRING_ADDR              0x3e40
#define     SERIAL_STRING_ADDR              0x3ea0
#define     DEVICE_PARING_INFO_ADDR         (DEVICE_CUSTOMIZATION_INFO_ADDR + 0x30)     //配对信息地址


/***********************************************************************************
OTP 中定制信息写错，修复机制开启  ：CUSTOM_DATA_ERR_FIX_ENABLE
		          最多可写定制信息次数 ：CUSTOM_DATA_MAX_COUNT
 **********************************************************************************/
#define		CUSTOM_DATA_ERR_FIX_ENABLE      1
#define		CUSTOM_DATA_MAX_COUNT		    2

//cust_crystal_12M: the first custom info 
//#define		cust_crystal_12M	( (*(unsigned char*) (DEVICE_CUSTOMIZATION_INFO_ADDR - 1)) == 0 )    //( custom_cfg_t->crystal_12M )
typedef struct{
	u16  vid;		  		  // 00~0x01 vendor id
	u16  id;		  		  // 02~0x03 device id
	u8	 cap;	           // 04 	crystal CAP setting
	u8	 paring_type;	   // 05 	paring_type
	u8   paring_limit_t;   // 06    manual_paring_time_set(4 s unit)   @@@@@as wrong_info flg if set to 0
	u8	 support_type;     // 07 	BIT7|BIT6  10:mouse_only   01:keyboard_only    11:mouse and keyboard
	u8	 rssi_threshold;   // 08
	u8   channal_msk;      // 09
	u8 	 report_type;      // 0a    default for mouse report rate 125,other for 250
	u16  vendor_id;        // 0b-0c
	u16  prodct_id;        // 0d-0e
	u8   tx_power_emi;     // 0f

	u8   memory_type;      // 10  0xff for otp,0 for flash
	u8   id_check;         // 11  0xff: check paring id,   0: not check	
	u8   emi_ini_patch;    // 12  0xff: patch nothing,   0: some-board must disable mspi driver strength to pass emi 192M

#if (CAVY_HAMSTER_REMINGTON_COMPATIBLE_EN)
	u8   chn_type;	   // 13  0xff: for hamster chn,  0: for cavy chn
#endif

} custom_cfg_t;

extern u32	custom_binding[2];
static inline u32 get_device_id_from_ram(int type){
	return custom_binding[(type - FRAME_TYPE_MOUSE)&1];
}

static inline u32 set_device_id_in_ram(int type,u32 id){
	custom_binding[(type - FRAME_TYPE_MOUSE)&1] = id;
}

static inline u32 get_mouse_id_from_ram(void){
	return custom_binding[0];
}

static inline void set_mouse_id_in_ram(u32 id){
	custom_binding[0] = id;
}

static inline u32 get_keyboard_id_from_ram(void){
	return custom_binding[1];
}

static inline void set_keyboard_id_in_ram(u32 id){
	custom_binding[1] = id;
}


extern custom_cfg_t   *p_custom_cfg;
static inline u32 custom_dongle_time_limit () {
	return (p_custom_cfg->paring_limit_t & 15)*4000000;
}

#define dongle_cust_tx_power_emi         ( (p_custom_cfg->tx_power_emi == 0xff) ? RF_POWER_8dBm : p_custom_cfg->tx_power_emi )

extern int dongle_support_mouse_enable;
extern int  dongle_support_keyboard_enable;

extern int     keyboard_paring_enable;
extern int     mouse_paring_enable;

#endif /* DONGLE_CUSTOM_H_ */
