/*
 * kb_custom.h
 *
 *  Created on: 2015-1-21
 *      Author: Administrator
 */

#ifndef KB_CUSTOM_H_
#define KB_CUSTOM_H_


typedef struct{
	u16	pipe_pairing;			// pipe 0,pairig
	u16 pipe_kb_data;			// pipe 2,kb data
	u32 did;					// device id

	u8	cap;  	//08
	u8  tx_power_paring;
    u8  tx_power;
	u8  tx_power_emi;

	u16 gpio_lvd;  //0c
	u16 gpio_scr;  //0e
	u16 gpio_cap;  //10
	u16 gpio_num;  //12

	u8  level_lvd; //14
	u8  level_scr;
	u8  level_cap;
	u8  level_num;

	u8  crystal_type; //18

#if (CAVY_HAMSTER_REMINGTON_COMPATIBLE_EN)
	u8  chn_type;	//19, 0xff: hamster chn, 0: cavy chn
#endif
}kb_custom_cfg_t;


extern kb_custom_cfg_t *p_kb_custom_cfg;

#define kb_cust_tx_power        ((p_kb_custom_cfg->tx_power 	   == 0xff) ? RF_POWER_8dBm   : p_kb_custom_cfg->tx_power)
#define kb_cust_tx_power_emi	((p_kb_custom_cfg->tx_power_emi	   == 0xff) ? RF_POWER_8dBm   : p_kb_custom_cfg->tx_power_emi)
#define kb_cust_tx_power_paring ((p_kb_custom_cfg->tx_power_paring == 0xff) ? RF_POWER_m24dBm : p_kb_custom_cfg->tx_power_paring )


#define KB_CUS_MAP_ADDR 		0x3c00
#define KB_SWITCH_SHIFT_ADDR	0x3e40
#define KB_SWITCH_KEY_ADDR		0x3ed0

#define KB_MAP_EXIST(addr)		( *((u32 *)(addr)) != U32_MAX  )

extern void kb_custom_init(void);

#endif /* KB_CUSTOM_H_ */
