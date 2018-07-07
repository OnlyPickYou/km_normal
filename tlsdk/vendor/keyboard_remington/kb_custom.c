/*
 * kb_custom.c
 *
 *  Created on: 2015-1-21
 *      Author: Administrator
 */

#include "../../proj/tl_common.h"
#include "../../proj_lib/pm.h"
#include "../../proj_lib/rf_drv.h"
#include "../common/rf_frame.h"

#include "kb_custom.h"
#include "kb_rf.h"


kb_custom_cfg_t *p_kb_custom_cfg;

/******************************************************************
 * function:
 * normal_addr -> 0x3c00  fn_addr -> 0x3c90
 * numlock_addr -> 0x3d20 fn_numlock_addr -> 0x3db0
 *
 * if you need to fix the cust addr then
 * normal_addr -> 0x3900  fn_addr -> 0x3990
 * numlock_addr -> 0x3920 fn_numlock_addr -> 0x39b0
 *
*****************************************************************/


void kb_map_addr_cust(void)
{
	extern u8 *	kb_p_map[4];
	extern u8 *kb_map_shift;

	extern kb_switch_key_t *kb_switch_key[8];
	u16 adr_offset = 0;
	if(KB_MAP_EXIST(0x3900)){
		adr_offset = 0x300;
	}
	if(KB_MAP_EXIST(KB_CUS_MAP_ADDR - adr_offset)){
		kb_p_map[0] = KB_CUS_MAP_ADDR - adr_offset;  //normal
		u8 fn_customed = KB_MAP_EXIST(KB_CUS_MAP_ADDR - adr_offset + 0x90);
		u8 num_customed = KB_MAP_EXIST(KB_CUS_MAP_ADDR - adr_offset + 0x120);
		kb_p_map[1] = fn_customed ? (KB_CUS_MAP_ADDR - adr_offset + 0x90) : kb_p_map[0];   //fn
		kb_p_map[2] = num_customed ? (KB_CUS_MAP_ADDR - adr_offset + 0x120) : kb_p_map[0];   //num
		kb_p_map[3] = KB_MAP_EXIST(KB_CUS_MAP_ADDR - adr_offset + 0x1b0) ? \
				(KB_CUS_MAP_ADDR - adr_offset + 0x1b0) : \
				(fn_customed ? kb_p_map[1] :  (num_customed ? kb_p_map[2] : kb_p_map[0]) ); //fn+num
	}

	kb_map_shift = KB_SWITCH_SHIFT_ADDR;

	for(int i=0; i<8; i++){
		kb_switch_key[i] =  (volatile  u8*)(KB_SWITCH_KEY_ADDR + i * 4);
	}

	if( *(volatile u32 *)(KB_SWITCH_SHIFT_ADDR) == 0xdeadbeef){
		kb_map_shift = KB_SWITCH_SHIFT_ADDR - 0x300;
	}
}
void kb_custom_init(void)
{
	//3f00  3f30  3f60
	for(int i=0;i<3;i++){
		p_kb_custom_cfg = (kb_custom_cfg_t *) (DEVICE_ID_ADDRESS+(i*0x30));
		if(p_kb_custom_cfg->cap != 0){
			break;
		}
	}

    if(p_kb_custom_cfg->pipe_pairing != U16_MAX){
		rf_set_access_code0 (rf_access_code_16to32(p_kb_custom_cfg->pipe_pairing));
	}

	if (p_kb_custom_cfg->did != U32_MAX) {
		pkt_pairing.did = p_kb_custom_cfg->did;
	}
#if (KEYBOARD_PIPE1_DATA_WITH_DID)
	pkt_km.did = pkt_pairing.did;
#endif

	analog_write(0x81,p_kb_custom_cfg->cap == U8_MAX ? 0xd8 : p_kb_custom_cfg->cap);

	kb_status.cust_tx_power = (p_kb_custom_cfg->tx_power == 0xff) ? RF_POWER_8dBm : p_kb_custom_cfg->tx_power;
	kb_status.tx_power = RF_POWER_2dBm;

	kb_status.led_gpio_lvd = (p_kb_custom_cfg->gpio_lvd == U16_MAX) ? GPIO_GP25 : p_kb_custom_cfg->gpio_lvd;

	kb_status.led_gpio_scr = (p_kb_custom_cfg->gpio_scr == U16_MAX) ? 0 : p_kb_custom_cfg->gpio_scr;
	kb_status.led_gpio_cap = (p_kb_custom_cfg->gpio_cap == U16_MAX) ? 0 : p_kb_custom_cfg->gpio_cap;
	kb_status.led_gpio_num = (p_kb_custom_cfg->gpio_num == U16_MAX) ? 0 : p_kb_custom_cfg->gpio_num;

	kb_status.led_level_lvd = (p_kb_custom_cfg->level_lvd == U8_MAX) ? 1 : 0; //0xff:high_valid,level = 1

	kb_status.led_level_scr = (p_kb_custom_cfg->level_scr == U8_MAX) ? 0 : 1; //0xff:high valid,level = 0
	kb_status.led_level_cap = (p_kb_custom_cfg->level_cap == U8_MAX) ? 0 : 1; //0xff:high valid,level = 0
	kb_status.led_level_num = (p_kb_custom_cfg->level_num == U8_MAX) ? 0 : 1; //0xff:high valid,level = 0

#if (CAVY_HAMSTER_REMINGTON_COMPATIBLE_EN)
	work_with_cavy_chn = (p_kb_custom_cfg->chn_type != U8_MAX) ? 1 : 0;
#endif

	kb_map_addr_cust();
/*
	if( *((volatile u8 *)0x3d00) != U8_MAX){
		kb_p_map[0] = kb_p_map[2] = 0x3d00;
		kb_p_map[4] = kb_p_map[6] = 0x3d90;
	}

	//3f90-3f97     kb_p_map[0]   normal
	//3f98-3f9f     kb_p_map[2]   numlock
	//3fa0-3fa7     kb_p_map[4]   fn
	//3fa8-3faf     kb_p_map[6]   fn+numlock

	u16 *p_cust_map_addr = 0x3f90;
#if 0
    for(int j = 0; j < sizeof(kb_p_map)/sizeof(int); j+=2 ){
        for(int i = 0; i < 4; i++ ){
            u16 map_cust = *( p_cust_map_addr + 4 * j + 2 * i );
            if( map_cust != U16_MAX ){
            	kb_p_map[j] = map_cust;
            }
        }
    }
#else
    for(int j = 0; j < 4; j++ ){
        for(int i = 0; i < 4; i++ ){
            u16 map_cust = *( p_cust_map_addr + 4 * j + i );
            if( map_cust != U16_MAX ){
            	kb_p_map[j*2] = map_cust;  //0 2 4 6
            }
        }
    }
#endif
*/
}

