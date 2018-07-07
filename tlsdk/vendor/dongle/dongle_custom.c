/*
 * dongle_custom.c
 *
 *  Created on: 2014-3-10
 *      Author: sihui
 */

#include "../../proj/tl_common.h"
#include "../../proj/mcu/watchdog_i.h"
#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"
#include "../common/rf_frame.h"
#include "../common/emi.h"

#include "dongle_custom.h"
#include "dongle_usb.h"

void (*device_program) (int adr, unsigned char id);
void (*device_program_on) (void);
void (*device_program_off) (void);

void otp_device_program_on (void);
void otp_device_program_off (void);
void otp_device_program(int , unsigned char);

void flash_device_program_on (void);
void flash_device_program_off (void);
void flash_device_program(int , unsigned char);


custom_cfg_t   *p_custom_cfg;

u32     rf_paring_tick = 0;
u32		custom_binding[2] = {0xffffffff, 0xffffffff};  //mouse_id   keyboard_id


s8 		custom_rssi_paring_th_offset;
s8 		custom_auto_paring_rssi;
u8      dongle_custom_cap;

int		custom_binding_idx;

int     id_check_flg;

int     rf_paring_enable;
int     keyboard_paring_enable;
int     mouse_paring_enable;

int     golden_dongle_enable;
int     auto_paring_enable;
int     auto_paring_enable_m2;

int     soft_paring_enable;
int     manual_paring_enable;

int     dongle_support_mouse_enable;
int     dongle_support_keyboard_enable;

u8      channel_mask_custom;




void dongle_emi_cust_init( void ){
    write_reg8( 0x800598, read_reg8(0x800598) & 0xf0 ); //8366 disable mspi IO drive-strenth to satisfy 192M
}

/****************************************************************************************************************
     custom_init
 ***************************************************************************************************************/
void	custom_init (void)
{
#if(CUSTOM_DATA_ERR_FIX_ENABLE)
	for(int i=0;i<CUSTOM_DATA_MAX_COUNT;i++){
		p_custom_cfg = (custom_cfg_t *) (DEVICE_CUSTOMIZATION_INFO_ADDR+(i*22));  //3f00
		if(p_custom_cfg->paring_limit_t != 0){
			break;
		}
	}
#else
	p_custom_cfg = (custom_cfg_t *)(DEVICE_CUSTOMIZATION_INFO_ADDR);  //3f00
#endif

	u16 custom_id;
	custom_id                      = p_custom_cfg->vid;
	if(custom_id != U16_MAX){
		rf_set_access_code0 (rf_access_code_16to32(custom_id));
	}
	custom_id                      = p_custom_cfg->id;
	if(custom_id != U16_MAX){
		u32 code = rf_access_code_16to32(custom_id);
		rf_set_access_code1 (code);
	}

#if DONGLE_CUS_EN
	dongle_custom_cap              = p_custom_cfg->cap;
    if ( p_custom_cfg->cap != U8_MAX )
        cap_internal_adjust( p_custom_cfg->cap );


    u8 pair_type =  ((p_custom_cfg->paring_type == U8_MAX) ? 0xaf : p_custom_cfg->paring_type);


	auto_paring_enable             = !(pair_type & CUSTOM_DONGLE_AUTO_PARING);
	soft_paring_enable             = !(pair_type & CUSTOM_DONGLE_SOFT_PARING);
	manual_paring_enable           = !(pair_type & CUSTOM_DONGLE_MANNUAL_PARING);
	golden_dongle_enable           = !(pair_type & CUSTOM_DONGLE_GOLDEN);

	auto_paring_enable_m2          = !(pair_type & CUSTOM_DONGLE_AUTO_PARING_M2);


#else
	auto_paring_enable = 1;
#endif																		//paring time limit

	dongle_support_mouse_enable    = 1;    //default : mouse only
#if DONGLE_CUS_EN
	dongle_support_keyboard_enable = (p_custom_cfg->support_type != U8_MAX);


	#if (CAVY_HAMSTER_REMINGTON_COMPATIBLE_EN)
		work_with_cavy_mouse = (p_custom_cfg->chn_type != U8_MAX);		//default hamster mouse
	#endif

	custom_rssi_paring_th_offset   = (p_custom_cfg->rssi_threshold == U8_MAX) ? 0 : p_custom_cfg->rssi_threshold;
	//custom_auto_paring_rssi        = (p_custom_cfg->rssi_threshold == U8_MAX) ? 65 : p_custom_cfg->rssi_threshold;
	channel_mask_custom            = p_custom_cfg->channal_msk;

	rf_paring_enable       = manual_paring_enable || auto_paring_enable_m2;



#else
	rf_paring_enable  = 1;
#endif
	keyboard_paring_enable = 0;
	mouse_paring_enable    = 0;

#if(USB_DESCRIPTER_CONFIGURATION_FOR_KM_DONGLE)  //for km dongle customization (add by sihui)
	get_usb_descripter_configuration();
#endif

#if(USB_ID_AND_STRING_CUSTOM)
	get_usb_id_and_string(p_custom_cfg->vendor_id,p_custom_cfg->prodct_id);
#endif

#if DONGLE_ID_IN_FW
	if (p_custom_cfg->memory_type == U8_MAX){
		device_program = otp_device_program;
		device_program_on = otp_device_program_on;
		device_program_off = otp_device_program_off;
	}
	else{
		device_program = flash_device_program;
		device_program_on = flash_device_program_on;
		device_program_off = flash_device_program_off;
	}
#endif

	id_check_flg = p_custom_cfg->id_check == U8_MAX;
    if ( p_custom_cfg->emi_ini_patch != U8_MAX )
        pf_emi_cust_init = dongle_emi_cust_init;


	u16 *p = (u16 *) DEVICE_PARING_INFO_ADDR;
	for (custom_binding_idx=0; custom_binding_idx<DID_PARING_MAX; custom_binding_idx++) {
		if (*p == 0xffff)
			break;
		custom_binding[p[0] & 1] = p[1] + (p[2] << 16);
		p += 3;
	}
}

#if DONGLE_ID_IN_FW
/****************************************************************************************************************
     set_device_id_in_firmware
     load paired device id into flash and ram
 ***************************************************************************************************************/
int set_device_id_in_firmware (int type,u32 id)
{
	int ret = 1;
	if (custom_binding[type] == id)
	{
		//do nothing
	}
	else if (custom_binding_idx < DID_PARING_MAX)
	{
		u32 adr = (DEVICE_PARING_INFO_ADDR + 6 * custom_binding_idx);


		device_program_on ();
		device_program (adr, type);
		device_program (adr+2, id);
		device_program (adr+3, id>>8);
		device_program (adr+4, id>>16);
		device_program (adr+5, id>>24);
		device_program_off ();


		if(id_check_flg){
			u8 temp = *(volatile u8*)(adr);
			temp = *(volatile u8*)(adr+1024);
			temp = *(volatile u8*)(adr+2*1024);

			u8 type_read =  *(volatile u8*)(adr);
			u16 id_read_low = *(volatile u16*)(adr+2);
			u16 id_read_high = *(volatile u16*)(adr+4);

			if( type_read != type || ((id_read_high<<16) | id_read_low) != id ){
				return 0;
			}
		}

		custom_binding[type & 1] = id;
		custom_binding_idx ++;
	}
	else {
		ret = 0;
	}
	return ret;
}


/****************************************************************************************************************
     device_program_on
     device_program
     device_program_off
 ***************************************************************************************************************/

//#if (OTP_PROGRAM)

//////////////////////////////////////////////////////
//copy from  dut otp code
//////////////////////////////////////////////////////
void otp_device_program_on (void){
	/* open vpp 6.75V */
	unsigned char irqst = read_reg8(0x800643);
	write_reg8(0x800643,0);

	write_reg8(0x800071, 0x13);;    // set DCDC 6.75 clk to 60M
	sleep_us(100);
	analog_write(0x85, 0x14);
	sleep_us(1000);   //wait at least 1000us
	analog_write(0x85, 0x54);
	sleep_us(100);

	write_reg8(0x800643,irqst);
}

void otp_device_program_off(void)
{
	analog_write(0x85, 0x0c);
	write_reg8(0x800071, 0x03);
}


_attribute_ram_code_ void otp_device_program(int addr, unsigned char value){
	unsigned char irqst = read_reg8(0x800643);
	write_reg8(0x800643,0);

	write_reg8(0x800012, 0x7e);
	write_reg16(0x800010, addr);
	write_reg8(0x80001a, 0x02);
	write_reg8(0x800013, value);
	WaitUs (100);
	write_reg8(0x800013, value);
	write_reg8(0x80001a, 0x0);
	WaitUs (20);

	write_reg8(0x800643,irqst);
}

//#else

void flash_device_program_on (void){}
void flash_device_program_off(void){}
//flash program to emulate OTP
_attribute_ram_code_ void flash_device_program (int adr, unsigned char id)
{
	unsigned char irqst = read_reg8(0x800643);
	write_reg8(0x800643,0);
	write_reg8 (0x80000d, 0x00);
	write_reg8 (0x80000c, 0x06);	//write enable
	while (read_reg8(0x80000d));
	write_reg8 (0x80000d, 0x01);

	WaitUs (1);

	write_reg8 (0x80000d, 0x00);
	write_reg8 (0x80000c, 0x02);	//write command
	while (read_reg8(0x80000d));

	write_reg8 (0x80000c, adr>>16);
	while (read_reg8(0x80000d));

	write_reg8 (0x80000c, adr>>8);
	while (read_reg8(0x80000d));

	write_reg8 (0x80000c, adr);
	while (read_reg8(0x80000d));

	write_reg8 (0x80000c, id);
	while (read_reg8(0x80000d));

	write_reg8 (0x80000d, 0x01);
#if 0
	write_reg8 (0x80000d, 0x00);
	write_reg8 (0x80000c, 0x05);	//read status
	while (read_reg8(0x80000d));
	int i;
	for (i=0; i<1000000; i++)
	{
		write_reg8 (0x80000c, 0x00);	//launch 8-cycle to trigger read
		while (read_reg8(0x80000d));
		if ( !(read_reg8(0x80000c) & 1) )
			       break;
	}
	write_reg8 (0x80000d, 0x01);
#else
	WaitUs (1000);
#endif
	write_reg8(0x800643,irqst);
}
//#endif
#endif

/****************************************************************************************************************
     paring_time_check
 ***************************************************************************************************************/
void paring_time_check(void)
{
	if(soft_paring_enable){
		if ( (keyboard_paring_enable || mouse_paring_enable) && clock_time_exceed(rf_paring_tick, 10000000)){
			keyboard_paring_enable = 0;
			mouse_paring_enable    = 0;
		}
	}

	else if(manual_paring_enable || auto_paring_enable_m2){
		if (rf_paring_enable && clock_time_exceed(rf_paring_tick, custom_dongle_time_limit())){
			rf_paring_enable = 0;
		}
	}

}

