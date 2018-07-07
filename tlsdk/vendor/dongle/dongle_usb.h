/*
 * dongle_usb.h
 *
 *  Created on: Feb 13, 2014
 *      Author: xuzhen
 */

#ifndef DONGLE_USB_H_
#define DONGLE_USB_H_


#define MAX_STRING_LEN  22

typedef struct
{
	u8 Size;
	u8 Type;
	wchar_t str[MAX_STRING_LEN];
}KM_USB_Descriptor_String_t;


void usb_data_report_proc(void);
void get_usb_descripter_configuration(void);
void get_usb_id_and_string(u16,u16);


extern u8 configuration_km_desc[];
extern u8 mouse_interface_number;
extern u8 keyboard_interface_number;

extern u8 configuration_desc_mouse[];
extern u8 configuration_desc_keyboard[];

#define USB_HID_DESCRIPTOR_LENGTH   	 16

extern u8 device_desc_km[];
extern KM_USB_Descriptor_String_t  vendor_desc_km;
extern KM_USB_Descriptor_String_t  prodct_desc_km;
extern KM_USB_Descriptor_String_t  serial_desc_km;

#endif /* DONGLE_USB_H_ */
