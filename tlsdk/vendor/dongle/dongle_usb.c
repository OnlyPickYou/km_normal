/*
 * dongle_usb.c
 *
 *  Created on: Feb 13, 2014
 *      Author: xuzhen
 */

#include "../../proj/tl_common.h"
#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"
#include "../common/rf_frame.h"
#include "dongle_usb.h"
#include "dongle_custom.h"

#if(USB_DESCRIPTER_CONFIGURATION_FOR_KM_DONGLE)  //for km dongle customization (add by sihui)

/*************************************************
 *  header   :   9
 *  mouse    :  25
 *  keyboard :  25
 *  printer  :  23
 *************************************************/

#define DESCRIPTOR_HEADER_LENGTH    9
#define DESCRIPTOR_MOUSE_LENGTH    	25
#define DESCRIPTOR_KEYBOARD_LENGTH  25
#define DESCRIPTOR_PRINTER_LENGTH   23


u8 configuration_km_desc[82] = {    //len = 9
							//USB_Descriptor_Configuration_Header_t
									0x09,			//sizeof(USB_Descriptor_Configuration_Header_t)
									0x02,  			//type= DTYPE_Configuration
									0x09,0x00,      //TotalConfigurationSize,   @@@@ will change @@@@
									0x03,           //TotalInterfaces           @@@@ will change @@@@
									1,      		//Configuration index
									0,      		//ConfigurationStrIndex = NO_DESCRIPTOR
									0xa0,           //ConfigAttributes = USB_CONFIG_ATTR_RESERVED | USB_CONFIG_ATTR_REMOTEWAKEUP
									0x19           	//MaxPowerConsumption = USB_CONFIG_POWER_MA(50)  MaxPower = 100mA
};

u8 configuration_desc_mouse[25] =  {       //len = 9 + 9 + 7 = 25
							//USB_Descriptor_Interface_t mouse_interface
									0x09, 			 //size = sizeof(USB_Descriptor_Interface_t)
									0x04, 	  		 //type = DTYPE_Interface
									2,               //InterfaceNumber          @@@@ will change @@@@
									0,     			 //AlternateSetting = 0
									1,  			 //TotalEndpoints = 1
									0x03,			 //Class = HID_CSCP_HIDClass
									0x01,        	 //SubClass = HID_CSCP_BootSubclass,
									0x02,			 //Protocol = HID_CSCP_MouseBootProtocol
									0,            	 //InterfaceStrIndex = NO_DESCRIPTOR
							//USB_HID_Descriptor_HID_t   mouse_hid
									0x09, 			 //size = sizeof(USB_HID_Descriptor_HID_t)
									0x21,       	 //type = HID_DTYPE_HID
									0x11,0x01,       //HIDSpec = 0x0111
									0x21, 			 //CountryCode = USB_HID_COUNTRY_US
									1,               //TotalReportDescriptors
									0x22,            //HIDReportType = HID_DTYPE_Report
									0x8e, 0x00,      //HIDReportLength[2] = {sizeof(mouse_report_desc), 0x00}
							//USB_Descriptor_Endpoint_t   mouse_in_endpoint
									0x07, 			//size = sizeof(USB_Descriptor_Endpoint_t)
									0x05,     		//type = DTYPE_Endpoint
									0x82,           //EndpointAddress = ENDPOINT_DIR_IN | USB_EDP_MOUSE,
									0x03,   		//Attributes = EP_TYPE_INTERRUPT
									0x08,0x00, 	 	//EndpointSize = 0x0008
									4    			//PollingIntervalMS = USB_MOUSE_POLL_INTERVAL
};

u8 configuration_desc_keyboard[25] = {      //len = 9 + 9 + 7 = 25
							//USB_Descriptor_Interface_t keyboardInterface
									0x09, 			 //size = sizeof(USB_Descriptor_Interface_t)
									0x04, 	 		 //type = DTYPE_Interface
									1,               //InterfaceNumber		    @@@@ will change @@@@
									0,     			 //AlternateSetting = 0
									1,  			 //TotalEndpoints = 1
									0x03,			 //Class = HID_CSCP_HIDClass
									0x01,            //SubClass = HID_CSCP_BootSubclass,
									0x01,            //Protocol = HID_CSCP_KeyboardBootProtocol,
									0,           	 //InterfaceStrIndex = NO_DESCRIPTOR
							//USB_HID_Descriptor_HID_t   keyboard_hid
									0x09,     		 //size = sizeof(USB_HID_Descriptor_HID_t)
									0x21,            //type = HID_DTYPE_HID
									0x11, 0x01, 	 // HIDSpec = 0x0111
									0x21, 			 //CountryCode = USB_HID_COUNTRY_US
									1, 		  		 //TotalReportDescriptors
									0x22, 			 //HIDReportType = HID_DTYPE_Report
									0x3b, 0x00,      //HIDReportLength[2] = {sizeof(keyboard_report_desc), 0x00}
							//USB_Descriptor_Endpoint_t  keyboard_in_endpoint
									0x07, 			 //size = sizeof(USB_Descriptor_Endpoint_t)
									0x05, 	    	 //type = DTYPE_Endpoint
									0x81,            //EndpointAddress = ENDPOINT_DIR_IN | USB_EDP_KEYBOARD_IN,
									0x03,    		 //Attributes = EP_TYPE_INTERRUPT
									0x08,0x00,		 //EndpointSize = 0x0008
									0x0a   		     //PollingIntervalMS = USB_KEYBOARD_POLL_INTERVAL = 10
};

u8 configuration_desc_printer[25] = {   //len = 9 + 7 + 7 = 23
							// USB_Descriptor_Interface_t   printer_interface
									0x09,     		//size = sizeof(USB_Descriptor_Interface_t)
									0x04,           //type = DTYPE_Interface
									0,				//InterfaceNumber           @@@@ will change @@@@
									0,            	//AlternateSetting = 0
									2,       		//TotalEndpoints = 2
									0x07,  			//Class = PRNT_CSCP_PrinterClass
									0x01,  			//SubClass = PRNT_CSCP_PrinterSubclass
									0x02,  			//Protocol = PRNT_CSCP_BidirectionalProtocol,// bInterfaceProtocol
									0,      		//InterfaceStrIndex = NO_DESCRIPTOR
							// USB_Descriptor_Endpoint_t   printer_in_endpoint
									0x07,      		//size = sizeof(USB_Descriptor_Endpoint_t)
									0x05,  			//type = DTYPE_Endpoint
									0x88,  			//EndpointAddress = ENDPOINT_DIR_IN | USB_EDP_PRINTER_IN
									0x02,  			//Attributes  = EP_TYPE_BULK
									0x40,0x00,      //EndpointSize = 0x0040
									0,              //PollingIntervalMS
							// USB_Descriptor_Endpoint_t  printer_out_endpoint
									0x07, 			 //size = sizeof(USB_Descriptor_Endpoint_t)
									0x05,     		 //type = DTYPE_Endpoint
									5, 				 //EndpointAddress = USB_EDP_PRINTER_OUT
									0x02,            //Attributes = EP_TYPE_BULK
									0x40,0x00,       //EndpointSize = 0x0040
									0	             //PollingIntervalMS
};

u8   interface_count;
u8   mouse_interface_number;
u8   keyboard_interface_number;
u8   printer_interface_number;

void  configuration_desc_add_device(u8 *device_desc_ptr,int device_desc_len)
{
	int i;
	int	total_desc_len = configuration_km_desc[2];

	for(i=0;i<device_desc_len;i++){
		configuration_km_desc[total_desc_len+i] = device_desc_ptr[i];
	}
	configuration_km_desc[2] = total_desc_len + device_desc_len;
}




void get_usb_descripter_configuration(void)
{
	mouse_interface_number = 0xff;
	keyboard_interface_number = 0xff;
	printer_interface_number = 0xff;

	interface_count = 0;

	if(dongle_support_mouse_enable){
		if(p_custom_cfg->report_type != U8_MAX){  //for report rate 250
			configuration_desc_mouse[24] = 2;
		}
		mouse_interface_number = interface_count;
		configuration_desc_mouse[2] = mouse_interface_number;
		interface_count ++;
		configuration_desc_add_device(configuration_desc_mouse,DESCRIPTOR_MOUSE_LENGTH);
	}

#if USB_KEYBOARD_ENABLE
	if(dongle_support_keyboard_enable){
		keyboard_interface_number = interface_count;
		configuration_desc_keyboard[2] = keyboard_interface_number;
		interface_count ++;
		configuration_desc_add_device(configuration_desc_keyboard,DESCRIPTOR_KEYBOARD_LENGTH);
	}
#endif

#if(USB_PRINTER_ENABLE)
	printer_interface_number = interface_count;
	configuration_desc_printer[2] = printer_interface_number;
	interface_count ++;
	configuration_desc_add_device(configuration_desc_printer,DESCRIPTOR_PRINTER_LENGTH);
#endif

	configuration_km_desc[4] = interface_count;
}


#endif





#if(USB_ID_AND_STRING_CUSTOM)

u8 device_desc_km[18] = {
		0x12,  		// sizeof(USB_Descriptor_Device_t)
		0x01,  		// Type = DTYPE_Device
		0x10,0x01, 	// 0x0110 : USBSpecification, USB 1.1
		0x00,       // Class : USB_CSCP_NoDeviceClass
		0x00, 		// SubClass : USB_CSCP_NoDeviceSubclass
		0x00, 		// Protocol : USB_CSCP_NoDeviceProtocol
		8,          // Endpoint0Size, Maximum Packet Size for Zero Endpoint. Valid Sizes are 8, 16, 32, 64
		0x8A,0x24,  // VendorID 蘇硉 0x248a   8  9
		0x66,0x83,  // ProductID defult value   mouse only:8366  mouse_keyboard_kit:8367
		0x00,0x01, 	// 0x0100 : ReleaseNumber
		1, 			// ManufacturerStrIndex = USB_STRING_VENDOR   @@@ pay attention
		2,			// ProductStrIndex = USB_STRING_PRODUCT		  @@@ pay attention
		0, 			// SerialNumStrIndex
		1           // NumberOfConfigurations
};


KM_USB_Descriptor_String_t  vendor_desc_km = {
			14,               //sizeof(L"Telink")=14
			0x03,			  //DTYPE_String=0x03
			L"Telink"
};

KM_USB_Descriptor_String_t  prodct_desc_km = {
			36,               //sizeof(L"Wireless Receiver")=36
			0x03,			  //DTYPE_String=0x03
			L"Wireless Receiver"
};

KM_USB_Descriptor_String_t  serial_desc_km = {
			18,               //sizeof(L"TLSR8566")=18
			0x03,			  //DTYPE_String=0x03
			L"TLSR8366"
};

void set_string_from_addr(KM_USB_Descriptor_String_t *ptr, u16 addr)
{
	u8 *p;

#if(CUSTOM_DATA_ERR_FIX_ENABLE)
	for(int i=0;i<CUSTOM_DATA_MAX_COUNT;i++){
		p = (u8 *)(addr+(i<<5));
		if(*p != 0){
			break;
		}
	}
#else
	p = (u8 *)(addr);
#endif

	int j = 0;
	if(*p != 0xff){
		while(*p != 0xff){
			ptr->str[j++] = *p++;
		}
		ptr->str[j] = 0;
		ptr->Size = (j+1)<<1;
	}

	/*
	u8 len;
	u8 *p = (u8 *)(addr);
	if(*p != U8_MAX){
		len = (*p++)%MAX_STRING_LEN;
		ptr->Size = (len+1)<<1;
		for(int i=0;i<len;i++){
			ptr->str[i] = *p++;
		}
		ptr->str[len] = 0;
	}
	*/
}

void get_usb_id_and_string(u16 vid,u16 pid)
{
	if(vid != U16_MAX){
		device_desc_km[8] =  vid&0xff;
		device_desc_km[9] =  (vid>>8)&0xff;
	}

	if(pid != U16_MAX){
		device_desc_km[10] =  pid&0xff;
		device_desc_km[11] =  (pid>>8)&0xff;
	}
#if USB_KEYBOARD_ENABLE
	else if(dongle_support_keyboard_enable){  //set default pid    mouse only:8366  mouse_keyboard_kit:8367
		device_desc_km[10] +=1;
	}
#endif

#if(USB_PRINTER_ENABLE)
	device_desc_km[10] +=2;
#endif

	set_string_from_addr(&vendor_desc_km,VENDOR_STRING_ADDR);
	set_string_from_addr(&prodct_desc_km,PRODCT_STRING_ADDR);
	set_string_from_addr(&serial_desc_km,SERIAL_STRING_ADDR);
}



#endif






void usb_data_report_proc(void)
{
	if ((reg_irq_src & FLD_IRQ_USB_PWDN_EN))
	{
		return;
	}

	extern void usbmouse_report_frame(void);
	extern void usbmouse_release_check(void);
	if(dongle_support_mouse_enable)
	{
		usbmouse_report_frame();
		usbmouse_release_check();
	}

#if USB_KEYBOARD_ENABLE
	extern void usbkb_report_frame(void);
	extern void usbkb_release_check(void);
	if(dongle_support_keyboard_enable)
	{
		usbkb_report_frame();
		usbkb_release_check();
	}
#endif
}

