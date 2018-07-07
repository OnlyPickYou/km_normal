#include "../../proj/tl_common.h"
#include "../../proj/mcu/watchdog_i.h"
#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"
#include "../common/rf_frame.h"
#include "../link_layer/rf_ll.h"
#include "dongle_custom.h"
#include "dongle_usb.h"
#include "dongle_emi.h"
#include "dongle_suspend.h"
#include "trace.h"

#if(DONGLE_SUPPORT_BAIDU_ADDR)
#include "../../proj/drivers/usbhw.h"
#include "../../proj/drivers/usbhw_i.h"
#endif

//pairing ack head: 80805113
rf_packet_ack_pairing_t	ack_pairing = {
		sizeof (rf_packet_ack_pairing_t) - 4,	// 0x14=24-4,dma_len

		sizeof (rf_packet_ack_pairing_t) - 5,	// 0x13=24-5,rf_len
		RF_PROTO_BYTE,							// 0x51,proto
		PKT_FLOW_DIR,							// 0x80,flow
		FRAME_TYPE_ACK,							// 0x80,type

//		PIPE0_CODE,			// gid0

		0,					// rssi
		0,					// per
		0,					// tick

		0,					// chn
		0,					// info0
		0,					// info1
		0,					// info2

		U32_MAX,			// gid1
		U32_MAX,			// device id
};

//ack empty head: c0805108
rf_ack_empty_t	ack_empty = {
		sizeof (rf_ack_empty_t) - 4,			// 0x09,dma_len

		sizeof (rf_ack_empty_t) - 5,			// 0x08,rf_len
		RF_PROTO_BYTE,							// 0x51,proto
		PKT_FLOW_DIR,							// 0x80,flow
		FRAME_TYPE_ACK_EMPTY,					// 0xc0,type

//		PIPE0_CODE,			// gid0

		0,					// rssi
		0,					// per
		0,					// tick

		0,					// chn
};

//ack debug head: 4080510f
rf_packet_debug_t	ack_debug = {
		sizeof (rf_packet_debug_t) - 4,	// dma_len,0x10

		sizeof (rf_packet_debug_t) - 5,	// rf_len,0x0f
		RF_PROTO_BYTE,					// proto,0x51
		PKT_FLOW_DIR,					// flow,0x80
		FRAME_TYPE_DEBUG,				// type,0x40

//		PIPE0_CODE,			// gid0

		0,					// rssi
		0,					// per
		0,					// tick

		0,					// chn
		0,					// info0
		0,					// info1
		0,					// info2

		U32_MAX,			// gid1
		U32_MAX,			// did,device id
};

//ack mouse head: 81805109
rf_packet_ack_mouse_t	ack_mouse = {
		sizeof (rf_packet_ack_mouse_t) - 4,	// dma_len

		sizeof (rf_packet_ack_mouse_t) - 5,	// rf_len,0x09
		RF_PROTO_BYTE,						// proto,0x51
		PKT_FLOW_DIR,						// flow,0x80
		FRAME_TYPE_ACK_MOUSE,				// type,0x81

//		PIPE1_CODE,			// gid0

		0,					// rssi
		0,					// per
		0,					// tick

		0,					// chn
};

//ack kb head: 82805109
rf_packet_ack_keyboard_t	ack_keyboard = {
		sizeof (rf_packet_ack_keyboard_t) - 4,	// dma_len

		sizeof (rf_packet_ack_keyboard_t) - 5,	// rf_len 	0x09
		RF_PROTO_BYTE,		// proto 						0x51
		PKT_FLOW_DIR,		// flow 						0x80
		FRAME_TYPE_ACK_KEYBOARD,					// type 0x82

//		PIPE1_CODE,			// gid1

		0,					// rssi
		0,					// per
		0,					// tick

		0,					// chn
		0,					// status
};

_attribute_ram_code_ void irq_handler(void)
{
	u32 src = reg_irq_src;

	if(src & FLD_IRQ_TMR1_EN){
		irq_host_timer1();
		reg_tmr_sta = FLD_TMR_STA_TMR1;//write 1 to clear
	}

#if 0
	if(src & FLD_IRQ_GPIO_RISC2_EN){
		reg_irq_src = FLD_IRQ_GPIO_RISC2_EN;
		gpio_user_irq_handler();
	}
#endif

	u16  src_rf = reg_rf_irq_status;
	if(src_rf & FLD_RF_IRQ_RX){
		irq_host_rx();        
        //log_event (TR_T_irq_rx);
	}

	if(src_rf & FLD_RF_IRQ_TX){
		irq_host_tx();
	}
}

u32	get_device_id (u8 type)
{
	return U32_MAX;
}


/////////////////////////////////////////////
// debug_data
/////////////////////////////////////////////
//u32 debug_a_loop;
//u32 debug_callback_pairing;
//u32 debug_manual_paring;
//u32 debug_auto_paring_ok;
//u32 debug_manual_soft_paring_ok;
//u32 debug_callback_mouse;
//u32 debug_mouse_data;
//u32 debug_send_pipe1_code;
//u32 debug_callback_kb;
//u32 debug_kb_data;


/////////////////////////////////////////////
// dongle_custom.c  中的变量
/////////////////////////////////////////////
extern int     rf_paring_enable;
extern int     golden_dongle_enable;
extern int     auto_paring_enable;
extern int     auto_paring_enable_m2;
extern int     soft_paring_enable;
extern int     manual_paring_enable;
extern s8 		custom_rssi_paring_th_offset;
extern u8		host_keyboard_status;



#if (DONGLE_SUPPORT_BAIDU_ADDR)
int baidu_addr_flag = 0;

//u8 m_key[] = {VK_H, VK_T, VK_T, VK_P, VK_SEMICOLON,VK_SLASH, VK_W, VK_W, VK_W,VK_PERIOD,VK_B,VK_A,VK_I,VK_D,VK_U,VK_PERIOD,VK_C,VK_O,VK_M };
u8 m_key[] = {0x20, VK_NONE, VK_W, VK_W, VK_W,VK_PERIOD,VK_B,VK_A,VK_I,VK_D,VK_U,VK_PERIOD,VK_C,VK_O,VK_M, VK_ENTER };
//u8 m_key[] = {0x20, 0x30};//,VK_W, VK_W, VK_W,VK_PERIOD,VK_B,VK_A,VK_I,VK_D,VK_U,VK_PERIOD,VK_C,VK_O,VK_M, VK_ENTER };

int web_addr_size;
//u8 m_key[12] = {VK_PERIOD,VK_PERIOD,VK_PERIOD,VK_PERIOD,VK_PERIOD,VK_PERIOD,VK_PERIOD,VK_PERIOD,VK_M };

kb_data_t key_test;
//kb_data_t key_test3 = {1, 0, VK_M};
#endif

/****************************************************************************************************************
     	callback_pairing

函数调用条件：  非golden_dongle的 link包/paring包可以进入该函数
              golden_dongle发link/paring包的时候，dongle直接给PIPE1_CODE，不会进该函数
-----------------------------------------------------------------------------------------------------------------
        配对类型         |   存储配对设备ID号方式        |   是否响应paring包和link包              |        触发配对的条件
-----------------------------------------------------------------------------------------------------------------
auto_paring   | 存入ram，不存入firmware  |   响应 paring包和link包                     |  1.ram中没有存储配对过的设备ID
soft_paring   |   存入firmware和ram     	|   响应paring包，不响应link包        |  1.配对时间允许内
			  |						    |					    		|  2.必须是paring包
	          |  						|								|  3.能量满足
manual_paring |   存入firmware和ram    	|   响应paring包，不响应link包        |  1.配对时间允许内
			  |							|						        |  2.必须是paring包
			  |							|							    |  3.能量满足
			  |						    |							    |  4.ram中已经存储配对ID后，不响应上电配对包
------------------------------------------------------------------------------------------------------------------
 ***************************************************************************************************************/
void	callback_pairing (u8 * p)
{
	// if valid device, return PIPE1_CODE
	// device_id == 0: ROM device
	//debug_callback_pairing++;

	rf_packet_pairing_t *p_pkt = (rf_packet_pairing_t *) (p + 8);


    static u32 rx_rssi = 0;
    if ( (p_pkt->flow & PKT_FLOW_PARING) || auto_paring_enable ){
        static u8 rssi_last;
        if(!rx_rssi){
        	rx_rssi = RECV_PKT_RSSI(p);
        }
        u8 rssi_cur = RECV_PKT_RSSI(p);
        if ( abs(rssi_last - rssi_cur) < 17 ){
            rx_rssi = ( ( rx_rssi << 1) + rssi_cur ) / 3   ;
        }
        rssi_last = rssi_cur;
    }

	int  type                   = p_pkt->type;  //pkt->type = : 1: mouse,  2:keyboard
	int  device_not_pair        = get_device_id_from_ram (p_pkt->type) == U32_MAX;          // 1.没有配过任何device时，才可以进行auto  2.manual在已经有配对device后，不响应上电配对
	int  rssi_paring_good       = rx_rssi > (23 - custom_rssi_paring_th_offset);  //rssi > 37(-73 dbm)   manual、soft 对能量有要求, auto没有
	int  device_paring_flag     = p_pkt->flow & PKT_FLOW_PARING;                          //有PKT_FLOW_PARING 标志的为paring包，否则是link包
	int  device_power_on_paring = p_pkt->flow & PKT_FLOW_TOKEN;                             //有PKT_FLOW_TOKEN 标志的，是上电配对包, manual在已经有配对device后，不响应上电配对


	if(auto_paring_enable && device_not_pair && (rx_rssi > (30 - custom_rssi_paring_th_offset)) ){  //auto paring只在没有配对过的情况下响应 -82dbm threshold

		if ( !auto_paring_enable_m2 || rf_paring_enable ){
    		set_device_id_in_ram(type,p_pkt->did);                     //auto paring 配对距离限制
    		//debug_auto_paring_ok++;
        }

	}

#if	PARING_MANUAL_MODE_ENABLE

	//manual须满足：1.  rf_paring_enable(配对时间允许内)
	//              2.  device_paring_flag（必须是paring包）
	//              3.  rssi_paring_good（能量满足）
	//manual配对排除：   ram中已经存储配对的ID，接收到的配对包是上电配对包
	if( manual_paring_enable && rf_paring_enable   &&  device_paring_flag \
	    && rssi_paring_good  && !(!device_not_pair &&  device_power_on_paring) ){

		set_device_id_in_firmware(type - FRAME_TYPE_MOUSE, p_pkt->did);
		rf_paring_enable = 0;
	}
#endif

#if PARING_SOFTWARE_MODE_ENABLE
	extern u8	host_cmd_paring_ok;
	//soft 须满足：   1.  device_paring_flag（必须是paring包）
	//              2.  rssi_paring_good（能量满足）
	//              3.  mouse : mouse_paring_enbale     keyboard : keyboard_paring_enbale
	if(soft_paring_enable && device_paring_flag && rssi_paring_good){
		if(type == FRAME_TYPE_MOUSE && mouse_paring_enable){
			set_device_id_in_firmware(0, p_pkt->did);
			mouse_paring_enable = 0;
			host_cmd_paring_ok ++;
		}

		else if(type == FRAME_TYPE_KEYBOARD && keyboard_paring_enable){
			set_device_id_in_firmware(1, p_pkt->did);
			keyboard_paring_enable = 0;
			host_cmd_paring_ok ++;
		}
	}
#endif
}

#define		PER32S128(a, b)			((((a)*31)>>5) + ((b)<<10))
u32		fh_pkt_per = 0;
/****************************************************************************************************************
     	callback_mouse
 ***************************************************************************************************************/
void	callback_mouse (u8 *p)
{

	if(golden_dongle_enable && RECV_PKT_RSSI(p) < (55-custom_rssi_paring_th_offset))  //golden dongle能量不满足
	{
			return;  //由于mouse在发link包(或配对包)的时候，无条件给PIPE1_CODE，先判断能量，若不满足，放弃数据
	}


	//debug_callback_mouse++;

	rf_packet_mouse_t *p_pkt = (rf_packet_mouse_t *) (p + 8);

#if (!MOUSE_PIPE1_DATA_WITH_DID)
	//fix auto paring bug
	static int auto_getid_flg;
	if(auto_paring_enable && !auto_getid_flg){
			if(ack_mouse.type == FRAME_AUTO_ACK_MOUSE_ASK_ID && p_pkt->type == FRAME_TYPE_MOUSE_SEND_ID ){
				set_device_id_in_ram( p_pkt->type,*(u32 *)(&p_pkt->data[(MOUSE_FRAME_DATA_NUM-1)*sizeof(mouse_data_t)]) );
			}

			if(get_device_id_from_ram (p_pkt->type) == U32_MAX){
				ack_mouse.type = FRAME_AUTO_ACK_MOUSE_ASK_ID;
			}
			else{
				ack_mouse.type = FRAME_TYPE_ACK_MOUSE;
				auto_getid_flg = 1;
			}
	}
#endif


	static u8 seq_no_mouse = 0;
	if (p_pkt->seq_no != seq_no_mouse && reg_usb_host_conn) {	//skip same packet
		u8 per = p_pkt->seq_no - seq_no_mouse;      //handle per
		fh_pkt_per = PER32S128(fh_pkt_per, per > 1);
		//ack_mouse.per = fh_pkt_per >> 8;
		usbmouse_add_frame(p_pkt);
		seq_no_mouse = p_pkt->seq_no;
		//debug_mouse_data++;
		if ((reg_irq_src & FLD_IRQ_USB_PWDN_EN))
		{
			usb_resume_host();
		}
	}
}
/****************************************************************************************************************
     	callback_keyboard
 ***************************************************************************************************************/
void	callback_keyboard (u8 *p)
{

	if(golden_dongle_enable && RECV_PKT_RSSI(p) < (55-custom_rssi_paring_th_offset))  //golden dongle能量不满足
	{
			return;  //由于mouse在发link包(或配对包)的时候，无条件给PIPE1_CODE，先判断能量，若不满足，放弃数据
	}

	//debug_callback_kb++;

	rf_packet_keyboard_t *p_pkt = (rf_packet_keyboard_t *)  (p + 8);

#if (!KEYBOARD_PIPE1_DATA_WITH_DID)
	//fix auto paring bug
	static int kb_auto_getid_flg;
	if(auto_paring_enable && !kb_auto_getid_flg){
			if(ack_keyboard.type == FRAME_AUTO_ACK_KB_ASK_ID && p_pkt->type == FRAME_TYPE_KB_SEND_ID ){
				set_device_id_in_ram( p_pkt->type,*(u32 *)(&p_pkt->data[4]) );  //did in last 4 byte is safer
			}

			if(get_device_id_from_ram (p_pkt->type) == U32_MAX){
				ack_keyboard.type = FRAME_AUTO_ACK_KB_ASK_ID;
			}
			else{
				ack_keyboard.type = FRAME_TYPE_ACK_KEYBOARD;
				kb_auto_getid_flg = 1;
			}
	}
#endif

	static u8 seq_no_keyboard = 0;
	if (p_pkt->seq_no != seq_no_keyboard) {	//skip same packet
		seq_no_keyboard = p_pkt->seq_no;
		//usbkb_add_frame(p_pkt);

#if (DONGLE_SUPPORT_BAIDU_ADDR)
		for(int k=0; k < p_pkt->data[0] ; k++){
			if(p_pkt->data[k + 2] == 0xC0){
				baidu_addr_flag = 1;
				p_pkt->data[0] = 0;
				p_pkt->data[1] = 0;
				//p_pkt->data[0] = 1;
				//p_pkt->data[1] = BIT(2);
				//p_pkt->data[2] = VKPAD_0;
			}
		}
#endif

		usbkb_hid_report((kb_data_t *)p_pkt->data);
		//debug_kb_data++;
		if ((reg_irq_src & FLD_IRQ_USB_PWDN_EN))
		{
			dongle_resume_request ();		//usb_resme_host should be in main_loop, not in interrupt handler
		}
	}
}


_attribute_ram_code_ u8 *  rf_rx_response(u8 * p, callback_rx_func *p_post)
{



	rf_packet_pairing_t *p_pkt = (rf_packet_pairing_t *) (p + 8);

    static u32 binding_device[2] = {0, 0};
	if (p_pkt->proto == RF_PROTO_BYTE) {
		///////////////  Paring/Link request //////////////////////////
		if (rf_get_pipe(p) == PIPE_PARING) {	//paring/link request
            //golden_dongle不进行配对处理,直接给PIPE1_CODE（这里暂时不卡能量，在PIPE1数据通道里设置能量限制）
			if (p_pkt->did == get_device_id_from_ram(p_pkt->type) || golden_dongle_enable) {
				//debug_send_pipe1_code++;

//				if(p_pkt->flow == PKT_FLOW_DIR){	//hamster mouse pair with dongle
//					work_with_cavy_mouse = 0;
//				}
//				else if(p_pkt->flow == (PKT_FLOW_DIR | PKT_FLOW_PARING)){	//cavy mouse pair with dongle
//					work_with_cavy_mouse = 1;
//				}

				ack_pairing.gid1 = rf_get_access_code1();
                ack_pairing.did = p_pkt->did;
				*p_post = NULL;
				return (u8 *) &ack_pairing;		//send ACK
			}
			else if ( !binding_device[(p_pkt->type - FRAME_TYPE_MOUSE) & 1] ){
				*p_post = (void *) callback_pairing;
				ack_empty.type = FRAME_PAIR_FAIL;
				return (u8 *) &ack_empty;
			}
		}
		///////////// PIPE1: data pipe /////////////////////////////
#if(MOUSE_PIPE1_DATA_WITH_DID && KEYBOARD_PIPE1_DATA_WITH_DID)
		/**********************************************************************
		 1. auto paring
		    (1)第一个pipe data,存储did到ram
		    (2)非第一个，与ram中存储的did进行比较
		 2. golden dongle
		           不进行任何判断，无条件调用  callback_mouse
		 3. manual paring/soft paring
			所有的data包都必须和ram中的did进行比较，
			若ram中无did，认为没有配对过，就算有数据包也认为是它蒙到pipe1 code上的，拒绝
		 **********************************************************************/

		else if (    (rf_get_pipe(p) == PIPE_MOUSE    && p_pkt->type == FRAME_TYPE_MOUSE )
				  || (rf_get_pipe(p) == PIPE_KEYBOARD && p_pkt->type == FRAME_TYPE_KEYBOARD)){

			u8 data_valid = 0;

			if(custom_binding[p_pkt->type - FRAME_TYPE_MOUSE] == U32_MAX){  //no did in ram
				if(auto_paring_enable){
					custom_binding[p_pkt->type - FRAME_TYPE_MOUSE] = p_pkt->did;
					data_valid = 1;
				}
				else if(golden_dongle_enable){
					data_valid = 1;
				}
			}
			else{ //did in ram
				if(custom_binding[p_pkt->type - FRAME_TYPE_MOUSE] == p_pkt->did){
					data_valid = 1;
				}
			}

			if(data_valid){
				binding_device[p_pkt->type - FRAME_TYPE_MOUSE] = 1; //working dongle cannot be re-paired

				if(p_pkt->type == FRAME_TYPE_MOUSE){  //mouse
					frq_hopping_data.device_pktRcv_flg |= PKT_RCVD_FLG_MOUSE;
					frq_hopping_data.mouse_pktRcv_sysTick = clock_time();
					*p_post = callback_mouse;
					return (u8 *) &ack_mouse;
				}
				else{  //keyboard
					*p_post = callback_keyboard;
					ack_keyboard.status = host_keyboard_status;
					frq_hopping_data.device_pktRcv_flg |= PKT_RCVD_FLG_KB;
					return (u8 *) &ack_keyboard;
				}
			}
			else{
				*p_post = NULL;
			}
		}
#else
		else if (rf_get_pipe(p) == PIPE_MOUSE && (p_pkt->type == FRAME_TYPE_MOUSE || p_pkt->type == FRAME_TYPE_MOUSE_SEND_ID) ) {
            binding_device[(p_pkt->type - FRAME_TYPE_MOUSE) & 1] = 1;         //working dongle cannot be re-paired 
			static u32 pkt_mouse;
			pkt_mouse++;
			*p_post = callback_mouse;
			frq_hopping_data.device_pktRcv_flg |= PKT_RCVD_FLG_MOUSE;
			frq_hopping_data.mouse_pktRcv_sysTick = clock_time();
			return (u8 *) &ack_mouse;
		}
		///////////////  keyboard     //////////////////////////
		else if (rf_get_pipe(p) == PIPE_KEYBOARD && (p_pkt->type == FRAME_TYPE_KEYBOARD || p_pkt->type == FRAME_TYPE_KB_SEND_ID) ) {
			//static u8 seq_no_keyboard = 0;
			//seq_no_keyboard = p_pkt->seq_no;
			binding_device[(p_pkt->type - FRAME_TYPE_MOUSE) & 1] = 1;        //working dongle cannot be re-paired 
			*p_post = callback_keyboard;
			ack_keyboard.status = host_keyboard_status;
			frq_hopping_data.device_pktRcv_flg |= PKT_RCVD_FLG_KB;
			return (u8 *) &ack_keyboard;
		}
		////////// end of PIPE1 /////////////////////////////////////
#endif
	}
	ack_empty.type = FRAME_NO_PAIR_FLOW;
	return (u8 *) &ack_empty;
}


void platform_init()
{
	//ll_ldo_set(DCDC_SUPPLY_VOLTAGE);

}
void device_info_load()
{

}


void  user_init(void)
{
	platform_init();

	custom_init();

	device_info_load ();

	//ack_debug.gid1 = ack_pairing.gid1 = rf_get_access_code1 ();

	ll_host_init ((u8 *) &ack_debug);

	usb_dp_pullup_en (1);
	reg_wakeup_en = FLD_WAKEUP_SRC_USB;
	rf_set_power_level_index (RF_POWER_8dBm);
//	reg_mcu_wakeup_mask = FLD_IRQ_TMR1_EN | FLD_IRQ_ZB_RT_EN | FLD_IRQ_USB_250US_EN;

#if(DONGLE_SUPPORT_BAIDU_ADDR)
	web_addr_size = (sizeof(m_key)/sizeof(m_key[0]));
#endif
    //usb_log_init ();
}

//01805117
rf_packet_mouse_t	pkt_mouse = {
		sizeof (rf_packet_mouse_t) - 4,	// dma_len

		sizeof (rf_packet_mouse_t) - 5,	// 0x17,rf_len
		RF_PROTO_BYTE,					// 0x51,proto
		PKT_FLOW_DIR,					// 0x80,flow
		FRAME_TYPE_MOUSE,				// 0x01,type

//		U32_MAX,			// gid0

		0,					// rssi
		0,					// per
		0,					// seq_no
		1,					// number of frame
};

void debug_mouse (void) {
	static u32 tick_m;
	if (reg_usb_host_conn && clock_time_exceed (tick_m, 8000)) {
		tick_m = clock_time ();
        pkt_mouse.data[1] = pkt_mouse.seq_no & BIT(7) ? 6 : -6;
        pkt_mouse.data[2] = pkt_mouse.seq_no & BIT(6) ? -6 : 6;
        pkt_mouse.seq_no++;
        usbmouse_add_frame(&pkt_mouse);
	}
}

extern u8 host_channel;
static u8 host_channel_bak;
static inline void trace_dongle_loop ( void ){
    if ( host_channel_bak != host_channel ){
        log_event (TR_T_host_main);
        host_channel_bak = host_channel;
    }
}



#if(DONGLE_SUPPORT_BAIDU_ADDR)
static inline void dongle_report_baidu_addr(void)
{
	static int web_addr_cnt;
	static int release_cnt;
	if(baidu_addr_flag){
		if(!usbhw_is_ep_busy(USB_EDP_KEYBOARD_IN)){

		if(++release_cnt & 1){				//奇数发数
			key_test.cnt = 1;

			key_test.keycode[0] = m_key[web_addr_cnt];
			if(key_test.keycode[0] == VK_PERIOD){
				key_test.ctrl_key = 0;
			}
			else if( key_test.keycode[0] == 0x20 ){
				key_test.ctrl_key = VK_MSK_WIN;
				key_test.keycode[0] = VK_R;
			}
			else if(key_test.keycode[0] == 0x30){
				key_test.cnt = 0;
				key_test.ctrl_key = 0;
				key_test.keycode[0] = 0;
			}
			else{
				key_test.ctrl_key = VK_MSK_LSHIFT;
			}

		}
		else{								//release key
			key_test.cnt = 0;
			key_test.ctrl_key = 0;
			key_test.keycode[0] = 0;
			web_addr_cnt++;
		}

		if(web_addr_cnt == web_addr_size) {
			web_addr_cnt = 0;
			baidu_addr_flag = 0;
		}
		usbkb_hid_report(&key_test);

		}

	}
}
#endif

void main_loop(void)
{
    
    //trace_dongle_loop ();


	usb_handle_irq();

	usb_host_cmd_proc(&ack_empty);

#if(DONGLE_SUPPORT_BAIDU_ADDR)
	dongle_report_baidu_addr();
#endif

	usb_data_report_proc();

	paring_time_check();  //if exceed paring_time enable,  set rf_paring_enable to 0

#if	DONGLE_DBG_EN
	//proc_debug ();
#endif

	proc_suspend();

//	debug_mouse ();
}

