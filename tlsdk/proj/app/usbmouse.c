
#include "../tl_common.h"
#if(USB_MOUSE_ENABLE)

#include "usbmouse.h"
#include "../drivers/usb.h"
#include "../drivers/usbhw.h"
#include "../drivers/usbhw_i.h"
#include "../../vendor/common/rf_frame.h"

#ifndef	USB_MOUSE_REPORT_SMOOTH
#define	USB_MOUSE_REPORT_SMOOTH	   1
#endif


#define  USBMOUSE_BUFF_DATA_NUM    8
static mouse_data_t mouse_dat_buff[USBMOUSE_BUFF_DATA_NUM];

static u8  usbmouse_wptr, usbmouse_rptr;
static u32 usbmouse_not_released;
static u32 usbmouse_data_report_time;

abs_mouse_dat_t abs_mouse_dat = {0, 0, 0, 0};

int sm_sum_x, sm_pre_x, sm_sum_y, sm_pre_y;
void iir_smoother(signed char *data_x, signed char *data_y){
	sm_sum_x = sm_sum_x - sm_pre_x + *data_x;
	sm_pre_x = sm_sum_x/2;
	*data_x = sm_pre_x;

    sm_sum_y = sm_sum_y - sm_pre_y + *data_y;
	sm_pre_y = sm_sum_y/2;
	*data_y = sm_pre_y;
}

static inline void iir_smoother_clear( void ){
	sm_sum_x = 0;
	sm_pre_x = 0;

    sm_sum_y = 0;
	sm_pre_y = 0;
}

const u8 sm_dyn_pth1 = 6;
const u8 sm_dyn_pth2 = 4;
u8 adaptive_smoother(signed char *data_x, signed char *data_y ){
    static u8 asm_flg = 0;    
    static u32 sm_last_smoother_tick = 0;
    
    //auto clear asm sum when no data for a long time
    if ( asm_flg && clock_time_exceed(sm_last_smoother_tick, 100000) ){           
        asm_flg = 0;
        iir_smoother_clear ( );
    }
    if ( !asm_flg ){
        if ( (abs(*data_x) > sm_dyn_pth1) || (abs(*data_y) > sm_dyn_pth1) ){
            asm_flg = 1;
            iir_smoother ( data_x, data_y );
        }
        else{
            asm_flg = 0;
            iir_smoother_clear ( );
        }
    }
    else{
        if ( (abs(*data_x) < sm_dyn_pth2) && (abs(*data_y) < sm_dyn_pth2) ){
           asm_flg = 0;
           iir_smoother_clear ( );
        }
        else{
           asm_flg = 1;
           iir_smoother ( data_x, data_y );
        }
    }
    
    if ( asm_flg ){
        sm_last_smoother_tick = clock_time();
    }
    
    return asm_flg;
}


#if 0
u8 mouse_button_debounce( u8 input, u8 debouce_len){
    static u8 btn_bak_bak = 0;    
    static u8 btn_bak = 0;
    u8 btn = btn_bak_bak & btn_bak & input;
    btn_bak_bak = btn_bak;
    btn_bak = input;
    return btn;
}
#else
static inline u8 mouse_button_debounce(u8 btn_cur, u8 debouce_len){
    static u8 btn_cnt = 0;
    static u8 btn_bak = 0;
	if(	(btn_cnt >= debouce_len) && (btn_bak != btn_cur) ){
		btn_bak = btn_cur;
		btn_cnt = 0;
	}else{
		if(btn_bak != btn_cur)
			btn_cnt ++;
		else
			btn_cnt = 0;
	}
    return btn_bak;
}
#endif


#if (SIMULATE_CAVY_DONGLE_ENABLE)
void usbmouse_add_frame (rf_packet_cavy_mouse_t *packet_mouse)
#elif (__PROJECT_DONGLE_ENC_8366__)
void usbmouse_add_frame (rf_packet_mouse_enc_t *packet_mouse)
#else
void usbmouse_add_frame (rf_packet_mouse_t *packet_mouse)
#endif
{

	u8 *mdata_ptr;

	u8 new_data_num = packet_mouse->pno;  //根据pno 获得最新数据的个数
	for(u8 i=0;i<new_data_num;i++)
	{

#if(1)  //sensor data smooth
			mdata_ptr = (u8 *)(&packet_mouse->data[i*sizeof(mouse_data_t)]);
            if ( *(mdata_ptr+1) || *(mdata_ptr+2) ){
			    adaptive_smoother ( (mdata_ptr+1), (mdata_ptr+2) );
            }
			*(mdata_ptr+0) = mouse_button_debounce( *(mdata_ptr+0), MOUSE_BUTTON_DEBOUNCE - 1 );
			memcpy4((int*)(&mouse_dat_buff[usbmouse_wptr]), (int*)mdata_ptr, sizeof(mouse_data_t));
#else
			memcpy4((int*)(&mouse_dat_buff[usbmouse_wptr]), (int*)(&packet_mouse->data[i*sizeof(mouse_data_t)]), sizeof(mouse_data_t));
#endif
			BOUND_INC_POW2(usbmouse_wptr,USBMOUSE_BUFF_DATA_NUM);
			if(usbmouse_wptr == usbmouse_rptr)
			{
					BOUND_INC_POW2(usbmouse_rptr,USBMOUSE_BUFF_DATA_NUM);
					break;
			}
	}
}


void usbmouse_release_check(){
	if(usbmouse_not_released && clock_time_exceed(usbmouse_data_report_time, USB_MOUSE_RELEASE_TIMEOUT)){
	    u8 release_data[MOUSE_REPORT_DATA_LEN + 2] = {0};

	    if(usbmouse_hid_report(USB_HID_MOUSE, (u8*)(&release_data), MOUSE_REPORT_DATA_LEN+2)){
		    usbmouse_not_released = 0;
	    }
	}
}


void usbmouse_report_frame(){

#if 	USB_MOUSE_REPORT_SMOOTH
	static u32 tick = 0;
	static u32 btn_tick = 0;

	u16 y_pos_diff = 0;
	u16 x_pos_abs = 0;
	u16 y_pos_abs = 0;
	if(usbhw_is_ep_busy(USB_EDP_MOUSE)) {
			tick = clock_time ();
	}

	u8 diff = (usbmouse_wptr - usbmouse_rptr) & (USBMOUSE_BUFF_DATA_NUM - 1);
    //buff cycle
    if ( usbmouse_rptr > usbmouse_wptr )
        diff = usbmouse_rptr - usbmouse_wptr + (USBMOUSE_BUFF_DATA_NUM - 1);
	if (diff < 3 && !clock_time_exceed (tick, 5000)) {
		return;
	}
#endif

	if(usbmouse_wptr != usbmouse_rptr){
        u32 data = *(u32*)(&mouse_dat_buff[usbmouse_rptr]);	// that is   >  0

        abs_mouse_dat.btn = (u8)data;
        abs_mouse_dat.wheel = (data >> 24);

        s8 x_pos = (data >> 8) & 0xff;
        s8 y_pos = (data >> 16) & 0xff;


        if(data == 0x01)
        	btn_tick = !btn_tick ? (clock_time() | 1) : btn_tick;
        else
        	btn_tick = 0;

#if 1
        if( (x_pos & 0x80) == 0x80 ){
        	x_pos_abs = ~(x_pos - 1);
        	abs_mouse_dat.x -= (x_pos_abs << 4);
           	if( abs_mouse_dat.x > 0x7fff )
           		abs_mouse_dat.x = 0;
        }
        else{
        	x_pos_abs = x_pos;
        	abs_mouse_dat.x += (x_pos_abs << 4);
           	if( abs_mouse_dat.x >= 0x7fff )
           		abs_mouse_dat.x = 0x7fff;
        }

        if( (y_pos & 0x80) == 0x80){
        	y_pos_abs = ~(y_pos - 1);
        	abs_mouse_dat.y -= (y_pos_abs << 4);
          	if( abs_mouse_dat.y  > 0x7fff )
                		abs_mouse_dat.y = 0;
        }
        else{

        	if( btn_tick  ){ //长按左键200ms以上，向下移动
        		if(!clock_time_exceed(btn_tick, 125000)){
        			BOUND_INC_POW2(usbmouse_rptr,USBMOUSE_BUFF_DATA_NUM);
        			return;
        		}
    			btn_tick = 0;
    			y_pos_diff = (u16)(rand() & 0x1f) << 2;
    			abs_mouse_dat.y += y_pos_diff;
        	}
        	else
        	{
        		y_pos_abs = y_pos;
             	abs_mouse_dat.y += (y_pos_abs << 4);
        	}

          	if(  abs_mouse_dat.y >= 0x7fff )
                		abs_mouse_dat.y = 0x7fff;
        }
#else
        if( (x_pos & 0x80) == 0x80 ){
               	x_pos_abs = (u16)(~(x_pos - 1));
               	x_pos_abs = (x_pos_abs << 4) & 0x07f0 ;
               	if(abs_mouse_dat.x < x_pos_abs){
               		abs_mouse_dat.x = 0;
               	}
               	else{
               		abs_mouse_dat.x -= x_pos_abs;
               	}

        }
        else{
               	x_pos_abs = x_pos;
               	x_pos_abs = (x_pos_abs << 4) & 0x07f0;
               	abs_mouse_dat.x += x_pos_abs;
                  	if( abs_mouse_dat.x >= 0x7fff )
                  		abs_mouse_dat.x = 0x7fff;
        }


        if( (y_pos & 0x80) == 0x80){
            y_pos_abs = (u16)(~(y_pos - 1));
            y_pos_abs = (y_pos_abs << 4) & 0x07f0;
            if (abs_mouse_dat.y < y_pos_abs)
            		   abs_mouse_dat.y = 0;
            else
            		   abs_mouse_dat.y -= y_pos_abs;

        }
        else{
        	if( btn_tick && clock_time_exceed(btn_tick, 200000) ){ //长按左键200ms以上，向下移动
               	btn_tick = 0;
               	y_pos_diff = (u16)(rand() & 0x3f) * 8;
               	abs_mouse_dat.y += y_pos_diff;
            }
            else{
                y_pos_abs = y_pos;
                y_pos_abs = (y_pos_abs << 4) & 0x07f0;
                abs_mouse_dat.y += y_pos_abs;
            }
            if(  abs_mouse_dat.y >= 0x7fff )
            	abs_mouse_dat.y = 0x7fff;
       }

#endif
        int ret = usbmouse_hid_report(USB_HID_MOUSE,(u8*)(&abs_mouse_dat), MOUSE_REPORT_DATA_LEN + 2);
		if(ret){
            BOUND_INC_POW2(usbmouse_rptr,USBMOUSE_BUFF_DATA_NUM);
		}
		if(0 == data && ret){			//  successfully  release the key
			usbmouse_not_released = 0;
		}else{
			usbmouse_not_released = 1;
			usbmouse_data_report_time = clock_time();
		}
		abs_mouse_dat.y -= y_pos_diff;
	}
	return;
}


int usbmouse_hid_report(u8 report_id, u8 *data, int cnt){
	//unsigned char crc_in[8];
	//unsigned short crc;
	//unsigned int crch;

#if(0)
	static int  previous_busy_flag = 0;
	static u32	usb_busy_start_time;
#endif


    assert(cnt<8);

    if (!reg_usb_host_conn)
    {
    	reg_usb_ep_ctrl(USB_EDP_MOUSE) = 0;
    	return 0;
    }

	if(usbhw_is_ep_busy(USB_EDP_MOUSE)){
#if(0)
		if(!previous_busy_flag){   //previous  not  busy
			usb_busy_start_time = clock_time();
		}
		if(clock_time_exceed(usb_busy_start_time,5000000)){  //5s busy  reset
			write_reg8(0x80006f,0x20);  //reset
		}
		previous_busy_flag = 1;
#endif
		return 0;
	}

#if(0)
	previous_busy_flag = 0;  //not busy
#endif

	reg_usb_ep_ptr(USB_EDP_MOUSE) = 0;

	// please refer to usbmouse_i.h mouse_report_desc
	extern u8 usb_mouse_report_proto;

	if (!usb_mouse_report_proto) {
		reg_usb_ep_dat(USB_EDP_MOUSE) = data[0];
		reg_usb_ep_dat(USB_EDP_MOUSE) = data[1];
		reg_usb_ep_dat(USB_EDP_MOUSE) = data[2];
	}
	else {
		reg_usb_ep_dat(USB_EDP_MOUSE) = report_id;
		foreach(i, cnt){
			reg_usb_ep_dat(USB_EDP_MOUSE) = data[i];
		}
	}
	reg_usb_ep_ctrl(USB_EDP_MOUSE) = FLD_EP_DAT_ACK;		// ACK

	return 1;
}


void usbmouse_init(){
}

#endif
