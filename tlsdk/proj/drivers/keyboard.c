
#include "../tl_common.h"
#include "../drivers/usbkeycode.h"
#include "keyboard.h"

#include "../../vendor/keyboard_remington/kb_led.h"

//#include "../os/ev.h"
//#include "../os/sys.h"

//#include "../../vendor/common/keyboard_cfg.h"
//#include "../../vendor/common/custom.h"

#if (__PROJECT_KEYBOARD__ || __PROJECT_REMINGTON_KEYBOARD__ || __PROJECT_REMINGTON_KEYBOARD_TW__|| __PROJECT_REMINGTON_XJGD__ )
extern kb_status_t kb_status;
#endif



#if (defined(KB_DRIVE_PINS) && defined(KB_SCAN_PINS))


#if(MCU_CORE_TYPE == MCU_CORE_8368 || MCU_CORE_TYPE == MCU_CORE_8266)
u16 drive_pins[] = KB_DRIVE_PINS;	//8
u16 scan_pins[] = KB_SCAN_PINS;		//18
#else
u32 drive_pins[] = KB_DRIVE_PINS;	//8
u32 scan_pins[] = KB_SCAN_PINS;		//18
#endif

kb_data_t	kb_event;
STATIC_ASSERT(IMPLIES((!KB_ONLY_SINGLEKEY_SUPP), KB_RETURN_KEY_MAX > 1));


#ifndef		KB_MAP_DEFAULT
#define		KB_MAP_DEFAULT		1
#endif

#ifndef		KB_LINE_MODE
#define		KB_LINE_MODE		0
#endif

#ifndef		KB_LINE_HIGH_VALID
#define		KB_LINE_HIGH_VALID		1
#endif


#ifndef		KB_HAS_CTRL_KEYS
#define		KB_HAS_CTRL_KEYS		1
#endif

#ifndef		KB_ONLY_SINGLEKEY_SUPP
#define		KB_ONLY_SINGLEKEY_SUPP		0
#endif

#ifndef		KB_RM_GHOST_KEY_EN
#define		KB_RM_GHOST_KEY_EN		1
#endif

#ifndef		KB_HAS_FN_KEY
#define		KB_HAS_FN_KEY		1
#endif

#ifndef		KB_DRV_DELAY_TIME
#define		KB_DRV_DELAY_TIME		10
#endif

#ifndef		KB_KEY_VALUE_MERGE
#define		KB_KEY_VALUE_MERGE		1
#endif


typedef u8 kb_k_mp_t[ARRAY_SIZE(drive_pins)]; //typedef u8 kb_k_mp_t[8]

#if		KB_MAP_DEFAULT


#ifndef			KB_MAP_NORMAL
static const u8 kb_map_normal[ARRAY_SIZE(scan_pins)][ARRAY_SIZE(drive_pins)] = {
	{VK_PAUSE,	 VK_POWER,	  VK_EURO,		VK_SLEEP,	 	VK_RCTRL,	  VK_WAKEUP,	VK_CTRL,	    VK_F5},
	{VK_Q,		 VK_TAB,	  VK_A,			VK_ESC,		    VK_Z,		  VK_NCHG,	  	VK_TILDE,	    VK_1},
	{VK_W,		 VK_CAPITAL,  VK_S,	        VK_K45,	    	VK_X,		  VK_CHG,	  	VK_F1,			VK_2},
	{VK_E,		 VK_F3,		  VK_D,		    VK_F4,		    VK_C,		  VK_ROMA,	  	VK_F2,			VK_3},
	{VK_R,		 VK_T,		  VK_F,			VK_G,			VK_V,		  VK_B,		  	VK_5,			VK_4},
	{VK_U,		 VK_Y,		  VK_J,			VK_H,			VK_M,		  VK_N,		 	VK_6,			VK_7},
	{VK_I,		 VK_RBRACE,	  VK_K,		    VK_F6,		    VK_COMMA,	  VK_K56,	  	VK_EQUAL,		VK_8},
	{VK_O,	     VK_F7,  	  VK_L,   	    VK_RMB,	 		VK_PERIOD,    VK_APP, 	 	VK_F8,			VK_9},
	{VK_P,		 VK_LBRACE,	  VK_SEMICOLON, VK_QUOTE,		VK_BACKSLASH, VK_SLASH,	  	VK_MINUS,		VK_0},
	{VK_SCR_LOCK,VK_C9R1,	  VK_FN,	    VK_ALT,		    VK_MMODE,	  VK_RALT,	  	VK_C9R6,		VK_PRINTSCREEN},
	{VK_K14,	 VK_BACKSPACE,VK_BACKSLASH,	VK_F11,		    VK_ENTER,	  VK_F12,	  	VK_F9,			VK_F10},
	{VKPAD_7,	 VKPAD_4,	  VKPAD_1,	    VK_SPACE,		VK_NUM_LOCK,  VK_DOWN,	  	VK_DELETE,		VK_POWER},
	{VKPAD_8,	 VKPAD_5,	  VKPAD_2,	    VKPAD_0,		VKPAD_SLASH,  VK_RIGHT,	  	VK_INSERT,		VK_SLEEP},
	{VKPAD_9,    VKPAD_6,	  VKPAD_3,		VKPAD_PERIOD,	VKPAD_ASTERIX,VKPAD_MINUS,	VK_PAGE_UP,		VK_PAGE_DOWN},
	{VKPAD_PLUS, VK_K107,	  VKPAD_ENTER,  VK_UP,		    VK_PLAY_PAUSE,VK_LEFT,	  	VK_HOME,	    VK_END},
	{VK_WAKEUP,	 VK_SHIFT,	  VK_RSHIFT,	VK_VOL_DN,	    VK_VOL_UP,	  VK_NEXT_TRK,	VK_PREV_TRK,	VK_MEDIA},
	{VK_MAIL,	 VK_WIN,	  VK_W_FORWRD,	VK_W_STOP,		VK_W_BACK,	  VK_W_REFRESH,	VK_W_MUTE,    	VK_W_SRCH},
	{VK_KCL,	 VK_W_FAV,	  VK_RWIN,		VK_MY_COMP,		VK_STOP,	  VK_CAL,	  	VK_WEB,	    	VK_KCR},
};
#else
static const u8 kb_map_normal[ARRAY_SIZE(scan_pins)][ARRAY_SIZE(drive_pins)] = KB_MAP_NORMAL;
#endif

#ifndef			KB_MAP_NUM
static const u8 kb_map_num[ARRAY_SIZE(scan_pins)][ARRAY_SIZE(drive_pins)] = {
	{VK_PAUSE,	 VK_POWER,	  VK_EURO,		VK_SLEEP,	 	VK_RCTRL,	  VK_WAKEUP,	VK_CTRL,	    VK_F5},
	{VK_Q,		 VK_TAB,	  VK_A,			VK_ESC,		    VK_Z,		  VK_NCHG,	  	VK_TILDE,	    VK_1},
	{VK_W,		 VK_CAPITAL,  VK_S,	        VK_K45,	    	VK_X,		  VK_CHG,	  	VK_F1,			VK_2},
	{VK_E,		 VK_F3,		  VK_D,		    VK_F4,		    VK_C,		  VK_ROMA,	  	VK_F2,			VK_3},
	{VK_R,		 VK_T,		  VK_F,		    VK_G,			VK_V,		  VK_B,		  	VK_5,			VK_4},
	{VK_U,		 VK_Y,		  VK_J,		    VK_H,			VK_M,		  VK_N,		 	VK_6,			VK_7},
	{VK_I,		 VK_RBRACE,	  VK_K,		    VK_F6,		    VK_COMMA,	  VK_K56,	  	VK_EQUAL,		VK_8},
	{VK_O,	     VK_F7,  	  VK_L,   	    VK_RMB,	 		VK_PERIOD,    VK_APP, 	 	VK_F8,			VK_9},
	{VK_P,		 VK_LBRACE,	  VK_SEMICOLON, VK_QUOTE,		VK_BACKSLASH, VK_SLASH,	  	VK_MINUS,		VK_0},
	{VK_SCR_LOCK,VK_C9R1,	  VK_FN,	    VK_ALT,		    VK_MMODE,	  VK_RALT,	  	VK_C9R6,		VK_PRINTSCREEN},
	{VK_K14,	 VK_BACKSPACE,VK_BACKSLASH,	VK_F11,		    VK_ENTER,	  VK_F12,	  	VK_F9,			VK_F10},
	{VKPAD_7,	 VKPAD_4,	  VKPAD_1,	    VK_SPACE,		VK_NUM_LOCK,  VK_DOWN,	  	VK_DELETE,		VK_POWER},
	{VKPAD_8,	 VKPAD_5,	  VKPAD_2,	    VKPAD_0,		VKPAD_SLASH,  VK_RIGHT,	  	VK_INSERT,		VK_SLEEP},
	{VKPAD_9,	 VKPAD_6,	  VKPAD_3,	    VKPAD_PERIOD,	VKPAD_ASTERIX,VKPAD_MINUS,	VK_PAGE_UP,		VK_PAGE_DOWN},
	{VKPAD_PLUS, VK_K107,	  VKPAD_ENTER,  VK_UP,		    VK_PLAY_PAUSE,VK_LEFT,	  	VK_HOME,	    VK_END},
	{VK_WAKEUP,	 VK_SHIFT,	  VK_RSHIFT,	VK_VOL_DN,	    VK_VOL_UP,	  VK_NEXT_TRK,	VK_PREV_TRK,	VK_MEDIA},
	{VK_MAIL,	 VK_WIN,	  VK_W_FORWRD,	VK_W_STOP,		VK_W_BACK,	  VK_W_REFRESH,	VK_W_MUTE,    	VK_W_SRCH},
	{VK_KCL,	 VK_W_FAV,	  VK_RWIN,		VK_MY_COMP,		VK_STOP,	  VK_CAL,	  	VK_WEB,	    	VK_KCR},
};
#else
static const u8 kb_map_num[ARRAY_SIZE(scan_pins)][ARRAY_SIZE(drive_pins)] = KB_MAP_NUM;
#endif

#ifndef			KB_MAP_FN
static const u8 kb_map_fn[ARRAY_SIZE(scan_pins)][ARRAY_SIZE(drive_pins)] = {
	{VK_PAUSE,	 VK_POWER,	  VK_EURO,		VK_SLEEP,	 	VK_RCTRL,	  VK_WAKEUP,	VK_CTRL,	    VK_F5},
	{VK_Q,		 VK_TAB,	  VK_A,		    VK_ESC,		    VK_Z,		  VK_NCHG,	  	VK_TILDE,	    VK_1},
	{VK_W,		 VK_CAPITAL,  VK_S,	        VK_K45,	    	VK_X,		  VK_CHG,	  	VK_F1,			VK_2},
	{VK_E,		 VK_F3,		  VK_D,		    VK_F4,		    VK_C,		  VK_ROMA,	  	VK_F2,			VK_3},
	{VK_R,		 VK_T,		  VK_F,		    VK_G,			VK_V,		  VK_B,		  	VK_5,			VK_4},
	{VK_U,		 VK_Y,		  VK_J,		    VK_H,			VK_M,		  VK_N,		 	VK_6,			VK_7},
	{VK_I,		 VK_RBRACE,	  VK_K,		    VK_F6,		    VK_COMMA,	  VK_K56,	  	VK_EQUAL,		VK_8},
	{VK_O,	     VK_F7,  	  VK_L,   	    VK_RMB,	 		VK_PERIOD,    VK_APP, 	 	VK_F8,			VK_9},
	{VK_P,		 VK_LBRACE,	  VK_SEMICOLON, VK_QUOTE,		VK_BACKSLASH, VK_SLASH,	  	VK_MINUS,		VK_0},
	{VK_SCR_LOCK,VK_C9R1,	  VK_FN,	    VK_ALT,		    VK_MMODE,	  VK_RALT,	  	VK_C9R6,		VK_PRINTSCREEN},
	{VK_K14,	 VK_BACKSPACE,VK_BACKSLASH,	VK_LOCK,		VK_ENTER,	  VK_F12,	  	VK_F9,			VK_F10},
	{VKPAD_7,	 VKPAD_4,	  VKPAD_1,	    VK_SPACE,		VK_NUM_LOCK,  VK_DOWN,	  	VK_DELETE,		VK_POWER},
	{VKPAD_8,	 VKPAD_5,	  VKPAD_2,	    VKPAD_0,		VKPAD_SLASH,  VK_RIGHT,	  	VK_INSERT,		VK_SLEEP},
	{VKPAD_9,	 VKPAD_6,	  VKPAD_3,	    VKPAD_PERIOD,	VKPAD_ASTERIX,VKPAD_MINUS,	VK_PAGE_UP,		VK_PAGE_DOWN},
	{VKPAD_PLUS, VK_K107,	  VKPAD_ENTER,  VK_UP,		    VK_PLAY_PAUSE,VK_LEFT,	  	VK_HOME,	    VK_END},
	{VK_WAKEUP,	 VK_SHIFT,	  VK_RSHIFT,	VK_VOL_DN,	    VK_VOL_UP,	  VK_NEXT_TRK,	VK_PREV_TRK,	VK_MEDIA},
	{VK_MAIL,	 VK_WIN,	  VK_W_FORWRD,	VK_W_STOP,		VK_W_BACK,	  VK_W_REFRESH,	VK_W_MUTE,    	VK_W_SRCH},
	{VK_KCL,	 VK_W_FAV,	  VK_RWIN,		VK_MY_COMP,		VK_STOP,	  VK_CAL,	  	VK_WEB,	    	VK_KCR},

};

#else
static const u8 kb_map_fn[ARRAY_SIZE(scan_pins)][ARRAY_SIZE(drive_pins)] = KB_MAP_FN;
#endif


#define small_keyboard_enable  		0


#if(__PROJECT_REMINGTON_XJGD__)
kb_k_mp_t *	kb_p_map[1] = {
		kb_map_normal,
};
#else
kb_k_mp_t *	kb_p_map[4] = {
		kb_map_normal,
		kb_map_fn,
		kb_map_num,
		kb_map_fn,
};
#endif

kb_k_mp_t *kb_map_shift;


#if(__PROJECT_REMINGTON_XJGD__)
#else
kb_switch_key_t  *kb_switch_key[8];
#endif


#if(KEYBOARD_WIN_LOCK_EN)
u8 kb_win_lock_key_pressed = 0;
#endif
///////////////////////////////////////////////////////////////////////
//////////// load configuration from flash/OTP ////////////////////////
///////////////////////////////////////////////////////////////////////
#else //line 62

#define		small_keyboard_enable			0

kb_k_mp_t *	kb_p_map[8] = {
		kb_map,							kb_map,
		big_kb_map_numlock_on,			kb_map_fn_off_numlock_on,
		big_kb_map_fn_on,				kb_map_fn_on_numlock_off,
		big_kb_map_fn_on_numlock_on,	kb_map_fn_on_numlock_on,
};

#endif

u32	scan_pin_need;
int kb_is_lock_pressed = 0;
int kb_is_unlock_pressed = 0;

u8 kb_is_fn_pressed = 0;
u8 switch0_key_pressed = 0;

//u8 (*kb_k_mp)[ARRAY_SIZE(drive_pins)];
kb_k_mp_t * kb_k_mp;


#if (__PROJECT_REMINGTON_KEYBOARD_TW__)


static u32 kb_rmv_ghost_key(u32 * pressed_matrix)
{
	u32 mix_final = 0;
	int matrix_no_empty = 0;

	foreach_arr(i, drive_pins){
		for(int j = (i+1); j < ARRAY_SIZE(drive_pins); ++j){
			u32 mix = (pressed_matrix[i] & pressed_matrix[j]);
			// >=2 鏍圭嚎閲嶅悎,  閭ｅ氨鏄�ghost key
			//four or three key at "#" is pressed at the same time, should remove ghost key
			if( mix && (!BIT_IS_POW2(mix) || (pressed_matrix[i] ^ pressed_matrix[j])) ){
				// remove ghost keys
				//pressed_matrix[i] &= ~mix;
				//pressed_matrix[j] &= ~mix;
				mix_final |= mix;
			}
		}
		pressed_matrix[i] &= ~mix_final;


		if(pressed_matrix[i]){
			matrix_no_empty = 1;
		}

	}


	if(mix_final && !matrix_no_empty){
		return 1;
	}
	else{
		return 0;
	}
}

extern u32     sys_host_status;

static u32 key_debounce_filter(u32 mtrx_cur[], u32 filt_en )
{
	u32 kc = 0;
    static u32 mtrx_pre[ARRAY_SIZE(drive_pins)];
    static u32 mtrx_last[ARRAY_SIZE(drive_pins)];
    foreach_arr(i, drive_pins){
        u32 mtrx_tmp = mtrx_cur[i];
        if( filt_en ){
            //mtrx_cur[i] = (mtrx_last[i] ^ mtrx_tmp) ^ (mtrx_last[i] | mtrx_tmp);  //key_matrix_pressed is valid when current and last value is the same
            mtrx_cur[i] = ( ~mtrx_last[i] & (mtrx_pre[i] & mtrx_tmp) ) | ( mtrx_last[i] & (mtrx_pre[i] | mtrx_tmp) );
        }
        if ( mtrx_cur[i] != mtrx_last[i] ) {
        	kc = 1;
        }
        mtrx_pre[i] = mtrx_tmp;
        mtrx_last[i] = mtrx_cur[i];
    }
    return kc;
}


// input:          pressed_matrix,
// key_code:   output keys array
// key_max:    max keys should be returned
//u8 a_debug_key = 0xee;

#if(KEYBOARD_WIN_LOCK_EN)
u8 cur_kb_win_lock;
u8 last_kb_win_lock;
#endif
static inline void kb_remap_key_row(int drv_ind, u32 m, int key_max, kb_data_t *kb_data)
{

	foreach_arr(i, scan_pins){
		//a_debug_key = 0xe5;
		if(m & 0x01){
			//a_debug_key = 0xe4;
			u8 kc = kb_k_mp[i][drv_ind];
#if(KB_HAS_CTRL_KEYS)

			if(kc >= VK_CTRL && kc <= VK_RWIN) //0xe0~0xe7 function key
				{
					kb_data->ctrl_key |= BIT(kc - VK_CTRL);
					//a_debug_key = 0xe0;
				}
			//else if(kc == VK_MEDIA_END)
				//lock_button_pressed = 1;
			else if(VK_ZOOM_IN == kc || VK_ZOOM_OUT == kc){//0xb6~0xb7
				kb_data->ctrl_key |= VK_MSK_LCTRL;
				kb_data->keycode[kb_data->cnt++] = (VK_ZOOM_IN == kc)? VK_EQUAL : VK_MINUS;
				//a_debug_key = 0xe1;
			}
#if(KEYBOARD_SWITCH_KEYCOD_EN)
			else if( kc >= VK_SWITCH_CTRL && kc <= VK_SWITCH_RWIN ){
				kc -= 0x10;							//swith to control key
				kb_k_mp_t *kb_k_mp_shift = kb_map_shift;
				kb_data->ctrl_key |= BIT(kc - VK_CTRL);
				kb_data->keycode[kb_data->cnt++] = kb_k_mp_shift[i][drv_ind];		//switch to shift map

			}
			else if(kc >= VK_SWITCH0 && kc <= VK_SWITCH7)
			{
				u8 ctrl_key_cnt = 0;
				u8 idx = (kc - VK_SWITCH0);

				switch0_key_pressed = idx ? 0 : 1;		//SWITCH0 has been pressed
				for(int dt=0; dt<(kb_switch_key[idx]->len); dt++){
					if(VK_CTRL <= kb_switch_key[idx]->data[dt] && VK_RWIN >= kb_switch_key[idx]->data[dt]){
						kb_data->ctrl_key |= BIT(kb_switch_key[idx]->data[dt] - VK_CTRL);
						ctrl_key_cnt++;
					}
					else{
						kb_data->keycode[dt - ctrl_key_cnt] = kb_switch_key[idx]->data[dt];
					}
				}
				kb_data->cnt = kb_switch_key[idx]->len - ctrl_key_cnt;

			}
#endif
			else if(kc != VK_FN)//fix fn ghost bug//!=0xff
			{
				if(!kc){
					kb_k_mp_t *kb_k_mp_normal = kb_p_map[0];  //normal
					kc = kb_k_mp_normal[i][drv_ind];
				}

				kb_data->keycode[kb_data->cnt++] = kc;
			}

#else
			kb_data->keycode[kb_data->cnt++] = kc;
#endif
			if(kb_data->cnt >= key_max){
				break;
			}
		}
		m = m >> 1;
		if(!m){
			break;
		}
	}
}
//u8 a_debug_remap = 0xcd;
static inline void kb_remap_key_code(u32 * pressed_matrix, int key_max, kb_data_t *kb_data, int numlock_status)
{

	kb_k_mp = kb_p_map[(kb_is_fn_pressed&1) | ( (numlock_status&1)<< 1)];
	foreach_arr(i, drive_pins){
		u32 m = pressed_matrix[i];
//		a_debug_remap = 0xcc;
		if(!m) continue;
//		a_debug_remap = 0xce;
		kb_remap_key_row(i, m, key_max, kb_data);
//		a_debug_remap = 0xcf;
		if(kb_data->cnt >= key_max){
			break;
		}
	}
}

static u32 kb_key_pressed(u8 * gpio)
{
	foreach_arr(i,drive_pins){
		gpio_write(drive_pins[i], KB_LINE_HIGH_VALID);
		gpio_set_output_en(drive_pins[i], 1);
	}
	sleep_us (25);
	gpio_read_all (gpio);

	u32 ret = 0;
	static u8 release_cnt = 0;
	static u32 ret_last = 0;

	foreach_arr(i,scan_pins){
		if(KB_LINE_HIGH_VALID != !gpio_read_cache (scan_pins[i], gpio)){
			ret |= (1 << i);
			release_cnt = 6;
			ret_last = ret;
		}
		//ret = ret && gpio_read(scan_pins[i]);
	}
	if(release_cnt){
		ret = ret_last;
		release_cnt--;
	}
	foreach_arr(i,drive_pins){
		gpio_write(drive_pins[i], 0);
		gpio_set_output_en(drive_pins[i], 0);
	}
	return ret;
}

#if(__PROJECT_KEYBOARD__)
_attribute_ram_code_
#endif
static u32 kb_scan_row(int drv_ind, u8 * gpio)
{
	/*
	 * set as gpio mode if using spi flash pin
	 * */
	u8 sr = irq_disable();


#if(!KB_LINE_MODE)
	u32 drv_pin = drive_pins[drv_ind];
	gpio_write(drv_pin, KB_LINE_HIGH_VALID);
	gpio_set_output_en(drv_pin, 1);
#endif

	u32 matrix = 0;
	foreach_arr(j, scan_pins){
		if(scan_pin_need & BIT(j)){
			int key = !gpio_read_cache (scan_pins[j], gpio);
			if(KB_LINE_HIGH_VALID != key) {
				if (KB_HAS_FN_KEY && (kb_k_mp[j][(drv_ind+7)&7] == VK_FN)) {
					kb_is_fn_pressed = 1;

				}
				matrix |= (1 << j);
			}
		}
	}
	//sleep_us(KB_DRV_DELAY_TIME);
	gpio_read_all (gpio);
	/*
	 * set as spi mode  if using spi flash pin
	 * */

#if(!KB_LINE_MODE)
	////////		float drive pin	////////////////////////////
	//sleep_us(KB_SCAN_DELAY_TIME);
	gpio_write(drv_pin, 0);
	gpio_set_output_en(drv_pin, 0);
#endif

	irq_restore(sr);
	return matrix;
}
/////////////////////external interface for keyboard module/////////////////////
#if(KB_KEY_VALUE_MERGE)

#define  KEY_BUF_SIZE   			8
typedef struct{
	u8	 		key_cnt;
	u8          valid_repeatKey;
	u8          rsv[2];
	u8          loop[KEY_BUF_SIZE];
	kb_data_t 	value[KEY_BUF_SIZE];
}keyEvt_buf_t;

keyEvt_buf_t keyEvt_buf;

#define KEY_SPECIAL(key) 			(key.cnt==0 || key.ctrl_key != 0)
#define KEY_MERGABLE(key1,key2)     (key1.cnt + key2.cnt <= KB_RETURN_KEY_MAX)

#define SAME_KEY_REMOVER_WHEN_MERGE     1  //@@@  remove sme key



inline void key_merge(int index,int max){

	int k;
	int j;
	int des_cnt = keyEvt_buf.value[index].cnt;
	int src_cnt = keyEvt_buf.value[index+1].cnt;
	for(j=0;j<src_cnt;j++){
#if (SAME_KEY_REMOVER_WHEN_MERGE)
		for(k=0;k<des_cnt;k++){
			if(keyEvt_buf.value[index+1].keycode[j] == keyEvt_buf.value[index].keycode[k]){
				break;
			}
		}
		if(k<des_cnt){  //same key,jump
			continue;
		}
#endif
		keyEvt_buf.value[index].keycode[des_cnt++] = keyEvt_buf.value[index+1].keycode[j];
	}
	keyEvt_buf.value[index].cnt = des_cnt;

	for(j=index+1;j<=max;j++){
		memcpy(&keyEvt_buf.value[j],&keyEvt_buf.value[j+1],sizeof(kb_data_t));
	}
}


inline void key_buf_merge(void)
{

	int i;
	int des_max = KEY_BUF_SIZE - 2;
	int src_max = KEY_BUF_SIZE - 1;
	for(i=0;i<=des_max;i++){
		if(KEY_SPECIAL(keyEvt_buf.value[i])){
			continue;
		}
		while(i+1 <= src_max){
			if(KEY_SPECIAL(keyEvt_buf.value[i+1])){
				i++;
				break;
			}
			if(KEY_MERGABLE(keyEvt_buf.value[i],keyEvt_buf.value[i+1])){  //merge
				key_merge(i,des_max);
				des_max--;
				src_max--;

				i --;  //@@@ back one step
			}
			else{
				break;
			}
		}
	}

	if(des_max ==  KEY_BUF_SIZE - 2){ //no merge,overwrite
		for(i=0;i<KEY_BUF_SIZE-1;i++){
			memcpy(&keyEvt_buf.value[i],&keyEvt_buf.value[i+1],sizeof(kb_event));
		}
		keyEvt_buf.key_cnt = KEY_BUF_SIZE - 1;
	}
	else{
		keyEvt_buf.key_cnt = des_max + 2;
	}

}

#endif

#if(!KB_KEY_VALUE_MERGE)
	u32 	matrix_buff[4][ARRAY_SIZE(drive_pins)];//matrix_buff[4][8]
	int		matrix_wptr, matrix_rptr;
#endif


#if(__PROJECT_KEYBOARD__)
_attribute_ram_code_
#endif

u32 gost_and_no_key = 0;
u32 kb_scan_key (int numlock_status, int read_key)
{

	//trace_event(TR_T_keyScan);

	int i;
	u8 gpio[8];

	gost_and_no_key = 0;

    scan_pin_need = kb_key_pressed (gpio);

	if(scan_pin_need){

		//trace_task_begin(TR_T_scanNeed);

		kb_event.cnt = 0;
		kb_event.ctrl_key = 0;
		kb_is_fn_pressed = 0;

		u32  key_changed = 0;

		u32 pressed_matrix[ARRAY_SIZE(drive_pins)] = {0};
		u32 pressed_matrix_backup[ARRAY_SIZE(drive_pins)] = {0};

		kb_k_mp = kb_p_map[0];

		kb_scan_row (0, gpio);
		for (int i=0; i<=ARRAY_SIZE(drive_pins); i++) {
			u32 r = kb_scan_row (i < ARRAY_SIZE(drive_pins) ? i : 0, gpio);
			if (i) {
				pressed_matrix[i - 1] = r;
				pressed_matrix_backup[i - 1] = r;

			}
		}

#if(KB_RM_GHOST_KEY_EN)
		gost_and_no_key = kb_rmv_ghost_key(&pressed_matrix[0]);
#endif
//in pairing mode key_debounce_filter cannot detect key pressed.
//need discussing
		static u32 data_same_cnt;

		keyEvt_buf.valid_repeatKey = 0;

		if(numlock_status & KB_NUMLOCK_STATUS_POWERON){
			key_changed = 1;
		}
		else{
			key_changed = key_debounce_filter(pressed_matrix, 1);
			//trace_event(TR_T_key_new);
		}

		if(key_changed){					//
			data_same_cnt = 0;
			if(kb_is_unlock_pressed & KB_STATUS_LOCK){
				kb_is_unlock_pressed = 0;
			}
		}
		else{
			data_same_cnt++;
			if( data_same_cnt >= (200/KB_MAIN_LOOP_TIME_MS)  ){  //200ms
				data_same_cnt = 0;
				if(!(kb_is_lock_pressed & KB_STATUS_LOCK) && !(kb_is_unlock_pressed & KB_STATUS_LOCK))
				{
					key_changed = 1;
					keyEvt_buf.valid_repeatKey = 1;
					//trace_event(TR_T_key_repeat);
				}
				else{
					key_changed = 0;
				}
			}
		}

#if(KB_KEY_VALUE_MERGE)

	#if 0
		//...
	#else  // save ram

		kb_data_t *key_ptr;

		if(key_changed && !gost_and_no_key){ //

			if(keyEvt_buf.key_cnt == KEY_BUF_SIZE){ //overwrite
				key_buf_merge();
			}
			key_ptr = (kb_data_t *)(&keyEvt_buf.value[keyEvt_buf.key_cnt]);
			keyEvt_buf.key_cnt++;
			key_ptr->cnt = 0;
			key_ptr->ctrl_key = 0;
			kb_remap_key_code(pressed_matrix, KB_RETURN_KEY_MAX,key_ptr,numlock_status);

			//trace_event(TR_T_remap);
			//trace_data(TR_24_REMAP_CNT, key_ptr->cnt);
			//trace_data(TR_24_REMAP_VALUE, key_ptr->keycode[0] | key_ptr->keycode[1]<<8 | key_ptr->keycode[2] <<16);
		}
	#endif


		//trace_task_end(TR_T_scanNeed);

	}


	if( (numlock_status & KB_NUMLOCK_STATUS_INVALID) && !(numlock_status & KB_STATUS_LOCK)
		#if(!__PROJECT_REMOTE__ && !__PROJECT_8267_REMOTE__)
		&& (kb_status.kb_mode == STATE_NORMAL) && (keyEvt_buf.key_cnt)
		#endif
		){  //KB_NUMLOCK_STATUS_ALL

		kb_event.cnt = 0;
		kb_event.ctrl_key = 0;
		return 1;
	}


	if (!keyEvt_buf.key_cnt || !read_key) {
		return 0;
	}
	memcpy(&kb_event,&keyEvt_buf.value[0],sizeof(kb_data_t));
	for(i=0;i<keyEvt_buf.key_cnt-1;i++){
		memcpy(&keyEvt_buf.value[i],&keyEvt_buf.value[i+1],sizeof(kb_data_t));
	}
	keyEvt_buf.key_cnt --;

	return 2;

#else

#endif


}



#else  //else of __PROJECT_REMINGTON_KEYBOARD_TW__


static void kb_rmv_ghost_key(u32 * pressed_matrix)
{
	u32 mix_final = 0;
	foreach_arr(i, drive_pins){
		for(int j = (i+1); j < ARRAY_SIZE(drive_pins); ++j){
			u32 mix = (pressed_matrix[i] & pressed_matrix[j]);
			// >=2 鏍圭嚎閲嶅悎,  閭ｅ氨鏄�ghost key
			//four or three key at "#" is pressed at the same time, should remove ghost key
			if( mix && (!BIT_IS_POW2(mix) || (pressed_matrix[i] ^ pressed_matrix[j])) ){
				// remove ghost keys
				//pressed_matrix[i] &= ~mix;
				//pressed_matrix[j] &= ~mix;
				mix_final |= mix;
			}
		}
		pressed_matrix[i] &= ~mix_final;
	}
}

extern u32     sys_host_status;

static u32 key_debounce_filter( u32 mtrx_cur[], u32 filt_en )
{
	u32 kc = 0;
    static u32 mtrx_pre[ARRAY_SIZE(drive_pins)];
    static u32 mtrx_last[ARRAY_SIZE(drive_pins)];
    foreach_arr(i, drive_pins){
        u32 mtrx_tmp = mtrx_cur[i];
        if( filt_en ){
            //mtrx_cur[i] = (mtrx_last[i] ^ mtrx_tmp) ^ (mtrx_last[i] | mtrx_tmp);  //key_matrix_pressed is valid when current and last value is the same
            mtrx_cur[i] = ( ~mtrx_last[i] & (mtrx_pre[i] & mtrx_tmp) ) | ( mtrx_last[i] & (mtrx_pre[i] | mtrx_tmp) );
        }
        if ( mtrx_cur[i] != mtrx_last[i] ) {
        	kc = 1;
        }
        mtrx_pre[i] = mtrx_tmp;
        mtrx_last[i] = mtrx_cur[i];
    }
    return kc;
}


// input:          pressed_matrix,
// key_code:   output keys array
// key_max:    max keys should be returned
//u8 a_debug_key = 0xee;

#if(KEYBOARD_WIN_LOCK_EN)
u8 cur_kb_win_lock;
u8 last_kb_win_lock;
#endif
static inline void kb_remap_key_row(int drv_ind, u32 m, int key_max, kb_data_t *kb_data)
{

	foreach_arr(i, scan_pins){
		//a_debug_key = 0xe5;
		if(m & 0x01){
			//a_debug_key = 0xe4;
			u8 kc = kb_k_mp[i][drv_ind];
#if(KB_HAS_CTRL_KEYS)

			if(kc >= VK_CTRL && kc <= VK_RWIN) //0xe0~0xe7 function key
				{
					kb_data->ctrl_key |= BIT(kc - VK_CTRL);
					//a_debug_key = 0xe0;
				}
			//else if(kc == VK_MEDIA_END)
				//lock_button_pressed = 1;
			else if(VK_ZOOM_IN == kc || VK_ZOOM_OUT == kc){//0xb6~0xb7
				kb_data->ctrl_key |= VK_MSK_LCTRL;
				kb_data->keycode[kb_data->cnt++] = (VK_ZOOM_IN == kc)? VK_EQUAL : VK_MINUS;
				//a_debug_key = 0xe1;
			}
#if(KEYBOARD_SWITCH_KEYCOD_EN)
			else if( kc >= VK_SWITCH_CTRL && kc <= VK_SWITCH_RWIN ){
				kc -= 0x10;							//swith to control key
				kb_k_mp_t *kb_k_mp_shift = kb_map_shift;
				kb_data->ctrl_key |= BIT(kc - VK_CTRL);
				kb_data->keycode[kb_data->cnt++] = kb_k_mp_shift[i][drv_ind];		//switch to shift map

			}
			else if(kc >= VK_SWITCH0 && kc <= VK_SWITCH7)
			{
				u8 ctrl_key_cnt = 0;
				u8 idx = (kc - VK_SWITCH0);

				switch0_key_pressed = idx ? 0 : 1;		//SWITCH0 has been pressed
				for(int dt=0; dt<(kb_switch_key[idx]->len); dt++){
					if(VK_CTRL <= kb_switch_key[idx]->data[dt] && VK_RWIN >= kb_switch_key[idx]->data[dt]){
						kb_data->ctrl_key |= BIT(kb_switch_key[idx]->data[dt] - VK_CTRL);
						ctrl_key_cnt++;
					}
					else{
						kb_data->keycode[dt - ctrl_key_cnt] = kb_switch_key[idx]->data[dt];
					}
				}
				kb_data->cnt = kb_switch_key[idx]->len - ctrl_key_cnt;

			}
#endif
			else if(kc != VK_FN)//fix fn ghost bug//!=0xff
			{
				if(!kc){
					kb_k_mp_t *kb_k_mp_normal = kb_p_map[0];  //normal
					kc = kb_k_mp_normal[i][drv_ind];
				}

				kb_data->keycode[kb_data->cnt++] = kc;
			}

#else
			kb_data->keycode[kb_data->cnt++] = kc;
#endif
			if(kb_data->cnt >= key_max){
				break;
			}
		}
		m = m >> 1;
		if(!m){
			break;
		}
	}
}
//u8 a_debug_remap = 0xcd;
static inline void kb_remap_key_code(u32 * pressed_matrix, int key_max, kb_data_t *kb_data, int numlock_status)
{

	kb_k_mp = kb_p_map[(kb_is_fn_pressed&1) | ( (numlock_status&1)<< 1)];
	foreach_arr(i, drive_pins){
		u32 m = pressed_matrix[i];
//		a_debug_remap = 0xcc;
		if(!m) continue;
//		a_debug_remap = 0xce;
		kb_remap_key_row(i, m, key_max, kb_data);
//		a_debug_remap = 0xcf;
		if(kb_data->cnt >= key_max){
			break;
		}
	}
}

static u32 kb_key_pressed(u8 * gpio)
{
	foreach_arr(i,drive_pins){
		gpio_write(drive_pins[i], KB_LINE_HIGH_VALID);
		gpio_set_output_en(drive_pins[i], 1);
	}
	sleep_us (20);
	gpio_read_all (gpio);

	u32 ret = 0;
	static u8 release_cnt = 0;
	static u32 ret_last = 0;

	foreach_arr(i,scan_pins){
		if(KB_LINE_HIGH_VALID != !gpio_read_cache (scan_pins[i], gpio)){
			ret |= (1 << i);
			release_cnt = 6;
			ret_last = ret;
		}
		//ret = ret && gpio_read(scan_pins[i]);
	}
	if(release_cnt){
		ret = ret_last;
		release_cnt--;
	}
	foreach_arr(i,drive_pins){
		gpio_write(drive_pins[i], 0);
		gpio_set_output_en(drive_pins[i], 0);
	}
	return ret;
}

#if(__PROJECT_KEYBOARD__)
_attribute_ram_code_
#endif
static u32 kb_scan_row(int drv_ind, u8 * gpio)
{
	/*
	 * set as gpio mode if using spi flash pin
	 * */
	u8 sr = irq_disable();

#if(!KB_LINE_MODE)
	u32 drv_pin = drive_pins[drv_ind];
	gpio_write(drv_pin, KB_LINE_HIGH_VALID);
	gpio_set_output_en(drv_pin, 1);
#endif

	u32 matrix = 0;
	foreach_arr(j, scan_pins){
		if(scan_pin_need & BIT(j)){
			int key = !gpio_read_cache (scan_pins[j], gpio);
			if(KB_LINE_HIGH_VALID != key) {
				if (KB_HAS_FN_KEY && (kb_k_mp[j][(drv_ind+7)&7] == VK_FN)) {
					kb_is_fn_pressed = 1;

				}
				matrix |= (1 << j);
			}
		}
	}
	//sleep_us(KB_DRV_DELAY_TIME);
	gpio_read_all (gpio);
	/*
	 * set as spi mode  if using spi flash pin
	 * */

#if(!KB_LINE_MODE)
	////////		float drive pin	////////////////////////////
	//sleep_us(KB_SCAN_DELAY_TIME);
	gpio_write(drv_pin, 0);
	gpio_set_output_en(drv_pin, 0);
#endif

	irq_restore(sr);
	return matrix;
}
/////////////////////external interface for keyboard module/////////////////////
#if(KB_KEY_VALUE_MERGE)
#define  KEY_BUF_SIZE   			8
typedef struct{
	u8	 		key_cnt;
	u8          rsv[3];
	u8          loop[KEY_BUF_SIZE];
	kb_data_t 	value[KEY_BUF_SIZE];
}keyEvt_buf_t;
keyEvt_buf_t keyEvt_buf;

#define KEY_SPECIAL(key) 			(key.cnt==0 || key.ctrl_key != 0)
#define KEY_MERGABLE(key1,key2)     (key1.cnt + key2.cnt <= KB_RETURN_KEY_MAX)

#define SAME_KEY_REMOVER_WHEN_MERGE     0

#define DBG_KEY_MERGE     0
#if DBG_KEY_MERGE
keyEvt_buf_t keyEvt_dbg;
#endif

inline void key_merge(int index,int max){

	int j;
	int des_cnt = keyEvt_buf.value[index].cnt;
	int src_cnt = keyEvt_buf.value[index+1].cnt;
	for(j=0;j<src_cnt;j++){
#if(SAME_KEY_REMOVER_WHEN_MERGE)
		for(k=0;k<des_cnt;k++){
			if(keyEvt_buf.value[index+1].keycode[j] == keyEvt_buf.value[index].keycode[k]){
				break;
			}
		}
		if(k<des_cnt){  //same key,jump
			continue;
		}
#endif
		keyEvt_buf.value[index].keycode[des_cnt++] = keyEvt_buf.value[index+1].keycode[j];
	}
	keyEvt_buf.value[index].cnt = des_cnt;

	for(j=index+1;j<=max;j++){
		memcpy(&keyEvt_buf.value[j],&keyEvt_buf.value[j+1],sizeof(kb_data_t));
	}
}


inline void key_buf_merge(void)
{
#if DBG_KEY_MERGE
	memcpy(&keyEvt_dbg,&keyEvt_buf,sizeof(keyEvt_buf_t));
#endif
	int i;
	int des_max = KEY_BUF_SIZE - 2;
	int src_max = KEY_BUF_SIZE - 1;
	for(i=0;i<=des_max;i++){
		if(KEY_SPECIAL(keyEvt_buf.value[i])){
			continue;
		}
		while(i+1 <= src_max){
			if(KEY_SPECIAL(keyEvt_buf.value[i+1])){
				i++;
				break;
			}
			if(KEY_MERGABLE(keyEvt_buf.value[i],keyEvt_buf.value[i+1])){  //merge
				key_merge(i,des_max);
				des_max--;
				src_max--;
			}
			else{
				break;
			}
		}
	}

	if(des_max ==  KEY_BUF_SIZE - 2){ //no merge,overwrite
		for(i=0;i<KEY_BUF_SIZE-1;i++){
			memcpy(&keyEvt_buf.value[i],&keyEvt_buf.value[i+1],sizeof(kb_event));
		}
		keyEvt_buf.key_cnt = KEY_BUF_SIZE - 1;
	}
	else{
		keyEvt_buf.key_cnt = des_max + 2;
	}
#if DBG_KEY_MERGE
	kb_device_led_setup(kb_led_cfg[6]);
#endif
}

#endif

u32 	matrix_buff[4][ARRAY_SIZE(drive_pins)];//matrix_buff[4][8]
int		matrix_wptr, matrix_rptr;



#if(__PROJECT_KEYBOARD__)
_attribute_ram_code_
#endif
u32 kb_scan_key (int numlock_status, int read_key)
{
	int i;
	u8 gpio[8];
    scan_pin_need = kb_key_pressed (gpio);

	if(scan_pin_need){
		kb_event.cnt = 0;
		kb_event.ctrl_key = 0;
		kb_is_fn_pressed = 0;

		u32 pressed_matrix[ARRAY_SIZE(drive_pins)] = {0};
		kb_k_mp = kb_p_map[0];

		kb_scan_row (0, gpio);
		for (int i=0; i<=ARRAY_SIZE(drive_pins); i++) {
			u32 r = kb_scan_row (i < ARRAY_SIZE(drive_pins) ? i : 0, gpio);
			if (i) {
				pressed_matrix[i - 1] = r;
			}
		}

#if(KB_RM_GHOST_KEY_EN)
		kb_rmv_ghost_key(&pressed_matrix[0]);
#endif
//in pairing mode key_debounce_filter cannot detect key pressed.
//need discussing
		static u32 data_same_cnt;

		u32  key_changed;
		if(numlock_status & KB_NUMLOCK_STATUS_POWERON){
			key_changed = 1;
		}
		else{
			key_changed = key_debounce_filter( pressed_matrix, 1);
		}

		if(key_changed){					//
			data_same_cnt = 0;
			if(kb_is_unlock_pressed & KB_STATUS_LOCK){
				kb_is_unlock_pressed = 0;
			}
		}
		else{
			data_same_cnt++;
			if( data_same_cnt >= (200/KB_MAIN_LOOP_TIME_MS)  ){  //200ms
				data_same_cnt = 0;
				if(!(kb_is_lock_pressed & KB_STATUS_LOCK) && !(kb_is_unlock_pressed & KB_STATUS_LOCK))
				{
					key_changed = 1;
				}
				else{
					key_changed = 0;
				}
			}
		}

#if(KB_KEY_VALUE_MERGE)
		u32 *pd;
		kb_data_t *key_ptr;
		if(key_changed){
			pd = matrix_buff[matrix_wptr&3];
			for (int k=0; k<ARRAY_SIZE(drive_pins); k++) {
				*pd++ = pressed_matrix[k];
			}
			matrix_wptr = (matrix_wptr + 1) & 7;
			if ( ((matrix_wptr - matrix_rptr) & 7) > 4 ) {	//overwrite older data
				matrix_rptr = (matrix_wptr - 4) & 7;
			}
		}
		if( (numlock_status & KB_NUMLOCK_STATUS_INVALID) && !(numlock_status & KB_STATUS_LOCK)
			#if(!__PROJECT_REMOTE__ && !__PROJECT_8267_REMOTE__)
			&& (kb_status.kb_mode == STATE_NORMAL)
			#endif
			){  //KB_NUMLOCK_STATUS_ALL
			return 1;
		}

		if(matrix_wptr != matrix_rptr){ //
			pd = matrix_buff[matrix_rptr&3];
			matrix_rptr = (matrix_rptr + 1) & 7;

			if(keyEvt_buf.key_cnt == KEY_BUF_SIZE){ //overwrite
				key_buf_merge();
			}
			key_ptr = (kb_data_t *)(&keyEvt_buf.value[keyEvt_buf.key_cnt]);
			keyEvt_buf.key_cnt++;
			key_ptr->cnt = 0;
			key_ptr->ctrl_key = 0;
			kb_remap_key_code(pd, KB_RETURN_KEY_MAX,key_ptr,numlock_status);
		}

		if (!keyEvt_buf.key_cnt || !read_key) {
			return 0;
		}
		memcpy(&kb_event,&keyEvt_buf.value[0],sizeof(kb_data_t));
		for(i=0;i<keyEvt_buf.key_cnt-1;i++){
			memcpy(&keyEvt_buf.value[i],&keyEvt_buf.value[i+1],sizeof(kb_data_t));
		}
		keyEvt_buf.key_cnt --;

#else
		///////////////////////////////////////////////////////////////////
		//	insert buffer here
		//       key mapping requires NUMLOCK status
		///////////////////////////////////////////////////////////////////
		u32 *pd;
		if (key_changed) {

			/////////// push to matrix buffer /////////////////////////
			pd = matrix_buff[matrix_wptr&3];
			for (int k=0; k<ARRAY_SIZE(drive_pins); k++) {
				*pd++ = pressed_matrix[k];
			}
			matrix_wptr = (matrix_wptr + 1) & 7;
			if ( ((matrix_wptr - matrix_rptr) & 7) > 4 ) {	//overwrite older data
				matrix_rptr = (matrix_wptr - 4) & 7;
			}
		}

		if (numlock_status & KB_NUMLOCK_STATUS_INVALID) {
			return 1;		//return empty key
		}

		////////// read out //////////
		if (matrix_wptr == matrix_rptr || !read_key) {
			return 0;			//buffer empty, no data
		}
		pd = matrix_buff[matrix_rptr&3];
		matrix_rptr = (matrix_rptr + 1) & 7;

		///////////////////////////////////////////////////////////////////

#if(KB_ONLY_SINGLEKEY_SUPP)
		kb_remap_key_code(pd, 2, &kb_event, numlock_status);
		int multikey_pressed = 0;
		// multi-key is invalid, that is assuming not press, that is last status
		// do not set kb_event.cnt to zero, which may occur an bug, press A --> press AB --> Releae B
		if(kb_event.cnt > 1){
			kb_event.cnt = 0;
			multikey_pressed = 1;
		}
#else
		kb_remap_key_code(pd, KB_RETURN_KEY_MAX, &kb_event, numlock_status);
#endif
#endif
		return 2;
	}
	//tick_kb_scan = clock_time () - t;
    return 0;
}





#endif //end of __PROJECT_REMINGTON_KEYBOARD_TW__



#endif

///////////////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////////////
