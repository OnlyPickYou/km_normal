
#pragma once

#if (__PROJECT_PM_TEST_5330__)	//  min environment for testing PM functionality.
	#define __PROJECT_PM_TEST__ 			1
	#include "example_8566.h"
#elif (__PROJECT_MOUSE__)
    #include "../mouse/mouse.h"
#elif (__PROJECT_MOUSE_8510__)
    #include "../mouse_cavy/cavym.h"
#elif (__PROJECT_DONGLE_8510__)
	#include "../dongle_cavy/dongle.h"
#elif (__PROJECT_KEYBOARD__)
	#include "../keyboard/kb_main.h"
    #include "../keyboard/kb_default_config.h"
#elif (__PROJECT_DONGLE__)
	#include "../dongle/dongle.h"
#elif (__PROJECT_DONGLE_RC__)
	#include "../dongle_IR/dongle.h"
#elif (__PROJECT_KM_SNIFFER__)
	#include "../km_sniffer/km_sniffer.h"
#elif (__PROJECT_PKT_SNIFFER__)
	#include "../pktsniffer/main_pktsniffer.h"
#elif (__PROJECT_REMOTE__)
	#include "../remote/app_config.h"
#elif (__PROJECT_DONGLE_REMOTE__)
	#include "../dongle_remote/app_config.h"
#elif (__PROJECT_REMINGTON_KEYBOARD__)
	#include "../keyboard_remington/kb_default_config.h"
	#include "../keyboard_remington/kb.h"
#elif (__PROJECT_REMINGTON_KEYBOARD_TW__)
	#include "../keyboard_remington_tw/kb_default_config.h"
	#include "../keyboard_remington_tw/kb.h"
#elif (__PROJECT_REMINGTON_RC__)
	#include "../rc_remington/kb_default_config.h"
	#include "../rc_remington/kb.h"
#elif (__PROJECT_VACUUM__)
    #include "../vacuum_controller/vacuum_controller.h"
#elif (__PROJECT_VACUUM_RECEIVER__)
	#include "../vacuum_receiver/vacuum_receiver.h"
#elif (__PROJECT_TX_8267__)
	#include "../bleplus_tx/tx.h"
#elif (__PROJECT_RX_8267__)
	#include "../bleplus_rx/rx.h"
#elif (__PROJECT_HAMSTER_RX__)
	#include "../hamster_rx/hmrx.h"
#elif (__PROJECT_8267_REMOTE__)
	#include "../bleplus_remote/app_config.h"
#elif (__PROJECT_MOUSE_ENC_8366__)
    #include "../8366_mouse_enc/mouse.h"
#elif (__PROJECT_DONGLE_ENC_8366__)
    #include "../8366_dongle_enc/dongle.h"
#elif (__PROJECT_KEYBOARD_ENC_8368__)
	#include "../8368_keyboard_enc/kb_default_config.h"
    #include "../8368_keyboard_enc/kb.h"
#elif(__PROJECT_REMINGTON_XJGD__)
	#include "../8368_xjgd_remote/kb_default_config.h"
	#include "../8368_xjgd_remote/kb.h"
#elif (__PROJECT_XJGD_DONGLE__)
	#include "../8366_xjgd_dongle/dongle.h"
#elif (__PROJECT_8366_MIIW_MOUSE__)
    #include "../8366_miiw_mouse/mouse.h"
#else
	#include "user_config_common.h"
#endif
