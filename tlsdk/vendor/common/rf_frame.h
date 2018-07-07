/*
 * led_rf_frame.h
 *
 *  Created on: Jan 13, 2014
 *      Author: xuzhen
 */

#ifndef _RF_FRAME_H_
#define _RF_FRAME_H_
#include "mouse_type.h"
#ifndef __PROJECT_REMINGTON_RC__
#include "../../proj/drivers/keyboard.h"
#else
#include "../rc_remington/keyboard.h"
#endif

#ifndef MOUSE_PIPE1_DATA_WITH_DID
#define MOUSE_PIPE1_DATA_WITH_DID	0
#endif

#ifndef KEYBOARD_PIPE1_DATA_WITH_DID
#define KEYBOARD_PIPE1_DATA_WITH_DID	0
#endif


#define		RF_PROTO_BYTE		0x51
#define		PIPE0_CODE			0x55556666
#define		PIPE1_CODE			0xaabbccdd


#define		RF_PROTO_VACUUM				0x5a

enum{
	PIPE_PARING			= 0x00,
	PIPE_MOUSE			= 0x01,
	PIPE_KEYBOARD		= 0x02,
	PIPE_AUDIO			= 0x03,
	PIPE_TOUCH			= 0x04,
	PIPE_RC				= 0x05,
	PIPE_VACUUM  		= 0x02,

};

#define BIT(n)                  		( 1<<(n) )

enum{
	FRAME_TYPE_DEVICE		= 0x00,
	FRAME_TYPE_MOUSE		= 0x01,
    FRAME_TYPE_KEYBOARD		= 0x02,
    FRAME_TYPE_AUDIO		= 0x03,
    FRAME_TYPE_TOUCH		= 0x04,
    FRAME_TYPE_VACUUM		= 0x05,
    FRAME_TYPE_VACUUM_CONF	= 0x06,
    FRAME_TYPE_VACUUM_MANN	= 0x07,
    FRAME_TYPE_VACUUM_RESET = 0x08,

    FRAME_TYPE_PARING		= 0x10,


    FRAME_TYPE_ACK        	= 0x80,
    FRAME_TYPE_ACK_MOUSE  	= FRAME_TYPE_ACK | FRAME_TYPE_MOUSE,	//0x81
    FRAME_TYPE_ACK_KEYBOARD	= FRAME_TYPE_ACK | FRAME_TYPE_KEYBOARD,	//0x82
    FRAME_TYPE_ACK_AUDIO	= FRAME_TYPE_ACK | FRAME_TYPE_AUDIO,	//0x83
    FRAME_TYPE_ACK_TOUCH	= FRAME_TYPE_ACK | FRAME_TYPE_TOUCH,	//0x84
    FRAME_TYPE_ACK_VACUUM	= FRAME_TYPE_ACK | FRAME_TYPE_VACUUM,	//0x85


    FRAME_TYPE_ACK_EMPTY	= FRAME_TYPE_ACK | BIT(6),				//0xc0

    FRAME_TYPE_DEBUG		= 0x40,
    FRAME_TYPE_MAX,			//0x41

    FRAME_PAIR_FAIL			= 0x20,
    FRAME_NO_PAIR_FLOW		= 0x30,

    FRAME_TYPE_REMOTE_KEYBOARD = 	BIT(5),
    FRAME_TYPE_REMOTE_MIC = 		BIT(6),

    FRAME_TYPE_MOUSE_SEND_ID	 = 0x09,  //  FRAME_TYPE_MOUSE | 0x08
    FRAME_AUTO_ACK_MOUSE_ASK_ID  = 0x89,  //  FRAME_TYPE_ACK_MOUSE | 0x08

    FRAME_TYPE_KB_SEND_ID	 = 0x0a,  // FRAME_TYPE_KEYBOARD | 0x08
	FRAME_AUTO_ACK_KB_ASK_ID  = 0x8a, //FRAME_TYPE_ACK_KEYBOARD | 0x08

};

enum{
	PKT_FLOW_CAL 	 = BIT(0),
	PKT_FLOW_SEARCH	 = BIT(2),
	PKT_FLOW_ACK_REQ = BIT(3),	
	PKT_FLOW_PARING	 = BIT(4),   //for paring
	PKT_FLOW_DATA	 = BIT(5),   //for data
	PKT_FLOW_TOKEN	 = BIT(6),
	PKT_FLOW_DIR	 = BIT(7),
};

enum{
	PKT_VACUUM_PARING	=	0x39,
	PKT_VACUUM_DATA		=   0xc6,
};

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8  rf_len;
	u8	proto;
	u8	flow;
	u8	type;

//	u32 gid0;		//pipe0 code,	used as sync code for control pipe in hamster

	u8	rssi;
	u8	per;
	u8	seq_no;
	u8	rsvd;

	u32 did;

}rf_packet_pairing_t;


#if (MOUSE_PIPE1_DATA_WITH_DID)
typedef struct {
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8  rf_len;
	u8	proto;
	u8	flow;
	u8	type;

//	u32 gid;		//pipe0 code,	used as sync code for control pipe in hamster

	u8	rssi;
	u8	per;
	u8	seq_no;
	u8	pno;

	u32 did;

	u8 data[MOUSE_FRAME_DATA_NUM*sizeof(mouse_data_t)]; //4*4 now the data length is variable, if the previous no ACK, data will send again in next time

}rf_packet_mouse_t;
#else
typedef struct {
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8  rf_len;
	u8	proto;
	u8	flow;
	u8	type;

//	u32 gid;		//pipe0 code,	used as sync code for control pipe in hamster

	u8	rssi;
	u8	per;
	u8	seq_no;
	u8	pno;
	u8 data[MOUSE_FRAME_DATA_NUM*sizeof(mouse_data_t)]; //4*4 now the data length is variable, if the previous no ACK, data will send again in next time

}rf_packet_mouse_t;
#endif

typedef struct {
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8  rf_len;
	u8	proto;
	u8	flow;
	u8	type;

	u8	rssi;
	u8	per;
	u8	seq_no;
	u8	pno;

	u8 data[MOUSE_FRAME_DATA_NUM*sizeof(mouse_data_t)]; //4*4 now the data length is variable, if the previous no ACK, data will send again in next time

	u32 did;
	u32 rid;

}rf_packet_cavy_mouse_t;

typedef struct {
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8  rf_len;
	u8	proto;
	u8	flow;
	u8	type;

//	u32 gid;		//pipe0 code,	used as sync code for control pipe in hamster

	u8	rssi;
	u8	per;
	u8	seq_no;
	u8	pno;
	u8 	data[MOUSE_FRAME_DATA_NUM*sizeof(mouse_data_t) + sizeof(kb_data_t) + 144]; //4*4 now the data length is variable, if the previous no ACK, data will send again in next time

}rf_packet_remote_t;

#if (KEYBOARD_PIPE1_DATA_WITH_DID)      //data with did
typedef struct {
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8  rf_len;
	u8	proto;
	u8	flow;
	u8	type;

//	u32 gid;		//pipe0 code,	used as sync code for control pipe in hamster

	u8	rssi;
	u8	per;
	u8	seq_no;
	u8	pno;

	u32 did;

	u8 data[sizeof(kb_data_t)]; //8 now the data length is variable, if the previous no ACK, data will send again in next time
}rf_packet_keyboard_t;

#else   //else of KEYBOARD_PIPE1_DATA_WITH_DID
typedef struct {
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8  rf_len;
	u8	proto;
	u8	flow;
	u8	type;

//	u32 gid;		//pipe0 code,	used as sync code for control pipe in hamster

	u8	rssi;
	u8	per;
	u8	seq_no;
	u8	pno;
	u8 data[sizeof(kb_data_t)]; //8 now the data length is variable, if the previous no ACK, data will send again in next time
}rf_packet_keyboard_t;
#endif   //end of KEYBOARD_PIPE1_DATA_WITH_DID


typedef struct {
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8  rf_len;
	u8	proto;
	u8	flow;
	u8	type;

	u8	rssi;
	u8	per;
	u8	rsvd;
	u8	pno;

	u32 seq_no;
	u32 did;
	u32 Mmac;
	u8 data[MOUSE_FRAME_DATA_NUM*sizeof(mouse_data_t)]; //4*4 now the data length is variable, if the previous no ACK, data will send again in next time


}rf_packet_mouse_enc_t;

typedef struct {
	u32 dma_len;

	u8  rf_len;
	u8	proto;
	u8	flow;
	u8	type;

	u8	rssi;
	u8	per;
	u8	rsvd;
	u8	pno;

	u32 seq_no;
	u32 did;
	u32 Mmac;

	u8 data[sizeof(kb_data_t)];
}rf_packet_keyboard_enc_t;


////////////////////////// host side ///////////////////////////////
typedef struct{
	u32 dma_len;

	u8  rf_len;
	u8	proto;
	u8	flow;
	u8	type;

//	u32 gid0;

	u8	rssi;
	u8	per;
	u16	tick;

	u8	chn;
	u8	info0;
	u8	info1;
	u8	info2;

	u32 gid1;
	u32 did;

}rf_packet_debug_t;

typedef struct {
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8  rf_len;
	u8	proto;
	u8	flow;
	u8	type;

//	u32 gid;		//pipe0 code,	used as sync code for control pipe in hamster

	u8	rssi;
	u8	per;
	u8	seq_no;
	u8	pno;
	//u8 data[sizeof(kb_data_t)]; //8 now the data length is variable, if the previous no ACK, data will send again in next time
	u8  data[2 * sizeof(kb_data_t)];
}rf_packet_vacuum_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number
	u8  rf_len;
	u8	proto;
	u8	flow;
	u8	type;

//	u32 gid0;		//pipe0 code,	used as sync code for control pipe in hamster

	u8	rssi;
	u8	per;
	u16	tick;
	u8	chn;
}rf_ack_empty_t;


typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8  rf_len;
	u8	proto;
	u8	flow;
	u8	type;

//	u32 gid0;		//pipe0 code,	used as sync code for control pipe in hamster

	u8	rssi;
	u8	per;
	u16	tick;
	u8	chn;
	u8	info0;
	u8	info1;
	u8	info2;

	u32 gid1;		//pipe1 code,	used as sync code for data pipe in hamster
	u32 did;

}rf_packet_ack_pairing_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8  rf_len;
	u8	proto;
	u8	flow;
	u8	type;

//	u32 gid;		//pipe0 code,	used as sync code for control pipe in hamster

	u8	rssi;
	u8	per;
	u16	tick;
    
	u8	chn;
    u8  info;
#if (DONGLE_SHOW_MOUSE_SEQNUM)
    u16  rsvd;
    u32 seq_num;
#endif
}rf_packet_ack_mouse_t;

typedef struct{
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8  rf_len;
	u8	proto;
	u8	flow;
	u8	type;

//	u32 gid;		//pipe0 code,	used as sync code for control pipe in hamster

	u8	rssi;
	u8	per;
	u16	tick;
    
	u8	chn;
	u8	status;		//host_keyboard_status,NumLock status
#if (DONGLE_SHOW_KB_SEQNUM)
    u16 rsvd;
    u32 seq_num;
#endif

}rf_packet_ack_keyboard_t;

#endif /* LED_RF_FRAME_H_ */
