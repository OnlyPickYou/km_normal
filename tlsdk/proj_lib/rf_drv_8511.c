
//  to detect MCU_CORE_TYPE in app mode
#if(!(__TL_LIB_8511__ ))
#include "../proj/config/user_config.h"
#include "../proj/mcu/config.h"		// must include this
#endif 

#if(__TL_LIB_8511__ || (MCU_CORE_TYPE && MCU_CORE_TYPE == MCU_CORE_8511))
#include "../proj/common/types.h"
#include "../proj/common/compatibility.h"
#include "../proj/common/bit.h"
#include "../proj/common/utility.h"
#include "../proj/common/static_assert.h"
#include "../proj/mcu/compiler.h"
#include "../proj/mcu/register.h"
#include "../proj/mcu/analog.h"
#include "../proj/mcu/anareg.h"
#include "rf_drv_8510.h"

// RF_FAST_MODE_2M
#define			RF_FAST_MODE				1

#define			RF_MANUAL_AGC_MAX_GAIN		1

#ifndef RF_CRYSTAL_12M_EN
#define			RF_CRYSTAL_12M_EN			0
#endif

#ifndef cust_crystal_12M
#define			cust_crystal_12M			0
#endif

#define 	    TX_GAIN    					0x93

#define         TP_GAIN0        0x3b
#define         TP_GAIN1        0x37


#define         C_TP_GAIN0      TP_GAIN0
#define         C_TP_GAIN1      TP_GAIN1


u8 emi_var[8];

unsigned char gain_tp[MAX_RF_CHANNEL];

const unsigned char rf_chn_cavy[MAX_RF_CHANNEL] = {
	FRE_OFFSET+ 5, FRE_OFFSET+ 8, FRE_OFFSET+11, FRE_OFFSET+14,
	FRE_OFFSET+17, FRE_OFFSET+48, FRE_OFFSET+51, FRE_OFFSET+54,
	FRE_OFFSET+57, FRE_OFFSET+60, FRE_OFFSET+63, FRE_OFFSET+66,
	FRE_OFFSET+69, FRE_OFFSET+72, FRE_OFFSET+75, FRE_OFFSET+78,
};

const unsigned char rf_chn[MAX_RF_CHANNEL] = {
	FRE_OFFSET+ 5, FRE_OFFSET+ 9, FRE_OFFSET+13, FRE_OFFSET+17,
	FRE_OFFSET+22, FRE_OFFSET+26, FRE_OFFSET+30, FRE_OFFSET+35,
	FRE_OFFSET+40, FRE_OFFSET+45, FRE_OFFSET+50, FRE_OFFSET+55,
	FRE_OFFSET+60, FRE_OFFSET+65, FRE_OFFSET+70, FRE_OFFSET+76,
};

//////////////////////////////////////////////////////////////////////////////
//  Setting Table
//////////////////////////////////////////////////////////////////////////////

#if 0

const unsigned char tbl_agc[] = {
0x59,0x59,0x59,0x5a,0x5a,0x5a,0x5a,0x5c, 0x5c,0x5c,0x5c,0x58,0x58,0x58,0x58,0x58,
0x60,0x60,0x60,0x60,0x68,0x68,0xa8,0xa8, 0xe8,0xe8,0x28,0x28,0x28,0x00,0x00,0x00,
0x02,0x03,0x04,0x04,0x05,0x06,0x07,0x05, 0x06,0x07,0x08,0x06,0x07,0x08,0x09,0x0a,
0x09,0x0a,0x0b,0x0c,0x0b,0x0c,0x0b,0x0c, 0x0b,0x0c,0x0b,0x0c,0x0c,0x00,0x43,0x75
};

#else

const unsigned char tbl_agc[] = {
	0x31,0x32,0x33,0x30,0x38,0x3c,0x2c,0x18 ,0x1c,0x0c,0x0c,0x00,0x00,0x00,0x00,0x00,
	0x0a,0x0f,0x15,0x1b,0x21,0x27,0x2e,0x32 ,0x38,0x3e
};

#endif

const TBLCMDSET  tbl_rf_ini[] = {
	0x80, 0x60,  TCMD_UNDER_BOTH | TCMD_WAREG,							  //@@@@@@@
	////////////////////////////8886 deepsleep analog register recover////////////////////////////
	0x06, 0x00,  TCMD_UNDER_BOTH | TCMD_WAREG, //power down control

	0x81, 0xd4,  TCMD_UNDER_BOTH | TCMD_WAREG, //crystal cap 40-5f       //@@ need change
	//0x82, 0x00,  TCMD_UNDER_BOTH | TCMD_WAREG, //enable rxadc clock
	0x83, 0x10,  TCMD_UNDER_BOTH | TCMD_WAREG, //enable bbpll clock

	0x8e, 0x6a,  TCMD_UNDER_BOTH | TCMD_WAREG, //tx/rx vco pre bias[2:0]
	0x8f, 0x1d,  TCMD_UNDER_BOTH | TCMD_WAREG, //tx/rx vco pre bias[2:0]  //@@@@@@@
	0x8d, 0x67,  TCMD_UNDER_BOTH | TCMD_WAREG, //tx/rx vco icur [2:0]     //@@@@@@@
	0xa0, 0x20,  TCMD_UNDER_BOTH | TCMD_WAREG, //dac datapath delay    	  //@@@@@@@
	0xa2, 0x2c,  TCMD_UNDER_BOTH | TCMD_WAREG, //pa_ramp_target
	0xa3, 0x10,  TCMD_UNDER_BOTH | TCMD_WAREG, //						  //--
	0xac, 0xcb,  TCMD_UNDER_BOTH | TCMD_WAREG, //filter center frequency  //@@@@@@@
	0xaa, 0x2e,  TCMD_UNDER_BOTH | TCMD_WAREG, //filter iq_swap           //--


	//new
	0x99, 0x31,  TCMD_UNDER_BOTH | TCMD_WAREG,
	0x95, 0x6f,  TCMD_UNDER_BOTH | TCMD_WAREG,
	0x82, 0x5f,  TCMD_UNDER_BOTH | TCMD_WAREG,
	0x9e, 0x82,  TCMD_UNDER_BOTH | TCMD_WAREG,
	0x93, 0x38,  TCMD_UNDER_BOTH | TCMD_WAREG,   //TP value

	0x04,   0x64, TCMD_UNDER_BOTH | TCMD_WAREG, // analog LDO to max: 64 => e4  //@@@@@@@@@

	0x042d, 0x33,	TCMD_UNDER_BOTH | TCMD_WRITE, //@@@@@@@-0x33


#if 1
	///////////// baseband //////////////////////
	0x0439, 0x72,	TCMD_UNDER_BOTH | TCMD_WRITE,	//
	0x0400, 0x0b,	TCMD_UNDER_BOTH | TCMD_WRITE,	// 2M mode
	0x042b, 0xf3,	TCMD_UNDER_BOTH | TCMD_WRITE,	// preamble
	0x043b, 0xec,	TCMD_UNDER_BOTH | TCMD_WRITE,	//enable timer stamp output
	0x074f, 0x01,	TCMD_UNDER_BOTH | TCMD_WRITE,	//enable system timer
	0x0f04, 0x50,	TCMD_UNDER_BOTH | TCMD_WRITE,	//tx settle time: 80us
	0x0f06, 0x00,	TCMD_UNDER_BOTH | TCMD_WRITE,	//rx wait settle time: 1us
	0x0f0c, 0x50,	TCMD_UNDER_BOTH | TCMD_WRITE,	//rx settle time: 80us
	0x0f10, 0x00,	TCMD_UNDER_BOTH | TCMD_WRITE,	//wait time on NAK

#if		RF_FAST_MODE
	0x0400, 0x0f,	TCMD_UNDER_BOTH | TCMD_WRITE,	// 2M mode
	0x0402, 0x28,	TCMD_UNDER_BOTH | TCMD_WRITE,	// 20-byte pre-amble
	0x042b, 0xf1,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code: 1

	0x0420, 0x1e,	TCMD_UNDER_BOTH | TCMD_WRITE,	// threshold
	0x0421, 0x04,	TCMD_UNDER_BOTH | TCMD_WRITE,	// no avg
	0x0422, 0x00,	TCMD_UNDER_BOTH | TCMD_WRITE,	// threshold
	0x0424, 0x12,	TCMD_UNDER_BOTH | TCMD_WRITE,	// number fo sync: bit[6:4]
	0x0464, 0x07,	TCMD_UNDER_BOTH | TCMD_WRITE,	// new sync: bit0
	0x04cd, 0x04,	TCMD_UNDER_BOTH | TCMD_WRITE,	// enable packet lenght = 0
#endif

	/////////////////////// 2m mode /////////////////////////////////////////////////
	0x0404, 0xca,	TCMD_UNDER_BOTH | TCMD_WRITE,	// head_mode/crc_mode: normal c0
	0x0f03, 0x36,	TCMD_UNDER_BOTH | TCMD_WRITE,	// bit3: crc2_en; normal 1e
//	0x04ca, 0xa0,	TCMD_UNDER_BOTH | TCMD_WRITE,	//head_chn, report rx sync channel in packet
	0x04ca, 0xa8,	TCMD_UNDER_BOTH | TCMD_WRITE,	//enable DC distance bit 3
	0x04cb, 0x04,	TCMD_UNDER_BOTH | TCMD_WRITE,	// set distance

	0x0405, 0x05,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code length 5
	0x0420, 0x26,	TCMD_UNDER_BOTH | TCMD_WRITE,	// sync threshold: 1e (4); 26 (5)
	//0x0430, 0x12,	TCMD_UNDER_BOTH | TCMD_WRITE,	// for 8us preamble
	//0x043d, 0x71,	TCMD_UNDER_BOTH | TCMD_WRITE,	// for 8us preamble

	0x0408, 0x71,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code byte3
	0x0409, 0x76,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code byte2
	0x040a, 0x51,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code byte1
	0x040b, 0x39,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code byte0
	0x042c, 0x38,	TCMD_UNDER_BOTH | TCMD_WRITE,	// maxiumum length 48-byte

#if RF_MANUAL_AGC_MAX_GAIN
	0x0433, 0x00,	TCMD_UNDER_BOTH | TCMD_WRITE,	// set mgain disable 01 -> 00
	0x0434, 0x01,	TCMD_UNDER_BOTH | TCMD_WRITE,	// pel0: 21 -> 01
	0x043a, 0x77,	TCMD_UNDER_BOTH | TCMD_WRITE,	// Rx signal power change threshold: 22 -> 77
	0x043e, 0xc9,	TCMD_UNDER_BOTH | TCMD_WRITE,	// set rx peak detect manual: 20 -> c9
	0x04cd, 0x00,	TCMD_UNDER_BOTH | TCMD_WRITE,	// fix rst_pga=0: 04 -> 00
#else
	0x043a, 0x44,	TCMD_UNDER_BOTH | TCMD_WRITE,	// Rx signal power change threshold: 22 -> 44
#endif

#endif
};

///////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////

const TBLCMDSET  tbl_tp_ini[] = {
	0x0e0c, 0x40,		TCMD_UNDER_BOTH | TCMD_WRITE,	//set PLL gain to center
	//0x0400, 0x4b,		TCMD_UNDER_BOTH | TCMD_WRITE,	//turn on TX test mode, output 0
	0x0065, 0x01,		TCMD_UNDER_BOTH | TCMD_WRITE,	//K500
};


const TBLCMDSET  tbl_tp_close[] = {
	0x0e0c, 0x40,		TCMD_UNDER_BOTH | TCMD_WRITE,	//set PLL gain to center
	//0x0400, 0x0b,		TCMD_UNDER_BOTH | TCMD_WRITE,	//set BB to 2M mode
	0x0065, 0x00,		TCMD_UNDER_BOTH | TCMD_WRITE,	//K500
};

//////////////////////////////////////////////////////////////////////
void rf_set_ble_channel (signed char chn) {}
void rf_set_rxmode_250k () {}
void rf_drv_2m () {}
void rf_drv_1m () {}
void rf_drv_250k () {}

//////////////////////////////////////////////////////////////////////
unsigned char gain_tp0;
unsigned char gain_tp1;


int rf_drv_init (int wakeup_from_suspend)
{
	unsigned char irqst = read_reg8(0x800643);
	write_reg8(0x800643,0x00);

	unsigned char i;

	if (!wakeup_from_suspend){

		LoadTblCmdSet (tbl_rf_ini, sizeof (tbl_rf_ini)/sizeof (TBLCMDSET));

		for (i=0; i<26; i++)
			write_reg8 (0x800480+i, tbl_agc[i]);	//set AGC table

		/////////////////////////////////////
		// Two Point Calibration
		/////////////////////////////////////

		gain_tp0 = C_TP_GAIN0;
		gain_tp1 = C_TP_GAIN1;

		for (i=0; i<RF_CHANNEL_MAX; i++) {
			gain_tp[i] = (i * gain_tp1 + (16 - i) * gain_tp0) >> 4;
		}
	}
	else {
		WriteAnalogReg (0x06,0x0);
		WaitUs (1);
	}
	write_reg8(0x800643,irqst);

	return 0;
}

/////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////
/*
	0x25, 0x7c, 0x67, 0x67, 	// 8.85  34.15
	0x1b, 0x7c, 0x67, 0x62, 	// 8.41  31.53
	0x14, 0x7c, 0x67, 0x61,		// 7.95  29.89
	0x0d, 0x7c, 0x67, 0x61,		// 7.35  28.31
	0x0a, 0x7c, 0x67, 0x61,		// 6.95  27.39
	0x08, 0x7c, 0x67, 0x61,		// 6.5   26.52
	0x06, 0x7c, 0x67, 0x64,		// 5.9   25.78
	0x06, 0x6c, 0x67, 0x64,		// 5.35  24.9
	0x06, 0x6c,	0x43, 0x64,		// 3.75  22.67
	0x06, 0x64, 0xc3, 0x64,		// 2.8   21.44
	0x06, 0x64, 0xc2, 0x64,		// -0.4  18.88
	0x09, 0x7c, 0xc1, 0x64,		// -2.85 17.2
	0x08, 0x7c, 0xc1, 0x64,		// -3.5  16.87
	0x06, 0x64,	0xc1, 0x64,		// -5.9  16.13
	0x05, 0x7c, 0x67, 0x61,		// -6.15 16.79
	0x04, 0x7c, 0x67, 0x61,		// -9.15 16.25
	0x03, 0x7c, 0x67, 0x61,		// -12.2 15.7
	0x02, 0x7c, 0x67, 0x67,		// -16   16.04
	0x01, 0x7c, 0x67, 0x67,		// -20   15.68
	0x00, 0x7c, 0x67, 0x67,		// -24   15.5
	0x00, 0x64, 0x67, 0x67,		// -25   15.07
	0x00, 0x64, 0x43, 0x67,		// -27.5 14.75
	0x00, 0x64, 0x43, 0x61,		// -31   14.07
*/

#define  ANALOG_REG04_SET_FOR_RX  0x00
const char tbl_rf_power[] = {
    //a2    04    a7    8d

	0x25, ANALOG_REG04_SET_FOR_RX | 0x7c, 0x43, 0x67,  // 4.4
    0x14, ANALOG_REG04_SET_FOR_RX | 0x7c, 0x43, 0x67,  // 3.2
    0x0d, ANALOG_REG04_SET_FOR_RX | 0x7c, 0x43, 0x67,  // 2.2
    0x0a, ANALOG_REG04_SET_FOR_RX | 0x7c, 0x43, 0x67,  // 1.55
    0x06, ANALOG_REG04_SET_FOR_RX | 0x7c, 0x43, 0x67,  // 0.2
    0x06, ANALOG_REG04_SET_FOR_RX | 0x64, 0xc3, 0x67,  // -2
    0x06, ANALOG_REG04_SET_FOR_RX | 0x7c, 0xc2, 0x67,  // -4.4
    0x09, ANALOG_REG04_SET_FOR_RX | 0x7c, 0xc1, 0x67,  // -6.6
    0x08, ANALOG_REG04_SET_FOR_RX | 0x7c, 0xc1, 0x67,  // -7.3
    0x06, ANALOG_REG04_SET_FOR_RX | 0x7c, 0xc1, 0x67,  // -10

};

void rf_set_power_level_index (int level)
{
	if (level + 1 > (sizeof (tbl_rf_power)>>2)) {
		level = (sizeof (tbl_rf_power)>>2) - 1;
	}
	u8 *p = tbl_rf_power + level * 4;
	analog_write (0xa2, *p ++);
	analog_write (0x04, *p ++);
	analog_write (0xa7, *p ++);
	analog_write (0x8d, *p ++);
}

_attribute_ram_code_ void	rf_power_enable (int en)
{
	analog_write (0x06, en ? 0x00 : 0xff);		//turn off transceiver
	analog_write (0x05, en ? 0x00 : 0x10);		//turn off transceiver
	// RC clock tracking enable
	//analog_write (0x24, en ? BIT(3) | BIT(1) : 0);
}

void	rf_trx_off ()
{
	analog_write (0x06, 0xfe);		//turn off transceiver
}

_attribute_ram_code_ void rf_set_channel (unsigned char chn, unsigned short set)
{
	u8 gain;
	/////////////////// turn on LDO and baseband PLL ////////////////
	analog_write (0x06, 0x00);
	write_reg8 (0x800f16, 0x29);

	write_reg8 (0x800428, RF_TRX_MODE);	// rx disable
	if ( set & RF_SET_TX_MANAUL ){
        write_reg8 (0x800f02, RF_TRX_OFF_MANUAL);  // reset tx/rx state machine
    }
    else{
	    write_reg8 (0x800f02, RF_TRX_OFF);	// reset tx/rx state machine
    }
    
	if (set & RF_CHN_TABLE) {
		gain = gain_tp[chn&15]; //set gain

		chn = rf_chn_cavy[chn];
	}
	else {
		gain =  gain_tp[((chn * 5) >> 5)&15]; //set gain
	}
#if(HIGH_FRE_ABOVE_2600)
	u32 fre = 2600 + chn;
#else
	u32 fre = 2400 + chn;
#endif
    write_reg16 (0x8004d6, fre);	// {intg_N}
    write_reg32 (0x8004d0, (fre - 2) * 58254 + 1125);	// {intg_N, frac}
    write_reg8 (0x8004d4, 07);  // {intg_N, frac} to fix emi carrier spur

    //analog_write (TX_GAIN, gain);
#if(HIGH_FRE_ABOVE_2600)
    analog_write (TX_GAIN, 0x30);
#else
    analog_write (TX_GAIN, gain);
#endif
}

_attribute_ram_code_ void rf_set_tx_rx_off ()
{
	/////////////////// turn on LDO and baseband PLL ////////////////
	//analog_write (0x06, 0xfe);
	write_reg8 (0x800f16, 0x29);
	write_reg8 (0x800428, RF_TRX_MODE);	// rx disable
	write_reg8 (0x800f02, RF_TRX_OFF);	// reset tx/rx state machine
}

void rf_set_rxmode ()
{
    write_reg8 (0x800428, RF_TRX_MODE | BIT(0));	// rx enable
    write_reg8 (0x800f02, RF_TRX_OFF | BIT(5));		// RX enable
}

void rf_set_txmode ()
{
	write_reg8  (0x800f02, RF_TRX_OFF | BIT(4));	// TX enable
}

void rf_send_packet (void* addr, u16 rx_waittime, u8 retry)
{
	//analog_write (0x06, 0x0);					//turn on RFPLL
	write_reg8  (0x800f00, 0x80);				// stop
	write_reg8  (0x800f14, retry);				// number of retry
	write_reg16 (0x80050c, (u16)((u32)addr));
	write_reg16 (0x800f0a, rx_waittime);
#if RF_FAST_MODE_2M
	write_reg16 (0x800f00, 0x3f83);				// start tx with PID = 0
#else
	write_reg16 (0x800f00, 0x3f81);				// start tx with PID = 0
	//write_reg8 (0x800524, 0x08);
#endif
}

void rf_send_packet_from_rx (void* addr)
{
	write_reg8 (0x800f02, RF_TRX_OFF);			// TRX manual mode off
	write_reg16 (0x80050c, (u16)((u32)addr));

#if RF_FAST_MODE_2M
	write_reg16 (0x800f00, 0x3f85);				// start tx with PID = 0, simple tx
#else
	write_reg16 (0x800f00, 0x3f81);				// start tx with PID = 0
	//write_reg8 (0x800524, 0x08);
#endif
}

void rf_send_single_packet (void* addr)
{
	write_reg8 (0x800f02, RF_TRX_OFF);			// TRX manual mode off
	analog_write (0x06, 0x0);					//turn on RFPLL
	write_reg8  (0x800f00, 0x80);				// stop
	write_reg16 (0x80050c, (u16)((u32)addr));

#if RF_FAST_MODE_2M
	write_reg16 (0x800f00, 0x3f85);				// start tx with PID = 0, simple tx
#else
	write_reg16 (0x800f00, 0x3f81);				// start tx with PID = 0
	//write_reg8 (0x800524, 0x08);
#endif
}

void rf_multi_receiving_init (u8 channel_mask)
{
//	write_reg8  (0x800f04, 0x50);			// set tx settle to 150us,
	write_reg8  (0x800f03, 0x30);			// disable rx timeout
	write_reg8  (0x800f15, 0xe0);			// disable TX manuuanl mode
	write_reg8  (0x800407, channel_mask);	// channel mask
}


void rf_receiving_pipe_enble(u8 channel_mask)
{
	write_reg8  (0x800407, channel_mask);	// channel mask
}

void rf_multi_receiving_start  (signed char chn, unsigned short set)
{
	write_reg8  (0x800f00, 0x80);			// stop
//	write_reg8  (0x800060, 0x80);			// reset baseband
//	write_reg8  (0x800060, 0x00);
	write_reg8 (0x800f02, RF_TRX_OFF);		// reset tx/rx state machine
	rf_set_channel (chn,  set);
	write_reg16  (0x800f00, 0x3f84);		// start rx with PID = 0
}

void rf_multi_receiving_send_packet  (void* addr)
{
//	write_reg8  (0x800060, 0x80);				// reset baseband
//	write_reg8  (0x800060, 0x00);
	write_reg8  (0x800f14, 0);					// number of retry
	write_reg16 (0x80050c, (u16)((u32)addr));
	write_reg16 (0x800f0a, 1);					// rx wait time
	write_reg16 (0x800f00, 0x3f83);				// start tx with PID = 0
	write_reg8 (0x800524, 0x08);
}

_attribute_ram_code_ void rf_start_stx  (void* addr, u32 tick)
{
//	write_reg32 (0x800f04, 0);							// tx wail & settle time: 0
	write_reg32(0x800f18, tick);						// Setting schedule trigger time
    write_reg8(0x800f16, read_reg8(0x800f16) | 0x04);	// Enable cmd_schedule mode
	write_reg16 (0x800f00, 0x3f85);						// single TX
	write_reg16 (0x80050c, (u16)((u32)addr));
}

void rf_start_stx2rx  (void* addr, u32 tick)
{
//	write_reg32 (0x800f04, 0x050);						// tx settle time: 80 us
	write_reg32(0x800f18, tick);						// Setting schedule trigger time
    write_reg8(0x800f16, read_reg8(0x800f16) | 0x04);	// Enable cmd_schedule mode
	write_reg16 (0x800f00, 0x3f87);						// single tx2rx
	write_reg16 (0x80050c, (u16)((u32)addr));
	//write_reg32 (0x800f04, 0x0);						// tx settle time: 80 us
}

void rf_start_btx (void* addr, u32 tick)
{
//	write_reg32 (0x800f04, 0x050);						// tx settle time: 80 us
	write_reg32(0x800f18, tick);						// Setting schedule trigger time
    write_reg8(0x800f16, read_reg8(0x800f16) | 0x04);	// Enable cmd_schedule mode
	write_reg8 (0x800f00, 0x81);						// ble tx
	write_reg16 (0x80050c, (u16)((u32)addr));
}

void rf_start_srx2tx  (void* addr, u32 tick)
{
//	write_reg32 (0x800f04, 0x50);						// tx_wait = 0; tx_settle = 80 us
	write_reg32 (0x800f28, 0x0fffffff);					// first timeout
	write_reg32(0x800f18, tick);						// Setting schedule trigger time
    write_reg8(0x800f16, read_reg8(0x800f16) | 0x04);	// Enable cmd_schedule mode
	write_reg16 (0x800f00, 0x3f88);						// single rx2tx
	write_reg16 (0x80050c, (u16)((u32)addr));
}

void rf_start_brx  (void* addr, u32 tick)
{
//	write_reg32 (0x800f04, 0x50);						// tx_wait = 0; tx_settle = 80 us
	write_reg32 (0x800f28, 0x0fffffff);					// first timeout
	write_reg32(0x800f18, tick);						// Setting schedule trigger time
    write_reg8(0x800f16, read_reg8(0x800f16) | 0x04);	// Enable cmd_schedule mode
	write_reg8 (0x800f00, 0x82);						// ble rx
	write_reg16 (0x80050c, (u16)((u32)addr));
}

_attribute_ram_code_ void rf_set_ack_packet  (void* addr)
{
	write_reg16 (0x80050c, (u16)((u32)addr));
}

//////////////////////////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////////////////////////
void SetRxMode (signed char chn, unsigned short set)
{
	rf_set_channel (chn, set);
	rf_set_rxmode ();
}

void SetTxMode (signed char chn, unsigned short set)
{
	rf_set_channel (chn, set);
	rf_set_txmode ();
}

void TxPkt (void* addr)
{
	write_reg16 (0x80050c, (u16)((u32)addr));
	write_reg8 (0x800524, 0x08);
}

/////////////////////////////////////////////////////////////
// Function to trigger and receive count values for the
// current freq setting
/////////////////////////////////////////////////////////////

#endif
