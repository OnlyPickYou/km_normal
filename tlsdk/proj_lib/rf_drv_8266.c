
//  to detect MCU_CORE_TYPE in app mode
#if(!(__TL_LIB_8266__ ))
#include "../proj/config/user_config.h"
#include "../proj/mcu/config.h"		// must include this
#endif 

#if(__TL_LIB_8266__ || (MCU_CORE_TYPE && MCU_CORE_TYPE == MCU_CORE_8266))
#include "../proj/common/types.h"
#include "../proj/common/compatibility.h"
#include "../proj/common/bit.h"
#include "../proj/common/utility.h"
#include "../proj/common/static_assert.h"
#include "../proj/mcu/compiler.h"
#include "../proj/mcu/register.h"
#include "../proj/mcu/analog.h"
#include "../proj/mcu/anareg.h"
#include "rf_drv_8266.h"

#ifndef			REVISION_8266_A2
#define			REVISION_8266_A2		0
#endif

#ifndef			REVISION_8266_A0
#define			REVISION_8266_A0		0
#endif

#if				REVISION_8266_A0
	#if (RF_FAST_MODE_1M)
		#define			TP_GAIN0		0x2a
		#define			TP_GAIN1		0x28
	#else
		#define			TP_GAIN0		0x55
		#define			TP_GAIN1		0x50
	#endif
#else
	#if (RF_FAST_MODE_1M)
		#define			TP_GAIN0		0x1d
		#define			TP_GAIN1		0x19
	#else
		#define			TP_GAIN0		0x40
		#define			TP_GAIN1		0x39
	#endif
#endif

#define		RF_MANUAL_AGC_MAX_GAIN		1


#define		TP_2M_G0			0x40
#define		TP_2M_G1			0x39

#define		TP_1M_G0			0x1d
#define		TP_1M_G1			0x19


#define     TX_GAIN    		0x93

u8 emi_var[8];

#define		TP_GET_GAIN(g0, g1)		((g0 - g1)*256/80)

int			rf_tp_base = TP_2M_G0;
int			rf_tp_gain = TP_GET_GAIN(TP_2M_G0, TP_1M_G1);


int 		xtalType_rfMode;
u8 		sar_adc_pwdn_en = 0;


const unsigned char rf_chn[MAX_RF_CHANNEL] = {
	FRE_OFFSET+ 5, FRE_OFFSET+ 9, FRE_OFFSET+13, FRE_OFFSET+17,
	FRE_OFFSET+22, FRE_OFFSET+26, FRE_OFFSET+30, FRE_OFFSET+35,
	FRE_OFFSET+40, FRE_OFFSET+45, FRE_OFFSET+50, FRE_OFFSET+55,
	FRE_OFFSET+60, FRE_OFFSET+65, FRE_OFFSET+70, FRE_OFFSET+76,
};

//////////////////////////////////////////////////////////////////////////////
//  Setting Table
//////////////////////////////////////////////////////////////////////////////
const unsigned char tbl_agc[] = {
	0x31,0x32,0x33,0x30,0x38,0x3c,0x2c,0x18 ,0x1c,0x0c,0x0c,0x00,0x00,0x00,0x00,0x00,
	0x0a,0x0f,0x15,0x1b,0x21,0x27,0x2e,0x32 ,0x38,0x3e
};


const TBLCMDSET  tbl_rf_ini_16M_Crystal_1m_Mode[] = {  //need confirm
	0x04eb, 0x60,  TCMD_UNDER_BOTH | TCMD_WRITE,
	0x99, 	0x31,  TCMD_UNDER_BOTH | TCMD_WAREG,
	0x82,	0x34,  TCMD_UNDER_BOTH | TCMD_WAREG,
	0x9e, 	0x41,  TCMD_UNDER_BOTH | TCMD_WAREG,
};

const TBLCMDSET  tbl_rf_ini_12M_Crystal_1m_Mode[] = { //same as 8267
	0x04eb, 0xe0,  TCMD_UNDER_BOTH | TCMD_WRITE,
	0x99, 	0xb1,  TCMD_UNDER_BOTH | TCMD_WAREG,
	0x82,	0x20,  TCMD_UNDER_BOTH | TCMD_WAREG,
	0x9e, 	0x56,  TCMD_UNDER_BOTH | TCMD_WAREG,
};

const TBLCMDSET  tbl_rf_ini_16M_Crystal_2m_Mode[] = { //need confirm
	0x04eb, 0x60,  TCMD_UNDER_BOTH | TCMD_WRITE,
	0x99, 	0x31,  TCMD_UNDER_BOTH | TCMD_WAREG,
	0x82,	0x34,  TCMD_UNDER_BOTH | TCMD_WAREG,
	0x9e, 	0x82,  TCMD_UNDER_BOTH | TCMD_WAREG,
};

const TBLCMDSET  tbl_rf_ini_12M_Crystal_2m_Mode[] = { //same as 8267
	0x04eb, 0xe0,  TCMD_UNDER_BOTH | TCMD_WRITE,
	0x99, 	0xb1,  TCMD_UNDER_BOTH | TCMD_WAREG,
	0x82,	0x20,  TCMD_UNDER_BOTH | TCMD_WAREG,
	0x9e, 	0xad,  TCMD_UNDER_BOTH | TCMD_WAREG,
};


const TBLCMDSET  tbl_rf_ini[] = {
	//RNG genertor
	0x045b, 0x02,	TCMD_UNDER_BOTH | TCMD_WRITE,	// RNG clock by 2
	0x0447, 0x08,	TCMD_UNDER_BOTH | TCMD_WRITE,	// RNG free running mode
	0x85,   0x00,  TCMD_UNDER_BOTH | TCMD_WAREG, 		//turn on RNG

	//crystal cap setting
	0x80, 0x61,  TCMD_UNDER_BOTH | TCMD_WAREG, //crystal cap 40-5f
	////////////////////////////8886 deepsleep analog register recover////////////////////////////
	0x06, 0x00,  TCMD_UNDER_BOTH | TCMD_WAREG, //power down control
	0x8f, 0x30,  TCMD_UNDER_BOTH | TCMD_WAREG, //boot rx vco current, temporary fix
//	0x88, 0x04,  TCMD_UNDER_BOTH | TCMD_WAREG, //48M PLL

	0x81, 0xd0,  TCMD_UNDER_BOTH | TCMD_WAREG, //crystal cap c0-df

#if REVISION_8266_A0		//	for A0
	0x8b, 0xe3,  TCMD_UNDER_BOTH | TCMD_WAREG, //boot rx vco current, temporary fix
	0x8e, 0x6b,  TCMD_UNDER_BOTH | TCMD_WAREG, //tx/rx vco pre bias[2:0]
	0x8d, 0x67,  TCMD_UNDER_BOTH | TCMD_WAREG, //tx/rx vco icur [2:0]
	0x0402, 0x2c,	TCMD_UNDER_BOTH | TCMD_WRITE,	// 20-byte pre-amble
#elif	REVISION_8266_A2
	0x8f, 0x38,  TCMD_UNDER_BOTH | TCMD_WAREG, //vcom
	0x8b, 0xe3,  TCMD_UNDER_BOTH | TCMD_WAREG, //boot rx vco current, temporary fix
	0x8e, 0x6b,  TCMD_UNDER_BOTH | TCMD_WAREG, //tx/rx vco pre bias[2:0]
	0x8d, 0x67,  TCMD_UNDER_BOTH | TCMD_WAREG, //tx/rx vco icur [2:0]
	0x0402, 0x26,	TCMD_UNDER_BOTH | TCMD_WRITE,	// 20-byte pre-amble
#else
	0x8f, 0x38,  TCMD_UNDER_BOTH | TCMD_WAREG, //vcom
	0x8b, 0xe3,  TCMD_UNDER_BOTH | TCMD_WAREG, //boot rx vco current, temporary fix
	0x8e, 0x6b,  TCMD_UNDER_BOTH | TCMD_WAREG, //tx/rx vco pre bias[2:0]
	0x8d, 0x67,  TCMD_UNDER_BOTH | TCMD_WAREG, //tx/rx vco icur [2:0]
	0x0402, 0x26,	TCMD_UNDER_BOTH | TCMD_WRITE,	// 20-byte pre-amble
#endif

	0xa0, 0x28,  TCMD_UNDER_BOTH | TCMD_WAREG, //dac datapath delay
	0xa2, 0x2c,  TCMD_UNDER_BOTH | TCMD_WAREG, //pa_ramp_target
	0xa3, 0x10,  TCMD_UNDER_BOTH | TCMD_WAREG, //pa_ramp_en = 1, pa ramp table max
	0xac, 0xa7,  TCMD_UNDER_BOTH | TCMD_WAREG, //@@@@@@@  rx bandwidth add 15%  fixed by sihui	0xaa, 0x2e,  TCMD_UNDER_BOTH | TCMD_WAREG, //filter iq_swap
	////////////// adc setting		////////////////////////////////////
#if 1
	0x0069,	0x04,	TCMD_UNDER_BOTH | TCMD_WRITE,	// ADC 5M clock
	0x006a,	0xc0,	TCMD_UNDER_BOTH | TCMD_WRITE,	// ADC 5M clock
	0x006b,	0x80,	TCMD_UNDER_BOTH | TCMD_WRITE,	// ADC 5M clock
	0x003c, 0x10,	TCMD_UNDER_BOTH | TCMD_WRITE,	// channel_m resolution[5:3]; m_tsamp[2:0]
	0x002f,	0x14,	TCMD_UNDER_BOTH | TCMD_WRITE,	// channel_lr resolution
	0x002b,	0x00,	TCMD_UNDER_BOTH | TCMD_WRITE,	// reference 1.3v (0); 3.3(1)
	0x0033,	0x27,	TCMD_UNDER_BOTH | TCMD_WRITE,	// enable l & r: l for amic; r for adc
#endif

	///////////// baseband //////////////////////
	0x0439, 0x72,	TCMD_UNDER_BOTH | TCMD_WRITE,	//
	0x0400, 0x0b,	TCMD_UNDER_BOTH | TCMD_WRITE,	// 2M mode
	0x042b, 0xf3,	TCMD_UNDER_BOTH | TCMD_WRITE,	// preamble
	0x043b, 0xfc,	TCMD_UNDER_BOTH | TCMD_WRITE,	//enable timer stamp & dc output
	0x0f04, 0x50,	TCMD_UNDER_BOTH | TCMD_WRITE,	//tx settle time: 80us
	0x0f06, 0x00,	TCMD_UNDER_BOTH | TCMD_WRITE,	//rx wait settle time: 1us
	0x0f0c, 0x50,	TCMD_UNDER_BOTH | TCMD_WRITE,	//rx settle time: 80us
	0x0f10, 0x00,	TCMD_UNDER_BOTH | TCMD_WRITE,	//wait time on NAK
//	0x0f16, 0x23,	TCMD_UNDER_BOTH | TCMD_WRITE,	//192M bbpll reset enable
//	0x0069, 0x05,	TCMD_UNDER_BOTH | TCMD_WRITE,	// set ADC clock to 4MHz
	0x0438, 0xb7, 	TCMD_UNDER_BOTH | TCMD_WRITE,     //pwrDft 0x37, BIT(7) : r_pkt_chg_disable ???

#if		RF_FAST_MODE
	0x0400, 0x0f,	TCMD_UNDER_BOTH | TCMD_WRITE,	// 2M mode
	0x042b, 0xf1,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code: 1

	0x0420, 0x1e,	TCMD_UNDER_BOTH | TCMD_WRITE,	// threshold
	0x0421, 0x04,	TCMD_UNDER_BOTH | TCMD_WRITE,	// no avg
	0x0422, 0x00,	TCMD_UNDER_BOTH | TCMD_WRITE,	// threshold
	0x0424, 0x12,	TCMD_UNDER_BOTH | TCMD_WRITE,	// number fo sync: bit[6:4]
	0x0464, 0x07,	TCMD_UNDER_BOTH | TCMD_WRITE,	// new sync: bit0
	0x04cd, 0x04,	TCMD_UNDER_BOTH | TCMD_WRITE,	// enable packet lenght = 0
#endif
	0x042d, 0x33,	TCMD_UNDER_BOTH | TCMD_WRITE,	// ********DC alpha=1/8, bit[6:4]
	0x04cb, 0x04,   TCMD_UNDER_BOTH | TCMD_WRITE,	// set distance
};


//		  poweron defalut	 ble_1m(12M crystal)
//ana_9e:		80				56
//ana_a3:		c0				f0
//ana_aa:		2a				26
const TBLCMDSET  tbl_rf_1m[] = {
	////////////////////// ble mode //////////////////////////////////////////////////
	0xa3, 0xf0,  TCMD_UNDER_BOTH | TCMD_WAREG, 		//pa_ramp_en = 1, pa ramp
	0xaa, 0x26,  TCMD_UNDER_BOTH | TCMD_WAREG,		 //filter iq_swap, 1M bandwidth

	0x0401, 0x08,	TCMD_UNDER_BOTH | TCMD_WRITE,	// pn enable
	0x0402, 0x24,	TCMD_UNDER_BOTH | TCMD_WRITE,	// 8-byte pre-amble
	0x0404, 0xf5,	TCMD_UNDER_BOTH | TCMD_WRITE,	// head_mode/crc_mode: normal c0

	//access code setting
	0x0405, 0x04,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code length 4
	0x0420, 0x1f,	TCMD_UNDER_BOTH | TCMD_WRITE,	// sync threshold: 1e (4); 26 (5)
	0x0408, 0x8e,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code byte3
	0x0409, 0x89,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code byte2
	0x040a, 0xbe,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code byte1
	0x040b, 0xd6,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code byte0

	0x0430, 0x12,	TCMD_UNDER_BOTH | TCMD_WRITE,	// for 8us preamble
	0x043d, 0xb1,	TCMD_UNDER_BOTH | TCMD_WRITE,	// for 8us preamble

	0x0f03, 0x1e,	TCMD_UNDER_BOTH | TCMD_WRITE,	// bit3: crc2 enable
	0x0f04, 0x68,	TCMD_UNDER_BOTH | TCMD_WRITE,	// tx settle time

	0x04ca, 0x88,	TCMD_UNDER_BOTH | TCMD_WRITE,   //i dont know how to set core_4ca in 1m_mode
	0x042c, 0x30,	TCMD_UNDER_BOTH | TCMD_WRITE,	// maxiumum length 48-byte


};



const TBLCMDSET  tbl_rf_2m[] = {
/////////////////////// nordic /////////////////////////////////////////////////
	0xa3, 0x10,  TCMD_UNDER_BOTH | TCMD_WAREG, 		//pa_ramp_en = 1, pa ramp
	0xaa, 0x2e,  TCMD_UNDER_BOTH | TCMD_WAREG,		 //filter iq_swap, 1M bandwidth
	0x0401, 0x00,	TCMD_UNDER_BOTH | TCMD_WRITE,	// pn disable
	0x0402, 0x26,	TCMD_UNDER_BOTH | TCMD_WRITE,	// 8-byte pre-amble
#if	RF_LONG_PACKET_EN
	0x0404, 0xc8,	TCMD_UNDER_BOTH | TCMD_WRITE,	// head_mode/crc_mode:
#else
	0x0404, 0xca,	TCMD_UNDER_BOTH | TCMD_WRITE,	// head_mode/crc_mode:
#endif

	//access code setting
	0x0405, 0x05,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code length 5
	0x0420, 0x26,	TCMD_UNDER_BOTH | TCMD_WRITE,	// sync threshold: 1e (4); 26 (5)
	0x0408, 0x71,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code byte3
	0x0409, 0x76,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code byte2
	0x040a, 0x51,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code byte1
	0x040b, 0x39,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code byte0

	0x0430, 0x12,	TCMD_UNDER_BOTH | TCMD_WRITE,	// for 8us preamble
	0x043d, 0xb1,	TCMD_UNDER_BOTH | TCMD_WRITE,	// for 	8us preamble
	0x04ca, 0xa8,	TCMD_UNDER_BOTH | TCMD_WRITE,	//head_chn, report rx sync channel in packet

	0x0f03, 0x36,	TCMD_UNDER_BOTH | TCMD_WRITE,	//initial prx pid 3 for Nordic mode,crc2 disable
	0x0f04, 0x50,	TCMD_UNDER_BOTH | TCMD_WRITE,	// tx settle time

	0x042c, 0x80,	TCMD_UNDER_BOTH | TCMD_WRITE,	// maxiumum length 128-byte(poweron default value)
};

void rf_drv_1m (void)
{
	rf_tp_base = TP_1M_G0;
	rf_tp_gain = TP_GET_GAIN(TP_1M_G0, TP_1M_G1);
	LoadTblCmdSet (tbl_rf_1m, sizeof (tbl_rf_1m)/sizeof (TBLCMDSET));

	if( IS_XTAL_12M(xtalType_rfMode) ){
		rf_set_12M_Crystal_1m_mode();
	}
	else{
		rf_set_16M_Crystal_1m_mode();
	}
}

void rf_drv_2m (void)
{
	rf_tp_base = TP_2M_G0;
	rf_tp_gain = TP_GET_GAIN(TP_2M_G0, TP_2M_G1);
	LoadTblCmdSet (tbl_rf_2m, sizeof (tbl_rf_2m)/sizeof (TBLCMDSET));

	if( IS_XTAL_12M(xtalType_rfMode) ){
		rf_set_12M_Crystal_2m_mode();
	}
	else{
		rf_set_16M_Crystal_2m_mode();
	}
}

const TBLCMDSET  tbl_rf_gain_manualMax[] = {
	0x0433, 0x00,	TCMD_UNDER_BOTH | TCMD_WRITE,	// set mgain disable 01 -> 00                 (fix)
	0x0434, 0x01,	TCMD_UNDER_BOTH | TCMD_WRITE,	// pel0: 21 -> 01                             (fix)
	0x043a, 0x77,	TCMD_UNDER_BOTH | TCMD_WRITE,	// Rx signal power change threshold: 22 -> 77 (fix)
	0x043e, 0xc9,	TCMD_UNDER_BOTH | TCMD_WRITE,	// set rx peak detect manual: 20 -> c9        (fix)
	0x04cd, 0x06,	TCMD_UNDER_BOTH | TCMD_WRITE,	// fix rst_pga=0: len = 0 enable              (fix)
//	0x04c0, 0x81,	TCMD_UNDER_BOTH | TCMD_WRITE,	// lowpow agc/sync: 87                        (fix)
};


const TBLCMDSET  tbl_rf_gain_AGC[] = {
	0x0433, 0x01,	TCMD_UNDER_BOTH | TCMD_WRITE,  // set mgain enable 00 -> 01
	0x0434, 0x21,	TCMD_UNDER_BOTH | TCMD_WRITE,
	0x043a, 0x22,	TCMD_UNDER_BOTH | TCMD_WRITE,
	0x043e, 0x20,	TCMD_UNDER_BOTH | TCMD_WRITE,
	0x04cd, 0x04,	TCMD_UNDER_BOTH | TCMD_WRITE,
};


void rf_update_tp_value (u8 tp0, u8 tp1)
{
	 rf_tp_base = tp0;
	 rf_tp_gain = TP_GET_GAIN(tp0, tp1);
}


void rf_drv_init (int xtal_type)  //1: 1M   0: 2M
{
	LoadTblCmdSet (tbl_rf_ini, sizeof (tbl_rf_ini)/sizeof (TBLCMDSET));


	xtalType_rfMode = xtal_type;

	if(IS_RF_1M_MODE(xtalType_rfMode)){
		rf_drv_1m();
	}
	else if(IS_RF_2M_MODE(xtalType_rfMode)){
		rf_drv_2m();
	}
	else{
		//250k
	}

	//rf_set_gain_manualMax();
	write_reg8(0x80043a, 0x44);  //AGC mode

	for (int i=0; i<26; i++){
		write_reg8 (0x800480+i, tbl_agc[i]);	//set AGC table
	}
}

/////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
// have to be located in direct access memory (non-cache)
/////////////////////////////////////////////////////////////////////

const char tbl_rf_power[] = {
  //a2    04    a7    8d
	0x25, 0x7c, 0x67, 0x67,		// 7 dBm
	0x0a, 0x7c, 0x67, 0x67,		// 5 dBm
	0x06, 0x74, 0x43, 0x61,		// -0.6
	0x06, 0x64, 0xc2, 0x61,		// -4.3
	0x06, 0x64, 0xc1, 0x61,		// -9.5
	0x05, 0x7c, 0x67, 0x67,		// -13.6
	0x03, 0x7c, 0x67, 0x67,		// -18.8
	0x02, 0x7c, 0x67, 0x67,		// -23.3
	0x01, 0x7c, 0x67, 0x67,		// -27.5
	0x00, 0x7c, 0x67, 0x67,		// -30
	0x00, 0x64, 0x43, 0x61,		// -37
	0x00, 0x64, 0xcb, 0x61,		// -max	power down PA & PAD
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

void	rf_power_down ()
{
	write_reg8 (0x800f16, 0x21);	//turn off baseband pll
	analog_write (0x06, 0xfe);		//turn off transceiver
}

_attribute_ram_code_ void	rf_power_enable (int en)
{
	//write_reg8 (0x800f16, 0x21);	//turn off baseband pll
	analog_write (0x06, en ? 0 : 0xff);		//turn off transceiver
#if (CLOCK_SYS_TYPE == CLOCK_TYPE_OSC)
	analog_write (0x05, en ? 0x02 : 0xe2);		//turn off transceiver
#endif
}

void	rf_trx_off ()
{
	analog_write (0x06, 0xfe);		//turn off transceiver
}

_attribute_ram_code_ void rf_set_channel (signed char chn, unsigned short set)
{
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
		chn = rf_chn[chn];
	}
	else {

	}
    write_reg16 (0x8004d6, 2400 + chn);	// {intg_N}
    analog_write (TX_GAIN, rf_tp_base - ( (chn * rf_tp_gain+128) >> 8));  //rounded to the nearest integer
#if		RF_FAST_MODE_1M
    chn = chn >> 1;
    if (chn < 2)
    	chn = 37;
    else if (chn < 13)
    	chn -= 2;
    else if (chn == 13)
    	chn = 38;
    else if (chn < 40)
    	chn -= 3;
    else
    	chn = 39;
    write_reg8 (0x80040d, chn);
#endif
}

void rf_set_tp_gain (s8 chn)
{
	analog_write (TX_GAIN, rf_tp_base - ( (chn * rf_tp_gain+128) >> 8));  //rounded to the nearest integer
}

void rf_set_ble_channel (signed char chn)
{
	write_reg8 (0x80040d, chn);

	if (chn < 11)
    	chn += 2;
    else if (chn < 37)
    	chn += 3;
    else if (chn == 37)
    	chn = 1;
    else if (chn == 38)
    	chn = 13;
    else
    	chn = 40;

    chn = chn << 1;

	/////////////////// turn on LDO and baseband PLL ////////////////
//	analog_write (0x06, 0x00);
    PHY_POWER_UP;
	write_reg8 (0x800f16, 0x29);

	write_reg8 (0x800428, RF_TRX_MODE);	// rx disable
	write_reg8 (0x800f02, RF_TRX_OFF);	// reset tx/rx state machine

    write_reg16 (0x8004d6, 2400 + chn);	// {intg_N}
    analog_write (TX_GAIN, rf_tp_base - ( (chn * rf_tp_gain+128) >> 8));  //rounded to the nearest integer
}

void rf_set_tx_rx_off ()
{
	/////////////////// turn on LDO and baseband PLL ////////////////
	//analog_write (0x06, 0xfe);
	write_reg8 (0x800f16, 0x29);
	write_reg8 (0x800428, RF_TRX_MODE);	// rx disable
	write_reg8 (0x800f02, RF_TRX_OFF);	// reset tx/rx state machine
}

void rf_set_rxmode ()
{
    write_reg8 (0x800428, RF_TRX_MODE | BIT(0));	// rx disable
    write_reg8 (0x800f02, RF_TRX_OFF | BIT(5));		// RX enable
}

void rf_set_txmode ()
{
	write_reg8  (0x800f02, RF_TRX_OFF | BIT(4));	// TX enable
}

void rf_send_packet (void* addr, u16 rx_waittime, u8 retry)
{
	analog_write (0x06, 0x0);					//turn on RFPLL
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
	//analog_write (0x06, 0x0);					//turn on RFPLL
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
//	write_reg8  (0x800f04, 0x50);			// set tx settle to 80us,
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

void rf_start_stx  (void* addr, u32 tick)
{
	//write_reg32 (0x800f04, 0);						// tx wail & settle time: 0
	write_reg32(0x800f18, tick);						// Setting schedule trigger time
    write_reg8(0x800f16, read_reg8(0x800f16) | 0x04);	// Enable cmd_schedule mode
#if RF_FAST_MODE_2M
	write_reg16 (0x800f00, 0x3f85);						// single TX
#else
	write_reg8 (0x800f00, 0x85);						// single TX
#endif
	write_reg16 (0x80050c, (u16)((u32)addr));
}

void rf_start_stx2rx  (void* addr, u32 tick)
{
//	write_reg32 (0x800f04, 0x050);						// tx settle time: 80 us
	write_reg32(0x800f18, tick);						// Setting schedule trigger time
    write_reg8(0x800f16, read_reg8(0x800f16) | 0x04);	// Enable cmd_schedule mode
#if RF_FAST_MODE_2M
	write_reg16 (0x800f00, 0x3f87);
#else
	write_reg8  (0x800f00, 0x87);
#endif
	// single tx2rx
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
//	write_reg32 (0x800f04, 0x56);						// tx_wait = 0; tx_settle = 86 us
	write_reg32 (0x800f28, 0x0fffffff);					// first timeout
	write_reg32(0x800f18, tick);						// Setting schedule trigger time
    write_reg8(0x800f16, read_reg8(0x800f16) | 0x04);	// Enable cmd_schedule mode
	write_reg8 (0x800f00, 0x82);						// ble rx
	write_reg16 (0x80050c, (u16)((u32)addr));
}

void rf_set_ack_packet  (void* addr)
{
	write_reg16 (0x80050c, (u16)((u32)addr));
}

//////////////////////////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////////////////////////
_attribute_ram_code_ void SetRxMode (signed char chn, unsigned short set)
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
