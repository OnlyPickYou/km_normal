
//  to detect MCU_CORE_TYPE in app mode
#if(!(__TL_LIB_8267__ ))
#include "../proj/config/user_config.h"
#include "../proj/mcu/config.h"		// must include this
#endif 

#if(__TL_LIB_8267__ || (MCU_CORE_TYPE && MCU_CORE_TYPE == MCU_CORE_8267))

#include "../proj/common/types.h"
#include "../proj/common/compatibility.h"
#include "../proj/common/bit.h"
#include "../proj/common/utility.h"
#include "../proj/common/static_assert.h"
#include "../proj/mcu/compiler.h"
#include "../proj/mcu/register.h"
#include "../proj/mcu/analog.h"
#include "../proj/mcu/anareg.h"
#include "rf_drv_8267.h"


#define			RF_MANUAL_AGC_MAX_GAIN		1
#define     	TX_GAIN    					0x93

#define		TP_2M_G0			0x40		//69
#define		TP_2M_G1			0x39		//57

#define		TP_1M_G0			0x1f		//0x1f
#define		TP_1M_G1			0x18		//0x18

#define		TP_GET_GAIN(g0, g1)		((g0 - g1)*256/80)

int			rf_tp_base = TP_1M_G0;
int			rf_tp_gain = TP_GET_GAIN(TP_1M_G0, TP_1M_G1);

int 		sar_adc_pwdn_en = 0;
int 		xtalType_rfMode;

u8 emi_var[8];

unsigned short	capt, capr;
unsigned char	rfdrv_tx_pa;
unsigned char	rf_tx_mode = RF_TX_MODE_NORMAL;
unsigned char	rfhw_tx_power = FR_TX_PA_MAX_POWER;

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

const TBLCMDSET  tbl_rf_ini_16M_Crystal_1m_Mode[] = {
	0x04eb, 0x60,  TCMD_UNDER_BOTH | TCMD_WRITE,
	0x99, 	0x31,  TCMD_UNDER_BOTH | TCMD_WAREG,
	0x82,	0x34,  TCMD_UNDER_BOTH | TCMD_WAREG,
	0x9e, 	0x41,  TCMD_UNDER_BOTH | TCMD_WAREG,
};

const TBLCMDSET  tbl_rf_ini_12M_Crystal_1m_Mode[] = {
	0x04eb, 0xe0,  TCMD_UNDER_BOTH | TCMD_WRITE,
	0x99, 	0xb1,  TCMD_UNDER_BOTH | TCMD_WAREG,
	0x82,	0x20,  TCMD_UNDER_BOTH | TCMD_WAREG,
	0x9e, 	0x56,  TCMD_UNDER_BOTH | TCMD_WAREG,
};

const TBLCMDSET  tbl_rf_ini_16M_Crystal_2m_Mode[] = {
	0x04eb, 0x60,  TCMD_UNDER_BOTH | TCMD_WRITE,
	0x99, 	0x31,  TCMD_UNDER_BOTH | TCMD_WAREG,
	0x82,	0x34,  TCMD_UNDER_BOTH | TCMD_WAREG,
	0x9e, 	0x82,  TCMD_UNDER_BOTH | TCMD_WAREG,
};

const TBLCMDSET  tbl_rf_ini_12M_Crystal_2m_Mode[] = {
	0x04eb, 0xe0,  TCMD_UNDER_BOTH | TCMD_WRITE,
	0x99, 	0xb1,  TCMD_UNDER_BOTH | TCMD_WAREG,
	0x82,	0x20,  TCMD_UNDER_BOTH | TCMD_WAREG,
	0x9e, 	0xad,  TCMD_UNDER_BOTH | TCMD_WAREG,
};

const TBLCMDSET  tbl_rf_ini[] = {
	0x06, 0x00,  TCMD_UNDER_BOTH | TCMD_WAREG, //power down control.
	0x81, 0xd0,  TCMD_UNDER_BOTH | TCMD_WAREG, //crystal cap c0-df
	0x8b, 0xe3,  TCMD_UNDER_BOTH | TCMD_WAREG, //boot rx vco current, temporary fix
	0x8f, 0x38,  TCMD_UNDER_BOTH | TCMD_WAREG, //boot rx vco current, temporary fix 				(fix)
	0xa0, 0x28,  TCMD_UNDER_BOTH | TCMD_WAREG, //dac datapath delay ******change  remington 0x26	(fix)
	0xac, 0xa7,  TCMD_UNDER_BOTH | TCMD_WAREG, //RX bandwidth setting: 0xaa -> 0xa7 ,widen 15%

	//set TX power, actually no need, rf_set_power_level_index()  will update the value
	0xa2, 0x2c,  TCMD_UNDER_BOTH | TCMD_WAREG, //pa_ramp_target ****0-5bit
	0x04, 0x7c,  TCMD_UNDER_BOTH | TCMD_WAREG,
	0xa7, 0x61,  TCMD_UNDER_BOTH | TCMD_WAREG,
	0x8d, 0x67,  TCMD_UNDER_BOTH | TCMD_WAREG,

	// Enhance 300k freq offset,
	0x04ca, 0x88,	TCMD_UNDER_BOTH | TCMD_WRITE,	// enable DC distance bit 3
	0x04cb, 0x04,	TCMD_UNDER_BOTH | TCMD_WRITE,	// set dcoc  distance  value[4-6], default is 6, why 4 here?
	0x042d, 0x33,	TCMD_UNDER_BOTH | TCMD_WRITE,	// DC alpha=1/8, [6:4] r_predcoc_bw: [0,6], pwrDft 0x63,6-> 3

	//pwrDft for 250k, set as 1m/2m
	0x0430, 0x12, 	TCMD_UNDER_BOTH | TCMD_WRITE,   //0x12 for 1m/2m, 0x17 for 250k, pwrDft 0x17
	0x043d, 0x71, 	TCMD_UNDER_BOTH | TCMD_WRITE,   //0x71 for 1m/2m, 0xfd for 250k, pwrDft 0xfd
	0x0464, 0x07,	TCMD_UNDER_BOTH | TCMD_WRITE,	//0x07 for 1m/2m, 0x06 for 250k, pwrDft 0x06

	// ?
	0x0438, 0xb7, 	TCMD_UNDER_BOTH | TCMD_WRITE,     //pwrDft 0x37, BIT(7) : r_pkt_chg_disable ???

	///////////// baseband //////////////////////
	0x0439, 0x72,	TCMD_UNDER_BOTH | TCMD_WRITE,	//RX RSSI offset, pwrDft 0x6e
	0x043b, 0xfc,	TCMD_UNDER_BOTH | TCMD_WRITE,	//enable timer stamp & dc output, pwrDft 0x0c

	//tx/rx  timing set
	0x0f04, 0x68,	TCMD_UNDER_BOTH | TCMD_WRITE,	//tx settle time: 80us
	0x0f06, 0x00,	TCMD_UNDER_BOTH | TCMD_WRITE,	//rx wait settle time: 1us
	0x0f0c, 0x50,	TCMD_UNDER_BOTH | TCMD_WRITE,	//rx settle time: 80us
	0x0f10, 0x00,	TCMD_UNDER_BOTH | TCMD_WRITE,	//wait time on NAK


	////////////// RNG genertor		////////////////////////////////////
	0x045b, 0x02,	TCMD_UNDER_BOTH | TCMD_WRITE,	// RNG clock by 2
	0x0447, 0x08,	TCMD_UNDER_BOTH | TCMD_WRITE,	// RNG free running mode
	0x85,   0x00,   TCMD_UNDER_BOTH | TCMD_WAREG, 	// turn on RNG

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
};


const TBLCMDSET  tbl_rf_ini_manual_maxGain[] = {               // 16M crystal                         (fix)
	0x0433, 0x00,	TCMD_UNDER_BOTH | TCMD_WRITE,	// set mgain disable 01 -> 00                 (fix)
	0x0434, 0x01,	TCMD_UNDER_BOTH | TCMD_WRITE,	// pel0: 21 -> 01                             (fix)
	0x043a, 0x77,	TCMD_UNDER_BOTH | TCMD_WRITE,	// Rx signal power change threshold: 22 -> 77 (fix)
	0x043e, 0xc9,	TCMD_UNDER_BOTH | TCMD_WRITE,	// set rx peak detect manual: 20 -> c9        (fix)
	0x04cd, 0x06,	TCMD_UNDER_BOTH | TCMD_WRITE,	// fix rst_pga=0: len = 0 enable              (fix)
//	0x04c0, 0x81,	TCMD_UNDER_BOTH | TCMD_WRITE,	// lowpow agc/sync: 87                        (fix)
};

const TBLCMDSET  tbl_rf_250k[] = {
		0x9e, 0xad,  TCMD_UNDER_BOTH | TCMD_WAREG, 		//reg_dc_mod (500K); ble: 250k
		0xa3, 0x10,  TCMD_UNDER_BOTH | TCMD_WAREG, 		//pa_ramp_en = 1, pa ramp table max
		0xaa, 0x2a,  TCMD_UNDER_BOTH | TCMD_WAREG,		//filter iq_swap, 2M bandwidth

		0x0400, 0x03,	TCMD_UNDER_BOTH | TCMD_WRITE,	// 250K mode
		0x0401, 0x40,	TCMD_UNDER_BOTH | TCMD_WRITE,	// pn enable
		0x0402, 0x26,	TCMD_UNDER_BOTH | TCMD_WRITE,	// 8-byte pre-amble
		0x0404, 0xc0,	TCMD_UNDER_BOTH | TCMD_WRITE,	// head_mode/crc_mode: normal c0
		0x0405, 0x04,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code length 4
		0x0408, 0xc9,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code byte3
		0x0409, 0x8a,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code byte2
		0x040a, 0x11,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code byte1
		0x040b, 0xf8,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code byte0

		0x0420, 0x90,	TCMD_UNDER_BOTH | TCMD_WRITE,	// sync threshold: 1e (4); 26 (5)
		0x0421, 0x00,	TCMD_UNDER_BOTH | TCMD_WRITE,	// no avg
		0x0422, 0x1a,	TCMD_UNDER_BOTH | TCMD_WRITE,	// threshold
		0x0424, 0x52,	TCMD_UNDER_BOTH | TCMD_WRITE,	// number for sync: bit[6:4]
		0x042b, 0xf3,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code: 1
		0x042c, 0x88,	TCMD_UNDER_BOTH | TCMD_WRITE,	// maxiumum length 48-byte

		0x0f03, 0x1e,	TCMD_UNDER_BOTH | TCMD_WRITE,	// bit3: crc2_en; normal 1e
};

const TBLCMDSET  tbl_rf_1m[] = {
	0xa3, 0xf0,  TCMD_UNDER_BOTH | TCMD_WAREG,//********[7:6] disable gauflt [5] LUT 2M or 1M
	0xaa, 0x26,  TCMD_UNDER_BOTH | TCMD_WAREG,//*******filter iq_swap, adjust the bandwidth*****remington 0x2e

	0x0400, 0x0f,	TCMD_UNDER_BOTH | TCMD_WRITE,	// ?
	0x0401, 0x08,	TCMD_UNDER_BOTH | TCMD_WRITE,	// pn enable
	0x0402, 0x24,	TCMD_UNDER_BOTH | TCMD_WRITE,	// 8-byte pre-amble
	0x0404, 0xf5,	TCMD_UNDER_BOTH | TCMD_WRITE,	// head_mode/crc_mode: normal c0; 0xf7 for RX shockburst
	0x0405, 0x04,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code length 4

	0x0408, 0x8e,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code byte3
	0x0409, 0x89,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code byte2
	0x040a, 0xbe,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code byte1
	0x040b, 0xd6,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code byte0	//0xd6be898e

	0x0420, 0x1f,	TCMD_UNDER_BOTH | TCMD_WRITE,	// threshold  31/32
	0x0421, 0x04,	TCMD_UNDER_BOTH | TCMD_WRITE,	// no avg
	0x0422, 0x00,	TCMD_UNDER_BOTH | TCMD_WRITE,	// threshold
	0x0424, 0x12,	TCMD_UNDER_BOTH | TCMD_WRITE,	// number fo sync: bit[6:4]
	0x042b, 0xf1,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code: 1
	0x042c, 0x30,	TCMD_UNDER_BOTH | TCMD_WRITE,	// maxiumum length 48-byte

	0x0f03, 0x1e,	TCMD_UNDER_BOTH | TCMD_WRITE,	// bit3: crc2_en; normal 1e
};

const TBLCMDSET  tbl_rf_2m[] = {
	0xa3, 0x10,  TCMD_UNDER_BOTH | TCMD_WAREG,//********[7:6] disable gauflt [5] LUT 2M or 1M
	0xaa, 0x2e,  TCMD_UNDER_BOTH | TCMD_WAREG,//*******filter iq_swap, adjust the bandwidth*****remington 0x2e

	0x0400, 0x0f,	TCMD_UNDER_BOTH | TCMD_WRITE,	// 1M mode
	0x0401, 0x00,	TCMD_UNDER_BOTH | TCMD_WRITE,	// pn disable
	0x0402, 0x26,	TCMD_UNDER_BOTH | TCMD_WRITE,	// 8-byte pre-amble
#if	RF_LONG_PACKET_EN
	0x0404, 0xc8,	TCMD_UNDER_BOTH | TCMD_WRITE,	// head_mode/crc_mode
#else
	#if (STANDARD_BLE_2M_MODE)
		0x0404, 0xc5,	TCMD_UNDER_BOTH | TCMD_WRITE,	//<4>_1:ble 1m,0:ble_2m; <5>:ble_wt; <7>:PN auto
	#else
		0x0404, 0xca,	TCMD_UNDER_BOTH | TCMD_WRITE,	// head_mode/crc_mode: normal c0
	#endif
#endif


#if (ACCESS_CODE_LEN == 5)
	0x0405, 0x05,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code length 5
#elif  (ACCESS_CODE_LEN == 4)
	0x0405, 0x05,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code length 4
#endif

	0x0408, 0x71,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code byte3
	0x0409, 0x76,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code byte2
	0x040a, 0x51,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code byte1
	0x040b, 0x39,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code byte0

#if (ACCESS_CODE_LEN == 5)
	0x0420, 0x26,	TCMD_UNDER_BOTH | TCMD_WRITE,	// threshold  38/40
#elif  (ACCESS_CODE_LEN == 4)
	0x0420, 0x1E,	TCMD_UNDER_BOTH | TCMD_WRITE,	// threshold  30/32
#endif
	0x0421, 0x04,	TCMD_UNDER_BOTH | TCMD_WRITE,	// no avg
	0x0422, 0x00,	TCMD_UNDER_BOTH | TCMD_WRITE,	// threshold
	0x0424, 0x12,	TCMD_UNDER_BOTH | TCMD_WRITE,	// number fo sync: bit[6:4]
	0x042b, 0xf1,	TCMD_UNDER_BOTH | TCMD_WRITE,	// access code: 1
	0x042c, 0x80,	TCMD_UNDER_BOTH | TCMD_WRITE,	// maxiumum length 128-byte

	0x0f03, 0x36,	TCMD_UNDER_BOTH | TCMD_WRITE,	// bit3: crc2_en; normal 1e
};

void rf_drv_250k (void)
{
	rf_tp_base = TP_2M_G0;
	rf_tp_gain = TP_GET_GAIN(TP_2M_G0, TP_2M_G1);
	LoadTblCmdSet (tbl_rf_250k, sizeof (tbl_rf_250k)/sizeof (TBLCMDSET));
}

void rf_drv_1m (void)
{
	rf_tp_base = TP_1M_G0;
	rf_tp_gain = TP_GET_GAIN(TP_1M_G0, TP_1M_G1);
	LoadTblCmdSet (tbl_rf_1m, sizeof (tbl_rf_1m)/sizeof (TBLCMDSET));
	rf_set_12M_Crystal_1m_mode();
}

void rf_drv_2m (void)
{
	rf_tp_base = TP_2M_G0;
	rf_tp_gain = TP_GET_GAIN(TP_2M_G0, TP_2M_G1);
	LoadTblCmdSet (tbl_rf_2m, sizeof (tbl_rf_2m)/sizeof (TBLCMDSET));
	rf_set_12M_Crystal_2m_mode();
}

void rf_update_tp_value (u8 tp0, u8 tp1)
{
	 rf_tp_base = tp0;
	 rf_tp_gain = TP_GET_GAIN(tp0, tp1);
}

///////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////

void rf_drv_init (int xtal_type)
{
	LoadTblCmdSet (tbl_rf_ini, sizeof (tbl_rf_ini)/sizeof (TBLCMDSET));

	xtalType_rfMode = xtal_type;
	if(!xtal_type){
		xtalType_rfMode = XTAL_12M_RF_2m_MODE;
	}


	if( IS_RF_2M_MODE(xtalType_rfMode) ){
		rf_drv_2m();
	}
	else{
		//rf_drv_1m();
	}


    /*MAX GAIN MODE: MAX_GAIN_MODE_ENABLE Have rssi max -58dbm limit*/
	LoadTblCmdSet (tbl_rf_ini_manual_maxGain, sizeof (tbl_rf_ini_manual_maxGain)/sizeof (TBLCMDSET));


	for (int i=0; i<26; i++){
		write_reg8 (0x800480+i, tbl_agc[i]);	//set AGC table
	}
}

/////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////


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

void	rf_power_enable (int en)
{
	//ana_06<0>: adc clock, can not turn off when audio on
	analog_write (0x06, en ? 0x0 : 0xfe);		//turn off transceiver
#if(CLOCK_SYS_TYPE == CLOCK_TYPE_OSC)
	analog_write (0x05, en ? 0x62 : 0xe2);
#endif
}

void	rf_trx_off ()
{
	analog_write (0x06, 0xfe);		//turn off transceiver
}

void rf_set_tp_gain (s8 chn)
{
	 analog_write (TX_GAIN, rf_tp_base - ((chn * rf_tp_gain + 128) >> 8));
}

_attribute_ram_code_ void rf_set_channel (signed char chn, unsigned short set)
{
	//u8 gain;
	/////////////////// turn on LDO and baseband PLL ////////////////
	analog_write (0x06, 0x00);
	write_reg8 (0x800f16, 0x29);

	write_reg8 (0x800428, RF_TRX_MODE);	// rx disable
	if ( set & RF_SET_TX_MANAUL ){
        write_reg8 (0x800f02, RF_TRX_OFF_MANUAL);  // reset tx/rx state machine
    }
    else{
	   //write_reg8 (0x800f02, RF_TRX_OFF);	// reset tx/rx state machine
    	write_reg8 (0x800f02, 0x44);

    }
    
	if (set & RF_CHN_TABLE) {
		chn = rf_chn[chn];
	}


	write_reg16 (0x8004d6, 2400 + chn);	// {intg_N}
	analog_write (TX_GAIN, rf_tp_base - ((chn * rf_tp_gain + 128) >> 8));
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
	//analog_write (0x06, 0x00);
    PHY_POWER_UP;
	write_reg8 (0x800f16, 0x29);

	write_reg8 (0x800428, RF_TRX_MODE);	// rx disable
	write_reg8 (0x800f02, RF_TRX_OFF);	// reset tx/rx state machine

    write_reg16 (0x8004d6, 2400 + chn);	// {intg_N}
    analog_write (TX_GAIN, rf_tp_base - ((chn * rf_tp_gain + 128) >> 8));
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

void rf_set_ack_packet  (void* addr)
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
