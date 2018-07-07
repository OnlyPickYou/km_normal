/*
 * kb_default_config.h
 *
 *  Created on: 2015-1-21
 *      Author: Administrator
 */

#ifndef KB_DEFAULT_CONFIG_H_
#define KB_DEFAULT_CONFIG_H_


#define _USER_CONFIG_DEFINED_	1	// must define this macro to make others known
#define	__LOG_RT_ENABLE__		0

//////////// product  Information  //////////////////////////////


#define CHIP_TYPE				CHIP_TYPE_8368

#define APPLICATION_DONGLE		0			// or else APPLICATION_DEVICE
#define	FLOW_NO_OS				1


#if (NEW_DPI_CONFIG || LR_SW_CPI)
#define	CS_OPT_TEST_MODE_LEVEL		3	//code size small
#define PARING_TIME_ON_POWER		48
#else
#define	CS_OPT_TEST_MODE_LEVEL		1
#endif


///////////////////  Hardware  //////////////////////////////

/////////////////// Clock  /////////////////////////////////
#define CLOCK_SYS_TYPE  		CLOCK_TYPE_OSC	//  one of the following:  CLOCK_TYPE_PLL, CLOCK_TYPE_OSC, CLOCK_TYPE_PAD, CLOCK_TYPE_ADC
#define CLOCK_SYS_CLOCK_HZ  	16000000

#define KEYBOARD_SWITCH_KEYCOD_EN	   1
////////////////////////////////////////////////////


#define KEYBOARD_PIPE1_DATA_WITH_DID   0

/////////////////// watchdog  //////////////////////////////
#define MODULE_WATCHDOG_ENABLE	1

///////////////////  interrupt  //////////////////////////////

///////////////////  GPIO  /////////////////////////////////
#define GPIO_CK                 0
#define GPIO_DI                 0

#define GPIO_DBG 				GPIO_GP26


//////////////////    RF configuration //////////////


///////////////////  ADC  /////////////////////////////////


///////////////////  USB   /////////////////////////////////


/////////////////// keyboard matrix //////////////////////////
#define  ADC_GPIO_23_27_EXCHANGE    1

#define  GPIO_NONE      0

#if ADC_GPIO_23_27_EXCHANGE
#define  KB_DRIVE_PINS  {GPIO_GP17, GPIO_GP18, GPIO_GP19, GPIO_GP20, \
						 GPIO_GP21, GPIO_GP22, GPIO_GP24, GPIO_GP27}
#else
#define  KB_DRIVE_PINS  {GPIO_GP17, GPIO_GP18, GPIO_GP19, GPIO_GP20, \
						 GPIO_GP21, GPIO_GP22, GPIO_GP23, GPIO_GP24}
#endif

#define  KB_SCAN_PINS   {GPIO_GP14, GPIO_GP32, GPIO_GP0, GPIO_GP1, \
						 GPIO_GP2,  GPIO_GP3,  GPIO_GP4, GPIO_GP5, \
						 GPIO_GP12, GPIO_GP13, GPIO_GP11,GPIO_GP9, \
						 GPIO_GP8,  GPIO_GP7,  GPIO_GP6, GPIO_GP10, \
						 GPIO_GP15, GPIO_GP16}


#define	MATRIX_ROW_PULL			PM_PIN_PULLUP_1M
#define	MATRIX_COL_PULL			PM_PIN_PULLDOWN_100K

#if 1
//drive pin 8
#define	PULL_WAKEUP_SRC_GPIO17		MATRIX_ROW_PULL
#define	PULL_WAKEUP_SRC_GPIO18		MATRIX_ROW_PULL
#define	PULL_WAKEUP_SRC_GPIO19		MATRIX_ROW_PULL
#define	PULL_WAKEUP_SRC_GPIO20		MATRIX_ROW_PULL
#define	PULL_WAKEUP_SRC_GPIO21		MATRIX_ROW_PULL
#define	PULL_WAKEUP_SRC_GPIO22		MATRIX_ROW_PULL

#if ADC_GPIO_23_27_EXCHANGE
#define	PULL_WAKEUP_SRC_GPIO27		MATRIX_ROW_PULL
#else
#define	PULL_WAKEUP_SRC_GPIO23		MATRIX_ROW_PULL
#endif

#define	PULL_WAKEUP_SRC_GPIO24		MATRIX_ROW_PULL

//scan pin 18
#define	PULL_WAKEUP_SRC_GPIO14		MATRIX_COL_PULL  //C0
#define	PULL_WAKEUP_SRC_GPIO32		MATRIX_COL_PULL
#define	PULL_WAKEUP_SRC_GPIO0		MATRIX_COL_PULL
#define	PULL_WAKEUP_SRC_GPIO1		MATRIX_COL_PULL
#define	PULL_WAKEUP_SRC_GPIO2		MATRIX_COL_PULL  //C4
#define	PULL_WAKEUP_SRC_GPIO3		MATRIX_COL_PULL
#define	PULL_WAKEUP_SRC_GPIO4		MATRIX_COL_PULL
#define	PULL_WAKEUP_SRC_GPIO5		MATRIX_COL_PULL
#define	PULL_WAKEUP_SRC_GPIO12		MATRIX_COL_PULL  //C8
#define	PULL_WAKEUP_SRC_GPIO13		MATRIX_COL_PULL
#define	PULL_WAKEUP_SRC_GPIO11		MATRIX_COL_PULL
#define	PULL_WAKEUP_SRC_GPIO9		MATRIX_COL_PULL
#define	PULL_WAKEUP_SRC_GPIO8		MATRIX_COL_PULL  //C12
#define	PULL_WAKEUP_SRC_GPIO7		MATRIX_COL_PULL
#define	PULL_WAKEUP_SRC_GPIO6		MATRIX_COL_PULL
#define	PULL_WAKEUP_SRC_GPIO10		MATRIX_COL_PULL

#define	PULL_WAKEUP_SRC_GPIO15		MATRIX_COL_PULL  //C16
#define	PULL_WAKEUP_SRC_GPIO16		MATRIX_COL_PULL
#endif


#define	PULL_WAKEUP_SRC_GPIO31		PM_PIN_PULLUP_1M  //GP31 pullup:detect single battery

#define GPIO_LED_IND            GPIO_NONE
#define GPIO_LED_CAP          	GPIO_NONE
#define GPIO_LED_SCR			GPIO_NONE



#define CAVY_HAMSTER_REMINGTON_COMPATIBLE_EN			1

#if (CAVY_HAMSTER_REMINGTON_COMPATIBLE_EN)
	#define		CAVY_RF_CHANNEL_MAP_ENABLE				1
#endif





#define DBG_ADC_DATA			0

/////////////////// keyscan  setting //////////////////////////
#define KB_MAIN_LOOP_TIME_MS    12

#define  LOW_TX_POWER_WHEN_SHORT_DISTANCE      1

//#define cust_crystal_12M  ( *(unsigned char*)0x3f18 != 0xff )
#define cust_crystal_12M  ( (*(unsigned char*)0x3f08 ? *(unsigned char*)0x3f18 : *(unsigned char*)0x3f48) != 0xff )

#endif /* KB_DEFAULT_CONFIG_H_ */
