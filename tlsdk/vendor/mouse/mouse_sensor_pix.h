/*
 * pan3204.h
 *
 *  Created on: 2012-3-8
 *      Author: zhi.wang
 */

#ifndef PAN3204_H_
#define PAN3204_H_


typedef struct{
	int mode;
	int value;
}CONFIG_PARA;


typedef struct{
	/**
	 * Add all possible config/init parameters here
	 * User app just init this struct and call your init function, then everything will done!
	 */
	CONFIG_PARA resolution;
	CONFIG_PARA led_current;
	CONFIG_PARA motion_level;
}OPTSensorST;


typedef enum{
	/**
	 *  mode: 1-->resolution,
 	 *  param: 0-->default(1000dpi), 1-->250dpi, 2-->500dpi, 3-->1250dpi, 4-->1500dpi, 5-->1750dpi
	 *  mode: 2-->LED current,
	 *  param: 0-->default(auto LED control), 1-->20mA, 2-->15mA, 3-->36mA, 4-->30m
	 *  mode: 3-->Motion control,
	 *  param: 0-->level low, 1-->edge low, 2-->level high, 3-->edge high
	 */
	CONFIG_MODE_RESOLUTION = 1,
	CONFIG_MODE_LED_CURRENT,
	CONFIG_MODE_MOTION_CONTROL
}OPTICAL_SENSOR_CONFIG_MODE;

void OPTSensor_hardware_init(unsigned int *sensor_hw );
unsigned int OPTSensor_Init(unsigned int mode);
void OPTSensor_configure(OPTICAL_SENSOR_CONFIG_MODE mode, int value );
unsigned int OPTSensor_dpi_update(unsigned int cpi_ctrl);
void OPTSensor_Shutdown(void);
unsigned int OPTSensor_recover(void);
void sif_init(void);

//---------------------------------------------
// Add by LiuWen. 2012/01/17.
// ---------------------------------------------

// -> decided by mouse shell type!!
/*
#ifdef OPTICAL_SENSOR_3204LL	//some the same as 3205DB
#define REG_PAN3204LL_PRODUCT_ID1	0x00
#define PAN3204LL_PRODUCT_ID1		0x30
#define REG_PAN3204LL_PRODUCT_ID2	0x01
#define PAN3204LL_PRODUCT_ID2 		0xd0		//PAW3204DB-TJ3L pid=0xd030 PAW3205 pid=0xd030
#endif

#ifdef OPTICAL_SENSOR_3204L
#define REG_PAN3204LL_PRODUCT_ID1	0x00
#define PAN3204LL_PRODUCT_ID1		0x30
#define REG_PAN3204LL_PRODUCT_ID2	0x01
#define PAN3204LL_PRODUCT_ID2 		0x50
#endif
*/

//---------------------------------------------
#define REG_PAN3204LL_PRODUCT_ID1           0x00
#define PAN3204LL_PRODUCT_ID1               0x30
#define PAN3204LL_PRODUCT_ID1_3065          0x31

#define PAN3204LL_PRODUCT_ID1_8204          0x38

//---------------------------------------------
#define REG_PAN3204LL_PRODUCT_ID2           0x01
#define PAN3204_PRODUCT_ID2                 0x50           // 3204: 0x50
#define PAN3204LL_PRODUCT_ID2				0xC0           // 3204LL: 0xC0
#define PAN3204UL_PRODUCT_ID2				0xD0           // 3204UL and 3205: 0xD0
#define PAW3207_PRODUCT_ID2				    0xE0
#define PAW3212_PRODUCT_ID2					0x02
#define S8321_PRODUCT_ID2					0x03

#define FCT3065_PRODUCT_OLD_ID2				0x00			//add by zjs
#define FCT3065_PRODUCT_NEW_ID2				0x70			//add by zjs

#define FCT3065_PRODUCT_ID1					0x31

//---------------------------------------------
#define REG_PAN3204LL_MOTION_STATUS         0x02
#define MOTION_STATUS_MOT                   0x80
#define MOTION_STATUS_DYOVF                 0x10
#define MOTION_STATUS_DXOVF                 0x08
#define MOTION_STATUS_RES                   0x07

//---------------------------------------------
#define REG_PAN3204LL_DELTA_X               0x03
#define REG_PAN3204LL_DELTA_Y               0x04

//---------------------------------------------
#define REG_PAN3204LL_OPERATION_MODE        0x05
#define OPERATION_LEDSHT_ENH                0xa0
#define OPERATION_SLP_ENH                   0x30
#define OPERATION_SLP2AU                    0x28
#define OPERATION_SLP2MU                    0x24
#define OPERATION_SLP1MU                    0x22
#define OPERATION_WAKEUP                    0x21

//---------------------------------------------
#define REG_PAN3204LL_CONFIGURATION         0x06
#define CONFIGURATION_RESET                 0x80
#define CONFIGURATION_MOTSWK                0x40
#define CONFIGURATION_PD_ENH                0x08
#define CONFIGURATION_CPI                   0x07


//---------------------------------------------
/*
 * add by jiusong, 2016-1-6
 */
#define PAW3212_OPERATION_SLP_ENH			0xb0
#define PAW3212_OPERATION_SLP2AU			0x08
#define PAW3212_OPERATION_SLP2MU			0x04
#define PAW3212_OPERATION_SLP1MU			0x02
#define PAW3212_OPERATION_WAKEUP			0x01		//PAW3212_OPERATION_SLP2MU/PAW3212_OPERATION_SLP1MU/PAW3212_OPERATION_WAKEUP only one can be set

#define PAW3212_CONFIGURATION_PD_ENH		0x08
#define PAW3212_CONFIGURATION_SLP3_ENH		0x20

#define REG_PAW3212_SLP1					0x0A
#define PAW3212_SLP1_FRQ					0x70		//(PAW3212_SLP1_FRQ>>4 + 1) * 4ms
#define	PAW3212_SLP1_ETIME					0x07		//(PAW3212_SLP1_ETME + 1) * 32ms

#define REG_PAW3212_SLP2					0x0B
#define PAW3212_SLP2_FRQ					0x10		//(PAW3212_SLP2_FRQ>>4 + 1 )* 64ms
#define PAW3212_SLP2_ETIME					0x00 		// (PAW3212_SLP2_ETIME +1) * 20.48s

#define REG_PAW3212_SLP3					0x0C
#define PAW3212_SLP3_FRQ					0x70		//(PAW3212_SLP3_FRQ>>4 + 1 )* 64ms
#define PAW3212_SLP3_ETIME					0x00 		// (PAW3212_SLP2_ETIME +1) * 20.48s

#define REG_PAW3212_SPI_MODE_SEL			0x26
#define PAW3212_2WIRE_SPI_MODE				0x34

#define REG_PAW3212_CPI_X					0x0D
#define REG_PAW3212_CPI_Y					0x0E


//---------------------------------------------
#define REG_PAN3204LL_IMAGE_QUALITY         0x07
#define REG_PAN3204LL_OPERATION_STATE       0x08
#define REG_PAN3204LL_WRITE_PROTECT         0x09
#define REG_PAN3204LL_SLEEP1_SETTING        0x0a
#define REG_PAN3204LL_ENTER_TIME            0x0b
#define REG_PAN3204LL_SLEEP2_SETTING        0x0c
#define REG_PAN3204LL_IMAGE_THRESHOLD       0x0d
#define REG_PAN3204LL_IMAGE_RECOGNITION     0x0e
//---------------------------------------------

//---------------------------------------------

/*
// Add by LiuWen. 2012/01/17.	//made a change according pid=0xd030
//---------------------------------------------
#ifdef OPTICAL_SENSOR_3204LL	//the same configuration as 3205
#define CPI600_3204LL     0x00
#define CPI800_3204LL     0x01
#define CPI1000_3204LL    0x02
#define CPI1200_3204LL    0x03	//made a change
#define CPI1600_3204LL    0x04
#endif

#ifdef OPTICAL_SENSOR_3204L
#define CPI800_3204LL     0x03
#define CPI1000_3204LL    0x04
#define CPI1200_3204LL    0x05
#define CPI1600_3204LL    0x06
#endif
*/

//---------------------------------------------
// CPI
//
// Add by LiuWen. 2012/07/16.
//---------------------------------------------
// 3204
#define CPI800_3204       0x03
#define CPI1000_3204      0x04
#define CPI1200_3204      0x05
#define CPI1600_3204      0x06

// 3204LL
#define CPI800_3204LL     0x00
#define CPI1000_3204LL    0x01
#define CPI1200_3204LL    0x02
#define CPI1600_3204LL    0x03

// 3204UL and 3205
#define CPI600_3204UL     0x00
#define CPI800_3204UL     0x01
#define CPI1000_3204UL    0x02
#define CPI1200_3204UL    0x03
#define CPI1600_3204UL    0x04

#define CPI800_SIGMA_8630    (0x00 | 0x04)
#define CPI1000_SIGMA_8630   (0x01 | 0x04)
#define CPI1200_SIGMA_8630   (0x02 | 0x04)
#define CPI1600_SIGMA_8630   (0x03 | 0x04)


//---------------------------------------------

#define MAX_SENSOR_DPI_RATE 3

#ifndef SENSOR_8640_ENABLE
#define SENSOR_8640_ENABLE	1
#endif

#ifndef	SUPPORT_M8589
#define	SUPPORT_M8589		1
#endif

#ifndef SENSOR_PAW3212_ENABLE
#define SENSOR_PAW3212_ENABLE	1
#endif


#ifndef	SENSOR_SMOOTH_CPI
#define	SENSOR_SMOOTH_CPI	0
#endif

#define WAKEUP_3204        (OPERATION_LEDSHT_ENH | OPERATION_SLP_ENH | OPERATION_SLP2AU | OPERATION_WAKEUP )
//add by zjs
#define WAKEUP_3212		   (PAW3212_OPERATION_SLP_ENH | PAW3212_OPERATION_SLP2AU | PAW3212_OPERATION_WAKEUP)
#define SLEEP_DISABLE_3204  OPERATION_LEDSHT_ENH        // OPERATION_SLP_ENH = 0, Reserved[1:0] = 01
#define CONFIG_POWERDOWN_3204   CONFIGURATION_PD_ENH
#define SIGMA_8630_SLEEP_2	0xbc

unsigned int I2C_PAN3204LL_ReadRegister(unsigned int cAddr);
void I2C_PAN3204LL_WriteRegister(unsigned int cAddr, unsigned int cData);
void Sensor3204LL_Optimization_Setting(void);
void Sensor3204_Shutdown(void);
int OPTSensor_resync(u32 retry);
int Sensor3204_Wakeup(u32 sensor);

unsigned int OPTSensor_motion_report( signed char *pBuf, u32 no_overflow );

#endif /* PAN3204_H_ */
