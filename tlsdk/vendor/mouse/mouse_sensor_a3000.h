#ifndef _ADNS_A3000_H_
#define _ADNS_A3000_H_


/* a3000 register */
#define SENSOR_A3000_REG_PID			0x00
#define SENSOR_A3000_REG_VID			0x01
#define SENSOR_A3000_REG_MOTION_ST		0x02
#define SENSOR_A3000_REG_DELTA_X		0x03
#define SENSOR_A3000_REG_DELTA_Y		0x04
#define SENSOR_A3000_REG_SQUAL			0x05
#define SENSOR_A3000_REG_SHUT_HI		0x06
#define SENSOR_A3000_REG_SHUT_LO		0x07
#define SENSOR_A3000_REG_PIX_MAX		0x08
#define SENSOR_A3000_REG_PIX_ACCUME		0x09
#define SENSOR_A3000_REG_MIN			0x0a
#define SENSOR_A3000_REG_GRAB			0x0b
#define SENSOR_A3000_REG_DELTA_XY_H		0x0c
#define SENSOR_A3000_REG_MOUSE_CTRL		0x0d
#define SENSOR_A3000_REG_RUN_DOWNSHIF	0x0e
#define SENSOR_A3000_REG_RST1_PERIOD	0x0f
#define SENSOR_A3000_REG_RST1_DOWNSHIF	0x10
#define SENSOR_A3000_REG_RST2_PERIOD	0x11
#define SENSOR_A3000_REG_RST2_DOWNSHIF	0x12
#define SENSOR_A3000_REG_RST3_PERIOD	0x13
#define SENSOR_A3000_REG_PERFORMANCE	0x22
#define SENSOR_A3000_REG_RESET			0x3a
#define SENSOR_A3000_REG_NOT_RID		0x3f
#define SENSOR_A3000_REG_LED_CTRL		0x40
#define SENSOR_A3000_REG_MOT_CTRL		0x41
#define SENSOR_A3000_REG_BURST_READ_FST	0x42
#define SENSOR_A3000_REG_REST_M_CONFIG	0x45
#define SENSOR_A3000_REG_MOT_BURST		0x63

/* register default value */
#define SENSOR_A3000_PID				0x2A

#define SENSOR_A3000_MOTION_STATUS_MOT  0x80

#define SENSOR_A3000_RESET_VALUE        0x5a

#define CPI_RES_EN			(0x01 << 5)

#define CPI1000_A3000       (0x00 << 2)
#define CPI250_A3000        (0x01 << 2)
#define CPI500_A3000        (0x02 << 2)
#define CPI1250_A3000       (0x03 << 2)
#define CPI1500_A3000       (0x04 << 2)
#define CPI1750_A3000       (0x05 << 2)
#define CPI2000_A3000       (0x06 << 2)

#define	MAX_SENSOR_DPI_RATE_A3000	3

#define A3000_DATA_DELTA_X_H 0xf0
#define A3000_DATA_DELTA_Y_H 0x0f

#define A3000_PID 0x2A

int A3000_OPTSensor_Init(unsigned int mode);
unsigned int	A3000_OPTSensor_motion_report(signed char *pBuf, u32 flow);
void			A3000_OPTSensor_Shutdown(void);
int A3000_OPTSensor_Wakeup(u32 sensor);

unsigned int	A3000_OPTSensor_dpi_update(unsigned int dpi_rate);
void			A3000_OPTSensor_recover(void);

#endif
