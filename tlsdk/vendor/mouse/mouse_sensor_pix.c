#include "../../vendor/common/user_config.h"


#ifndef MOUSE_OPTICAL_EN
#define MOUSE_OPTICAL_EN   0
#endif

#if (MOUSE_OPTICAL_EN)

#define		PAN3204_EN	1
#if (PAN3204_EN)
#include "../../proj/tl_common.h"
#include "../../proj/mcu/watchdog_i.h"
#include "mouse_sensor_pix.h"
#include "mouse_sensor.h"
#include "trace.h"
#include "mouse_custom.h"

#include "../common/device_power.h"

#ifndef		OPTSENSOR_LOWPOWER_RESYNC
#define		OPTSENSOR_LOWPOWER_RESYNC		1
#endif
#define		SENSOR_RECOVER_FAIL				1

#if ( CHIP_TYPE	== CHIP_TYPE_8366 )
#define sif_spi_clk_low    do{ (*((volatile u32*)0x800584)) &= (~PIN_SIF_SCL); }while(0)
#define sif_spi_clk_high   do{ (*((volatile u32*)0x800584)) |= PIN_SIF_SCL; }while(0)

#define sif_spi_sda_low    do{ (*((volatile u32*)0x800584)) &= (~PIN_SIF_SDA); }while(0)
#define sif_spi_sda_high   do{ (*((volatile u32*)0x800584)) |= PIN_SIF_SDA; }while(0)

#define sif_spi_sda_output_en   do{ (*((volatile u32*)0x800588)) |= PIN_SIF_SDA; }while(0)
#define sif_spi_sda_output_disable   do{ (*((volatile u32*)0x800588)) &= (~PIN_SIF_SDA); }while(0)
#else
#define sif_spi_clk_low    gpio_write(PIN_SIF_SCL, 0)
#define sif_spi_clk_high   gpio_write(PIN_SIF_SCL, 1)

#define sif_spi_sda_low    gpio_write(PIN_SIF_SDA, 0)
#define sif_spi_sda_high   gpio_write(PIN_SIF_SDA, 1)

#define sif_spi_sda_output_en   gpio_set_output_en(PIN_SIF_SDA, 1)
#define sif_spi_sda_output_disable   gpio_set_output_en(PIN_SIF_SDA, 0)
#endif

////////////////////////////////////////////////////////////////////////////
// serial interface function
////////////////////////////////////////////////////////////////////////////
#define		DLY_200NS    asm("tnop");asm("tnop")
#define		DLY_100NS    asm("tnop")

u16	check_spi_counter;
u8	check_resync_counter;
u8	product_id1, product_id2,product_id3;
//u8  reg_0d;
static u8	sensor_type = 0;
extern sleep_cfg_t device_sleep;

#ifndef MOUSE_SENSOR_IO_FIXED
#define MOUSE_SENSOR_IO_FIXED	0
#endif

#if MOUSE_SENSOR_IO_FIXED
#ifndef		PIN_SIF_SCL
#define		PIN_SIF_SCL		GPIO_CK
#endif
#ifndef		PIN_SIF_SDA
#define		PIN_SIF_SDA		GPIO_DI
#endif
void OPTSensor_hardware_init(unsigned int *sensor_hw ){
}
#else
unsigned int m_sif_clk;
unsigned int m_sif_data;
#undef      PIN_SIF_SCL
#define		PIN_SIF_SCL		m_sif_clk
#undef		PIN_SIF_SDA
#define		PIN_SIF_SDA		m_sif_data
void OPTSensor_hardware_init(unsigned int *sensor_hw ){
	m_sif_data = *sensor_hw;
	m_sif_clk = *(sensor_hw+1);

	gpio_setup_up_down_resistor(m_sif_data, IIC_1M_PULLUP_EN ? PM_PIN_PULLUP_1M : PM_PIN_PULLUP_10K );	//pull up register is 1M
	gpio_setup_up_down_resistor(m_sif_clk, IIC_1M_PULLUP_EN ? PM_PIN_PULLUP_1M : PM_PIN_PULLUP_10K);


#if MOUSE_GPIO_FULL_RE_DEF    
    gpio_set_func( m_sif_data, AS_GPIO);
    gpio_set_input_en(m_sif_data,0);
    gpio_set_output_en(m_sif_data,1);
    gpio_set_func( m_sif_clk, AS_GPIO);
#endif
    gpio_set_input_en(m_sif_clk,0);
    gpio_set_output_en(m_sif_clk,1);
}
#endif

void sif_init(void){
	//init in mouse.h
	gpio_write (PIN_SIF_SCL, 1);
	gpio_set_output_en (PIN_SIF_SCL, 1);
}

void  sif_resyn(void){
	gpio_write (PIN_SIF_SCL, 1);
	WaitUs(3);
	gpio_write (PIN_SIF_SCL, 0);
	WaitUs(2);
    gpio_write (PIN_SIF_SCL, 1);
    WaitUs(2000);
}

unsigned int sif_ReadByte(void){
	unsigned int ii = 0;
	unsigned int dat = 0;

	gpio_set_output_en (PIN_SIF_SDA, 0);
	for(ii=0; ii<8; ii++){
		gpio_write (PIN_SIF_SCL, 0);
		DLY_200NS;
		gpio_write (PIN_SIF_SCL, 1);
		DLY_200NS;
		if(gpio_read (PIN_SIF_SDA)){
			dat |= (1<<(7-ii));
		}else{
			dat &= ~(1<<(7-ii));
		}
	}
	return dat;
}

void sif_SendByte( unsigned int data)
{
	unsigned int ii = 0;

	gpio_set_output_en (PIN_SIF_SDA, 1);
	for(ii=0; ii<8; ii++){
		gpio_write (PIN_SIF_SCL, 0);
		if(data&(1<<(7-ii))){
			gpio_write (PIN_SIF_SDA, 1);
		}else{
			gpio_write (PIN_SIF_SDA, 0);
		}
		DLY_100NS;

		gpio_write (PIN_SIF_SCL, 1);
		DLY_100NS;
	}
	gpio_set_output_en (PIN_SIF_SDA, 0);
}

unsigned int I2C_PAN3204LL_ReadRegister(unsigned int cAddr)
{
	//sif_SendByte((cAddr&0x7f));	
    unsigned int dat = 0;
	cAddr = cAddr & 0x7f;
    unsigned int ii = 0;
    sif_spi_sda_output_en;
	for(ii=0; ii<8; ii++){
        sif_spi_clk_low;
		if(cAddr&(1<<(7-ii))){
            sif_spi_sda_high;
		}else{
            sif_spi_sda_low;
		}
		DLY_100NS;
        sif_spi_clk_high;
		DLY_100NS;
	}
    
    sif_spi_sda_output_disable;
    //gpio_write (PIN_SIF_SDA, 1);    //pull up
    
	//WaitUs(1);       //this delay is necessary!

	//gpio_set_output_en (PIN_SIF_SDA, 0);
	for(ii=0; ii<8; ii++){
        sif_spi_clk_low;
		DLY_200NS;
        sif_spi_clk_high;
		DLY_200NS;
		if(gpio_read (PIN_SIF_SDA)){
			dat |= (1<<(7-ii));
		}else{
			dat &= ~(1<<(7-ii));
		}
	}
	return dat;
}

void I2C_PAN3204LL_WriteRegister(unsigned int cAddr, unsigned int cData)
{
	sif_SendByte((cAddr|0x80));
	WaitUs(1);
	sif_SendByte(cData);
	WaitUs(1);
}


//----------------------------------------------------------------------
// FUNCTION NAME: Sensor3204LL_Optimization_Setting
//
// DESCRIPTION:
//      Do the optimization of the sensor PAN3204LL to consume less current.
//      Also, if need, set a better tracking performance on Critical surfaces.
//
//----------------------------------------------------------------------
void DownloadConfigTable (const unsigned char *ptbl, unsigned int len)
{
	unsigned int i, addr;

	for (i=0; i<len; i+= 2) {
		addr = ptbl[0];
		I2C_PAN3204LL_WriteRegister(addr, ptbl[1]);
		ptbl+=2;
	}
}

const unsigned char Config_3204[] = {
		0x09, 0x5A,
		0x0A, 0xF2,
		0x0B, 0x00,
		0x0D, 0x14,
		0x12, 0x1D,
		0x27, 0x4B,
		0x42, 0x85,
		0x28, 0xEF,
		0x2A, 0xAA,
		0x2B, 0xA0,
		0x2C, 0x8C,
		0x2D, 0x82,
		0x09, 0x00,
};


const unsigned char Config_3204LL[] = {
		0x09, 0x5A,
		0x0A, 0x40,
		0x0B, 0x10,
		0x0C, 0xE0,
		0x24, 0x1F,
		0x64, 0x4E,
		0x50, 0x07,
		0x4D, 0x81,
		0x0D, 0x0A,
		0x09, 0x00,
};

const unsigned char Config_3204UL[] = {
		0x09, 0x5A,
		0x0A, 0xf0,
		0x0D, 0x0F,
		0x1D, 0xE3,
		0x28, 0xB4,
		0x29, 0x46,
		0x2A, 0x96,
		0x2B, 0x8C,
		0x2C, 0x6E,
		0x2D, 0x64,
		0x38, 0x5F,
		0x39, 0x0F,
		0x3A, 0x32,
		0x3B, 0x47,
		0x42, 0x10,
		0x54, 0x2E,
		0x55, 0xF2,
		0x61, 0xF4,
		0x63, 0x70,
		0x75, 0x52,
		0x76, 0x41,
		0x77, 0xED,
		0x78, 0x23,
		0x79, 0x46,
		0x7A, 0xE5,
		0x7C, 0x48,
		0x7D, 0xD2,
		0x7E, 0x77,
		0x1B, 0x35,
		0x7F, 0x01,
		0x0B, 0x00,
		0x7F, 0x00,
		
#if SENSOR_QB_READ		
		0x15, 0x84,
		0x19, 0x1c,
		0x12, 0x5d,  //ini value:0x12 = 0x1d
#endif		

		0x09, 0x00,
};

const unsigned char Config_3205[] = {
#if 0
// E ÔªËØmouse settting
        0x09, 0x5a,  
        0x0b, 0x12, 
        0x0c, 0x20, 
        0x28, 0xb4, 
        0x29, 0x46,
        0x2a, 0x96,
        0x2b, 0x8c, 
        0x2c, 0x6e,

        0x2d, 0x64,
        0x38, 0x5f,
        0x39, 0x0f,
        0x3a, 0x32,
        0x3b, 0x47,
        0x42, 0x10,
        0x4b, 0x13,
        0x54, 0x2e,
        0x55, 0xf2,
        0x61, 0xf4,
        0x63, 0x70,
        0x75, 0x52,
        0x76, 0x41,
        0x77, 0xed,
        0x78, 0x23,
        0x79, 0x46,
        0x7a, 0xe5,
        0x7c, 0x48,
        0x7d, 0x80,
        0x7e, 0x77,
        0x7f, 0x01,
        0x0b, 0x00,
        0x7f, 0x00,
        0x09, 0x00,
#elif 0       
//Below is the setting from YiHaiXing which cannot improve the current significantly.
//But this setting can improve the code size about 52 bytes.
		0x09, 0x5A,
		0x0A, 0x70,
		0x0B, 0x10,
		0x0C, 0x70,
		0x0D, 0x0A,
		0x0E, 0xE5,
		0x09, 0x00,
#else
		0x09, 0x5A,
 	    0x0D, 0x0A,
		0x1B, 0x35, 
        0x1D, 0xDB,

		0x28, 0xB4,
		0x29, 0x46,
		0x2A, 0x96,
		0x2B, 0x8C,
		0x2C, 0x6E,
		0x2D, 0x64,
		0x38, 0x5F,
		0x39, 0x0F,
		0x3A, 0x32,
		0x3B, 0x47,
		0x42, 0x10,

		0x43, 0x09,	
		0x54, 0x2E,
		0x55, 0xF2,
		0x61, 0xF4,
		0x63, 0x70,
		0x75, 0x52,
		0x76, 0x41,
		0x77, 0xED,
		0x78, 0x23,
		0x79, 0x46,
		0x7A, 0xE5,
		0x7C, 0x48,

        0x7D, 0x80,    
		0x7E, 0x77,

		0x7F, 0x01,	
		0x0B, 0x00,

		0x7F, 0x00,
		0x09, 0x00,
#endif
};

const unsigned char Config_AN3205[] = {
    0x09, 0xA5,
    0x40, 0x1B,
    0x41, 0x04,
    0x46, 0x43,
    0x48, 0x77,
    0x4D, 0x08,
    0x4E, 0x80,
    0x09, 0x00,
};

//3207 default setting is the same
#if 0
const unsigned char Config_3207[] = {
        0x09,  0x5A,      
        0x0D,  0x0A,      
        0x0E,  0xC5,      
        0x20,  0x2B,      
        0x24,  0x2D,      
        0x28,  0xB4,      
        0x29,  0x46,      
        0x2A,  0x96,      
        0x2B,  0x8C,      
        0x2C,  0x6E,      
        0x2D,  0x64,      
        0x38,  0x58,      
        0x42,  0x22,      
        0x43,  0x09,      
        0x4C,  0xC2,      
        0x4D,  0x81,      
        0x4F,  0xD0,      
        0x56,  0xA1,      
        0x61,  0xF4,      
        0x62,  0x10,      
        0x63,  0x07,      
        0x67,  0x00,      
        0x75,  0x52,      
        0x76,  0x41,      
        0x77,  0x6D,      
        0x78,  0x23,      
        0x79,  0x46,      
        0x09,  0x00,      
};
#endif

const unsigned char Config_8630[] = {
	0x7f , 0xa5,
	0x1c , 0x14,
	0x1f , 0x3a,	
	0x7f , 0x00,
};

const unsigned char Config_3207[] = {

};

const unsigned char Config_VT108[] = {		//KA8
	0x09,  0x5a,
	0x0d,  0x0c,
	0x09,  0x00,
};

const unsigned char Config_M8589[] = {

};

const unsigned char Config_KA9[] = {
		0x09,  0x5a,
		0x0d,  0x14,
		0x1b,  0x35,
		0x1d,  0xdb,
		0x28,  0xb4,
		0x29,  0x46,
		0x2a,  0x96,
		0x2b,  0x8c,
		0x2c,  0x6e,
		0x2d,  0x64,
		0x38,  0x5f,
		0x39,  0x0f,
		0x3a,  0x32,
		0x3b,  0x47,
		0x42,  0x10,
		0x54,  0x2e,
		0x55,  0xf2,
		0x61,  0xf4,
		0x63,  0x70,
		0x75,  0x52,
		0x76,  0x41,
		0x77,  0xed,
		0x78,  0x23,
		0x79,  0x46,
		0x7a,  0xe5,
		0x7c,  0x48,
		0x7d,  0x80,
		0x7e,  0x77,
		0x1b,  0x35,
		0x7f,  0x01,
		0x0b,  0x00,
		0x7f,  0x00,
		0x09,  0x00,
};

const unsigned char ShutDown_Config_KA9[] = {
		0x09,  0x5a,
		0x4b,  0x13,
		0x09,  0x00,
};

const unsigned char WakeUp_Config_KA9[] = {
		0x09,  0x5a,
		0x4b,  0x1b,
		0x09,  0x00,
};


#if 0
const unsigned char Config_3212[] = {
	0x09,  0x5a,
	0x26,  0x04,
	0x09,  0x00,
};
#else
const unsigned char Config_3212[] = {
	0x09,  0x5a,
	0x06,  0xa0,
	0x26,  0x34,
	0x09,  0x00,
};

#endif

typedef struct{
	const unsigned char *ptbl;
	unsigned int len;
}SENSOR_OPT;

const SENSOR_OPT Pan_opt[]={
		{Config_3204, sizeof(Config_3204)},
		{Config_3204LL, sizeof(Config_3204LL)},
		{Config_3204UL, sizeof(Config_3204UL)},
		{Config_3205, sizeof(Config_3205)},		
		{Config_AN3205, sizeof(Config_AN3205)},		
		{Config_8630, sizeof(Config_8630)},
		{Config_3207, sizeof(Config_3207)},
		{Config_VT108, sizeof(Config_VT108)},
		{Config_M8589, sizeof(Config_M8589)},
		{Config_KA9, sizeof(Config_KA9)},
};
#define OPTIM_SENSOR_NUM    ( sizeof(Pan_opt) / sizeof(SENSOR_OPT) )

#if 0
void Sensor3204LL_Optimization_Setting(void)
{
	DownloadConfigTable(Pan_opt[sensor_type].ptbl, Pan_opt[sensor_type].len);

}
#else

#define Sensor3204LL_Optimization_Setting()    DownloadConfigTable(Pan_opt[sensor_type].ptbl, Pan_opt[sensor_type].len)
#define Sensor3212_Optimization_Setting()	   DownloadConfigTable(Config_3212, sizeof(Config_3212))
#define SensorKA9_Shutdown_Setting()		   DownloadConfigTable(ShutDown_Config_KA9, sizeof(ShutDown_Config_KA9))
#define SensorKA9_WakeUp_Setting()             DownloadConfigTable(WakeUp_Config_KA9, sizeof(WakeUp_Config_KA9))
#endif


#if 0
void OPTSensor_re_init()
{
	sif_init();
	sif_resyn();
}
#endif


int OPTSensor_resync(u32 retry)
{
	check_spi_counter = 0;
	while ( ((product_id1 = I2C_PAN3204LL_ReadRegister(REG_PAN3204LL_PRODUCT_ID1)) != PAN3204LL_PRODUCT_ID1)\
			&& (product_id1 != PAN3204LL_PRODUCT_ID1_8204)\
			&& (product_id1 != PAN3204LL_PRODUCT_ID1_3065)\
			){
		wd_clear();

		gpio_write(PIN_SIF_SCL, 1);
		WaitUs(3);
		gpio_write(PIN_SIF_SCL, 0);
		if(mouse_cust_fct3065xy){
			WaitUs(10000);//FCT3065-XY SCL Low 10ms, then high 512ms
			gpio_write(PIN_SIF_SCL, 1);
			WaitUs(40000);
		}
		else{
			WaitUs(3);
			gpio_write(PIN_SIF_SCL, 1);
		}


		if( ++check_spi_counter >= retry )
		{
			check_resync_counter++;
			return SENSOR_RECOVER_FAIL;
		}
	}
	return 0;
}

static inline int sensor_type_identify( unsigned char product_id2 ){
    int sensor_type = 0;
#if (SENSOR_PAW3212_ENABLE)
    if((product_id2 & 0xff) == 0x02){
    	sensor_type = SENSOR_PAW3212;
    	return sensor_type;
    }
#endif
    //detect senor type FCT3065
    if( (product_id2 == FCT3065_PRODUCT_NEW_ID2)  || (product_id2 == FCT3065_PRODUCT_OLD_ID2 )){
    	if(product_id1 == FCT3065_PRODUCT_ID1){
    		sensor_type = SENSOR_3205;
    		return sensor_type;
    	}
    }
    else if( (product_id1 == PAN3204LL_PRODUCT_ID1_8204) && (product_id2 == S8321_PRODUCT_ID2)){
    	sensor_type = SENSOR_VT108;
    	return sensor_type;
    }

    switch( product_id2 &0xF0 ){
        case PAN3204_PRODUCT_ID2://0x50
            if(product_id2 == 0x54){
                sensor_type =SENSOR_VT108;
                break;
            }
            else if(product_id2 == 0x52){
            	sensor_type = SENSOR_OM16;
            	break;
            }
            else if(product_id2 == 0x50){			//Identify YS8006
            	u8 reg_17 = I2C_PAN3204LL_ReadRegister(0x17);
//            	u8 reg_18 = I2C_PAN3204LL_ReadRegister(0x18);
//            	u8 reg_19 = I2C_PAN3204LL_ReadRegister(0x19);
//
            	if(reg_17 == 0x5c)
            		sensor_type = SENSOR_YS8006;
            	break;

            }
            else{
                sensor_type = SENSOR_3204;
                unsigned  int reg_old, reg_new;
#if SUPPORT_M8589
                reg_old = I2C_PAN3204LL_ReadRegister(0x9);
                reg_new = 0;
                do{
                    I2C_PAN3204LL_WriteRegister(0x09, 0xa5);        //first reg 06 write 0 to BIT2
                    if(++reg_new>100000)
                        break;
                } while(0xa5 != I2C_PAN3204LL_ReadRegister(0x9));
                reg_new = I2C_PAN3204LL_ReadRegister(REG_PAN3204LL_PRODUCT_ID2);
                I2C_PAN3204LL_WriteRegister( 0x09, reg_old );       //first reg 06 write 0 to BIT2
                if(reg_new == 0x49){
                    sensor_type = SENSOR_M8589;
                    break;
                }
#endif
#if SENSOR_8640_ENABLE
                reg_old = I2C_PAN3204LL_ReadRegister(0x06);
                I2C_PAN3204LL_WriteRegister( 0x06, 0 );     //first reg 06 write 0 to BIT2
                reg_new = I2C_PAN3204LL_ReadRegister(0x06);
                I2C_PAN3204LL_WriteRegister( 0x06, reg_old );
                if( ( BIT(2) & reg_new ) == BIT(2) ){       //Sigma 8630 reg 06 is always 1, 3205/3205 reg 06 is 0
                    sensor_type = SENSOR_SIGMA_8630;
                    break;
                }
#endif
            break;
            }
        case PAN3204LL_PRODUCT_ID2://0xC0
            sensor_type = SENSOR_3204LL;
            break;
        case PAN3204UL_PRODUCT_ID2://0xD0
            if( I2C_PAN3204LL_ReadRegister(0x1E) & 0x01 ){
                sensor_type = SENSOR_3204UL;
            }
            else if(I2C_PAN3204LL_ReadRegister(0x1B) == 0x1D)
            {
        		sensor_type = SENSOR_YS8008;
        	}
            else if(product_id1 == PAN3204LL_PRODUCT_ID1){			//add KA9
            	sensor_type = SENSOR_KA9;
            }
            else{
                sensor_type = SENSOR_3205;
                u8 reg_20 = I2C_PAN3204LL_ReadRegister(0x20);
                u8 reg_21 = I2C_PAN3204LL_ReadRegister(0x21);
                if ( (reg_20 == 0x58) && ((reg_21 & 0xf0) == 0x20) ){
                    sensor_type = SENSOR_AN3205;
                }
            }
            break;
       case PAW3207_PRODUCT_ID2://0xE0
            sensor_type = SENSOR_PAW3207;
            break;
        default:
            break;
    }
    return sensor_type;
}

//called when power on, or mouse waked up from deep-sleep
unsigned int OPTSensor_Init(unsigned int poweron){
    
    // Do the full chip reset.

	sif_init();	
	if ( OPTSensor_resync (1024) ) {
		return SENSOR_MODE_A3000;
	}

    if ( poweron ){
        //reset sensor would clear all optimization

        I2C_PAN3204LL_WriteRegister(REG_PAN3204LL_CONFIGURATION, CONFIGURATION_RESET);
        WaitUs(1000);
    }
    //get sensor id
    product_id1 = I2C_PAN3204LL_ReadRegister(REG_PAN3204LL_PRODUCT_ID1);    //power-on must re-read product_id1, or it would make mistake
    product_id2 = I2C_PAN3204LL_ReadRegister(REG_PAN3204LL_PRODUCT_ID2);    
    product_id3 = I2C_PAN3204LL_ReadRegister(0x60);				//

    sensor_type = sensor_type_identify( product_id2 );

    if(product_id1 == PAN3204LL_PRODUCT_ID1 &&  sensor_type == SENSOR_PAW3212){
    	Sensor3212_Optimization_Setting();		//PAW3212 do special optimization
    }

    //do sensor optimization only when power on
    if( poweron ){
    	if( (sensor_type < OPTIM_SENSOR_NUM) || ((sensor_type == SENSOR_SIGMA_8630) && (I2C_PAN3204LL_ReadRegister(0x1f) != 0x3a)) ){
            if ( product_id1 == PAN3204LL_PRODUCT_ID1 ){    //FCT donnot do optimizataion
#if (SENSOR_PAW3212_ENABLE)
            	if(sensor_type == SENSOR_PAW3212){



            	}
            	else
#endif
            	{
            		Sensor3204LL_Optimization_Setting();
            	}

            }
    	}
    }
    //reg_0d = I2C_PAN3204LL_ReadRegister(0x0d);
	return sensor_type;
}

static inline void OPTSensor_surface_optimiz_3207( void ){
    static u8  surface_bak;
    u8  surface;   
    surface = I2C_PAN3204LL_ReadRegister(0x33) & 0x1F; 
    if ( (surface < 4) && (surface_bak >= 4) ){
        I2C_PAN3204LL_WriteRegister(0x09, 0x5A);      
        I2C_PAN3204LL_WriteRegister(0x0D, 0x1B);
        I2C_PAN3204LL_WriteRegister(0x09, 0x00);
    } 
    else if ( (surface >= 4) && (surface_bak < 4) ){
        I2C_PAN3204LL_WriteRegister(0x09, 0x5A);      
        I2C_PAN3204LL_WriteRegister(0x0D, 0x16);
        I2C_PAN3204LL_WriteRegister(0x09, 0x00);
    }
    surface_bak = surface;
}

// Param: pBuf-->the buffer where coordinates have been stored, 2 bytes
// return: 1-->available coordiates read, 0-->read fail
// only Delta_X and Delta_Y has been read and stored
// maybe should adjust the value based on the senario of mouse movement
unsigned int OPTSensor_motion_report( signed char *pBuf, u32 no_overflow ){
	unsigned int optical_status = 0;
	unsigned int reg_x, reg_y;

//sensor ph5205 no overflow read

	if ((product_id1 == 0x28) || (((product_id3&0x10) == 0x10) && sensor_type == SENSOR_3205)
		|| (sensor_type == SENSOR_PAW3212) || (sensor_type == SENSOR_OM16))	//damn 5205 has the exact same config with 3205 even its own IDs!!!

		no_overflow = 1;

	static u16 resync_cnt = 0;

	if(mouse_cust_fct3065xy){
//		if ( (resync_cnt++ & 0x3ff) == 0 ){
//			if (OPTSensor_resync (33)){
//				return 0;
//			}
//		}
	}
	else{
		if ( (resync_cnt++ & 0x1f) == 0 ){
			if (OPTSensor_resync (33)){
				return 0;
			}
			if ( sensor_type == SENSOR_PAW3207 ){
				OPTSensor_surface_optimiz_3207();
			}
		}
	}
	//---------------------------------------------------------
	// Read MOTION_STATUS regisgter, and then DELTA_X register, and
	// third DELTA_Y regisgter. The sequence is not suggested to change.
	//---------------------------------------------------------
	optical_status = I2C_PAN3204LL_ReadRegister(REG_PAN3204LL_MOTION_STATUS);
	if( (optical_status & MOTION_STATUS_MOT) \
        || ( no_overflow && ((optical_status & MOTION_STATUS_DXOVF) || (optical_status & MOTION_STATUS_DYOVF)) )\
       ){
		reg_x = I2C_PAN3204LL_ReadRegister(REG_PAN3204LL_DELTA_X);	// DIRECTION_CLOCK_12 X
		reg_y = I2C_PAN3204LL_ReadRegister(REG_PAN3204LL_DELTA_Y);	// DIRECTION_CLOCK_12 Y

		if(reg_x==0x80)
			reg_x = 0x81;
		if(reg_y==0x80)
			reg_y = 0x81;

		if( !no_overflow ){
			if(optical_status & MOTION_STATUS_DXOVF)
				reg_x = ((signed char)reg_x>=0)? 0x81:0x7f;   //x overflow, current<0,0x7f, current>0, 0xff

			if(optical_status & MOTION_STATUS_DYOVF)
				reg_y = ((signed char)reg_y>=0)? 0x81:0x7f;   //y overflow, current<0,0x7f, current>0, 0xff
		}

/*
		if(sensor_type == SENSOR_YS8006){
			pBuf[0] = (mouse_status.cpi==2)  ? (signed char)(reg_x*3/4) : reg_x;
			pBuf[1] = (mouse_status.cpi==2)  ? (signed char)(reg_y*3/4) : reg_y;
		}
*/
		if(sensor_type == SENSOR_YS8008){
			if((reg_x == 0) && (reg_y == 0)&&(device_sleep.mode & M_SUSPEND_100MS))
			{
				reg_x = 1;
				reg_y = 0;

			}

			 //read YS8008 operation state, if it is not normal state, set reg_x=0,reg_y=0;
			if((I2C_PAN3204LL_ReadRegister(0x08)&0xFF)||(optical_status & 0x60))
			{
				reg_x = 0;
				reg_y = 0;
			}

			if(((abs((signed char)reg_x)>0x01)||(abs((signed char)reg_y)>0x01))\
					&&(abs((signed char)reg_x)<0x5f)&&(abs((signed char)reg_y)<0x5f))
			//if((abs((signed char)reg_x)>dxy)||(abs((signed char)reg_y)>dxy))
			{
				pBuf[0] = (signed char)reg_x/2;
				pBuf[1] = (signed char)reg_y/2;
			}
			else
			{
				pBuf[0] = reg_x;
				pBuf[1] = reg_y;
			}

		}
		else

		{

	 		pBuf[0] = reg_x;
	 		pBuf[1] = reg_y;
		}


		return 1;
	}
	else{
	    return 0;
	}
}

//----------------------------------------------------------------------
// FUNCTION NAME: Sensor3204_Shutdown
//
// DESCRIPTION:
//      Shut down the mouse sensor. The mouse will shut off LED at the bottom.
//
// ARGUMENTS:
//      none
//
// RETURNS:
//      none
//
// PROCEDURE:
//      1 )
//      2 )
//
// AUTHOR & DATE:
// 	   LiuWen. 2012/01/17.
//
//----------------------------------------------------------------------

// Must clear SLP_ENH bit in the OPERATION_MODE register before using Power-Down mode.
void OPTSensor_Shutdown(void)
{
	unsigned int i;
	for( i=0; i < 16; i++ ){
#if SENSOR_8640_ENABLE
		if( sensor_type == SENSOR_SIGMA_8630 ){
			I2C_PAN3204LL_WriteRegister(REG_PAN3204LL_OPERATION_MODE, SIGMA_8630_SLEEP_2 ); //must set sleep2				
			WaitUs (5001); 		//must wait 2000us in sleep2 mode before set power down
			I2C_PAN3204LL_WriteRegister(REG_PAN3204LL_CONFIGURATION, CONFIG_POWERDOWN_3204);
			return;
		}
		else
#endif
		{
			I2C_PAN3204LL_WriteRegister(REG_PAN3204LL_OPERATION_MODE, SLEEP_DISABLE_3204);
			I2C_PAN3204LL_WriteRegister(REG_PAN3204LL_CONFIGURATION, CONFIG_POWERDOWN_3204);


#if(MOUSE_SENSOR_YS8008_EN)
			if(sensor_type == SENSOR_YS8008){
				return;
			}
			else if(sensor_type == SENSOR_KA9){
				SensorKA9_Shutdown_Setting();
			}
#endif
		}

		if( I2C_PAN3204LL_ReadRegister(REG_PAN3204LL_CONFIGURATION) != CONFIG_POWERDOWN_3204 ){
			OPTSensor_resync (33);
		}
		else{
			i = 16;
		}
	}
}

u8 dbg_sensor_cpi;
//unsigned char PAN3204LL_dpi_ctrl_table[] = {CPI800_3204LL, CPI1200_3204LL, CPI1600_3204LL, 0};

#if(MOUSE_SENSOR_YS8008_EN)
const unsigned char DPI_convert_table[LAST_SUPPORT_SENSOR] = {3, 0, 1, 1, 1, 4, 0, 3, 3, 0, 0, 3, 0, 0, 0, 0};	//paw3212 = 0x09
#else
const unsigned char DPI_convert_table[LAST_SUPPORT_SENSOR] = {3, 0, 1, 1, 1, 4, 0, 3, 3, 0, 3, 0};	//paw3212 = 0x09
#endif
//const unsigned char DPI_convert_table[LAST_SUPPORT_SENSOR] = {3, 0, 1, 1, 1, 4, 0, 3, 3, 0};


unsigned int OPTSensor_dpi_update( unsigned int cpi_ctrl ){
	unsigned int dpi_tbl, dpi_reg;


#if 0
    if ( product_id1 != PAN3204LL_PRODUCT_ID1 )        
    	if(product_id2 != 0x70)	//sendor 3065
		{
			dpi_tbl = cpi_ctrl + 1;  //sensor 3065(old) and 3205-TJDN convert value is 1
		}
		else		//sensor 3065 new
		{
			dpi_tbl = cpi_ctrl ;
		}
#else
    if ( product_id1 == PAN3204LL_PRODUCT_ID1_3065 ){		//FCT3065 sensor
    		if(product_id2 != 0x70)	//sendor 3065
    		{
    			dpi_tbl = cpi_ctrl + 1;  //sensor 3065(old) and 3205-TJDN convert value is 1
    		}
    		else		//sensor 3065 new
    		{
    			dpi_tbl = cpi_ctrl ;
    		}
    }
#endif
    else if (product_id1 == 0x28 )

    	dpi_tbl = cpi_ctrl + 3;  //sensor ph5205

    else if (product_id2 == 0x02)
    	dpi_tbl = cpi_ctrl;

	else{

	    dpi_tbl = cpi_ctrl + DPI_convert_table[sensor_type];
	}
	int timeout = 0;
#if (SENSOR_PAW3212_ENABLE)
	if(sensor_type == SENSOR_PAW3212){
		unsigned int dpi_reg_1,dpi_reg_2;
		do{
			I2C_PAN3204LL_WriteRegister(REG_PAW3212_CPI_X,	dpi_tbl);
			I2C_PAN3204LL_WriteRegister(REG_PAW3212_CPI_Y,  dpi_tbl);
			dpi_reg_1 = I2C_PAN3204LL_ReadRegister(REG_PAW3212_CPI_X);
			dpi_reg_2 = I2C_PAN3204LL_ReadRegister(REG_PAW3212_CPI_Y);

		}while( (dpi_reg_1 == dpi_reg_2) && (dpi_reg_1 !=dpi_tbl) && (++timeout<32) );
		dbg_sensor_cpi = dpi_reg_1;
		return (dpi_reg_1 == dpi_tbl);
	}
	else
#endif
	{
		do{
			I2C_PAN3204LL_WriteRegister(REG_PAN3204LL_CONFIGURATION, dpi_tbl);
			dpi_reg = I2C_PAN3204LL_ReadRegister(REG_PAN3204LL_CONFIGURATION);
		}while( (dpi_reg!=dpi_tbl) && (++timeout<32) );
		dbg_sensor_cpi = dpi_reg;
		return (dpi_reg == dpi_tbl);
	}
}

int Sensor3204_Wakeup(u32 sensor){

    I2C_PAN3204LL_WriteRegister(REG_PAN3204LL_CONFIGURATION, 2);    //clear BIT(3) to exit Power down mode, deault cpi-rate-2
#if(MOUSE_SENSOR_YS8008_EN)
    if(SENSOR_YS8008 == sensor_type){
    	WaitUs(5001);
    }
#endif

    I2C_PAN3204LL_WriteRegister(REG_PAN3204LL_OPERATION_MODE, WAKEUP_3204);			//sleep enable and wakeup
    if(sensor_type == SENSOR_KA9){
    	SensorKA9_WakeUp_Setting();
    }
	return 1;
}


#endif
#endif

