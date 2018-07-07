#include "../../vendor/common/user_config.h"
#include "../../proj/tl_common.h"
#include "../../proj/mcu/watchdog_i.h"
#include "mouse_sensor_a3000.h"
#include "mouse_sensor.h"
#include "trace.h"

#if MOUSE_SENSOR_A3000_EN

#define		CS_OPT_RESET_XY_BUFFER			0

#define		DLY_200NS    asm("tnop");asm("tnop")/*;asm("tnop");asm("tnop");asm("tnop");asm("tnop");asm("tnop");asm("tnop");asm("tnop");asm("tnop")*/
#define		DLY_100NS    asm("tnop") ; /*asm("tnop"); asm("tnop"); asm("tnop");asm("tnop")**/


#ifndef		PIN_SIF_CK
#define		PIN_SIF_CK		GPIO_GP3
#endif
#ifndef		PIN_SIF_DI
#define		PIN_SIF_DI		GPIO_MSDO
#endif
#ifndef		PIN_SIF_DO
#define		PIN_SIF_DO		GPIO_GP2
#endif
#ifndef		PIN_SIF_CN
#define		PIN_SIF_CN		GPIO_DP
#endif

#if ( CHIP_TYPE	== CHIP_TYPE_8366 )
#define sif_spi_clk_low    do{ (*((volatile u32*)0x800584)) &= (~PIN_SIF_CK); }while(0)
#define sif_spi_clk_high   do{ (*((volatile u32*)0x800584)) |= PIN_SIF_CK; }while(0)

#define sif_spi_mosi_low    do{ (*((volatile u32*)0x800584)) &= (~PIN_SIF_DO); }while(0)
#define sif_spi_mosi_high   do{ (*((volatile u32*)0x800584)) |= PIN_SIF_DO; }while(0)

#define sif_spi_read_input(pin) ( (*((volatile u32*)0x800580)) & (pin) )

static u8 irq_bak;
#define sif_spi_msdo_as_gpio   do{  irq_bak = reg_irq_en;\
                                    reg_irq_en = 0;\
                                    ( *((volatile u8*)0x80058c) = 0x9f );\    
                               }while(0)                                 //msdo output must set gpio
#define sif_spi_msdo_as_spi   do{ ( *((volatile u8*)0x80058c) = 0x9e ); reg_irq_en = irq_bak; }while(0)  //msdo  as spi do, revoer from gpio

#define sif_spi_msdo_reuse_dis_irq   do{  irq_bak = reg_irq_en; reg_irq_en = 0;}while(0)
#define sif_spi_msdo_reuse_restore_irq   do{ reg_irq_en = irq_bak; }while(0)

#else
#define sif_spi_clk_low    (gpio_write (PIN_SIF_CK, 0))
#define sif_spi_clk_high   (gpio_write (PIN_SIF_CK, 1))
#define sif_spi_mosi_low   (gpio_write (PIN_SIF_DO, 0))
#define sif_spi_mosi_high  (gpio_write (PIN_SIF_DO, 1))

#endif

#if 0
const int resolution_table[7] = {400, 500, 600, 800, 1000, 1200, 1600};
const int ledcurrent_table[4] = {0, 1, 2, 3};
const int motionctrl_table[2] = {0, 1};
unsigned char product_id_A3000[2] = {0, 0};
#endif

void a3000_spi_init(unsigned int *sensor_hw){
    
	// set in header file

	//gpio_set_func( PIN_SIF_DI, AS_GPIO);
	//gpio_set_input_en (PIN_SIF_DI, 1);
	//gpio_set_output_en (PIN_SIF_DI, 0);

	//gpio_set_func( PIN_SIF_DO, AS_GPIO);
	//gpio_set_input_en(PIN_SIF_DO, 0);
	gpio_set_output_en (PIN_SIF_DO, 1);

	//gpio_set_func( PIN_SIF_CN, AS_GPIO);
	gpio_set_input_en(PIN_SIF_CN, 0);
	gpio_set_output_en (PIN_SIF_CN, 1);

	//gpio_set_input_en(PIN_SIF_CK, 0);
	gpio_write (PIN_SIF_CK, 1);
	gpio_set_output_en (PIN_SIF_CK, 1);

}

//PIN_SIF_DI operation must be in ram, when re-use flash spi pin MSDO as PIN_SIF_DI
_attribute_ram_code_ unsigned int spi_ReadByte(void){
	unsigned char ii = 0;
	unsigned char dat = 0;
    sif_spi_msdo_reuse_dis_irq;
	for(ii=0; ii<8; ii++){
        sif_spi_clk_low;
		//DLY_200NS;
        sif_spi_clk_high;
		if( sif_spi_read_input(PIN_SIF_DI) ){
			dat |= (1<<(7-ii));
		}else{
			dat &= ~(1<<(7-ii));
		}
		//DLY_200NS;
	}
    sif_spi_msdo_reuse_restore_irq;
	return dat;
}

void spi_SendByte( unsigned int data){
	unsigned int ii = 0;
	for(ii=0; ii<8; ii++){
        sif_spi_clk_low;
		if(data&(1<<(7-ii))){
            sif_spi_mosi_high;
		}else{
            sif_spi_mosi_low;
		}
        sif_spi_clk_high;
	}
}

unsigned int sensor_reg_read(unsigned int cAddr){
	unsigned char dat;
	gpio_write(PIN_SIF_CN, 0);
	spi_SendByte((cAddr&0x7f));
	//WaitUs(4);       //4u read delay is necessary!
	//function jump > 4.6us
	WaitUs(1);     //1us margine
	dat = (spi_ReadByte());
	gpio_write(PIN_SIF_CN, 1);
	return dat;
}

void sensor_reg_write(unsigned int cAddr, unsigned int cData){
	gpio_write(PIN_SIF_CN, 0);;
	DLY_200NS;
	spi_SendByte((cAddr|0x80));
	spi_SendByte(cData);
	gpio_write(PIN_SIF_CN, 1);
	WaitUs(4);
}
#define A3000_12_BIT_MODE 1

#define KEY_DPI_VALID 0xf0
#define SENSOR_X_REVERSE      0x80
#define SENSOR_Y_REVERSE      0x40
#define SENSOR_XY_SWITCH      0x20
#define SENSOR_XY_REVERSE     0x10
const unsigned char sensor_a3000_settings[] = {
#if( A3000_12_BIT_MODE )
		0x0d, 0xa1,
#else	
		0x0d, 0x21,
#endif

		0x47, 0x52,
		0x48, 0x68,
		0x49, 0x20,
		0x6d, 0x41,
		0x6e, 0xa0,
		0x70, 0x85,
		0x71, 0x64,
		0x72, 0x46,
		0x73, 0x37,
		0x74, 0x41,
		0x75, 0x28,
		0x76, 0x16,
		0x77, 0x0f,
		0x64, 0xf0,
		0x03, 0x03,
		0x48, 0x60,
		0x41, 0x00,
};

volatile unsigned char sensor_a3000_pid = 0;
static inline u32 mouse_sensor_a3000_identify( void ){
	/* reset sensor */
	sensor_reg_write(SENSOR_A3000_REG_RESET, 0x5a);
	WaitUs(5000);
	sensor_reg_write(SENSOR_A3000_REG_RESET, 0x00);
	WaitUs(5000);

	/* read pid/vid */
	u32 a3000_pid = sensor_reg_read(SENSOR_A3000_REG_PID);
    sensor_a3000_pid = a3000_pid;
	if (A3000_PID != a3000_pid)
		return 0;
    else
        return 1;
}

int A3000_OPTSensor_Init(unsigned int mode){
	int i;
	unsigned int addr;
	unsigned char *ptbl = sensor_a3000_settings;
    if ( !mouse_sensor_a3000_identify() ){
        return U8_MAX;
    }
	u32 a3000_vid = sensor_reg_read(SENSOR_A3000_REG_VID);

	/* init */
	for (i = 0; i < sizeof(sensor_a3000_settings); i += 2) {
		addr = ptbl[0];
		sensor_reg_write(addr, ptbl[1]);
		ptbl+=2;
	}
	return SENSOR_A3000;
}

int cur_cpi;
void A3000_OPTSensor_recover( )
{
	if( sensor_reg_read(SENSOR_A3000_REG_PID) != A3000_PID ){
		return A3000_OPTSensor_Init(cur_cpi);
	}
}

#define MAX_REPORT_XY 127
unsigned int A3000_OPTSensor_motion_report(signed char *pBuf, u32 flow)    // ????????????????    *pBuf[] ???
{
	unsigned char optical_status = 0;

	//---------------------------------------------------------
	// Read MOTION_STATUS regisgter, and then DELTA_X register, and
	// third DELTA_Y regisgter. The sequence is not suggested to change.
	//
	// Modified by LiuWen. 2012/01/17.
	//---------------------------------------------------------
	optical_status = sensor_reg_read(SENSOR_A3000_REG_MOTION_ST);
	if(optical_status & SENSOR_A3000_MOTION_STATUS_MOT){
		
		unsigned char delta_xy_high, delta_x_l, delta_y_l, delta_x_h, delta_y_h;
		signed short delta_x_16, delta_y_16;
		delta_x_l = sensor_reg_read(SENSOR_A3000_REG_DELTA_X);
		delta_y_l = sensor_reg_read(SENSOR_A3000_REG_DELTA_Y);
		delta_xy_high = sensor_reg_read(SENSOR_A3000_REG_DELTA_XY_H);
		
		if( delta_xy_high & 0x80  )
			delta_x_h = ( (A3000_DATA_DELTA_X_H & delta_xy_high) >> 4 ) | 0xf0;
		else
			delta_x_h = ( (A3000_DATA_DELTA_X_H & delta_xy_high) >> 4 );
		delta_x_16 = (delta_x_h << 8) | delta_x_l;

		if( delta_xy_high & 0x08  )
			delta_y_h = ( A3000_DATA_DELTA_Y_H & delta_xy_high ) | 0xf0;
		else
			delta_y_h = ( A3000_DATA_DELTA_Y_H & delta_xy_high );
		delta_y_16 = ( (delta_y_h << 8) | delta_y_l );

		if(delta_x_16 > MAX_REPORT_XY || delta_y_16 > MAX_REPORT_XY || delta_x_16 < -MAX_REPORT_XY || delta_y_16 < -MAX_REPORT_XY)
		{			
			if( abs(delta_x_16) > abs(delta_y_16) )
			{					
				pBuf[0] = (delta_x_16 > MAX_REPORT_XY) ? -MAX_REPORT_XY : MAX_REPORT_XY;
				pBuf[1] = -(delta_y_16*MAX_REPORT_XY) / abs(delta_x_16);					
			}
			else
			{		
				pBuf[0] = -(delta_x_16*MAX_REPORT_XY) / abs(delta_y_16);					
				pBuf[1] = (delta_y_16 > MAX_REPORT_XY) ? -MAX_REPORT_XY : MAX_REPORT_XY;
			}
		}//------------------------------------------Ð±ÂÊ²»±ä»»Ëã--------------------
		else
		{
			pBuf[0] = -delta_x_16;
			pBuf[1] = -delta_y_16;
		}
		
		return 1;
		
	}
	else{
#if CS_OPT_RESET_XY_BUFFER
		*pBuf = 0;
	    *(++pBuf)=0;
#endif
	    return 0;
	}
}


void A3000_OPTSensor_Shutdown(void){
	sensor_reg_write( SENSOR_A3000_REG_MOUSE_CTRL, \
					  sensor_reg_read(SENSOR_A3000_REG_MOUSE_CTRL) | BIT(1)
					);//power down

}

int A3000_OPTSensor_Wakeup(u32 sensor){
    //a3000 wakeuped when sensor init
    //sensor_reg_write( SENSOR_A3000_REG_RESET, SENSOR_A3000_RESET_VALUE );
	return 1;
}

#if NEW_DPI_CONFIG
unsigned char dpi_ctrl_table[] = { CPI1000_A3000, CPI1500_A3000, CPI2000_A3000, 0};
void A3000_OPTSensor_dpi_table(int segment,unsigned char* pVal)
{
	int i;
	for(i=0;i<segment;i++){
		dpi_ctrl_table[i] = *(pVal+i);
	}
}
#else
const unsigned char dpi_ctrl_table[] = { CPI1000_A3000, CPI1250_A3000, CPI1500_A3000, CPI2000_A3000, };
#endif

static inline void sensor_dpi_rate_update(unsigned int dpi_rate){

	unsigned char dpi_ctrl_val = CPI_RES_EN | \
								 dpi_ctrl_table[dpi_rate] | \
								 ((~0x1c) & sensor_reg_read(SENSOR_A3000_REG_MOUSE_CTRL));
	
	sensor_reg_write( SENSOR_A3000_REG_MOUSE_CTRL, dpi_ctrl_val );

}

unsigned int A3000_OPTSensor_dpi_update(unsigned int dpi_rate){
	{
		sensor_dpi_rate_update(dpi_rate);
		return 1;
	}

}

#endif

