/*
 * mouse_wheel.c
 *
 *  Created on: Feb 12, 2014
 *      Author: xuzhen
 */

#include "../../proj/tl_common.h"
#include "mouse_wheel.h"

#if(CHIP_8366_A1)

/**************************************************************************
GPIO_GP0  = BIT(8)		00
GPIO_GP1  = BIT(9)     	01
GPIO_GP2  = BIT(10)    	02
GPIO_GP3  = BIT(11)    	03
GPIO_GP4  = BIT(12)    	04
GPIO_GP5  = BIT(13)    	05
GPIO_GP6  = BIT(14)		06
GPIO_GP7  = BIT(4) 		07
GPIO_GP8  = BIT(5) 		08
GPIO_GP9  = BIT(6) 		09
GPIO_GP10 = BIT(7) 		0a
**************************************************************************/
static inline u8 gpio_to_wheel_value(u32 pin)
{
	if(pin & 0x7f00){
		pin>>=8;
	}
	else if(pin & 0xf0){
		pin<<=3;
	}

	u8 i = 0;
	for(i=0;i<12;i++){
		if(pin &(1<<i)){
			return	i;
		}
	}

	return 0;
}



void mouse_wheel_init(mouse_hw_t *mouse_hw)
{
#if MOUSE_SENSOR_IO_FIXED
	gpio_setup_up_down_resistor(M_HW_WHEEL_Z0,1);
	gpio_setup_up_down_resistor(M_HW_WHEEL_Z1,1);

	write_reg8(0x8000d2,gpio_to_wheel_value(M_HW_WHEEL_Z0));
	write_reg8(0x8000d3,gpio_to_wheel_value(M_HW_WHEEL_Z1));
#else
	gpio_setup_up_down_resistor(mouse_hw->wheel[0], 1 );
	gpio_setup_up_down_resistor(mouse_hw->wheel[1], 1 );

	//write_reg8(0x8000d2,0x05);   //wheel1  GP5
	//write_reg8(0x8000d3,0x00);   //wheel2  GP0
	write_reg8(0x8000d2,gpio_to_wheel_value(mouse_hw->wheel[0]));
	write_reg8(0x8000d3,gpio_to_wheel_value(mouse_hw->wheel[1]));

#endif
	write_reg8(0x800065,0xff);   //wheel clk enable
#if(WHEEL_TWO_STEP_PROC)
	write_reg8(0x8000d7,0x03);   //BIT(0) 0: 1¸ñ        1:  2¸ñ      	BIT(1)  wakeup enable
#else
	write_reg8(0x8000d7,0x02);
#endif
	write_reg8(0x8000d1,0x00);   //filter   00-07     00 is best
 }




#else

s8		wheel_cnt = 0;

int		wheel_irq = 0;
int		wheel_level;

u32 mouse_wheel0;
u32 mouse_wheel1;
volatile u32 wheel_src;



///////////////////////////////////////////////////////////
// for wheel interrupt
///////////////////////////////////////////////////////////

void gpio_user_irq_handler(void)
{

	u8 src0_v,src1_v; //debug
	int dir,level;

	src0_v = gpio_read(mouse_wheel0) ? 1:0;
	src1_v = gpio_read(mouse_wheel1) ? 1:0;

	//gpio_clr_interrupt(wheel_src);
	gpio_clr_interrupt_and_wakeup(wheel_src);

	if(wheel_src == mouse_wheel0){
		wheel_src = mouse_wheel1;
		level = !src1_v;
		dir = -1;
	}
	else{
		wheel_src = mouse_wheel0;
		level = !src0_v;
		dir = 1;
	}

	if(wheel_level ^ level){
		wheel_cnt += dir;
	}

	wheel_level = level;
	//gpio_set_interrupt(wheel_src,wheel_level);
	gpio_set_interrupt_and_wakeup(wheel_src,wheel_level);
}



void mouse_wheel_init(mouse_hw_t *mouse_hw)
{
	//////////////////////////////////////////////////////////
	//setup wheel interrupt
	//////////////////////////////////////////////////////////
	//mouse_wheel_pullup_en (1);
	wheel_cnt = 0;
	wheel_irq = 0;
	gpio_setup_up_down_resistor(mouse_hw->wheel[0], 1 );
	gpio_setup_up_down_resistor(mouse_hw->wheel[1], 1 );
	mouse_wheel0 = mouse_hw->wheel[0];
	mouse_wheel1 = mouse_hw->wheel[1];

	wheel_src = mouse_wheel0;
	wheel_level = !gpio_read (mouse_wheel0);
	//gpio_set_interrupt(wheel_src,wheel_level);
	gpio_set_interrupt_and_wakeup(wheel_src,wheel_level);


#if(IRQ_GPIO0_ENABLE)
	reg_irq_mask |= FLD_IRQ_GPIO_RISC2_EN;	//FLD_IRQ_GPIO_RISC2;
	reg_irq_src = FLD_IRQ_GPIO_RISC2_EN;	//clear GPIO interrupt flag
#endif
}


#endif

void mouse_wheel_detect(mouse_status_t  * mouse_status)
{

}



