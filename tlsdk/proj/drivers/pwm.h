
#pragma once

#include "../tl_common.h"

/**
 *  Enable pwm Module clock
 *
 */
static inline void pwm_clk_en(int en)
{
	if(en){
		BM_SET(reg_clk_en, FLD_CLK_PWM_EN);
		BM_CLR(	reg_rst_clk0, FLD_RST_PWM);
	}
	else{
		BM_CLR(reg_clk_en, FLD_CLK_PWM_EN);
	}
}

/**
 *  setting the pwm clock source rate
 *  @param             sys_clk_div, the ratio of system clock and PWM clock source
 */
static inline void pwm_master_clk_div_set(u32 sys_clk_div)
{
	sys_clk_div ++;
	reg_pwm_clk = sys_clk_div;
}

#ifndef MCU_CORE_8366
static inline void pwm_set(int id, u16 max_tick, u16 cmp_tick){
	reg_pwm_cycle(id) = MASK_VAL(FLD_PWM_CMP, cmp_tick, FLD_PWM_MAX, max_tick);
}

static inline void pwm_set_cmp(int id, u16 cmp){
	reg_pwm_cmp(id) = cmp;
}

static inline void pwm_start(int id){
	BM_SET(reg_pwm_enable, BIT(id));
}

static inline void pwm_stop(int id){
	BM_CLR(reg_pwm_enable, BIT(id));
}
#endif

/**
 *
 * @param pwm_id      PWM ID, must be 0, 1, 2, 3
 * @param frequency   PWM output frequency, unit is HZ
 * @param duty_cyc    PWM duty cycle, accurate to 1/10000
 * @param phase       PWM phase offset
 */
void pwm_setting(u32 pwm_id, u32 frequency, u32 duty_cyc, u32 phase);


