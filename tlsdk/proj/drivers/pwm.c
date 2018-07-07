
#include "../tl_common.h"
#include "pwm.h"


/**
 *
 * @param pwm_id      PWM ID, must be 0, 1, 2, 3
 * @param frequency   PWM output frequency, unit is HZ
 * @param duty_cyc    PWM duty cycle, accurate to 1/10000
 * @param phase       PWM phase offset
 */

#ifndef MCU_CORE_8366
static const u32 gpio_pwm_pin[]={GPIO_GP9, GPIO_GP10, GPIO_GP16, GPIO_GP20};
void pwm_setting(u32 pwm_id, u32 frequency, u32 duty_cyc, u32 phase)
{
	u32 cmp_tick, max_tick, div;

	assert(pwm_id<4);

	gpio_set_func(gpio_pwm_pin[pwm_id], AS_PWM);

	div = reg_pwm_clk;
	div++;
	max_tick = CLOCK_SYS_CLOCK_HZ/div/frequency;
	cmp_tick = max_tick*duty_cyc/10000;
	phase = max_tick*phase/10000;

	reg_pwm_phase(pwm_id) = phase;
	reg_pwm_cycle(pwm_id) = MASK_VAL(FLD_PWM_CMP, cmp_tick, FLD_PWM_MAX, max_tick);


}
#endif
