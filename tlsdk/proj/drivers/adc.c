
#include "../mcu/register.h"
#include "../common/assert.h"
#include "../common/static_assert.h"
#include "../mcu/analog.h"
#include "syshw.h"
#include "adc.h"
#include "../mcu/clock.h"
#include "dfifo.h"
#include "../common/compatibility.h"
#include "../common/utility.h"

#if (MCU_CORE_TYPE == MCU_CORE_8266)


//set chn0 sampling cycles and resolution
void adc_set_chn0_sampling(int cyc0, int res0){
	reg_adc_ch0_samp_clk_res = MASK_VAL(FLD_ADC_CHN0_SAMP_CYCLE,cyc0, FLD_ADC_CHN0_SAMP_RESOL, res0);
}

//set chn1 and chn2  sampling cycles and resolution
void adc_set_ch12_sampling(int cyc1_2, int res1_2){
	reg_adc_ch12_samp_res = MASK_VAL(FLD_ADC_CHN12_SAMP_RESOL,cyc1_2);
	reg_adc_ch12_samp_clk = MASK_VAL( FLD_ADC_CHN12_SAMP_CYCLE, res1_2);
}


void adc_set_clk_freq(u8 mhz){
	assert((reg_adc_mod_h & FLD_ADC_MOD_H) == 0 && (reg_adc_mod_h & FLD_ADC_MOD_H_STEP) == 0);	// use low bits only
	reg_adc_mod = MASK_VAL(FLD_ADC_MOD, CLK_FHS_MZ, FLD_ADC_CLK_EN, 1);
	reg_adc_step_l = mhz;
}

void adc_clk_en(int en){
    if(en){
	    SET_FLD(reg_adc_mod_h, FLD_ADC_MOD_H_CLK);
	}else{
	    CLR_FLD(reg_adc_mod_h, FLD_ADC_MOD_H_CLK);
	}
}

#endif
