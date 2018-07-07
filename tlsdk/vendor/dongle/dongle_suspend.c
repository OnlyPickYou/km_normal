/*
 * dongle_suspend.c
 *
 *  Created on: 2014-7-10
 *      Author: Telink
 */

#include "../../proj/tl_common.h"
#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"
#include "dongle_suspend.h"


extern  u8  usb_has_suspend_irq;

int resume_host_flg;

#ifndef USB_REMOTE_WAKEUP_FEATURE_ENABLE
#define USB_REMOTE_WAKEUP_FEATURE_ENABLE	1
#endif

void	dongle_resume_request ()
{
	resume_host_flg = 1;
}

int suspend_and_wakeup (void)
{

	static u32 tick = 0;
	static u32 tick_wakeup = 0;
	static int wakeup = 1;
	int allow_suspend = 0;

	if (!allow_suspend && clock_time_exceed (tick, 2000000))  //suspend allowed after power on for 2 seconds
		allow_suspend = 1;


	if (resume_host_flg)
	{
		if (reg_irq_src & FLD_IRQ_USB_PWDN_EN)
		{
			usb_resume_host ();
		}
		resume_host_flg = 0;
	}

	if (!(reg_irq_src & FLD_IRQ_USB_PWDN_EN)) {
		usb_has_suspend_irq = 0;
	}

	if (allow_suspend && usb_has_suspend_irq && wakeup
			&& clock_time_exceed (tick_wakeup, 15*1000)) {  //PM_USB_WAKEUP_TIME
		wakeup = 0;

		u8 r = irq_disable();			// must

		if (reg_usb_mdev & BIT(2)) {
			cpu_sleep_wakeup_rc (0 , PM_WAKEUP_CORE | PM_WAKEUP_TIMER, 200);
		}
		else {
			cpu_sleep_wakeup_rc (0 ,PM_WAKEUP_CORE, 200);
		}

		//restore
		clock_init();
		rf_drv_init(1);
		reg_tmr_ctrl |= FLD_TMR1_EN;
		extern  u8	host_channel;
		rf_set_channel (host_channel, RF_CHN_TABLE);
		rf_set_rxmode ();
		irq_restore(r);
		tick_wakeup = clock_time ();
		wakeup = 1;

	}
	return 1;
}


void usb_resume_host(void)
{
	if ((reg_usb_mdev & BIT(2)) && (reg_irq_src & FLD_IRQ_USB_PWDN_EN) ){   // (reg_usb_mdev & BIT(2))
		reg_wakeup_en = FLD_WAKEUP_SRC_USB_RESM;
		reg_wakeup_en = FLD_WAKEUP_SRC_USB;
	}
}

void proc_suspend(void)
{
	if(usb_has_suspend_irq){
		suspend_and_wakeup();
	}
}
