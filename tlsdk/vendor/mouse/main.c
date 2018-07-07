
#include "../../proj/tl_common.h"

#if(!__PROJECT_PM_TEST__)

#include "../../proj/mcu/watchdog_i.h"
#include "../../vendor/common/user_config.h"

#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"

extern void user_init();



int main (void) {

	cpu_wakeup_init();
	//usb_dp_pullup_en (1);     //mouse should disable dp pull-up

	//while (1);
	clock_init();

	dma_init();

	gpio_init();

	irq_init();


#if (MODULE_USB_ENABLE)
	usb_init();
#endif

#if (MODULE_RF_ENABLE)
	rf_drv_init(0);
#endif

    user_init ();

    irq_enable();

	while (1) {
#if(MODULE_WATCHDOG_ENABLE)
		wd_clear();
#endif
		main_loop ();
	}
}

#endif


