
#pragma once

#ifndef _USER_CONFIG_DEFINED_
//#error	user_config.h must be include before this
#endif

#define GPIO_PULL_UP_0		0
#define GPIO_PULL_UP_1M		1
#define GPIO_PULL_UP_10K	2
#define GPIO_PULL_DN_100K	3


#ifndef GPIO1_OUTPUT_ENABLE
#define GPIO1_OUTPUT_ENABLE 	0
#endif
#ifndef GPIO2_OUTPUT_ENABLE
#define GPIO2_OUTPUT_ENABLE 	0
#endif
#ifndef GPIO3_OUTPUT_ENABLE
#define GPIO3_OUTPUT_ENABLE 	0
#endif
#ifndef GPIO4_OUTPUT_ENABLE
#define GPIO4_OUTPUT_ENABLE 	0
#endif
#ifndef GPIO5_OUTPUT_ENABLE
#define GPIO5_OUTPUT_ENABLE 	0
#endif
#ifndef GPIO6_OUTPUT_ENABLE
#define GPIO6_OUTPUT_ENABLE 	0
#endif
#ifndef GPIO7_OUTPUT_ENABLE
#define GPIO7_OUTPUT_ENABLE 	0
#endif



#ifndef GPIO1_INPUT_ENABLE
#define GPIO1_INPUT_ENABLE 	1
#endif
#ifndef GPIO2_INPUT_ENABLE
#define GPIO2_INPUT_ENABLE 	1
#endif
#ifndef GPIO3_INPUT_ENABLE
#define GPIO3_INPUT_ENABLE 	1
#endif
#ifndef GPIO4_INPUT_ENABLE
#define GPIO4_INPUT_ENABLE 	1
#endif
#ifndef GPIO5_INPUT_ENABLE
#define GPIO5_INPUT_ENABLE 	1
#endif
#ifndef GPIO6_INPUT_ENABLE
#define GPIO6_INPUT_ENABLE 	1
#endif
#ifndef GPIO7_INPUT_ENABLE
#define GPIO7_INPUT_ENABLE 	1
#endif


#ifndef GPIO1_DATA_STRENGTH
#define GPIO1_DATA_STRENGTH 	1
#endif
#ifndef GPIO2_DATA_STRENGTH
#define GPIO2_DATA_STRENGTH 	1
#endif
#ifndef GPIO3_DATA_STRENGTH
#define GPIO3_DATA_STRENGTH 	1
#endif
#ifndef GPIO4_DATA_STRENGTH
#define GPIO4_DATA_STRENGTH 	1
#endif
#ifndef GPIO5_DATA_STRENGTH
#define GPIO5_DATA_STRENGTH 	1
#endif
#ifndef GPIO6_DATA_STRENGTH
#define GPIO6_DATA_STRENGTH 	1
#endif
#ifndef GPIO7_DATA_STRENGTH
#define GPIO7_DATA_STRENGTH 	1
#endif


#ifndef GPIO1_DATA_OUT
#define GPIO1_DATA_OUT 		0
#endif
#ifndef GPIO2_DATA_OUT
#define GPIO2_DATA_OUT 		0
#endif
#ifndef GPIO3_DATA_OUT
#define GPIO3_DATA_OUT 		0
#endif
#ifndef GPIO4_DATA_OUT
#define GPIO4_DATA_OUT 		0
#endif
#ifndef GPIO5_DATA_OUT
#define GPIO5_DATA_OUT 		0
#endif
#ifndef GPIO6_DATA_OUT
#define GPIO6_DATA_OUT 		0
#endif
#ifndef GPIO7_DATA_OUT
#define GPIO7_DATA_OUT 		0
#endif


#ifndef MSDO_OUTPUT_ENABLE
#define MSDO_OUTPUT_ENABLE 		0
#endif
#ifndef MSDI_OUTPUT_ENABLE
#define MSDI_OUTPUT_ENABLE 		0
#endif
#ifndef MCLK_OUTPUT_ENABLE
#define MCLK_OUTPUT_ENABLE 		0
#endif
#ifndef MSCN_OUTPUT_ENABLE
#define MSCN_OUTPUT_ENABLE 		0
#endif
#ifndef SWS_OUTPUT_ENABLE
#define SWS_OUTPUT_ENABLE 		0
#endif



#ifndef MSDO_INPUT_ENABLE
#define MSDO_INPUT_ENABLE 		1
#endif
#ifndef MSDI_INPUT_ENABLE
#define MSDI_INPUT_ENABLE 		1
#endif
#ifndef MCLK_INPUT_ENABLE
#define MCLK_INPUT_ENABLE 		1
#endif
#ifndef MSCN_INPUT_ENABLE
#define MSCN_INPUT_ENABLE 		1
#endif
#ifndef SWS_INPUT_ENABLE
#define SWS_INPUT_ENABLE 		1
#endif



#ifndef MSDO_DATA_STRENGTH
#define MSDO_DATA_STRENGTH 		1
#endif
#ifndef MSDI_DATA_STRENGTH
#define MSDI_DATA_STRENGTH 		1
#endif
#ifndef MCLK_DATA_STRENGTH
#define MCLK_DATA_STRENGTH 		1
#endif
#ifndef MSCN_DATA_STRENGTH
#define MSCN_DATA_STRENGTH 		1
#endif
#ifndef SWS_DATA_STRENGTH
#define SWS_DATA_STRENGTH 		1
#endif


#ifndef MSDO_DATA_OUT
#define MSDO_DATA_OUT 			0
#endif
#ifndef MSDI_DATA_OUT
#define MSDI_DATA_OUT 			0
#endif
#ifndef MCLK_DATA_OUT
#define MCLK_DATA_OUT 			0
#endif
#ifndef MSCN_DATA_OUT
#define MSCN_DATA_OUT 			0
#endif
#ifndef SWS_DATA_OUT
#define SWS_DATA_OUT 			0
#endif


#ifndef GPIO1_FUNC
#define	GPIO1_FUNC		AS_GPIO
#endif
#ifndef GPIO2_FUNC
#define	GPIO2_FUNC		AS_GPIO
#endif
#ifndef GPIO3_FUNC
#define	GPIO3_FUNC		AS_GPIO
#endif
#ifndef GPIO4_FUNC
#define	GPIO4_FUNC		AS_GPIO
#endif
#ifndef GPIO5_FUNC
#define	GPIO5_FUNC		AS_GPIO
#endif
#ifndef GPIO6_FUNC
#define	GPIO6_FUNC		AS_GPIO
#endif
#ifndef GPIO7_FUNC
#define	GPIO7_FUNC		AS_GPIO
#endif
#ifndef MSDO_FUNC		// MSDI_FUNC, MCLK_FUNC, MSCN_FUNC, must the same as MSDO_FUNC
#define	MSDO_FUNC		AS_MSPI		// AS_GPIO, AS_MSPI
#endif
#ifndef MSDI_FUNC		// MSDI_FUNC, MCLK_FUNC, MSCN_FUNC, must the same as MSDO_FUNC
#define	MSDI_FUNC		AS_MSPI		// AS_GPIO, AS_MSPI
#endif
#ifndef MCLK_FUNC		// MSDI_FUNC, MCLK_FUNC, MSCN_FUNC, must the same as MSDO_FUNC
#define	MCLK_FUNC		AS_MSPI		// AS_GPIO, AS_MSPI
#endif
#ifndef MSCN_FUNC		// MSDI_FUNC, MCLK_FUNC, MSCN_FUNC, must the same as MSDO_FUNC
#define	MSCN_FUNC		AS_MSPI		// AS_GPIO, AS_MSPI
#endif
#ifndef SWS_FUNC
#define	SWS_FUNC		AS_SWIRE	// AS_GPIO, AS_SWIRE, AS_UART
#endif



#ifndef PULLUP_WAKEUP_SRC_GPIO567
#define PULLUP_WAKEUP_SRC_GPIO567	  	0				//gp5/gp6/gp7 disable 1M pull up
#endif

#ifndef PULLDOWN_WAKEUP_SRC_GPIO567
#define PULLDOWN_WAKEUP_SRC_GPIO567		0				//gp5/gp6/gp7 disable 100K pull down
#endif

#ifndef PULLUP_WAKEUP_SRC_MSDI
#define	PULLUP_WAKEUP_SRC_MSDI			0				//msdi disable 1M pull up
#endif


#ifndef PULL_WAKEUP_SRC_MSDO
#define	PULL_WAKEUP_SRC_MSDO	0
#endif

#ifndef PULL_WAKEUP_SRC_MSCN
#define	PULL_WAKEUP_SRC_MSCN	0
#endif
#ifndef PULL_WAKEUP_SRC_MCLK
#define	PULL_WAKEUP_SRC_MCLK	0
#endif
#ifndef PULL_WAKEUP_SRC_SWS
#define	PULL_WAKEUP_SRC_SWS	0
#endif

#define	PM_PIN_PULLUP_1M		1
#define	PM_PIN_PULLUP_10K		2
#define	PM_PIN_PULLDOWN_100K	3
#define	PM_PIN_UP_DOWN_FLOAT	0xff

