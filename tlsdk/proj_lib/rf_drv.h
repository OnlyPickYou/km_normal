
#pragma once

#if(MCU_CORE_TYPE == MCU_CORE_8266)
#include "rf_drv_8266.h"
#elif(MCU_CORE_TYPE == MCU_CORE_8366)
#include "rf_drv_8366.h"
#elif(MCU_CORE_TYPE == MCU_CORE_8368)
#include "rf_drv_8368.h"
#elif(MCU_CORE_TYPE == MCU_CORE_8510)
#include "rf_drv_8510.h"
#elif(MCU_CORE_TYPE == MCU_CORE_8511)
#include "rf_drv_8511.h"
#elif(MCU_CORE_TYPE == MCU_CORE_8267)
#include "rf_drv_8267.h"
#endif

