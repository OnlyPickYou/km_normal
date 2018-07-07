#pragma once

#if(MCU_CORE_TYPE == MCU_CORE_8266)
#include "../mcu_spec/register_8266.h"
#elif(MCU_CORE_TYPE == MCU_CORE_8366)
#include "../mcu_spec/register_8366.h"
#elif(MCU_CORE_TYPE == MCU_CORE_8368)
#include "../mcu_spec/register_8368.h"
#elif(MCU_CORE_TYPE == MCU_CORE_8510)
#include "../mcu_spec/register_8510.h"
#elif(MCU_CORE_TYPE == MCU_CORE_8511)			//add for cavy dongle
#include "../mcu_spec/register_8511.h"
#elif(MCU_CORE_TYPE == MCU_CORE_8267)
#include "../mcu_spec/register_8267.h"
#endif
