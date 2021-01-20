/** 
 * Malyan M300 (Monoprice Mini Delta) BSP for Marlin 
 * Copyright (C) 2019 Aegean Associates, Inc.
 */

// locate the application to 0x8002000 to use/accommodate the bootloader

#ifndef STM32_USE_BOOTLOADER
#define STM32_USE_BOOTLOADER
#endif

// CMSIS

#ifndef STM32F070xB
#define STM32F070xB
#endif

#ifndef ARM_MATH_CM0
#define ARM_MATH_CM0
#endif

#include "stm32f0xx.h"
#include "arm_math.h"

/* we use the default configuration as is */
#include "stm32f0xx_hal_conf_template.h"

/* HACK!
   the 10-2020-q4 toolchain generates a warning when compiling
   the HAL crc module. Since we don't use this module, disable
   it rather than fix it to avoid the warning and limit changes
   to STM32Cube-1.10.1 to those that are essential.
*/
#undef HAL_CRC_MODULE_ENABLED
