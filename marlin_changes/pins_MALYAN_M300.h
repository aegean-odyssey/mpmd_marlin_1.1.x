/** 
 * Malyan M300 (Monoprice Mini Delta) BSP for Marlin 
 * Copyright (C) 2019 Aegean Associates, Inc.
 *
 * pins description
 */

/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/* Malyan M300 (Monoprice MP Mini Delta) pin assigments
 * 15 AUG 2019 Created. Aegean Associates, Inc. (odyssey@aegean.com)
 * 18 NOV 2019 NOTE! comments refer the *stock* orientation of the
 *             tower axes (X,Y,Z stepper motors). Refer to the files,
 *             HAL32_stm.h and HAL_stm32.c, to see how the mapping 
 *             from pin number to actual GPIO is adjusted to support
 *             the ROTATE_TOWER_AXES compile-time option.
 */

#define BOARD_NAME          "Malyan M300"
#define MAX_EXTRUDERS       1

// limit switches

#define X_MAX_PIN           23  // GPIOC,GPIO_PIN_13
#define Y_MAX_PIN           24  // GPIOC,GPIO_PIN_14
#define Z_MAX_PIN           25  // GPIOC,GPIO_PIN_15

// z probe

#define Z_MIN_PIN           26  // GPIOB,GPIO_PIN_7
#define Z_MIN_PROBE_PIN     26  // GPIOB,GPIO_PIN_7

// stepper motors

#define X_DIR_PIN           13  // GPIOB,GPIO_PIN_13
#define X_STEP_PIN          14  // GPIOB,GPIO_PIN_14
#define X_ENABLE_PIN        10  // GPIOB,GPIO_PIN_10

#define Y_DIR_PIN           11  // GPIOB,GPIO_PIN_11
#define Y_STEP_PIN          12  // GPIOB,GPIO_PIN_12
#define Y_ENABLE_PIN        10  // GPIOB,GPIO_PIN_10

#define Z_DIR_PIN           15  // GPIOB,GPIO_PIN_1
#define Z_STEP_PIN          16  // GPIOB,GPIO_PIN_2
#define Z_ENABLE_PIN        10  // GPIOB,GPIO_PIN_10

#define E0_DIR_PIN          17  // GPIOA,GPIO_PIN_6
#define E0_STEP_PIN         18  // GPIOA,GPIO_PIN_7
#define E0_ENABLE_PIN       19  // GPIOB,GPIO_PIN_0

// temperature sensors

#define TEMP_0_PIN           8  // GPIOA,GPIO_PIN_0 (analog)
#define TEMP_BED_PIN         9  // GPIOA,GPIO_PIN_4 (analog)

// heaters / fans

#define HEATER_0_PIN        21  // GPIOA,GPIO_PIN_1
#define HEATER_BED_PIN      22  // GPIOA,GPIO_PIN_5

#if CONFIGURE_FAN_AS_PART_COOLING
#undef  E0_AUTO_FAN_PIN
#define E0_AUTO_FAN_PIN     -1  // GPIOA,GPIO_PIN_8
#define FAN_PIN             20  // GPIOA,GPIO_PIN_8
#else
#undef  E0_AUTO_FAN_PIN
#define E0_AUTO_FAN_PIN     20
#define FAN_PIN             20
#endif

// leds

#define STAT_LED_RED_PIN    27 // GPIOB,GPIO_PIN_15
#define STAT_LED_BLUE_PIN   28 // GPIOB,GPIO_PIN_9
#define STAT_LED_GREEN_PIN  29 // GPIOB,GPIO_PIN_8

// misc pins

#define SD_DETECT_PIN       -1
#define SDPOWER             -1
#define SDSS                30  // GPIOB,GPIO_PIN_6
#define LED_PIN             27  // GPIOB,GPIO_PIN_15
#define PS_ON_PIN           -1
#define SUICIDE_PIN         -1
#define KILL_PIN            34  // GPIOA,GPIO_PIN_15
#define CASE_LIGHT_PIN      -1

#if CONFIGURE_FAN1_AND_EXTRA_IO
#define FAN1_PIN            35  // GPIOA,GPIO_PIN_3
#define FREE_PIN            36  // GPIOA,GPIO_PIN_2
#endif

// spi

#define SCK_PIN   31  // GPIOB,GPIO_PIN_3
#define MISO_PIN  32  // GPIOB,GPIO_PIN_4
#define MOSI_PIN  33  // GPIOB,GPIO_PIN_5
#define SS_PIN    30  // GPIOB,GPIO_PIN_6
