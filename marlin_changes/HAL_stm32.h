/** 
 * Malyan M300 (Monoprice Mini Delta) BSP for Marlin 
 * Copyright (C) 2019 Aegean Associates, Inc.
 *
 * replacement for HAL.h  (see NOTE, below)
 */

/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware
 * [https://github.com/MarlinFirmware/Marlin]
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* NOTE: include this file before all others (e.g. gcc -include HAL_stm32.h)
 * to select the cpu device (CMSIS, STM32F070xB) and to configure the HAL
 * driver appropriately. ALSO, this file provides several work-arounds to
 * "preempt" the inclusion of a few of the original Marlin include files
 * that would otherwise cause problems when compiling for the STM32F070xB.
 * An included file that is NOT part of Marlin (e.g.'include <avr/eeprom.h>')
 * is created as an empty file and placed in the "included files search path"
 * to satisfy the reference to the file. Using these techniques, then, our
 * port requires only minimal changes to the original Marlin source files. 
 *
 * Highlights of the required changes (*file compatible with the original):
 * boards.h                      - added MALYAN_M300 board definition*
 * pins.h                        - include MALYAN_M300 pins description*
 * pins_MALYAN_M300.h            - created, the MALYAN_M300 pins description
 * Configuration.h               - Marlin configuration for MALYAN_M300
 * Configuration_adv.h           - Marlin configuration for MALYAN_M300
 * delay.h                       - minor (benign) change for MALYAN_M300*
 * cardreader.h                  - minor (benign) change for MALYAN_M300*
 * stepper.h                     - minor (benign) change for MALYAN_M300*
 * stepper.cpp                   - minor (benign) change for MALYAN_M300*
 * temperature.h                 - minor (benign) change for MALYAN_M300*
 * temperature.cpp               - minor (benign) change for MALYAN_M300*
 * Marlin_main.cpp               - add inverted KILL pin for MALYAN_M300*
 * HAL_stm32.h                   - replacement for HAL.h
 * HAL_stm32.c                   - HAL "glue" to support the STM32F070xB
 * MarlinSerial_stm32.cpp        - replacement for MarlinSerial.cpp
 * Sd2Card_stm32.cpp             - replacement for Sd2Card.cpp
 * configuration_store_stm32.cpp - replacement for configuration_store.cpp
 * watchdog_stm32.cpp            - replacement for watchdog.cpp
 * malyanlcd_stm32.cpp           - replacement for malyanlcd.cpp
 */

/* HAL ------------------------------------------------------------------ */

#ifndef _HAL_AVR_H_
#define _HAL_AVR_H_
// same as HAL.h, so we can preempt '#include "HAL.h"'

#ifdef __cplusplus
extern "C" {
#endif

#define _FASTIO_ARDUINO_H_
// same as fastio.h, so we can preempt '#include "fastio.h"'

#define WATCHDOG_H
// same as watchdog.h, so we can preempt '#include "watchdog.h"'
void watchdog_init(void);
void watchdog_reset(void);

#define _ENDSTOP_INTERRUPTS_H_
// same as endstop_interrupts.h, preempt '#include "endstop_interrupts.h"'

void setup_endstop_interrupts(void);
void endstop_isr(void);

// HAL

#include "stm32f0xx_hal.h"
#include "usbd_cdc.h"
#include "arm_math.h"

void HAL_setup(void);
void HAL_reboot(void);

// DEBUG

// extra malyan 300 user interface elements
// red:27, blue:28, green:29, off:1, on:0
void debug_led(uint8_t l, uint8_t s);
void debug_red(void);
void debug_green(void);
void debug_blue(void);

// simple ui (one switch, one led)
void led_0_solid(void);  // black (off)
void led_W_solid(void);  // white
void led_R_solid(void);  // red
void led_G_solid(void);  // green
void led_B_solid(void);  // blue
void led_Y_solid(void);  // yellow
void led_C_solid(void);  // cyan
void led_M_solid(void);  // magenta
void led_W_flash(void);  // flashing white
void led_R_flash(void);  // flashing red
void led_G_flash(void);  // flashing green
void led_B_flash(void);  // flashing blue
void led_Y_flash(void);  // flashing yellow
void led_C_flash(void);  // flashing cyan
void led_M_flash(void);  // flashing magenta
uint8_t pushbutton_pressed(void);
    
// GPIO 

int HAL_gpio_init(void);
void HAL_gpio_config(GPIO_TypeDef *gpio, uint16_t pin, uint16_t mode);
void HAL_gpio_write(GPIO_TypeDef *gpio, uint16_t pin, uint16_t state);
uint8_t HAL_gpio_read(GPIO_TypeDef *gpio, uint16_t pin);

void HAL_gpio_config_indirect(uint8_t pin, uint16_t mode);
void HAL_gpio_write_indirect(uint8_t pin, uint16_t state);
uint8_t HAL_gpio_read_indirect(uint8_t pin);

// FLASH

#define FLASHSTORE_ADDRESS  0x0801f800
#define E2END  0x400  // storage size (see configuration_store.h)

int HAL_flashstore_init(void);
int HAL_flashstore_write(const uint8_t * source, uint32_t length, int erase);

// ADC

#define ADC_TEMP_0_PIN    ADC_CHSELR_CHSEL0
#define ADC_TEMP_BED_PIN  ADC_CHSELR_CHSEL4
//#define HAL_ANALOG_SELECT(pin)  // defined below
#define HAL_START_ADC(pin) HAL_adc_start(ADC_ ## pin)
#define HAL_ADC_READY() HAL_adc_ready()
#define HAL_READ_ADC() HAL_adc_read()

int HAL_adc_init(void);
int HAL_adc_ready(void);
void HAL_adc_start(uint16_t ch);
uint16_t HAL_adc_read(void);

// TIM

int HAL_tim1_init(void);
void HAL_tim1_pwm(uint8_t v);

int HAL_tim6_init(void);
int HAL_tim7_init(void);

// IWDT

#define FAUX_TIMEOUT  4000  // 4.0s
#define IWDG_TIMEOUT  1500  // 4.8s
 
int HAL_iwdg_init(void);
void HAL_iwdg_refresh(void);
void faux_watchdog_interrupt(void);

// SPI1

int HAL_spi_init(uint8_t rate);
void HAL_spi_config(uint8_t rate);
uint8_t HAL_spi_recv(void);
void HAL_spi_send(uint8_t b);
void HAL_spi_read(uint8_t * buf, uint16_t len);
void HAL_spi_send_block(uint8_t token, const uint8_t * buf);

// USBD (CDC)

// FIXME!? see usbd_conf.c  
void HAL_PCD_MspInit(PCD_HandleTypeDef * pcd);
void HAL_PCD_MspDeInit(PCD_HandleTypeDef * pcd);

// USART1
// USART2

#define USART_RXNE  USART_ISR_RXNE
#define USART_TXE   USART_ISR_TXE
#define USART_TC    USART_ICR_TCCF
#define USART_FE    USART_ICR_FECF
#define USART_ORE   USART_ICR_ORECF

extern void ptimer_isr(void);
extern void qtimer_isr(void);
extern void usart1_isr(void);
extern void usart2_isr(void);

int HAL_usart_init (USART_TypeDef * usart, uint32_t rate);
uint8_t HAL_usart_check (USART_TypeDef * usart, uint32_t flags);
void HAL_usart_clear (USART_TypeDef * usart, uint32_t flags);
uint8_t HAL_usart_read (USART_TypeDef * usart);
void HAL_usart_send (USART_TypeDef * usart, uint8_t c);
void HAL_usart_txe_1(USART_TypeDef * usart);
void HAL_usart_txe_0(USART_TypeDef * usart);

// MARLIN (entry points)
void setup(void);
void loop(void);

#ifdef __cplusplus
}
#endif

/* MARLIN --------------------------------------------------------------- */

// The Marlin firmware makes some presumptions about F_CPU, so we use
// F_CPU_marlin and F_CPU_actual along with F_CPU to workaround some
// of the difficulties.
#define F_CPU_marlin  16000000ul
#define F_CPU_actual  48000000ul
#define F_CPU  F_CPU_actual

// The 60-watt power supply of the Monoprice Mini Delta doesn't provide
// sufficient power to heat the nozzle and the build plate at the same
// time. We slightly modify the temperature isr (see temperature.cpp)
// to ensure that only one heater is active at a time. Our VERY simple
// implementation, though, limits "full-on" for the heat bed to 99.2%,
// and always gives priority to hotend -- hopefully not a problem.
#define ONLY_ONE_HEATER_AT_A_TIME  1

#if MAKE_05ALIMIT
#undef  ONLY_ONE_HEATER_AT_A_TIME
#define ONLY_ONE_HEATER_AT_A_TIME  1
#endif

#if MAKE_10ALIMIT
#undef  ONLY_ONE_HEATER_AT_A_TIME
#define ONLY_ONE_HEATER_AT_A_TIME  0
#endif

// Marlin expects the "kill" pin input to be asserted low. Since this
// is not the case in the Monoprice Mini Delta, we add a compile-time
// switch to work-around the situation. (see Marlin_main.cpp)
#define KILL_PIN_ASSERTED_HIGH  1

// Configure IO as Marlin directs (as opposed to configuring all of the
// IO at the program's start) -- nice idea, but requires a workaround to
// use hardware pwm (FAN_USES_HARDWARE_PWM). (see the GPIO_20 hack below)
#define CONFIGURE_IO_INDIVIDUALLY  0

// Configure the fan output to use hardware pwm
#define FAN_USES_HARDWARE_PWM  1

// The single fan of the Monoprice Mini Delta is normally configured as
// an auto-cooling extruder fan. Set CONFIGURE_FAN_AS_PART_COOLING to 1
// to use M106/M107 to control the fan (NOT RECOMMENDED).
#define CONFIGURE_FAN_AS_PART_COOLING  0

// The stock firmware of the Monoprice Mini Delta places the "front" of
// the build plate away from the LCD display. Set ROTATE_TOWER_AXES to 1
// to redefine the stepper motor axes and thus rotate the tower axes.
// Specifically, X<=Y, Y<=Z, and Z<=X (where A<=B means A becomes B)
#define ROTATE_TOWER_AXES  1

// Invert the direction of a stepper motor by setting the corresponding
// bit(X,Y,Z,E) in STEPPER_DIRECTION_XYZE. It seems that Monoprice does
// not configure the stepper motors consistently, so it may be necessary
// adjust this value. Use the M503 report from the stock firmware to
// determine an appropriate value.
// e.g. if the stock firmware M503 reports:
// (M562 XYZE) XYZABCD---+... use 0b0001 (0x1) 
// (M562 XYZE) XYZABCD+++-... use 0b1110 (0xe)
// (M562 XYZE) XYZABCD----... use 0b0000 (0x0)
// (M562 XYZE) XYZABCD++++... use 0b1111 (0xf)
//
// NOTE! IMPORTANT! Be careful here, if we rotate the tower axes
// (#define ROTATE_TOWER_AXES  1), then the output from the stock
// M503 (M562) command must be adjusted (mapped) accordingly.
// e.g. if the stock firmware M503 reports:
// (M562 XYZE) XYZABCD+---... use 0b0100 (0x4) 
// (M562 XYZE) XYZABCD-+--... use 0b0010 (0x2)
// (M562 XYZE) XYZABCD--+-... use 0b1000 (0x8)
// 
#ifndef INVERT_STEPPER_DIRECTION_XYZE
#define INVERT_STEPPER_DIRECTION_XYZE  0b0001
#endif

// Custom codes, M988 and M989, open and close an output log file
// in the current working directory. Use a DOS 8.3 name for the
// file. "#define OVERLY_SIMPLISTIC_OUTPUT_LOGGING_HACK  1" to
// enable this feature.
// e.g.
// M988 logfile.txt  ; start writing output to "logfile.txt"
// M503              ; report settings
// M989              ; stop writing (close) the log file
//
#define OVERLY_SIMPLISTIC_OUTPUT_LOGGING_HACK  1



// must match pin description file, pins_MALYAN_M300.h
#define GPIO(pin) GPIO_ ##pin
#define GPIO_8   GPIOA,GPIO_PIN_0   // TEMP_0_PIN (analog)
#define GPIO_9   GPIOA,GPIO_PIN_4   // TEMP_BED_PIN (analog)
#define GPIO_10  GPIOB,GPIO_PIN_10  // X,Y,Z_ENABLE_PIN

#if ROTATE_TOWER_AXES
#define GPIO_11  GPIOB,GPIO_PIN_1   // Y_DIR_PIN
#define GPIO_12  GPIOB,GPIO_PIN_2   // Y_STEP_PIN
#define GPIO_13  GPIOB,GPIO_PIN_11  // X_DIR_PIN
#define GPIO_14  GPIOB,GPIO_PIN_12  // X_STEP_PIN
#define GPIO_15  GPIOB,GPIO_PIN_13  // Z_DIR_PIN
#define GPIO_16  GPIOB,GPIO_PIN_14  // Z_STEP_PIN
#else
#define GPIO_11  GPIOB,GPIO_PIN_11  // Y_DIR_PIN
#define GPIO_12  GPIOB,GPIO_PIN_12  // Y_STEP_PIN
#define GPIO_13  GPIOB,GPIO_PIN_13  // X_DIR_PIN
#define GPIO_14  GPIOB,GPIO_PIN_14  // X_STEP_PIN
#define GPIO_15  GPIOB,GPIO_PIN_1   // Z_DIR_PIN
#define GPIO_16  GPIOB,GPIO_PIN_2   // Z_STEP_PIN
#endif

#define GPIO_17  GPIOA,GPIO_PIN_6   // E0_DIR_PIN
#define GPIO_18  GPIOA,GPIO_PIN_7   // E0_STEP_PIN
#define GPIO_19  GPIOB,GPIO_PIN_0   // E0_ENABLE_PIN
#define GPIO_20  GPIOA,GPIO_PIN_8   // FAN_PIN, E0_AUTO_FAN_PIN
#define GPIO_21  GPIOA,GPIO_PIN_1   // HEATER_0_PIN
#define GPIO_22  GPIOA,GPIO_PIN_5   // HEATER_BED_PIN

#if ROTATE_TOWER_AXES
#define GPIO_23  GPIOC,GPIO_PIN_14  // X_MAX_PIN
#define GPIO_24  GPIOC,GPIO_PIN_15  // Y_MAX_PIN
#define GPIO_25  GPIOC,GPIO_PIN_13  // Z_MAX_PIN
#else
#define GPIO_23  GPIOC,GPIO_PIN_13  // X_MAX_PIN
#define GPIO_24  GPIOC,GPIO_PIN_14  // Y_MAX_PIN
#define GPIO_25  GPIOC,GPIO_PIN_15  // Z_MAX_PIN
#endif

#define GPIO_26  GPIOB,GPIO_PIN_7   // Z_MIN_PROBE_PIN, Z_MIN_PIN
#define GPIO_27  GPIOB,GPIO_PIN_15  // STAT_LED_RED_PIN, LED_PIN
#define GPIO_28  GPIOB,GPIO_PIN_9   // STAT_LED_BLUE_PIN
#define GPIO_29  GPIOB,GPIO_PIN_8   // STAT_LED_GREEN_PIN
#define GPIO_30  GPIOB,GPIO_PIN_6   // SS_PIN, SDSS
#define GPIO_31  GPIOB,GPIO_PIN_3   // SCK_PIN
#define GPIO_32  GPIOB,GPIO_PIN_4   // MISO_PIN
#define GPIO_33  GPIOB,GPIO_PIN_5   // MOSI_PIN
#define GPIO_34  GPIOA,GPIO_PIN_15  // KILL_PIN

#if CONFIGURE_IO_INDIVIDUALLY
// workaround, disable the config mechanism for the fan pin
#undef  GPIO_20
#define GPIO_20  NULL,GPIO_PIN_8    // FAN_PIN
#endif

#if ROTATE_TOWER_AXES
// x stepper
#define GPIO_X_DIR_PIN           GPIOB,GPIO_PIN_11
#define GPIO_X_STEP_PIN          GPIOB,GPIO_PIN_12
#define GPIO_X_ENABLE_PIN        GPIOB,GPIO_PIN_10
// y stepper
#define GPIO_Y_DIR_PIN           GPIOB,GPIO_PIN_1
#define GPIO_Y_STEP_PIN          GPIOB,GPIO_PIN_2
#define GPIO_Y_ENABLE_PIN        GPIOB,GPIO_PIN_10
// z stepper
#define GPIO_Z_DIR_PIN           GPIOB,GPIO_PIN_13
#define GPIO_Z_STEP_PIN          GPIOB,GPIO_PIN_14
#define GPIO_Z_ENABLE_PIN        GPIOB,GPIO_PIN_10
#else
// x stepper
#define GPIO_X_DIR_PIN           GPIOB,GPIO_PIN_13
#define GPIO_X_STEP_PIN          GPIOB,GPIO_PIN_14
#define GPIO_X_ENABLE_PIN        GPIOB,GPIO_PIN_10
// y stepper
#define GPIO_Y_DIR_PIN           GPIOB,GPIO_PIN_11
#define GPIO_Y_STEP_PIN          GPIOB,GPIO_PIN_12
#define GPIO_Y_ENABLE_PIN        GPIOB,GPIO_PIN_10
// z stepper
#define GPIO_Z_DIR_PIN           GPIOB,GPIO_PIN_1
#define GPIO_Z_STEP_PIN          GPIOB,GPIO_PIN_2
#define GPIO_Z_ENABLE_PIN        GPIOB,GPIO_PIN_10
#endif

// xyz common enable
#define GPIO_XYZ_ENABLE_PIN      GPIOB,GPIO_PIN_10

// extruder stepper
#define GPIO_E0_DIR_PIN          GPIOA,GPIO_PIN_6
#define GPIO_E0_STEP_PIN         GPIOA,GPIO_PIN_7
#define GPIO_E0_ENABLE_PIN       GPIOB,GPIO_PIN_0

// push button (kill) switch
#define GPIO_KILL_PIN            GPIOA,GPIO_PIN_15

#if ROTATE_TOWER_AXES
// max endstops
#define GPIO_X_MAX_PIN           GPIOC,GPIO_PIN_14
#define GPIO_Y_MAX_PIN           GPIOC,GPIO_PIN_15
#define GPIO_Z_MAX_PIN           GPIOC,GPIO_PIN_13
#else
// max endstops
#define GPIO_X_MAX_PIN           GPIOC,GPIO_PIN_13
#define GPIO_Y_MAX_PIN           GPIOC,GPIO_PIN_14
#define GPIO_Z_MAX_PIN           GPIOC,GPIO_PIN_15
#endif

// nozzle as z probe
#define GPIO_Z_MIN_PIN           GPIOB,GPIO_PIN_7
#define GPIO_Z_MIN_PROBE_PIN     GPIOB,GPIO_PIN_7

// led outputs
#define GPIO_STAT_LED_RED_PIN    GPIOB,GPIO_PIN_15
#define GPIO_STAT_LED_BLUE_PIN   GPIOB,GPIO_PIN_9
#define GPIO_STAT_LED_GREEN_PIN  GPIOB,GPIO_PIN_8

// temperature control
#define GPIO_HEATER_BED_PIN      GPIOA,GPIO_PIN_5
#define GPIO_HEATER_0_PIN        GPIOA,GPIO_PIN_1

// fan speed control
#define GPIO_FAN_PIN             GPIOA,GPIO_PIN_8
#define GPIO_E0_AUTO_FAN_PIN     GPIOA,GPIO_PIN_8

// temperature (analog input)
#define GPIO_TEMP_BED_PIN        GPIOA,GPIO_PIN_4
#define GPIO_TEMP_0_PIN          GPIOA,GPIO_PIN_0

// sd card (spi)
#define GPIO_SS_PIN              GPIOB,GPIO_PIN_6
#define GPIO_SCK_PIN             GPIOB,GPIO_PIN_3
#define GPIO_MISO_PIN            GPIOB,GPIO_PIN_4
#define GPIO_MOSI_PIN            GPIOB,GPIO_PIN_5

// these must match IOdefs[] (see HAL_stm32.c)
#define PIN_INPUT   0
#define PIN_OUTPUT  1
#define PIN_PULLUP  2
#define PIN_ANALOG  3

#define DO_NOTHING

#if CONFIGURE_IO_INDIVIDUALLY
#define SET_OUTPUT(pin)        HAL_gpio_config(GPIO(pin), PIN_OUTPUT)
#define SET_INPUT_PULLUP(pin)  HAL_gpio_config(GPIO(pin), PIN_PULLUP)
#define SET_INPUT(pin)         HAL_gpio_config(GPIO(pin), PIN_INPUT)
#define pinMode(pin, mode)     HAL_gpio_config_indirect(pin, mode)
#define HAL_ANALOG_SELECT(pin) HAL_gpio_config(GPIO(pin), PIN_ANALOG)
#else
#define SET_OUTPUT(pin)        DO_NOTHING
#define SET_INPUT_PULLUP(pin)  DO_NOTHING
#define SET_INPUT(pin)         DO_NOTHING
#define pinMode(pin, mode)     DO_NOTHING
#define HAL_ANALOG_SELECT(pin) DO_NOTHING
#endif
#define WRITE(pin, v)          HAL_gpio_write(GPIO(pin), v)
#define READ(pin)              HAL_gpio_read(GPIO(pin))
#define OUT_WRITE(pin,v)       WRITE(pin, v)

#define HIGH  1
#define LOW   0

#define INPUT                  PIN_INPUT
#define OUTPUT                 PIN_OUTPUT
#define INPUT_PULLUP           PIN_PULLUP

#define analogWrite(pin, v)    if (pin == FANPIN__) HAL_tim1_pwm(v)
#define digitalRead(pin)       HAL_gpio_read_indirect(pin)
#define digitalWrite(pin, v)   HAL_gpio_write_indirect(pin, v)


// which pins are analog input?
#define analogInputToDigitalPin(pin)  (((pin) == 8) || ((pin) == 9))  

// NOTE!?? may be needed (see buzzer.h, servo.cpp)
#undef CRITICAL_SECTION_START
#undef CRITICAL_SECTION_END
#define CRITICAL_SECTION_START  DO_NOTHING
#define CRITICAL_SECTION_END    DO_NOTHING

#define cli()  __disable_irq()
#define sei()  __enable_irq()

#define ISRS_ENABLED()  (!__get_PRIMASK())
#define ENABLE_ISRS()   __enable_irq()
#define DISABLE_ISRS()  __disable_irq()

typedef int8_t byte;
typedef int8_t pin_t;
typedef uint16_t hal_timer_t;

// timers
#define HAL_TIMER_TYPE_MAX          0xffff
#define HAL_TIMER_RATE              ((F_CPU_marlin) / 8)  // 2MHz or 2.5MHz

#define TEMP_TIMER_NUM              0
#define STEP_TIMER_NUM              1
#define PULSE_TIMER_NUM             STEP_TIMER_NUM

#define TEMP_TIMER_FREQUENCY        ((F_CPU_marlin) / 64.0 / 256.0)

#define STEPPER_TIMER_RATE          HAL_TIMER_RATE
#define STEPPER_TIMER_PRESCALE      8
#define STEPPER_TIMER_TICKS_PER_US  (STEPPER_TIMER_RATE /1000000)
// NOTE *_TICK_PER_US cannot be of type double

#define PULSE_TIMER_RATE            STEPPER_TIMER_RATE
#define PULSE_TIMER_PRESCALE        STEPPER_TIMER_PRESCALE
#define PULSE_TIMER_TICKS_PER_US    STEPPER_TIMER_TICKS_PER_US

__STATIC_INLINE uint8_t NVIC_IsEnabledIRQ(IRQn_Type IRQn) {
#define IRQnMASK (uint32_t) (1UL << (((uint32_t) IRQn) & 0x1f))
    return (uint8_t) ((NVIC->ISER[0] & IRQnMASK) != 0);
}

#define DISABLE_IRQ(IRQn)  HAL_NVIC_DisableIRQ(IRQn); __DSB(); __ISB()

#define ENABLE_STEPPER_DRIVER_INTERRUPT()   HAL_NVIC_EnableIRQ(TIM6_IRQn)
#define DISABLE_STEPPER_DRIVER_INTERRUPT()  DISABLE_IRQ(TIM6_IRQn)
#define STEPPER_ISR_ENABLED()               NVIC_IsEnabledIRQ(TIM6_IRQn)

#define ENABLE_TEMPERATURE_INTERRUPT()      HAL_NVIC_EnableIRQ(TIM7_IRQn)
#define DISABLE_TEMPERATURE_INTERRUPT()     DISABLE_IRQ(TIM7_IRQn)
#define TEMPERATURE_ISR_ENABLED()           NVIC_IsEnabledIRQ(TIM7_IRQn)

#define TIM_CCR_0  TIM7->ARR
#define TIM_CNT_0  TIM7->CNT
#define TIM_EGR_0  TIM7->EGR |= TIM_EGR_UG
#define TIM_INI_0  HAL_tim7_init()

#define TIM_CCR_1  TIM6->ARR
#define TIM_CNT_1  TIM6->CNT
#define TIM_EGR_1  TIM6->EGR |= TIM_EGR_UG
#define TIM_INI_1  HAL_tim6_init()

#define _CAT(a, ...) a ## __VA_ARGS__
#define HAL_timer_start(t,f)        _CAT(TIM_INI_, t)
#define HAL_timer_get_count(t)      ((uint16_t) _CAT(TIM_CNT_, t))
#define HAL_timer_get_compare(t)    ((uint16_t) _CAT(TIM_CCR_, t))
#define HAL_timer_set_compare(t,v)  _CAT(TIM_CCR_, t) = (uint16_t) v; \
    if (! ((uint16_t) v < _CAT(TIM_CNT_, t))) _CAT(TIM_EGR_, t)

/**
 * The note below is from the original HAL.h. It does not apply
 * for our Monoprice Delta Mini port, but we'll match the priority
 * scheme for the interrupts.
 *
 * On AVR there is no hardware prioritization and preemption of
 * interrupts, so this emulates it. The UART has first priority
 * (otherwise, characters will be lost due to UART overflow).
 * Then: Stepper, Endstops, Temperature, and -finally- all others.
 */

#define HAL_timer_isr_prologue(TIMER_NUM)  DO_NOTHING
#define HAL_timer_isr_epilogue(TIMER_NUM)  DO_NOTHING

#define HAL_STEP_TIMER_ISR  void HAL_step_timer_isr(void) 
#define HAL_TEMP_TIMER_ISR  void HAL_temp_timer_isr(void)

#ifdef __cplusplus
extern "C" {
#endif
extern HAL_STEP_TIMER_ISR;
extern HAL_TEMP_TIMER_ISR;
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
// Marlin redefines this
#undef UNUSED

#define PROGMEM
#define PGM_P                   const char *
#define pgm_read_ptr(p)         ((const void *) (p))
#define pgm_read_byte(p)        *((const uint8_t *) (p))
#define pgm_read_word(p)        *((const uint16_t *) (p))
#define pgm_read_dword(p)       *((const uint32_t *) (p))
#define pgm_read_float(p)       *((const float *) (p))
#define pgm_read_byte_near(p)   *((const uint8_t *) ((uint32_t) (p)))
#define pgm_read_word_near(p)   *((const uint16_t *) ((uint32_t) (p)))
#define pgm_read_dword_near(p)  *((const uint32_t *) ((uint32_t) (p)))
#define pgm_read_float_near(p)  *((const float *) ((uint32_t) (p)))
#define PSTR(s)                 ((const char *) (s))
#define PGM_PATCH_uint16_t      uint32_t
#define strstr_P                strstr
#define strcpy_P                strcpy
#define strncpy_P               strncpy
#define strlen_P                strlen
#define printf_P                printf
#define sprintf_P               sprintf

#define millis()                HAL_GetTick()
#define _delay_ms(x)            HAL_Delay(x)
#define delay(x)                HAL_Delay(x)
#define abs(x)                  ((x)>0?(x):-(x))
#define constrain(amt,lo,hi)    ((amt)<(lo)?(lo):((amt)>(hi)?(hi):(amt)))
#define round(x)                ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg)            ((deg)*DEG_TO_RAD)
#define degrees(rad)            ((rad)*RAD_TO_DEG)
#define sq(x)                   ((x)*(x))
#define random(a, b)            ((((b) - (a)) * rand()) /RAND_MAX)
#define randomSeed(s)           srand(s)

#define eeprom_read_block(d, s, n)  memcpy(d, s, n)

#include <ctype.h>    // tolower()
#include <strings.h>  // strcasecmp()

// fake a String Class
class String {
public:
    String() { };
    int length (void) const {
	return 0;
    };
    char operator[] (int) const {
	return '\0';
    };
};

// fake a Print Class
class Print {
public:
    Print() {};
    // needed by malyanlcd.cpp
    static void write(const char * s, uint8_t n);
};

// needed by malyanlcd.cpp
class CustomSerial : public Print {
public:
    CustomSerial() { };
    static void begin(const long baud);
    static void end(void);
    static int peek(void);
    static int read(void);
    static uint16_t available(void);
    static void flush(void);
    static void flushTX(void);
    static void write(const uint8_t c);
    static void write(const char *str);
    static void write(const uint8_t *buf, size_t size);
    static void write(const char *buf, size_t size) {
	write((uint8_t *) buf, size);
    }
};
extern CustomSerial Serial1;

// marlin's speed_lookuptable.h expects 16MHz or 20MHz,
// so we'll include it here, adjusting F_CPU accordingly
#undef  F_CPU
#define F_CPU  F_CPU_marlin
#include "speed_lookuptable.h"
#undef  F_CPU
#define F_CPU  F_CPU_actual

#if OVERLY_SIMPLISTIC_OUTPUT_LOGGING_HACK
bool MarlinSerial_log(const char * file_name);
#endif

#endif  // __cplusplus
#endif  // _HAL_AVR_H_
