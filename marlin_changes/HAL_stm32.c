/** 
 * Malyan M300 (Monoprice Mini Delta) BSP for Marlin 
 * Copyright (C) 2019 Aegean Associates, Inc.
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
 */

#include "HAL_stm32.h"

// prevent drivers.h from pulling in MarlinConfig.h
#define MARLIN_CONFIG_H
#undef UNUSED
#include "macros.h"
#include "drivers.h"
#include "Configuration.h"
#include "Configuration_adv.h"

#ifndef CORE_CPU_FREQ
#define CORE_CPU_FREQ  48000000ul
#endif

#define PRIORITY_EXTI4_15  1
#define PRIORITY_TIM6      3
#define PRIORITY_TIM7      3
#define PRIORITY_TICK      0
#define PRIORITY_USART1    2
#define PRIORITY_USART2    2
#define PRIORITY_USB       1

void debug_led(uint8_t l, uint8_t s)
{   // matches pin description file, pins_MALYAN_M300.h
    // red:27, blue:28, green:29, off:1, on:0
    HAL_gpio_write_indirect(l, s);
}

// matches pin description file, pins_MALYAN_M300.h
#define RED_CLR  0x80000000UL
#define RED_SET  0x00008000UL
#define BLU_CLR  0x02000000UL
#define BLU_SET  0x00000200UL
#define GRN_CLR  0x01000000UL
#define GRN_SET  0x00000100UL

void debug_black(void)
{   // matches pin description file, pins_MALYAN_M300.h
    GPIOB->BSRR = RED_SET | BLU_SET | GRN_SET;
}

void debug_red(void)
{   // matches pin description file, pins_MALYAN_M300.h
    GPIOB->BSRR = RED_CLR | BLU_SET | GRN_SET;
}

void debug_green(void)
{   // matches pin description file, pins_MALYAN_M300.h
    GPIOB->BSRR = RED_SET | BLU_SET | GRN_CLR;
}

#if 1
void debug_blue(void)
{   // matches pin description file, pins_MALYAN_M300.h
    GPIOB->BSRR = BLU_CLR;  // only affect blue led
}
#else
void debug_blue(void)
{   // matches pin description file, pins_MALYAN_M300.h
    GPIOB->BSRR = RED_SET | BLU_CLR | GRN_SET;
}
#endif

void debug_yellow(void)
{   // matches pin description file, pins_MALYAN_M300.h
    GPIOB->BSRR = RED_CLR | BLU_SET | GRN_CLR;
}

void debug_cyan(void)
{   // matches pin description file, pins_MALYAN_M300.h
    GPIOB->BSRR = RED_SET | BLU_CLR | GRN_CLR;
}

void debug_magenta(void)
{   // matches pin description file, pins_MALYAN_M300.h
    GPIOB->BSRR = RED_CLR | BLU_CLR | GRN_SET;
}

void debug_white(void)
{   // matches pin description file, pins_MALYAN_M300.h
    GPIOB->BSRR = RED_CLR | BLU_CLR | GRN_CLR;
}

static uint8_t debug_pushbutton(void)
{   // matches pin description file, pins_MALYAN_M300.h
    return ((GPIOA->IDR & 0x8000) != 0);
}

#define WAIT_OFF   100
#define WAIT_RED   150
#define HOLD_TIME  2000
#define HOLD_TEST  (HOLD_TIME / (WAIT_OFF + WAIT_RED))

#if !HOLD_TEST
#error "HOLD_TEST cannot be zero"
#endif

void debug_wait_on_pushbutton(void)
{
    for (uint8_t u = 0; u < HOLD_TEST; u++) {
#if ENABLED(USE_WATCHDOG)
	watchdog_reset();
#endif
	HAL_Delay(WAIT_RED);
	debug_black();
	HAL_Delay(WAIT_OFF);
	debug_red();
	if (! debug_pushbutton())
	    u = 0;
    }
}

void HAL_setup(void)
{
    HAL_InitTick(PRIORITY_TICK);
    HAL_gpio_init();
    HAL_tim1_init();
    debug_green();  // leds to green
    HAL_flashstore_init();
}

inline void HAL_reboot(void)
{
    // Hmm, NVIC_SystemReset() doesn't seem to work, so
    // we'll just wait for the watchdog to reset things.
    //__NVIC_SystemReset();
    while(1);
}

/**
 * GPIO
 */

typedef struct {
    GPIO_TypeDef * Port;
    GPIO_InitTypeDef InitStruct;
} GPIO_InitPinType;

#define IO_af  GPIO_MODE_AF_PP,     GPIO_PULLDOWN, GPIO_SPEED_FREQ_HIGH
#define IO_AF  GPIO_MODE_AF_PP,     GPIO_PULLUP,   GPIO_SPEED_FREQ_HIGH
#define IO_OM  GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   GPIO_SPEED_FREQ_MEDIUM
#define IO_OL  GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   GPIO_SPEED_FREQ_LOW
#define IO_IN  GPIO_MODE_INPUT,     GPIO_NOPULL,   0
#define IO_IU  GPIO_MODE_INPUT,     GPIO_PULLUP,   0
#define IO_ID  GPIO_MODE_INPUT,     GPIO_PULLDOWN, 0
#define IO_IT  GPIO_MODE_IT_RISING_FALLING, GPIO_PULLUP, 0
#define IO_AI  GPIO_MODE_ANALOG,    GPIO_NOPULL,   0

#define P(p)   GPIO_PIN_ ## p
#define XYZ_STEPPER_PINS  P(1)|P(2)|P(11)|P(12)|P(13)|P(14)

const GPIO_InitPinType IOdefs[] = {
#if CONFIGURE_IO_INDIVIDUALLY
#define IODEF_OFFSET  4
  { NULL,  { 0,                 IO_ID, 0 }},  // digital input (pull-down)
  { NULL,  { 0,                 IO_OM, 0 }},  // digital output
  { NULL,  { 0,                 IO_IU, 0 }},  // digital input with pull-up
  { NULL,  { 0,                 IO_AI, 0 }},  // analog input
#else
#define IODEF_OFFSET  0
#endif
#if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
#define EXTI_PINS  P(13)|P(14)|P(15)|P(7)
  { GPIOC, { P(13)|P(14)|P(15), IO_IT, 0 }},  // x, y, z max endstops
  { GPIOB, { P(7),              IO_IT, 0 }},  // z probe 
#else
  { GPIOC, { P(13)|P(14)|P(15), IO_IU, 0 }},  // x, y, z max endstops
  { GPIOB, { P(7),              IO_IU, 0 }},  // z probe 
#endif
  { GPIOA, { P(0)|P(4),         IO_AI, 0 }},  // ADC
  { GPIOA, { P(11)|P(12),       IO_AF, GPIO_AF2_USB    }},  // USB (CDC)
  { GPIOA, { P(9)|P(10),        IO_AF, GPIO_AF1_USART1 }},  // USART1 (LCD)
  { GPIOA, { P(2)|P(3),         IO_AF, GPIO_AF1_USART2 }},  // USART2 (DBG)
  { GPIOB, { P(3)|P(4)|P(5),    IO_af, GPIO_AF0_SPI1   }},  // SPI1 (sck,mi,mo)
  { GPIOB, { P(6),              IO_OM, 0 }},  // SPI1 (ss) (SD_CS)
  { GPIOB, { P(8)|P(9)|P(15),   IO_OL, 0 }},  // status led outputs
#if ! CONFIGURE_IO_INDIVIDUALLY
  { GPIOB, { XYZ_STEPPER_PINS,  IO_OM, 0 }},  // x, y, z steppers
  { GPIOA, { P(6)|P(7),         IO_OM, 0 }},  // e stepper
  { GPIOB, { P(10)|P(0),        IO_OM, 0 }},  // common xyz, e stepper enables
  { GPIOA, { P(15),             IO_ID, 0 }},  // push button (kill) switch
#if FAN_USES_HARDWARE_PWM
  { GPIOA, { P(1)|P(5),         IO_OL, 0 }},  // heater controls
#else
  { GPIOA, { P(1)|P(5)|P(8),    IO_OL, 0 }},  // heater and fan controls
#endif
#endif
#if FAN_USES_HARDWARE_PWM
  { GPIOA, { P(8),              IO_AF, GPIO_AF2_TIM1 }},  // fan control (pwm)
#endif
};

#define IODEF_COUNT  (sizeof(IOdefs)/sizeof(GPIO_InitPinType))
#define NEXT_IODEF   IOdefs[IODEF_COUNT]
#define STOPS_IODEF  IOdefs[IODEF_OFFSET + 0]
#define PROBE_IODEF  IOdefs[IODEF_OFFSET + 1]
#define ADC_IODEF    IOdefs[IODEF_OFFSET + 2]
#define USB_IODEF    IOdefs[IODEF_OFFSET + 3]
#define UART1_IODEF  IOdefs[IODEF_OFFSET + 4]
#define UART2_IODEF  IOdefs[IODEF_OFFSET + 5]
#define SPI1_IODEF   IOdefs[IODEF_OFFSET + 6]
#define SD_CS_IODEF  IOdefs[IODEF_OFFSET + 7]
#define LEDS_IODEF   IOdefs[IODEF_OFFSET + 8]
#define FAN_IODEF    IOdefs[IODEF_COUNT - 1]

#define IOCAST(x)  (GPIO_InitTypeDef *) x

#if CONFIGURE_IO_INDIVIDUALLY
void HAL_gpio_config(GPIO_TypeDef *gpio, uint16_t pin, uint16_t mode)
{
    if ((gpio != NULL) && (mode < IODEF_COUNT)) {
	GPIO_InitTypeDef iodef = IOdefs[mode].InitStruct;
	iodef.Pin = pin;
	HAL_GPIO_Init(gpio, &iodef);
    }
}
#endif

inline static void __HAL_gpio_init(GPIO_TypeDef * gpio,
				   const GPIO_InitTypeDef * initstruct)
{
    if (initstruct != NULL) {
	GPIO_InitTypeDef iodef = *initstruct;
	HAL_GPIO_Init(gpio, &iodef);
    }
}

uint8_t HAL_gpio_read(GPIO_TypeDef * gpio, uint16_t pin)
{
    return (uint8_t) HAL_GPIO_ReadPin(gpio, pin);
}

void HAL_gpio_write(GPIO_TypeDef * gpio, uint16_t pin, uint16_t state)
{
    HAL_GPIO_WritePin(gpio, pin, state);
}

int HAL_gpio_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

#if CONFIGURE_IO_INDIVIDUALLY
    // seems Marlin doesn't initialize the status LEDs
    HAL_GPIO_Init(LEDS_IODEF.Port, IOCAST(&(LEDS_IODEF.InitStruct)));
#else
    for (GPIO_InitPinType * p = (GPIO_InitPinType *) IOdefs;
	 p < &NEXT_IODEF; p++) {
	if (p->Port != NULL) {
	    HAL_GPIO_Init(p->Port, &(p->InitStruct));
	    if (p->InitStruct.Mode == GPIO_MODE_OUTPUT_PP)
		(p->Port)->BRR = p->InitStruct.Pin;
	}
    }
#if 0
    // adjust the initial state of some "asserted low" outputs
    HAL_gpio_write(GPIO_E0_ENABLE_PIN, HIGH);
    HAL_gpio_write(GPIO_XYZ_ENABLE_PIN, HIGH);
#else  // faster, smaller hard-coded version
    // matches pin description file, pins_MALYAN_M300.h
    GPIOB->BSRR = 0x0401;  // stepper motor XYZ+E disabled
#endif
#endif
    return 0;
}

// slower, indirectly referenced pin functions

#define xPIN0   8
#define xGPIOA  0x10000
#define xGPIOB  0x20000
#define xGPIOC  0x30000
#define xMASK   0x30000

static const uint32_t gpio_indirect[] = {
// must match pin description file, pins_MALYAN_M300.h
   xGPIOA | GPIO_PIN_0,   // GPIO_8,  TEMP_0_PIN (analog)
   xGPIOA | GPIO_PIN_4,   // GPIO_9,  TEMP_BED_PIN (analog)
   xGPIOB | GPIO_PIN_10,  // GPIO_10, X,Y,Z_ENABLE_PIN
#if ROTATE_TOWER_AXES
   xGPIOB | GPIO_PIN_1,   // GPIO_11, Y_DIR_PIN
   xGPIOB | GPIO_PIN_2,   // GPIO_12, Y_STEP_PIN
   xGPIOB | GPIO_PIN_11,  // GPIO_13, X_DIR_PIN
   xGPIOB | GPIO_PIN_12,  // GPIO_14, X_STEP_PIN
   xGPIOB | GPIO_PIN_13,  // GPIO_15, Z_DIR_PIN
   xGPIOB | GPIO_PIN_14,  // GPIO_16, Z_STEP_PIN
#else
   xGPIOB | GPIO_PIN_11,  // GPIO_11, Y_DIR_PIN
   xGPIOB | GPIO_PIN_12,  // GPIO_12, Y_STEP_PIN
   xGPIOB | GPIO_PIN_13,  // GPIO_13, X_DIR_PIN
   xGPIOB | GPIO_PIN_14,  // GPIO_14, X_STEP_PIN
   xGPIOB | GPIO_PIN_1,   // GPIO_15, Z_DIR_PIN
   xGPIOB | GPIO_PIN_2,   // GPIO_16, Z_STEP_PIN
#endif
   xGPIOA | GPIO_PIN_6,   // GPIO_17, E0_DIR_PIN
   xGPIOA | GPIO_PIN_7,   // GPIO_18, E0_STEP_PIN
   xGPIOB | GPIO_PIN_0,   // GPIO_19, E0_ENABLE_PIN
   xGPIOA | GPIO_PIN_8,   // GPIO_20, FAN_PIN
   xGPIOA | GPIO_PIN_1,   // GPIO_21, HEATER_0_PIN
   xGPIOA | GPIO_PIN_5,   // GPIO_22, HEATER_BED_PIN
#if ROTATE_TOWER_AXES
   xGPIOC | GPIO_PIN_14,  // GPIO_23, X_MAX_PIN
   xGPIOC | GPIO_PIN_15,  // GPIO_24, Y_MAX_PIN
   xGPIOC | GPIO_PIN_13,  // GPIO_25, Z_MAX_PIN
#else
   xGPIOC | GPIO_PIN_13,  // GPIO_23, X_MAX_PIN
   xGPIOC | GPIO_PIN_14,  // GPIO_24, Y_MAX_PIN
   xGPIOC | GPIO_PIN_15,  // GPIO_25, Z_MAX_PIN
#endif
   xGPIOB | GPIO_PIN_7,   // GPIO_26, Z_MIN_PROBE_PIN, Z_MIN
   xGPIOB | GPIO_PIN_15,  // GPIO_27, STAT_LED_RED_PIN, LED_PIN
   xGPIOB | GPIO_PIN_9,   // GPIO_28, STAT_LED_BLUE_PIN
   xGPIOB | GPIO_PIN_8,   // GPIO_29, STAT_LED_GREEN_PIN
   xGPIOB | GPIO_PIN_6,   // GPIO_30, SS_PIN, SDSS
   xGPIOB | GPIO_PIN_3,   // GPIO_31, SCK_PIN
   xGPIOB | GPIO_PIN_4,   // GPIO_32, MISO_PIN
   xGPIOB | GPIO_PIN_5,   // GPIO_33, MOSI_PIN
   xGPIOA | GPIO_PIN_15,  // GPIO_34, KILL_PIN
};

#define xPINS  (sizeof(gpio_indirect)/sizeof(uint32_t))

static void gpio_map(uint8_t pn, GPIO_TypeDef ** gpio, uint16_t * pin)
{
    uint32_t xx = ((pn - xPIN0) < xPINS) ? gpio_indirect[pn - xPIN0] : 0;

    switch(xx & xMASK) {
    case xGPIOA: *gpio = GPIOA; break;
    case xGPIOB: *gpio = GPIOB; break;
    case xGPIOC: *gpio = GPIOC; break;
    default:     *gpio = NULL;
    }
    *pin = (uint16_t) (xx & 0xffff);
}

#if CONFIGURE_IO_INDIVIDUALLY
void HAL_gpio_config_indirect(uint8_t pn, uint16_t mode)
{
    GPIO_TypeDef * gpio;
    uint16_t pin;
    gpio_map(pn, &gpio, &pin);
    HAL_gpio_config(gpio, pin, mode);
}
#endif

void HAL_gpio_write_indirect(uint8_t pn, uint16_t state)
{
    GPIO_TypeDef * gpio;
    uint16_t pin;
    gpio_map(pn, &gpio, &pin);
    HAL_gpio_write(gpio, pin, state);
}

uint8_t HAL_gpio_read_indirect(uint8_t pn)
{
    GPIO_TypeDef * gpio;
    uint16_t pin;
    gpio_map(pn, &gpio, &pin);
    return HAL_gpio_read(gpio, pin);
}


#if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
// replace functions defined in the unused, "endstops_interrupts.h"

void setup_endstop_interrupts(void)
{
#if CONFIGURE_IO_INDIVIDUALLY
    HAL_GPIO_Init(STOPS_IODEF.Port, IOCAST(&(STOPS_IODEF.InitStruct)));
    HAL_GPIO_Init(PROBE_IODEF.Port, IOCAST(&(PROBE_IODEF.InitStruct)));
#endif

    HAL_NVIC_SetPriority(EXTI4_15_IRQn, PRIORITY_EXTI4_15, 0);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}
#endif


/**
 * TMR1 (fan control)
 */

// PWM frequency (48MHz /8     /256) = 23.4375kHz
// PWM frequency (48MHz /9     /256) = 20.8333kHz
// PWM frequency (48MHz /5682  /256) = 32.9989Hz

#define TIM1_PRESCALE  5682
#define TIM1_PWM_MAX   255

int HAL_tim1_init(void)
{
#if CONFIGURE_IO_INDIVIDUALLY
    HAL_GPIO_Init(FAN_IODEF.Port, IOCAST(&(FAN_IODEF.InitStruct)));
#endif

#if 0 // not necessary
    __HAL_RCC_TIM1_FORCE_RESET();
    __HAL_RCC_TIM1_RELEASE_RESET();
#endif
    __HAL_RCC_TIM1_CLK_ENABLE();

    TIM1->ARR = TIM1_PWM_MAX;
    TIM1->PSC = (TIM1_PRESCALE -1);
    TIM1->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE);
    TIM1->CCER |= TIM_CCER_CC1E;
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CCR1 = 0;  // pwm off
    TIM1->CR1 |= (TIM_CR1_ARPE | TIM_CR1_CMS_0 | TIM_CR1_CEN);
    TIM1->EGR |= TIM_EGR_UG;
    return 0;
}

void HAL_tim1_pwm(uint8_t v)
{
    TIM1->CCR1 = v;
}

/**
 * TIM6 (stepper interrupt)
 */

// from the original HAL.h ...
//
// "Generally we use a divider of 8, resulting in a 2MHz timer frequency
//  on a 16MHz MCU. If you are going to change this, be sure to regenerate
//  speed_lookuptable.h with create_speed_lookuptable.py"

#if F_CPU_marlin == 20000000
#define TIM6_PRESCALE  19  // emulate 20MHz AVR (not exact)
#else
#define TIM6_PRESCALE  24  // emulate 16MHz AVR by default
#endif
#define TIM6_COMPARE   16384

int HAL_tim6_init(void)
{
#if 0 // not necessary
    __HAL_RCC_TIM6_FORCE_RESET();
    __HAL_RCC_TIM6_RELEASE_RESET();
#endif
    __HAL_RCC_TIM6_CLK_ENABLE();

    TIM6->PSC = TIM6_PRESCALE -1;
    TIM6->ARR = TIM6_COMPARE -1;
    TIM6->DIER |= TIM_DIER_UIE;
    TIM6->SR = 0;
    TIM6->CR1 |= TIM_CR1_CEN;

    HAL_NVIC_SetPriority(TIM6_IRQn, PRIORITY_TIM6, 0);
    HAL_NVIC_EnableIRQ(TIM6_IRQn);
    return 0;
}

/**
 * TIM7 (temperature interrupt)
 */

// mimic the AVR's 8-bit counter T0
// avr:     16MHz /64  /256 = 976.5625Hz (/128) = 7.629Hz
// stm32f0: 48MHz /192 /256 = 976.5625Hz (/128) = 7.629Hz

#define TIM7_PRESCALE  192
#define TIM7_COMPARE   256

int HAL_tim7_init(void)
{
#if 0 // not necessary
    __HAL_RCC_TIM7_FORCE_RESET();
    __HAL_RCC_TIM7_RELEASE_RESET();
#endif
    __HAL_RCC_TIM7_CLK_ENABLE();

    TIM7->PSC = TIM7_PRESCALE -1;
    TIM7->ARR = TIM7_COMPARE -1;
    TIM7->DIER = TIM_DIER_UIE;
    TIM7->SR = 0;
    TIM7->CR1 |= TIM_CR1_CEN;

    HAL_NVIC_SetPriority(TIM7_IRQn, PRIORITY_TIM7, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
    return 0;
}

/**
 * FLASHSTORE
 */

#define FLASHSTORE_MAXSIZE  0x0800
#ifndef FLASHSTORE_ADDRESS
#define FLASHSTORE_ADDRESS  0x0801f800
#endif

int HAL_flashstore_init(void)
{
    // note, internal RC clk HSI must be enabled
    __HAL_RCC_HSI_ENABLE();
    __HAL_RCC_FLITF_CLK_ENABLE();
    FLASH->ACR |= (FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY);
}

int HAL_flashstore_write(const uint8_t * source, uint32_t length, int erase)
{
    uint16_t * p = (uint16_t *) FLASHSTORE_ADDRESS;
    uint16_t * s = (uint16_t *) source;
    int e = 1;  // preset for error

    if (length > FLASHSTORE_MAXSIZE)
	return e;

    // program halfwords, not bytes
    length = (length+1)/2;
    do {
	// unlock
	while (FLASH->SR & FLASH_SR_BSY);
	if (FLASH->CR & FLASH_CR_LOCK) {
	    FLASH->KEYR = FLASH_KEY1;
	    FLASH->KEYR = FLASH_KEY2;
	}
	if (erase) {
	    FLASH->CR &= ~FLASH_CR_MER;
	    FLASH->CR |= FLASH_CR_PER;
	    FLASH->AR = (uint32_t) p;
	    FLASH->CR |= FLASH_CR_STRT;
	    while (FLASH->SR & FLASH_SR_BSY); 
	    if (! (FLASH->SR & FLASH_SR_EOP))
		goto _abort;
	    FLASH->SR = FLASH_SR_EOP;
	    FLASH->CR &= ~FLASH_CR_PER;
	}
	// write
	FLASH->CR |= FLASH_CR_PG;
	while (length-- > 0) {
	    *p++ = *s++;
	    while (FLASH->SR & FLASH_SR_BSY);
	    if (! (FLASH->SR & FLASH_SR_EOP))
		goto _abort;
	    FLASH->SR = FLASH_SR_EOP;
	}
	FLASH->CR &= ~FLASH_CR_PG;
	e = 0;
    } while (0);
 _abort:
    FLASH->CR &= ~(FLASH_CR_PG | FLASH_CR_PER | FLASH_CR_MER);
    FLASH->CR |= FLASH_CR_LOCK;
    return e;
}


/**
 * ADC
 */

uint16_t HAL_adc_read(void)
{
    return (uint16_t) (ADC1->DR);
}

int HAL_adc_ready(void)
{
    // test for end of sequence
    return ADC1->ISR & ADC_ISR_EOS;
}

#define ADC_ISR_FLAGS  (ADC_ISR_EOS | ADC_ISR_EOC | ADC_ISR_OVR)

void HAL_adc_start(uint16_t ch)
{
#if 1 // ??? shouldn't be necessary, but just in case...
    if ((ADC1->CR & (ADC_CR_ADEN|ADC_CR_ADDIS|ADC_CR_ADSTART)) != ADC_CR_ADEN)
	HAL_adc_init();
#endif
    ADC1->CHSELR = ch;
    ADC1->ISR |= ADC_ISR_FLAGS;  // write 1 to clear 
    ADC1->CR |= ADC_CR_ADSTART;
}

#define ADC_SMPR_239_5  7

int HAL_adc_init(void)
{
#if CONFIGURE_IO_INDIVIDUALLY
    HAL_GPIO_Init(ADC_IODEF.Port, IOCAST(&(ADC_IODEF.InitStruct)));
#endif

    // ensure initial state
    __HAL_RCC_ADC1_FORCE_RESET();
    __HAL_RCC_ADC1_RELEASE_RESET();

    __HAL_RCC_ADC1_CLK_ENABLE();

    // configure the adc clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->CR2 |= RCC_CR2_HSI14ON;
    while (! (RCC->CR2 & RCC_CR2_HSI14RDY));

    // calibrate the adc
    if (ADC1->CR & ADC_CR_ADEN) ADC1->CR |= ADC_CR_ADDIS;
    while (ADC1->CR & ADC_CR_ADEN);
    ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;
    ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE;
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL);

#if 0
    // enable the adc
    ADC1->ISR |= ADC_ISR_ADRDY;  // write 1 to clear 
    ADC1->CR |= ADC_CR_ADEN;
    while (! (ADC1->ISR & ADC_ISR_ADRDY));
#else
    // enable the adc (per device errata)
    ADC1->ISR |= ADC_ISR_ADRDY;  // write 1 to clear 
    ADC1->CR |= ADC_CR_ADEN;
    while (! (ADC1->ISR & ADC_ISR_ADRDY))
	ADC1->CR |= ADC_CR_ADEN;
#endif
    // configure the adc, use 10b resolution to mimic avr
    ADC1->CFGR1 |= (ADC_CFGR1_OVRMOD | ADC_RESOLUTION_10B);
    ADC1->SMPR |= ADC_SMPR_239_5;
    return 0;
}


#if ENABLED(USE_WATCHDOG)
/**
 * IWDG
 */

#if ENABLED(WATCHDOG_RESET_MANUAL)
extern void faux_watchdog_interrupt(void);
#endif

// 40kHz clk, 128 prescale, scale = t * 40000 / 128
#define IWDB_TMO(t)  (((t) * 625) / 2) 

#ifndef IWDG_TIMEOUT
#define IWDG_TIMEOUT  1500  // ~4.8s
#endif

const IWDG_HandleTypeDef iwdg =
    { IWDG, { IWDG_PRESCALER_128, IWDG_TIMEOUT, IWDG_WINDOW_DISABLE }};

int HAL_iwdg_init(void)
{
    HAL_IWDG_Init((IWDG_HandleTypeDef *) &iwdg);
    return 0;
}

void HAL_iwdg_refresh(void)
{
    HAL_IWDG_Refresh((IWDG_HandleTypeDef *) &iwdg);
}
#endif


/**
 * SPI1
 */

int HAL_spi_init(uint8_t rate)
{
#if CONFIGURE_IO_INDIVIDUALLY
    HAL_GPIO_Init(SPI1_IODEF.Port, IOCAST(&(SPI1_IODEF.InitStruct)));
    HAL_GPIO_Init(SD_CS_IODEF.Port, IOCAST(&(SD_CS_IODEF.InitStruct)));
#endif

    // ensure intial state (function called more than once)
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();

    __HAL_RCC_SPI1_CLK_ENABLE();

    // SSM and SSI to avoid MODF (mode fault) errors
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR2 = 0x0700 | SPI_CR2_FRXTH;
    HAL_spi_config(rate);
    SPI1->CR1 |= SPI_CR1_SPE;
    return 0;
}

void HAL_spi_config(uint8_t rate)
{
    // HACK! adjust the rate for spi mode selection on the sd card
    if (rate++ > 4) rate = 7;

    volatile uint8_t b;
    while (SPI1->SR & (SPI_SR_BSY | SPI_SR_FTLVL | SPI_SR_FRLVL))
	b = (uint8_t) SPI1->DR;

    SPI1->CR1 &= ~((       7) << 3);
    SPI1->CR1 |=  ((rate & 7) << 3);
}

static uint8_t HAL_spi_txrx(uint8_t b)
{
    while (! (SPI1->SR & SPI_SR_TXE));
    *((uint8_t *) &(SPI1->DR)) = b;    
    while (! (SPI1->SR & SPI_SR_RXNE));
    return (uint8_t) SPI1->DR;
}

uint8_t HAL_spi_recv(void)
{
    return HAL_spi_txrx(0xff);
}

void HAL_spi_send(uint8_t b)
{
    HAL_spi_txrx(b);
}

void HAL_spi_read(uint8_t * buf, uint16_t len)
{
    while (len--)
	*buf++ = HAL_spi_recv();
}

void HAL_spi_send_block(uint8_t token, const uint8_t * buf)
{
    uint16_t i = 512;
    uint8_t * p = (uint8_t *) buf;

    *((uint8_t *) &(SPI1->DR)) = token;    
    while (i--) {
	while (! (SPI1->SR & SPI_SR_TXE));
	*((uint8_t *) &(SPI1->DR)) = *p++;
    }
    // wait on transmitter, clear overflow error
    volatile uint8_t b;
    while (SPI1->SR & (SPI_SR_BSY | SPI_SR_FTLVL | SPI_SR_RXNE))
	b = (uint8_t) SPI1->DR;
}


/**
 * USART1
 * USART2
 * USB
 */

extern void ptimer_isr(void);
extern void qtimer_isr(void);
extern void usart1_isr(void);
extern void usart2_isr(void);

#define USART_ICR_ALL  0x20a5f

inline uint8_t HAL_usart_check(USART_TypeDef * usart, uint32_t flags)
{
    return ((usart->ISR & flags) != 0);
}

inline void HAL_usart_clear(USART_TypeDef * usart, uint32_t flags)
{
    usart->ICR |= flags;
}

inline uint8_t HAL_usart_read(USART_TypeDef * usart)
{
    return usart->RDR;
}

inline void HAL_usart_send(USART_TypeDef * usart, uint8_t c)
{
    usart->TDR = c;
}

inline void HAL_usart_txe_1(USART_TypeDef * usart)
{   // enable transmitter interrupt
    usart->CR1 |= USART_CR1_TXEIE;
}

inline void HAL_usart_txe_0(USART_TypeDef * usart)
{   // disable transmitter interrupt
    usart->CR1 &= ~USART_CR1_TXEIE;
}

static int HAL_usart1_init(uint32_t fq)
{
#if CONFIGURE_IO_INDIVIDUALLY
    HAL_GPIO_Init(UART1_IODEF.Port, IOCAST(&(UART1_IODEF.InitStruct)));
#endif
    __HAL_RCC_USART1_CLK_ENABLE();

    USART1->BRR = CORE_CPU_FREQ / fq;
    USART1->ICR |= USART_ICR_ALL;
    USART1->CR1 |= (USART_CR1_UE|USART_CR1_RE|USART_CR1_RXNEIE|USART_CR1_TE);

    HAL_NVIC_SetPriority(USART1_IRQn, PRIORITY_USART1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    return 0;
}

static int HAL_usart2_init(uint32_t fq)
{
#if CONFIGURE_IO_INDIVIDUALLY
    HAL_GPIO_Init(UART2_IODEF.Port, IOCAST(&(UART2_IODEF.InitStruct)));
#endif
    __HAL_RCC_USART2_CLK_ENABLE();

    USART2->BRR = CORE_CPU_FREQ / fq;
    USART2->ICR |= USART_ICR_ALL;
    USART2->CR1 |= (USART_CR1_UE|USART_CR1_RE|USART_CR1_RXNEIE|USART_CR1_TE);

    HAL_NVIC_SetPriority(USART2_IRQn, PRIORITY_USART2, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    return 0;
}

int HAL_usart_init(USART_TypeDef * usart, uint32_t rate)
{
    if (usart == USART1) return HAL_usart1_init(rate);
    if (usart == USART2) return HAL_usart2_init(rate);
    return 1;
}

/**
 * PCD (low level USB support for USB CDC)
 * see usbd_conf.h, usbd_conf.d
 *
 * FIXME!? should use PCD or USBD, not both?
 */

void HAL_PCD_MspInit(PCD_HandleTypeDef * pcd)
{
#if CONFIGURE_IO_INDIVIDUALLY
    HAL_GPIO_Init(USB_IODEF.Port, IOCAST(&(USB_IODEF.InitStruct)));
#endif

    __HAL_RCC_USB_CLK_ENABLE();

    HAL_NVIC_SetPriority(USB_IRQn, PRIORITY_USB, 0);
    HAL_NVIC_EnableIRQ(USB_IRQn);
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef * pcd)
{
    __HAL_RCC_USB_CLK_DISABLE();
}

/**
 * Interrupt Handlers
 */

void SysTick_Handler(void)
{
#if ENABLED(USE_WATCHDOG)
#if ENABLED(WATCHDOG_RESET_MANUAL)
    faux_watchdog_interrupt();
#endif
#endif
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
    ptimer_isr();
}

#if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
#define BLUE  (RED_SET | BLU_CLR | GRN_SET)
#define WHITE (RED_CLR | BLU_CLR | GRN_CLR)
#define PROBE (P(7))
void EXTI4_15_IRQHandler(void)
{
    // matches pin description file, pins_MALYAN_M300.h
    GPIOB->BSRR = (GPIOB->IDR & PROBE) ? WHITE : BLUE ;
    endstop_isr();
    EXTI->PR |= EXTI_PINS;
}
#endif

void TIM6_IRQHandler(void)
{
    HAL_step_timer_isr();
    TIM6->SR = 0;
}

void TIM7_IRQHandler(void)
{
    HAL_temp_timer_isr();
    qtimer_isr();
    TIM7->SR = 0;
}

void USART1_IRQHandler(void)
{
    usart1_isr();
}

void USART2_IRQHandler(void)
{
    usart2_isr();
}

#if 0
/* defined in usbd_conf.c */
void USB_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&_pcd);
}
#endif

/* other (unused) exceptions/interrupts are non-recoverable */
void HardFault_Handler(void)
{
#define INTERRUPT_INUSE(x)
#define NON_RECOVERABLE(x)  ".global " x "\n" x ":\n"
    asm(
	NON_RECOVERABLE("NMI_Handler")
	INTERRUPT_INUSE("HardFault_Handler")
	NON_RECOVERABLE("SVC_Handler")
	NON_RECOVERABLE("PendSV_Handler")
	INTERRUPT_INUSE("SysTick_Handler")
	NON_RECOVERABLE("WWDG_IRQHandler")
	NON_RECOVERABLE("PVD_VDDIO2_IRQHandler")  /* stm32f072xb */
	NON_RECOVERABLE("RTC_IRQHandler")
	NON_RECOVERABLE("FLASH_IRQHandler")
	NON_RECOVERABLE("RCC_IRQHandler")
	NON_RECOVERABLE("EXTI0_1_IRQHandler")
	NON_RECOVERABLE("EXTI2_3_IRQHandler")
	INTERRUPT_INUSE("EXTI4_15_IRQHandler")
	NON_RECOVERABLE("TSC_IRQHandler" )        /* stm32f072xb */
	NON_RECOVERABLE("DMA1_Channel1_IRQHandler")
	NON_RECOVERABLE("DMA1_Channel2_3_IRQHandler")
	NON_RECOVERABLE("DMA1_Channel4_5_IRQHandler")
	NON_RECOVERABLE("ADC1_IRQHandler")
	NON_RECOVERABLE("TIM1_BRK_UP_TRG_COM_IRQHandler")
	NON_RECOVERABLE("TIM1_CC_IRQHandler")
	NON_RECOVERABLE("TIM2_IRQHandler")        /* stm32f072xb */
	NON_RECOVERABLE("TIM3_IRQHandler")
	INTERRUPT_INUSE("TIM6_IRQHandler")
	INTERRUPT_INUSE("TIM7_IRQHandler")
	NON_RECOVERABLE("TIM14_IRQHandler")
	NON_RECOVERABLE("TIM15_IRQHandler")
	NON_RECOVERABLE("TIM16_IRQHandler")
	NON_RECOVERABLE("TIM17_IRQHandler")
	NON_RECOVERABLE("I2C1_IRQHandler")
	NON_RECOVERABLE("I2C2_IRQHandler")
	NON_RECOVERABLE("SPI1_IRQHandler")
	NON_RECOVERABLE("SPI2_IRQHandler")
	INTERRUPT_INUSE("USART1_IRQHandler")
	INTERRUPT_INUSE("USART2_IRQHandler")
	NON_RECOVERABLE("USART3_4_IRQHandler")
	NON_RECOVERABLE("CEC_CAN_IRQHandler")     /* stm32f072xb */
	INTERRUPT_INUSE("USB_IRQHandler")
	:::);
    // use red status led in marlin to flag an unexpected fault
    // note: not ideal, the output pin is not guaranteed to be
    // configured until HAL_setup() is executed
    while(1) debug_red();
}
