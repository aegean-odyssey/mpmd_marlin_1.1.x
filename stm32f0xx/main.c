/** 
 * Malyan M300 (Monoprice Mini Delta) BSP for Marlin 
 * Copyright (C) 2019 Aegean Associates, Inc.
 */

#ifndef STM32F070xB
#define STM32F070xB
#endif

#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"

/* marlin entry points */
extern void setup (void);
extern void loop (void);

static void system_clocks_config(void)
{
    RCC_OscInitTypeDef osc;
    RCC_ClkInitTypeDef clk;

    // use the external 8Mhz osc, configure PLL for 48Mhz
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PREDIV = RCC_PREDIV_DIV2;
    osc.PLL.PLLMUL = RCC_PLL_MUL12;
    osc.HSEState = RCC_HSE_ON;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
	while(1);

    // use PLL as system clock source and configure HCLK, PCLK1 dividers
    clk.ClockType =RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) != HAL_OK)
	while(1);
}


#ifdef STM32_USE_BOOTLOADER
typedef struct {
    uint32_t address[48];
} IVT_t;

extern __IO IVT_t g_pfnVectors;
__IO IVT_t VectorTable  __attribute__((section(".RAMVectorTable")));
#endif


int main(void)
{
#ifdef STM32_USE_BOOTLOADER
    // disable all interrupts
    __disable_irq();
    NVIC->ICER[0U] = (uint32_t)(~0UL);
    // reset all peripherals
    __HAL_RCC_APB1_FORCE_RESET();
    __HAL_RCC_APB1_RELEASE_RESET();
    __HAL_RCC_APB2_FORCE_RESET();
    __HAL_RCC_APB2_RELEASE_RESET();
    __HAL_RCC_AHB_FORCE_RESET();
    __HAL_RCC_AHB_RELEASE_RESET();
    // clear any pending interrupts
    NVIC->ICPR[0U] = (uint32_t)(~0UL);
    // swap in our interrupt vectors
    VectorTable = g_pfnVectors;
    __HAL_SYSCFG_REMAPMEMORY_SRAM();
    __HAL_RCC_SRAM_CLK_ENABLE();
    __enable_irq();
    // tickle the watchdog, just in case
    IWDG->KR = 0x0000AAAA;
#endif

    HAL_Init();
    system_clocks_config();

    setup();

    for(;;) {
	loop();
    }
}
