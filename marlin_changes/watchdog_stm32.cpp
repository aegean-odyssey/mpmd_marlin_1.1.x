/** 
 * Malyan M300 (Monoprice Mini Delta) BSP for Marlin 
 * Copyright (C) 2019 Aegean Associates, Inc.
 *
 * replacement for watchdog.cpp (and endstop_interrupts.h, see below)
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

#include "Marlin.h"
#include "watchdog.h"


#ifndef FAUX_TIMEOUT
#define FAUX_TIMEOUT  5859  // ~6s, depends on temperature isr rate
#endif

void watchdog_init(void)
{
    HAL_iwdg_init();
    watchdog_reset();
}

void watchdog_reset(void)
{
#if ENABLED(WATCHDOG_RESET_MANUAL)
    faux_watchdog = FAUX_TIMEOUT;
#endif
    HAL_iwdg_refresh();
}

#if ENABLED(WATCHDOG_RESET_MANUAL)
static uint16_t faux_watchdog = 0;

void faux_watchdog_interrupt(void)
{
    if (! faux_watchdog) return;
    if (faux_watchdog--) return;

    watchdog_reset();
    SERIAL_ERROR_START();
    SERIAL_ERRORLNPGM("Watchdog barked, please turn off the printer.");
    kill(PSTR("ERR:Watchdog"));
    while (1);
}
#endif
