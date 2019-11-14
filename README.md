# mpmd_marlin_1.1.x
a fork of Marlin firmware (bugfix-1.1.x) for the Monoprice MP Mini Delta 3d printer

> **IMPORTANT** mpmd_marlin_1.1.x is a work in progress. There is a great deal of experimenting, testing, and refinement in the works. ***The firmware is largely untested -- USE AT YOUR OWN RISK!***

## Background

It all began with my effort to understand the build process for the [Marlin4MPMD](https://github.com/mcheah/Marlin4MPMD) project. You see, I wanted to tinker with the firmware of my Monoprice MP Mini Delta 3d printer and studied the Marlin4MPMD project to learn how to proceed. During one of the many periods of confusion, self-doubts, and desperation, I turned to the [Marlin firmware](https://www.marlinfw.org) project as an alternative. Of course, there really is no substitute for actually understanding how things work, so after poring over both projects, I figured I'd developed enough understanding to attempt a "new" port of Marlin firmware for the Monoprice MP Mini Delta 3d printer.

Here is the result.

> **BTW**, the mpmd_marlin_1.1.x project would not be possible without the extensive information curated in the Marlin4MPMD source code and at the Marlin4MPMD GitHub repository ([mcheah/Marlin4MPMD](https://github.com/mcheah/Marlin4MPMD)).

## Highlights/ Features

The mpmd_marlin_1.1.x firmware differs from the MP Mini Delta's stock firmware and from the alternative Marlin4MPMD firmware in features and behavior. Here are a few highlights:

* derived from the Marlin firmware codebase (bugfix-1.1.x branch);
* wifi, as found in the stock Monoprice firmware, is NOT implemented;
* two flavors, 5A limit and 10A limit firmware (similar to Marlin4MPMD);
* adds a "kill" switch function missing in the stock Monoprice firmware;
* adds arc support (G2/G3) gcodes;
* adds filament change (M600) gcode;
* adds manual progress update (M73) gcode;
* improves LCD and SD card operation (compared to Marlin4MPMD firmware);
* improves delta calibration (G33) gcode;
* modified automatic bed leveling (G29) gcode; and
* most "safety" features of Marlin enabled.

### Installation

* *coming...*

*In the interim, please see [mcheah/Marlin4MPMD](https://github.com/mcheah/Marlin4MPMD) -- the installation procedure is more or less the same.*

### User Guide

* *coming...*

### G-code Support

* *coming...*

## Development

For the intrepid maker, hacker, programmer,... here is some information on how this firmware is being developed.

### Build System

The build system is linux based, specifically [Debian "buster"](https://www.debian.org/releases/buster/) with the following additional packages installed:

*(ARM bare metal compiler, tools, and libraries)*

```sh
$ apt-get install build-essential emacs-nox gcc-arm-none-eabi binutils-arm-none-eabi libstdc++\-arm-none-eabi-newlib libnewlib-arm-none-eabi libnewlib-dev gdb-multiarch openocd telnet
```
*(basic git -- for GitHub, ssh, ...)*

```sh
$ apt-get install git quilt patchutils openssh-client ca-certificates gnupg wget curl
```

*(optional, but I find useful)*

```sh
$ apt-get install sudo rename xsltproc picocom zip unzip p7zip-full p7zip-rar
```
### Compiling

The "Makefile" documents the entire build process. The default target (make) and the "all" target (make all) are good places to start. The "Makefile" also defines the release number (e.g. RELEASE = 00).

Generate two (2) "firmware.bin" files -- one for use with a 60watt power adapter (05Alimit) and one for use with a 120watt power adapter (10Alimit). 

```sh
$ make all
```
Generate a single "firmware.bin" file with the prevailing configuration of the original Marlin4MPDM source (currently matches the 10Alimit configuration).

```sh
$ make
```

### Source Notes

*(excerpt from HAL_stm32.h)*

```C
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
```

### Configuration Options

*(excerpt from HAL_stm32.h)*

```C
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
#ifndef INVERT_STEPPER_DIRECTION_XYZE
#define INVERT_STEPPER_DIRECTION_XYZE  0b0001
#endif
```

## More Resources

+ [Marlin4MPMD firmware GitHub](https:/github.com/mcheah/Marlin4MPMD)
+ [MP Mini Delta (Unofficial) Wiki](https://www.mpminidelta.com)
+ [Marlin Firmware](https://www.marlinfw.org)
+ [my Marlin4MPMD port](https://github.com/aegean-odyssey/marlin4mpmd_1.3.3)
+ [Debian "buster"](https://www.debian.org/releases/buster/)
+ [STM32CubeF0 (STMicroelectronics)](https://www.st.com/content/st_com/en/products/embedded-software/mcu-mpu-embedded-software/stm32-embedded-software/stm32cube-mcu-mpu-packages/stm32cubef0.html)
