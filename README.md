# mpmd_marlin_1.1.x
a fork of Marlin firmware (bugfix-1.1.x) for the Monoprice MP Mini Delta 3d printer

> **IMPORTANT** mpmd_marlin_1.1.x is a work in progress. There is a great deal of experimenting, testing, and refinement in the works. ***The firmware is largely untested -- USE AT YOUR OWN RISK!***

## Background

It all began with my effort to understand the build process for the [Marlin4MPMD](https://github.com/mcheah/Marlin4MPMD) project. You see, I wanted to tinker with the firmware of my Monoprice MP Mini Delta 3d printer and studied the Marlin4MPMD project to learn how to proceed. During one of the many periods of confusion, self-doubts, and desperation, I turned to the [Marlin firmware](https://www.marlinfw.org) project as an alternative. Of course, there really is no substitute for actually understanding how things work, so after poring over both projects, I figured I'd developed enough understanding to attempt a "new" port of Marlin firmware for the Monoprice MP Mini Delta 3d printer.

Here is the result.

> **BTW**, the mpmd_marlin_1.1.x project would not be possible without the extensive information curated by @mcheah in the Marlin4MPMD source code and at the Marlin4MPMD GitHub repository ([mcheah/Marlin4MPMD](https://github.com/mcheah/Marlin4MPMD)).

## Highlights/ Features

The mpmd_marlin_1.1.x firmware differs from the MP Mini Delta's stock firmware and from the alternative Marlin4MPMD firmware in features and behavior. Here are a few highlights:

* derived from the Marlin firmware codebase (bugfix-1.1.x branch);
* wifi, as found in the stock Monoprice firmware, is _**NOT**_ implemented;
* two flavors, 5A limit and 10A limit firmware (similar to Marlin4MPMD);
* adds a "kill" switch function missing in the stock Monoprice firmware;
* adds arc support (G2/G3) gcodes;
* adds filament change (M600) gcode;
* adds manual progress update (M73) gcode;
* improves LCD and SD card operation (compared to Marlin4MPMD firmware);
* improves delta calibration (G33) gcode;
* modified automatic bed leveling (G29) gcode;
* most "safety" features of Marlin enabled; and
* calibrate, adjust, etc. from the SD card menu.


## Quick Start

* __Step 1. Install the firmware__
	* place files, firmware.bin and fcupdate.flg, in the root
	directory of the sd card
	* power up the printer -- led flashes white while programming
	* printer resets -- led is green briefly, then solid white
	* delete the file, fcupdate.flg

* __Step 2. Calibrate the printer__
	* perform the initial calibration, G33
	* create a bed leveling grid, G29 
	* adjust the probe Z offset, M851
	* save, M500

* __Step 3. Print__
	* install a (Mini Delta) printer profile in your slicing program 
	* configure the print start and end G-code in your slicing program
	* slice models, and print from the micro sd card


## Installing Firmware

* *more details belong here... for now,*

_Please see [mcheah/Marlin4MPMD](https://github.com/mcheah/Marlin4MPMD) -- the installation procedure is more or less the same._

## Calibrating the Printer

Once installed, the mpmd_marlin_1.1x firmware requires an initial calibration. 
This calibration process can be executed in two ways, with a serial terminal 
program via the USB port or directly on the printer via a micro SD card:

### via the USB port

* calibrate the machine parameters
```gcode
G33         ; automatically calibrate
M500        ; save
```

* produce a bed level (bi-linear) mesh
```gcode
M851 Z0     ; reset the probe offset
G28         ; home (w/ new probe offset)
G29         ; compute a bed level mesh
M851 Z0.6   ; set a nominal probe offset
M500        ; save
```

* adjust the z offset (machine specific)
```gcode
M851 Z0.450 ; machine specific offset
M500        ; save
```

### via the SD card

The setup_gcode folder contains a set of "command" (gcode) files that 
extend the capabilities of the printer. Place these files on the micro
SD card and use the printer's "print" function to access them.

Select print on the Mini Delta and choose the file, AUTO_CALIBRATE.gcode,
(in the folder, setup_gcode) to perform the initial calibration for the
printer. This command will calibrate the printer geometry, create a bed 
leveling grid, and save the settings to flash memory. The output that is
normally sent to the serial port during this process is captured and 
saved to a file, CALIBRAT.txt, on the micro sd card.

The initial calibration sets the "Z-height" to 0.6mm, which by design
places the nozzle above the build plate when the nozzle is moved to the
origin (X0 Y0 Z0). This offset can be adjusted with the "M851 Zn.nnn" 
M-code command and stored to the flash memory with the M500 M-code 
command. "Print" the files, M851_Znnn.gcode and M500_SAVE.gcode, 
respectively, to perform these functions.

_**to recap**_

* calibrate the machine and bed level parameters
```
AUTO_CALIBRATE.gcode
```

* adjust the z offset (machine specific) and save
```
M851_Zxxx.gcode         ; where "xxx" is the offset (e.g. 450 is 0.450mm)
M500_SAVE.gcode
```

#### Other command files
```sh
/fcupdate.flg                      ; file to signal the bootloader to update
/firmware.bin                      ; mpmd_marlin_1.1.x firmware update
/setup_gcode/
	AUTO_CALIBRATE.gcode       ; initial calibration (creates CALIBRAT.TXT)
	CREATE_FCUPDATE.gcode      ; create the fcupdate.flg file
	DELETE_FCUPDATE.gcode      ; delete the fcupdate.flg file
	FILAMENT_LOAD.gcode        ; (currently unimplemented)
	FILAMENT_UNLOAD.gcode      ; (currently implemented)
	G0_X0_Y0_Z0.gcode          ; move the nozzle to X0 Y0 Z0
	G28_HOME.gcode             ; move to the home position (home)
	G29_BED_LEVEL.gcode        ; calculate a bed level grid (creates BEDLEVEL.TXT)
	M500_SAVE.gcode            ; save the current settings to flash
	M501_RESTORE.gcode         ; load the saved settings from flash
	M502_FACTORY.gcode         ; load the factory default settings
	M503_REPORT.gcode          ; configuration info (creates REPORT.TXT)
	M851_Z000.gcode            ; set the "probe" Z offset to 0.000mm 
	M851_Z300.gcode            ; set the "probe" Z offset to 0.300mm
	M851_Z350.gcode            ; set the "probe" Z offset to 0.350mm
	M851_Z400.gcode            ; set the "probe" Z offset to 0.400mm
	M851_Z450.gcode            ; set the "probe" Z offset to 0.450mm
	M851_Z500.gcode            ; set the "probe" Z offset to 0.500mm
	M851_Z550.gcode            ; set the "probe" Z offset to 0.550mm
	M851_Z600.gcode            ; set the "probe" Z offset to 0.600mm
	M851_Z650.gcode            ; set the "probe" Z offset to 0.650mm
	M851_Z700.gcode            ; set the "probe" Z offset to 0.700mm
	M851_Z750.gcode            ; set the "probe" Z offset to 0.750mm
	M851_Z800.gcode            ; set the "probe" Z offset to 0.800mm
```

## Configuring a Print

* *more details belong here ...*

### Start/End G-code

* sample start g-code
```gcode
; mpmd_marlin_1.1.x firmware
; set and wait on the hot end temperature
M104 S[first_layer_temperature] T0
M109 S[first_layer_temperature] T0
; set and wait on the bed temperature
M140 S[first_layer_bed_temperature]
M190 S[first_layer_bed_temperature]
; home axes, probe/adjust z-offset, and pause 4s
G28
G29 P0
G0 X0 Y0 Z80 F3600
G4 S4
; extrude a priming strip outside of the perimeter
G92 E0
G1 X-54 Y0 Z0.32 F2700
G3 X0 Y-54 I54 E20 F900
G92 E0
```

* sample end g-code
```gcode
; mpmd_marlin_1.1.x firmware
; heaters off, home, motors off
M104 S0
M140 S0
G28
M84
```

## G/M-code Support

### Marlin Index

 •[G0‑G1: Linear Move ](http://marlinfw.org/docs/gcode/G000-G001.html) 
 •[G2‑G3: Controlled Arc Move ](http://marlinfw.org/docs/gcode/G002-G003.html) 
 •[G4: Dwell ](http://marlinfw.org/docs/gcode/G004.html) 
 •[G27: Park toolhead ](http://marlinfw.org/docs/gcode/G027.html) 
 •[G28: Auto Home ](http://marlinfw.org/docs/gcode/G028.html) 
 •[G29: Bed Leveling (Automatic) ](http://marlinfw.org/docs/gcode/G029-abl.html) 
 •[G30: Single Z‑Probe ](http://marlinfw.org/docs/gcode/G030.html) 
 •[G33: Delta Auto Calibration ](http://marlinfw.org/docs/gcode/G033.html) 
 •[G90: Absolute Positioning ](http://marlinfw.org/docs/gcode/G090.html) 
 •[G91: Relative Positioning ](http://marlinfw.org/docs/gcode/G091.html) 
 •[M0‑M1: Unconditional stop ](http://marlinfw.org/docs/gcode/M000-M001.html) 
 •[M16: Expected Printer Check ](http://marlinfw.org/docs/gcode/M016.html) 
 •[M17: Enable Steppers ](http://marlinfw.org/docs/gcode/M017.html) 
 •[M18‑M84: Disable steppers ](http://marlinfw.org/docs/gcode/M018.html) 
 •[M20: List SD Card ](http://marlinfw.org/docs/gcode/M020.html) 
 •[M21: Init SD card ](http://marlinfw.org/docs/gcode/M021.html) 
 •[M22: Release SD card ](http://marlinfw.org/docs/gcode/M022.html) 
 •[M23: Select SD file ](http://marlinfw.org/docs/gcode/M023.html) 
 •[M24: Start or Resume SD print ](http://marlinfw.org/docs/gcode/M024.html) 
 •[M25: Pause SD print ](http://marlinfw.org/docs/gcode/M025.html) 
 •[M26: Set SD position ](http://marlinfw.org/docs/gcode/M026.html) 
 •[M27: Report SD print status ](http://marlinfw.org/docs/gcode/M027.html) 
 •[M28: Start SD write ](http://marlinfw.org/docs/gcode/M028.html) 
 •[M29: Stop SD write ](http://marlinfw.org/docs/gcode/M029.html) 
 •[M30: Delete SD file ](http://marlinfw.org/docs/gcode/M030.html) 
 •[M31: Print time ](http://marlinfw.org/docs/gcode/M031.html) 
 •[M32: Select and Start ](http://marlinfw.org/docs/gcode/M032.html) 
 •[M33: Get Long Path ](http://marlinfw.org/docs/gcode/M033.html) 
 •[M73: Set Print Progress ](http://marlinfw.org/docs/gcode/M073.html) 
 •[M75: Start Print Job Timer ](http://marlinfw.org/docs/gcode/M075.html) 
 •[M76: Pause Print Job ](http://marlinfw.org/docs/gcode/M076.html) 
 •[M77: Stop Print Job Timer ](http://marlinfw.org/docs/gcode/M077.html) 
 •[M81: Power Off ](http://marlinfw.org/docs/gcode/M081.html) 
 •[M82: E Absolute ](http://marlinfw.org/docs/gcode/M082.html) 
 •[M83: E Relative ](http://marlinfw.org/docs/gcode/M083.html) 
 •[M92: Set Axis Steps‑per‑unit ](http://marlinfw.org/docs/gcode/M092.html) 
 •[M104: Set Hotend Temperature ](http://marlinfw.org/docs/gcode/M104.html) 
 •[M105: Report Temperatures ](http://marlinfw.org/docs/gcode/M105.html) 
 •[M106: Set Fan Speed ](http://marlinfw.org/docs/gcode/M106.html) 
 •[M107: Fan Off ](http://marlinfw.org/docs/gcode/M107.html) 
 •[M108: Break and Continue ](http://marlinfw.org/docs/gcode/M108.html) 
 •[M109: Wait for Hotend Temperature ](http://marlinfw.org/docs/gcode/M109.html) 
 •[M110: Set Line Number ](http://marlinfw.org/docs/gcode/M110.html) 
 •[M111: Debug Level ](http://marlinfw.org/docs/gcode/M111.html) 
 •[M112: Emergency Stop ](http://marlinfw.org/docs/gcode/M112.html) 
 •[M113: Host Keepalive ](http://marlinfw.org/docs/gcode/M113.html) 
 •[M114: Get Current Position ](http://marlinfw.org/docs/gcode/M114.html) 
 •[M115: Firmware Info ](http://marlinfw.org/docs/gcode/M115.html) 
 •[M117: Set LCD Message ](http://marlinfw.org/docs/gcode/M117.html) 
 •[M118: Serial print ](http://marlinfw.org/docs/gcode/M118.html) 
 •[M119: Endstop States ](http://marlinfw.org/docs/gcode/M119.html) 
 •[M125: Park Head ](http://marlinfw.org/docs/gcode/M125.html)
 •[M140: Set Bed Temperature ](http://marlinfw.org/docs/gcode/M140.html) 
 •[M155: Temperature Auto‑Report ](http://marlinfw.org/docs/gcode/M155.html) 
 •[M190: Wait for Bed Temperature ](http://marlinfw.org/docs/gcode/M190.html) 
 •[M201: Set Print Max Acceleration ](http://marlinfw.org/docs/gcode/M201.html) 
 •[M203: Set Max Feedrate ](http://marlinfw.org/docs/gcode/M203.html) 
 •[M204: Set Starting Acceleration ](http://marlinfw.org/docs/gcode/M204.html) 
 •[M205: Set Advanced Settings ](http://marlinfw.org/docs/gcode/M205.html) 
 •[M220: Set Feedrate Percentage ](http://marlinfw.org/docs/gcode/M220.html) 
 •[M221: Set Flow Percentage ](http://marlinfw.org/docs/gcode/M221.html) 
 •[M301: Set Hotend PID ](http://marlinfw.org/docs/gcode/M301.html) 
 •[M302: Cold Extrude ](http://marlinfw.org/docs/gcode/M302.html) 
 •[M303: PID autotune ](http://marlinfw.org/docs/gcode/M303.html) 
 •[M304: Set Bed PID ](http://marlinfw.org/docs/gcode/M304.html) 
 •[M400: Finish Moves ](http://marlinfw.org/docs/gcode/M400.html) 
 •[M401: Deploy Probe ](http://marlinfw.org/docs/gcode/M401.html) 
 •[M402: Stow Probe ](http://marlinfw.org/docs/gcode/M402.html) 
 •[M410: Quickstop ](http://marlinfw.org/docs/gcode/M410.html) 
 •[M420: Bed Leveling State ](http://marlinfw.org/docs/gcode/M420.html) 
 •[M428: Home Offsets Here ](http://marlinfw.org/docs/gcode/M428.html) 
 •[M500: Save Settings ](http://marlinfw.org/docs/gcode/M500.html) 
 •[M501: Restore Settings ](http://marlinfw.org/docs/gcode/M501.html) 
 •[M502: Factory Reset ](http://marlinfw.org/docs/gcode/M502.html) 
 •[M503: Report Settings ](http://marlinfw.org/docs/gcode/M503.html) 
 •[M504: Validate EEPROM contents ](http://marlinfw.org/docs/gcode/M504.html) 
 •[M524: Abort SD print ](http://marlinfw.org/docs/gcode/M524.html) 
 •[M600: Filament Change ](http://marlinfw.org/docs/gcode/M600.html) 
 •[M603: Configure Filament Change ](http://marlinfw.org/docs/gcode/M603.html) 
 •[M665: Delta Configuration ](http://marlinfw.org/docs/gcode/M665.html) 
 •[M666: Set Delta endstop adjustments ](http://marlinfw.org/docs/gcode/M666.html) 
 •[M851: XYZ Probe Offset ](http://marlinfw.org/docs/gcode/M851.html) 
 •[M928: Start SD Logging ](http://marlinfw.org/docs/gcode/M928.html) 
 •[M999: STOP Restart ](http://marlinfw.org/docs/gcode/M999.html) 


### Additions/ Changes

G/M-code|Note
-|-
`G29 P0` | auto bed level, adjust for height (Z) only
`M106 ...` | only available with part-cooling fan option
`M107 ...` | only available with part-cooling fan option
`M118 {`_<_string_>_`}` | send a control string to the lcd ui
`M988 `_<_filename_>_ | capture output to file (DOS 8.3 name)
`M989 ` | close the capture file
`M0/M1 ...` | use via pushbutton led<br>_blue flashing: waiting for user_
`M600 ...` | use via pushbutton led _**(untested!)**_<br>_cyan flashing: waiting for user to insert filament;_<br>_yellow flashing: waiting for user to re-heat nozzle;_<br>_yellow solid: nozzle re-heating, please stand-by_
`M117 ...` | non-functional on lcd (future enhancement?) 


## Development

For the intrepid maker, hacker, programmer,... here is some information on how this firmware is being developed.

### Build System

The build system is linux based, specifically [Debian "buster"](https://www.debian.org/releases/buster/) with the following additional packages installed:

*(ARM bare metal compiler, libraries, and tools)*

```sh
$ apt-get install build-essential gcc-arm-none-eabi binutils-arm-none-eabi 
$ apt-get install libnewlib-arm-none-eabi libnewlib-dev libstdc++\-arm-none-eabi-newlib 
$ apt-get install gdb-multiarch openocd telnet emacs-nox
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
```

## More Resources

+ [Marlin4MPMD firmware GitHub](https:/github.com/mcheah/Marlin4MPMD)
+ [MP Mini Delta (Unofficial) Wiki](https://www.mpminidelta.com)
+ [Marlin Firmware](https://www.marlinfw.org)
+ [my Marlin4MPMD port](https://github.com/aegean-odyssey/marlin4mpmd_1.3.3)
+ [Debian "buster"](https://www.debian.org/releases/buster/)
+ [STM32CubeF0 (STMicroelectronics)](https://www.st.com/content/st_com/en/products/embedded-software/mcu-mpu-embedded-software/stm32-embedded-software/stm32cube-mcu-mpu-packages/stm32cubef0.html)
