/** 
 * Malyan M300 (Monoprice Mini Delta) BSP for Marlin 
 * Copyright (C) 2019 Aegean Associates, Inc.
 *
 * replacement for malyanlcd.cpp
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

/**
 * malyanlcd.cpp
 *
 * LCD implementation for Malyan's LCD, a separate ESP8266 MCU running
 * on Serial1 for the M200 board. This module outputs a pseudo-gcode
 * wrapped in curly braces which the LCD implementation translates into
 * actual G-code commands.
 *
 * Added to Marlin for Mini/Malyan M200
 * Unknown commands as of Jan 2018: {H:}
 * Not currently implemented:
 * {E:} when sent by LCD. Meaning unknown.
 *
 * Notes for connecting to boards that are not Malyan:
 * The LCD is 3.3v, so if powering from a RAMPS 1.4 board or
 * other 5v/12v board, use a buck converter to power the LCD and
 * the 3.3v side of a logic level shifter. Aux1 on the RAMPS board
 * has Serial1 and 12v, making it perfect for this.
 * Copyright (c) 2017 Jason Nelson (xC0000005)
 */

#include "MarlinConfig.h"

#if ENABLED(MALYAN_LCD)

#if ENABLED(SDSUPPORT)
  #include "cardreader.h"
  #include "SdFatConfig.h"
#else
  #define LONG_FILENAME_LENGTH  0
#endif

#include "temperature.h"
#include "planner.h"
#include "stepper.h"
#include "duration_t.h"
#include "printcounter.h"
#include "parser.h"
#include "configuration_store.h"

#include "Marlin.h"

// from Marlin_main.cpp
extern bool is_relative_mode(void);

#ifndef BUILD_DISPLAY
#define BUILD_DISPLAY  "99"
#endif

// on the Malyan M300, this will be Serial1
#define LCD_SERIAL  Serial1

// longest sys command + one filename, and some extra just in case
#define MAX_CURLY_COMMAND  ((32 + LONG_FILENAME_LENGTH) * 2)

#if MAX_CURLY_COMMAND > 384
// we count on a fairly large stack (2k), sanity check here
#error "MAX_CURLY_COMMAND may be too large for stack variables"
#endif

#define FILE_LIST_LIMIT  63

void malyan_ui_write(const char * const message)
{
    // everything written needs the high bit set
    for (char * p = (char *) message; *p; p++)
	LCD_SERIAL.write(*p | 0x80);
}

void malyan_ui_write_sys_started(void)
{
    malyan_ui_write("{SYS:STARTED}");
}

void malyan_ui_write_sys_build(void)
{
    malyan_ui_write("{SYS:BUILD}");
}

void malyan_ui_write_sys_canceling(void)
{
    malyan_ui_write("{SYS:CANCELING}");
}

void malyan_ui_write_percent(uint16_t p)
{
    char s[16]; // small buffer

    if (p > 100) p = 100;
    if (! p) malyan_ui_write_sys_build();

    sprintf(s, "{TQ:%03i}", p);
    malyan_ui_write(s);
}

void malyan_ui_write_temperatures(void)
{
    char s[MAX_CURLY_COMMAND];

#if HAS_HEATED_BED
    sprintf(s,
	    "{T0:%03.0f/%03i}{T1:000/000}{TP:%03.0f/%03i}",
	    thermalManager.degHotend(0),
	    thermalManager.degTargetHotend(0),
	    thermalManager.degBed(),
	    thermalManager.degTargetBed());
#else
    sprintf(s,
	    "{T0:%03.0f/%03i}{T1:000/000}{TP:%03.0f/%03i}",
	    thermalManager.degHotend(0),
	    thermalManager.degTargetHotend(0),
	    0,
	    0);
#endif
    malyan_ui_write(s);
}

void malyan_ui_write_printfile(char * fn)
{
    char s[MAX_CURLY_COMMAND];

    sprintf(s, "{PRINTFILE:%.30s}", fn);
    malyan_ui_write(s);
}

#if ENABLED(SDSUPPORT)
/**
 * send a list of filenames/directories to the malyan lcd
 *
 * e.g.
 * {DIR:..}  // used for "parent" button
 * {FILE:buttons.gcode}
 * {DIR:folder}
 * {DIR:++}  // indicate more files than can be displayed 
 * {SYS:OK}
 *
 * ??? (note from marlin4mpmd port) malyan lcd crashes if you
 * send filenames longer than 20 characters to the device ???
 */
static void list_directory(uint16_t n)
{
    char s[MAX_CURLY_COMMAND];
    uint16_t u;
    
    /* ??? from the original malyanlcd.cpp ???
       A more efficient way to do this would be to implement a callback
       in the ls_SerialPrint code, but that requires changes to the core
       cardreader class that would not benefit the majority of users.
       Since one can't select a file for printing during a print, 
       there's little reason not to do it this way.
    */
    u = card.get_num_Files() - n;
    if (u > FILE_LIST_LIMIT)
	u = FILE_LIST_LIMIT;

    if (card.getDirDepth())
	malyan_ui_write("{DIR:..}");

    for (uint16_t i = 0; i < u; i++) {
	card.getfilename(n++);
	sprintf(s,
		card.filenameIsDir ? "{DIR:%.30s}" : "{FILE:%.30s}",
		card.longest_filename());
	malyan_ui_write(s);
    }

    if (! (u < FILE_LIST_LIMIT))
	malyan_ui_write("{DIR:++}");

    malyan_ui_write("{SYS:OK}");
}
#endif

/**
 * process an lcd 'C' command
 * (command portion begins after the colon)
 * {C:T0190}  // set temp for hotend to 190
 * {C:P050}   // set temp for bed to 50
 * {C:S09}    // set feedrate to 90 %.
 * {C:S12}    // set feedrate to 120 %.
 */
inline static void process_lcd_c_command(const char * command)
{
    switch (*command) {
    case 'S':
	feedrate_percentage = atoi(command + 1) * 10;
	feedrate_percentage = constrain(feedrate_percentage, 10, 999);
	break;
    case 'T':
	thermalManager.setTargetHotend(atoi(command + 1), 0);
	break;
    case 'P':
	thermalManager.setTargetBed(atoi(command + 1));
	break;
    default:
	SERIAL_ECHOLNPAIR("UNKNOWN C COMMAND", command);
    }
}

/**
 * process an lcd 'B' command.
 * we respond to the {B:0} request with {T0:008/195}{T1:000/000}
 * {TP:000/000}{TQ:000C}{TT:000000}. T0/T1 are hot end temperatures,
 * TP is bed, TQ is percent, and TT is probably time remaining 
 * (HH:MM:SS). The ui can't handle displaying a second hotend,
 * but the stock firmware always sends it, and it's always zero.
 * ??? {TR:000000} (time remaining) ???
 */
inline static void process_lcd_eb_command(const char * command)
{
    char s[16];
    duration_t t;

    switch (*command) {
    case '0':
	malyan_ui_write_temperatures();
	t = print_job_timer.duration();
	sprintf(s, "{TT:%02u%02u%02u}",
		(uint16_t) t.hour(),
		(uint16_t) t.minute() % 60,
		(uint16_t) t.second());
	malyan_ui_write(s);
#if ENABLED(SDSUPPORT)
	if (card.sdprinting)
	    malyan_ui_write_percent(card.percentDone());
#endif
	break;
    default:
	SERIAL_ECHOLNPAIR("UNKNOWN E/B COMMAND", command);
    }
}

/**
 * process an lcd 'J' command, movement commands
 * (the command portion begins after the colon)
 * axes: X, Y, Z, A (extruder)
 * e.g.
 * {J:E}{J:X-200}{J:E}
 * {J:E}{J:X+200}{J:E}
 */
inline static void process_lcd_j_command(const char * command)
{
    // only allow movement commands if we're not printing
    if (card.sdprinting)
	return;

    char axis = *command;
    char s[24];
    
    switch (axis) {
    case 'E':
	// toggle stepper motor enable. ??? FIXME? ???
	// original code "toggles" -- this seems quite
	// problematic to me, so to achieve "compatible"
	// functionality, we always *disable* here, and
	// always "enable" for the actual motion request
	enqueue_and_echo_command("M18");
	break;
    case 'A': axis = 'E';
    case 'Y':
    case 'Z':
    case 'X':
	enqueue_and_echo_command("M17");
	sprintf(s, "G1 %c%03.1f", axis, atof(command+1));
	if (is_relative_mode()) {
	    enqueue_and_echo_command(s);
	} else {
	    enqueue_and_echo_command("G91");
	    enqueue_and_echo_command(s);
	    enqueue_and_echo_command("G90");
	}
	break;
    default:
	SERIAL_ECHOLNPAIR("UNKNOWN J COMMAND", command);
    }
}

/**
 * process an lcd 'P' command, related to homing and printing
 * 
 * {P:H}   // home all axes
 * {P:X}   // cancel print
 * -> {SYS:CANCELING}
 * -> {SYS:STARTED}
 * {P:P}   // pause print
 * -> {SYS:PAUSE}
 * -> {SYS:PAUSED}
 * {P:R}   // resume print
 * -> {SYS:RESUME}
 * -> {SYS:RESUMED}
 * {P:nnn} // print 3-digit file number nnn (e.g. {P:000})
 * // if P:nnn is a file, display "filename", goto build screen 
 * // OR, if P:nnn is a directory, list directory (see S:L) 
 * -> {PRINTFILE:"filename"}
 * -> {SYS:BUILD}
 */
static void process_lcd_p_command(const char * command)
{
    uint8_t cc = *command;
    
    switch (cc) {
    case 'X':
	// cancel print
        malyan_ui_write_sys_canceling();
#if ENABLED(SDSUPPORT)
	card.abort_sd_printing = true;
	loop(); // make it so
#endif
        malyan_ui_write_sys_started();
	// notify octoprint to cancel print
        MYSERIAL0.write("//action:cancel\n");
	break;

    case 'H':
	// home all axis
	enqueue_and_echo_command("G28");
	break;

    case 'P':
	// pause print
	malyan_ui_write("{SYS:PAUSE}");
#if ENABLED(SDSUPPORT)
    	card.pauseSDPrint();
#endif
    	malyan_ui_write("{SYS:PAUSED}");
	// notify octoprint to pause print
    	MYSERIAL0.write("//action:paused\n");
    	break;

    case 'R':
	// resume print
    	malyan_ui_write("{SYS:RESUME}");
#if ENABLED(SDSUPPORT)
    	card.startFileprint();
#endif
    	malyan_ui_write("{SYS:RESUMED}");
	// notify octoprint to resume print
    	MYSERIAL0.write("//action:resumed\n");
	break;

    default:
	/* ??? note from original malyanlcd.cpp
	   Print file "000", a three digit number, indicating which
	   file to print from the SD card. If it's a directory, then
	   switch to the directory. Find the name of the file to print
	   in order to echo it via the PRINTFILE option. The {S:L}
	   response should ensure that the SD card was mounted.
	*/
	/* ??? note from the original malyanlcd.cpp
	   There may be a difference in how V1 and V2 LCDs handle sub-
	   directory prints. Investigate more. This matches the V1 motion
	   controller actions but the V2 LCD switches to "print" mode on
	   {SYS:DIR} response.
	*/
	if (isdigit(cc)) {
#if ENABLED(SDSUPPORT)
	    int fi = atoi(command);
	    if ((fi == 0) && card.getDirDepth()) {
		card.updir();
		list_directory(0);
		break;
	    }
	    if (fi == FILE_LIST_LIMIT) {
		// *!* FIXME! cannot handle a folder with more
		// than FILE_LIST_LIMIT number of files/folders
		malyan_ui_write_sys_canceling();
		malyan_ui_write_sys_started();
		malyan_ui_write("{E:too many files}");
		break;
	    }
	    if (card.getDirDepth())
		fi--;
	    card.getfilename(fi);
	    if (card.filenameIsDir) {
		card.chdir(card.filename);
		list_directory(0);
		break;
	    }
	    malyan_ui_write_printfile(card.longest_filename());
	    malyan_ui_write_sys_build();
	    card.openAndPrintFile(card.filename);
	    // notify octoprint that we're starting a print
	    MYSERIAL0.write("//action:resumed\n");
#endif
	    break;
	}
	SERIAL_ECHOLNPAIR("UNKNOWN P COMMAND", command);
    }
}

/**
 * process an lcd 'S' command
 *
 * // temperature request
 * {S:I} -> {T0:999/000}{T1:000/000}{TP:004/000}
 * // list files request (return a list of files/directories)
 * {S:L} -> {FILE:"filename"}...{DIR:"directory"}...{SYS:OK}
 */
inline static void process_lcd_s_command(const char * command)
{
    switch (*command) {
    case 'H':
	// home all axis
	enqueue_and_echo_command("G28");
	break;

    case 'I':
	// temperature information
	malyan_ui_write_temperatures();
	break;

    case 'L':
	// list files
#if ENABLED(SDSUPPORT)
	// FIXME? 
	// without card change detect, we must always init 
        card.initsd();
	list_directory(0);
#endif
	break;

    case 'U':
	// sent after {R:R}, no idea what it means
	malyan_ui_write_sys_started();
	break;

#if 0 // ignore
    case 'S':
	// repeatly sent *if* in {SYS:BUILD} *and* a
	// message {E:<message>} is displayed, waiting
	// for the user to press "OK"
	break;
#endif

    default:
	SERIAL_ECHOLNPAIR("UNKNOWN S COMMAND", command);
    }
}

/**
 * Receive a curly brace command and translate to G-code.
 * ??? Currently {E:0} is not handled. Its function is unknown,
 * but it occurs during the temp window after a sys build. ???
 */
inline static void process_lcd_command(const char * command)
{
    char * p = (char *) command;

    // sync up just pass the leading {,
    // the trailing } is already gone
    while ((*p != '{') && (*p != 0)) p++;
    while ((*p == '{') && (*p != 0)) p++;

    uint8_t cc = *p++;
    if (cc && (*p++ == ':')) {
	switch (cc) {
	case 'S':
	    process_lcd_s_command(p);
	    break;
	case 'J':
	    process_lcd_j_command(p);
	    break;
	case 'P':
	    process_lcd_p_command(p);
	    break;
	case 'C':
	    process_lcd_c_command(p);
	    break;
	case 'B':
	case 'E':
	    process_lcd_eb_command(p);
	    break;
	case 'V':
	    malyan_ui_write("{VER:" BUILD_DISPLAY "}");
	}
    }
    else {
	SERIAL_ECHOLNPAIR("UNKNOWN COMMAND", command);
    }
}

static void malyan_ui_process_incoming(void)
{
    static char s[MAX_CURLY_COMMAND];
    static uint16_t n = 0;

    // process incoming characters
    while (LCD_SERIAL.available()) {
	char b = LCD_SERIAL.read() ^ 0x80;
	if (b & 0x80) continue;
	if (b == '}') b = 0;
	s[n++] = b;
	if ((b == 0) || (! (n < MAX_CURLY_COMMAND))) { 
	    s[MAX_CURLY_COMMAND-1] = 0;
	    process_lcd_command(s);
	    n = 0;
	}
    }
}
    
#if ENABLED(SDSUPPORT)
/**
 * simplistic progress bar updating (during sd card printing)
 * The malyan ui needs to see at least one TQ which is not 100% and
 * then when the print is complete, one which is. If a file is not
 * open, set last_percent_done to 100 to ensure that a final print
 * status (i.e. {TQ:100}) is sent to the display (to show the ui's
 * "print complete" screen).
 */
static void malyan_ui_update_progress_bar(void)
{
    static uint16_t last_percent_done = 100;

    uint16_t k = last_percent_done;

    if (card.sdprinting)
	k = card.percentDone();

    if (! card.isFileOpen())
	k = 100;

    if (k != last_percent_done) {
	malyan_ui_write_percent(k);
	last_percent_done = k;
    }
}
#endif

static void malyan_ui_update_usb_status(void)
{
    static uint16_t usb;

    // update the usb status indicator
    uint16_t u = HAL_usb_IsConfigured();
    if (usb != u) {
	// R:UC (connected), R:UD (disconnected)
	malyan_ui_write((char *) (u ? "{R:UC}" : "{R:UD}"));
	usb = u;
    }
}

/**
 * Initialize the lcd display, plus read the curly-brace commands
 * from the malyan ui computer (on serial 1) and translate into g-code
 * where appropriate. Also show/hide the USB status indicator on the
 * display. We consider the "configured" usb as being "connected". 
 * {R:R} *may* be a command to reset the lcd display. The lcd display
 * responds with a string of x's along with {S:U} followed by a number
 * {V:0}'s -- all of which the purpose is unknown. We use {S:U} as an
 * indication that the display is booting, and {V:0} as a request for a
 * version number. The x's could be an unused "boot" string -- for now,
 * we ignore them (and any character input without its high bit set).
 */

void lcd_init(void)
{
    LCD_SERIAL.begin(500000);
    malyan_ui_write("{R:R}");  // ??? seems to be reset
}

void lcd_update(void)
{
    static volatile uint16_t busy;

    if (! busy) {
	busy = 1;

	malyan_ui_update_usb_status();
	malyan_ui_process_incoming();
#if ENABLED(SDSUPPORT)
	malyan_ui_update_progress_bar();
#endif

	busy = 0;
    }
}


/**
 * set an alert
 */
void lcd_setalertstatusPGM(const char * message)
{
    char s[MAX_CURLY_COMMAND];

    sprintf(s, "{E:%.30s}", message);
    malyan_ui_write(s);
}

#endif
