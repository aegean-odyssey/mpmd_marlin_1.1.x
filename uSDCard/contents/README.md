# mpmd_marlin_1.1.x
__An open-source upgrade for the Monoprice MP Mini Delta 3d printer__

Use the files in this zip archive to enhance your Monoprice MP Mini Delta
3d printer. The
[mpmd_marlin_1.1.x project](https:/github.com/aegean-odyssey/mpmd_marlin_1.1.x)
combines Marlin firmware 1.1.9 along with a few customizations to
create a truly open-source firmware for the printer. 


## Card Contents

* mpmd_marlin_1.1.x firmware

  _featuring_
  * Marlin Firmware 1.1.9
  * improve UI and SD card operation
  * automatic delta calibration
  * modified automatic bed leveling
  * most "safety" features of Marlin enabled
  * calibrate, adjust, etc. from the SD card menu.
  * and more...
  
* G-code command files

  The setup_gcode folder contains a set of "command" (gcode) files
that extend the capabilities of the printer. Use the printer's
"print" function to perform calibrations, change or save settings,
and provide other capabilities that are normally not available on
the Monoprice Mini Delta printer.

* Example 3d models 

  * monoprice_cat &nbsp;&nbsp;--
_the Monoprice demo gcode file, with modified start gcode_
  * 3dBenchy &nbsp;&nbsp;--
_[#3DBenchy by CreativeTools.se](https://www.thingiverse.com/thing:763622)_
  * money_cat &nbsp;&nbsp;--
_[maneki-neko -money cat- by bs3](https://www.thingiverse.com/thing:923108)_
  * money_cat_fill &nbsp;&nbsp;--
_[maneki-neko -money cat- by bs3](https://www.thingiverse.com/thing:923108)_
  * treefrog_45_cut &nbsp;&nbsp;--
_[Treefrog by MorenaP](https://www.thingiverse.com/thing:18479)_
  

## Getting Started

* Step 1. Install the firmware
  * place the zip file contents in the root directory of the sd card
  * power up the printer -- led flashes white while programming
  * printer resets -- led is green briefly, then solid white
  * delete the file, fcupdate.flg (use DELETE_FCUPDATE.gcode)

* Step 2. Calibrate the printer
  * perform the initial calibration, G33 (use AUTO_CALIBRATE.gcode)
  * create a bed leveling grid, G29 (done by AUTO_CALIBRATE.gcode)
  * adjust the probe Z offset, M851 (use an M851_Znnn.gcode file)
  * save, M500 (use M500_SAVE.gcode)

* Step 3. Print
  * install a (Mini Delta) printer profile in your slicing program
  * configure the print start and end G-code in your slicing program
  * slice models, and print from the micro sd card


## Additional Information

* _(more belongs here...)_
