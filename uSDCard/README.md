_mpmd_marlin_1.1.x_

## MPMD_MARLIN_1.1.X

__an open-source upgrade for the Monoprice MP Mini Delta 3d printer__

The [mpmd_marlin_1.1.x](https:/github.com/aegean-odyssey/mpmd_marlin_1.1.x)
project combines Marlin firmware 1.1.9 along with a few customizations to
create a useful open-source firmware for the printer. Use the files in this
zip archive to enhance your Monoprice MP Mini Delta 3d printer.

## GETTING STARTED

Copy the contents of this folder to the ROOT directory of the micro SD card,
i.e. this README.md file should be in the card's ROOT directory. With the
micro SD card, you can install the firmware, perform an initial printer
calibration, and print a few test models. Complete instructions, tips, and
help are avaiable at the wiki.

Please see [the wiki](
https://github.com/aegean-odyssey/mpmd_marlin_1.1.x/wiki)
for more information.

## CARD CONTENTS

##### mpmd_marlin_1.1.x firmware

The several variants of the mpmd_marlin_1.1.x firmware are located in the
`/firmware` folder. To install the firmware, ONE of the .bin files should
be copied to the SD card's ROOT directory and RENAMED to `/firmware.bin`.

Please see [Which Firmware?](
https://github.com/aegean-odyssey/mpmd_marlin_1.1.x/wiki/Which-Firmware%3f)
to help determine an appropriate firmware variant for your printer.
  
##### G-code command files

The `/setup_gcode` folder contains a set of "command" (gcode) files
that extend the capabilities of the printer. Use the printer's
"print" function to perform calibrations, change or save settings,
and provide other capabilities that are normally not available on
the Monoprice Mini Delta printer.

##### Example 3d models

There are a few example .stl and .gcode files in the `/models` folder.
The .gcode files contain start and end gcode suited to the firmware and
the MP Mini Delta printer, and are ready to print directly from the
micro SD card.

* monoprice_cat _(
  the Monoprice demo gcode file, with modified start gcode)_

* 3dBenchy _(ref: [#3DBenchy by CreativeTools.se](
  https://www.thingiverse.com/thing:763622))_

* money_cat _(ref: [maneki-neko -money cat- by bs3](
  https://www.thingiverse.com/thing:923108))_

* money_cat_fill _(ref: [maneki-neko -money cat- by bs3](
  https://www.thingiverse.com/thing:923108))_

* treefrog_45_cut _(ref: [Treefrog by MorenaP](
  https://www.thingiverse.com/thing:18479))_

##### OctoPrint-plugin

Use the plugin located in `/miscellany/OctoPrint` to allow for
faster uploads to the printer's micro SD card from within OctoPrint.
Copy the single plugin file, `ao_m990_upload_to_scard.py`, to
OctoPrint's plugin folder.

Please see [OctoPrint-plugin](
https://github.com/aegean-odyssey/mpmd_marlin_1.1.x/wiki/OctoPrint-plugin)
for more information.

##### Pronterface macro

The M990 g-code can be used from Pronterface to upload the currently
loaded file to the printer's micro SD card. To configure Pronterface
to allow for faster uploads, use the following steps:

* Copy the command, `/miscellany/ao_m990.py`, to a location that is
  included in your computer's command search path (make sure that the
  file is executable).

* Create a macro (e.g. AO_M990) in Pronterface to hold the follow script:

  ```
  ! self.ao = 'ao_m990.py -d "{}" -s "{}" "$s"'.format(self.p.port,self.p.baud)
  ! self.disconnect()
  ! print('AO_M990: {}'.format(self.filename))
  ! self.do_run_script(self.ao)
  ! self.connect()
  ```

* Create a custom button/command (e.g. Upload to SD) in Pronterface to
  call the macro you created in the previous step.

Please see [Pronterface Tips](
https://github.com/aegean-odyssey/mpmd_marlin_1.1.x/wiki/Pronterface-Tips)
for more information.
