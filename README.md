<img alt="Marlin logo" height="100" align="right"
 src="https://github.com/MarlinFirmware/Marlin/blob/2.0.x/buildroot/share/pixmaps/logo/marlin.svg" />

# mpmd_marlin_1.1.x
__an open-source upgrade for the Monoprice MP Mini Delta 3d printer__<br/>
_a fork of Marlin firmware (bugfix-1.1.x) for the Monoprice MP Mini Delta 3d printer_

[```Latest Release```](https://github.com/aegean-odyssey/mpmd_marlin_1.1.x/releases/latest)
[```Quick Start```](https://github.com/aegean-odyssey/mpmd_marlin_1.1.x/wiki/Quick-Start)

<img alg="Monoprice Mini Delta" height="240" align="right"
 src="https://github.com/aegean-odyssey/PrusaSlicer-settings/blob/master/Monoprice_MiniDelta.png" />
 
The mpmd_marlin_1.1.x project is a port of the very popular [Marlin firmware](https://www.marlinfw.org). This port, from Aegean Odyssey, specifically targets the 32-bit motherboard found in the [Monoprice MP Mini Delta 3d printer](https://www.monoprice.com/product?p_id=21666). It is Marlin firmware tailored to the strengths and weaknesses of the Mini Delta.

<p clear="both">&nbsp;</p>

> **PLEASE NOTE** mpmd_marlin_1.1.x is a work in progress. There is a great deal of experimenting, testing, and refinement in the works. Though the firmware seems to work well, it is largely untested -- ***use at your own risk***.


### Should you upgrade?

I'd like to say everyone with a Monoprice MP Mini Delta should try this firmware upgrade, 
but it is not for everyone. Follow the link, ["Should you upgrade?"](https://github.com/aegean-odyssey/mpmd_marlin_1.1.x/wiki/Should-you-upgrade%3f), for information to help you decide.

We believe this open-source alternative is worthwhile. Naturally, the choice is yours.
To give it a try. please see the wiki page, ["Quick Start"](https://github.com/aegean-odyssey/mpmd_marlin_1.1.x/wiki/Quick-Start), for instructions.


### Why another Marlin port?

It all began with my effort to understand the build process for the [Marlin4MPMD](https://github.com/mcheah/Marlin4MPMD) project. You see, I wanted to tinker with the firmware of my Monoprice MP Mini Delta 3d printer and studied the Marlin4MPMD project to learn how to proceed. During one of the many periods of confusion, self-doubts, and desperation, I turned to the [Marlin firmware](https://www.marlinfw.org) project as an alternative. Of course, there really is no substitute for actually understanding how things work, so after poring over both projects, I figured I'd developed enough understanding to attempt a "new" port of Marlin firmware for the Monoprice MP Mini Delta 3d printer.

The mpmd_marlin-1.1.x project is the result.

By the way, the mpmd_marlin_1.1.x project would not be possible without the extensive information curated by @mcheah in the Marlin4MPMD source code and at the Marlin4MPMD GitHub repository ([mcheah/Marlin4MPMD](https://github.com/mcheah/Marlin4MPMD)).


### More Resources

+ [MP Mini Delta (Unofficial) Wiki](https://www.mpminidelta.com)
+ [Marlin Firmware](https://www.marlinfw.org)
+ [Marlin4MPMD firmware GitHub](https:/github.com/mcheah/Marlin4MPMD)
+ [Marlin4MPMD (AO port)](https://github.com/aegean-odyssey/marlin4mpmd_1.3.3)
+ [STM32CubeF0 (STMicroelectronics)](https://www.st.com/content/st_com/en/products/embedded-software/mcu-mpu-embedded-software/stm32-embedded-software/stm32cube-mcu-mpu-packages/stm32cubef0.html)
+ [Debian "buster"](https://www.debian.org/releases/buster/)
