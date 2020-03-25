<img alt="Marlin logo" height="100" align="right"
 src="https://github.com/aegean-odyssey/mpmd_marlin_1.1.x/wiki/marlin_logo.svg" />

# mpmd_marlin_1.1.x
__an open-source upgrade for the Monoprice MP Mini Delta 3d printer__<br/>
_a fork of Marlin firmware (bugfix-1.1.x) for the Monoprice MP Mini Delta 3d printer_

[```Latest Release```](https://github.com/aegean-odyssey/mpmd_marlin_1.1.x/releases/latest)
&nbsp; [```Quick Start```](https://github.com/aegean-odyssey/mpmd_marlin_1.1.x/wiki/Quick-Start)

<img alg="Monoprice Mini Delta" height="240" align="right"
 src="https://github.com/aegean-odyssey/mpmd_marlin_1.1.x/wiki/mpminidelta.png" />
 
The mpmd_marlin_1.1.x project is a port of the very popular [Marlin firmware](https://www.marlinfw.org). This port, from Aegean Odyssey, specifically targets the 32-bit motherboard found in the [Monoprice MP Mini Delta 3d printer](https://www.monoprice.com/product?p_id=21666). It is Marlin firmware tailored to the strengths and weaknesses of the Mini Delta.

> **`PLEASE NOTE:`** mpmd_marlin_1.1.x is a work in progress. There is a great deal of experimenting, testing, and refinement in the works. Though the firmware seems to work well, it is largely untested -- ***use at your own risk***.
<br clear="both"/>

### Should you upgrade?

I'd like to say everyone with a Monoprice MP Mini Delta should try this firmware upgrade, 
but it is not for everyone. Follow the link, ["Should you upgrade?"](https://github.com/aegean-odyssey/mpmd_marlin_1.1.x/wiki/Should-you-upgrade%3f), for information to help you decide. We believe this open-source alternative is worthwhile. Naturally, the choice is yours. To give it a try. please see the wiki page, 
["Quick Start"](https://github.com/aegean-odyssey/mpmd_marlin_1.1.x/wiki/Quick-Start), for instructions.


### Why another Marlin port?

My initial experience with the Monoprice MP Mini Delta was mostly frustration. I just could not get the printer to print reliably. But when it did work, it was great fun. Installing [Marlin4MPMD](https://github.com/mcheah/Marlin4MPMD) greatly improved the printing, though there were still some usability issues that I thought might be fixable. After spending some time understanding the build process for the Marlin4MPMD project, I opted to work out a simpler command-line build for stable [Marlin firmware](https://www.marlinfw.org), and to code the relatively simple low-level interface to the printer's 32-bit controller board. The goal is to produce a very useable "open source upgrade" (firmware, supporting files, and utilities) for the stock Monoprice MP Mini Delta printer.

The mpmd_marlin-1.1.x project is the result.

By the way, the mpmd_marlin_1.1.x project would not be possible without the extensive information curated by @mcheah in the Marlin4MPMD source code and at its GitHub repository, [mcheah/Marlin4MPMD](https://github.com/mcheah/Marlin4MPMD).

__`LINKS:`__
&nbsp; [`MP Mini Delta (Unofficial) Wiki`](https://www.mpminidelta.com)
&nbsp; [`Marlin Firmware`](https://marlinfw.org)
&nbsp; [`Marlin4MPMD firmware GitHub`](https://github.com/mcheah/Marlin4MPMD)

<!--
### Other Resources
+ [Marlin4MPMD (AO port)](https://github.com/aegean-odyssey/marlin4mpmd_1.3.3)
+ [STM32CubeF0 (STMicroelectronics)](https://www.st.com/content/st_com/en/products/embedded-software/mcu-mpu-embedded-software/stm32-embedded-software/stm32cube-mcu-mpu-packages/stm32cubef0.html)
+ [Debian "buster"](https://www.debian.org/releases/buster/)
-->
