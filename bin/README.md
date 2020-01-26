# ~/bin
### m990.py
__transfer a file from the host to the printer's SD card__

The m990.py python script is an example of a host (utility) program to transfer a G-code file to the micro SD card of the Monoprice MP Mini Delta printer via its USB port. It uses the custom M990 G/M-code to significantly increase the transfer speed as compared to the normal Marlin upload mechanism. 

### arm-none-eabi-size (wrapper)

The arm-none-eabi-size script here (in ~/bin) is a "pretty print" wrapper for the output from the original arm-none-eabi-size program (in /usr/bin). It combines both styles of the program's output (-A -B) and provides hexadecimal address and decimal size information.

So, instead of

```sh
$ /usr/bin/arm-none-eabi-size -A mpmd_marlin_1.1.x-119r00-10Alimit.elf

mpmd_marlin_1.1.x-119r00-10Alimit.elf  :
section                 size        addr
.isr_vector              192   134225920
.text                 108580   134226112
.rodata                10860   134334696
.ARM.extab                 0   134345556
.ARM                       0   134345556
.preinit_array             0   134345556
.init_array                8   134345556
.fini_array                4   134345564
.data                    924   536871104
UNUSED_FLASH_MEMORY      260   134346492
.bss                    7456   536872028
._user_heap_stack       2052   536879484
.ARM.attributes           40           0
.RAMVectorTable          192   536870912
.comment                 174           0
.debug_info           254520           0
.debug_abbrev          49281           0
.debug_loc            114246           0
.debug_aranges          3640           0
.debug_ranges           4936           0
.debug_line            46078           0
.debug_str             19492           0
.debug_frame            6336           0
Total                 629271
```

or


```sh
$ /usr/bin/arm-none-eabi-size -B mpmd_marlin_1.1.x-119r00-10Alimit.elf

text	   data	    bss	    dec	    hex	filename
 119632	    936	   9960	 130528	  1fde0	mpmd_marlin_1.1.x-119r00-10Alimit.elf
```

the wrapper outputs

```sh
$ arm-none-eabi-size  mpmd_marlin_1.1.x-119r00-10Alimit.elf

MPMD_MARLIN_1.1.X-119R00-10ALIMIT.ELF

section                    addr    size
.isr_vector          0x08002000      c0 (192)
.text                0x080020c0   1a824 (108580)
.rodata              0x0801c8e8    2a6c (10860)
.ARM.extab           0x0801f354       0 (0)
.ARM                 0x0801f354       0 (0)
.preinit_array       0x0801f354       0 (0)
.init_array          0x0801f354       8 (8)
.fini_array          0x0801f35c       4 (4)
.data                0x200000c0     39c (924)
UNUSED_FLASH_MEMORY  0x0801f6fc     104 (260)
.bss                 0x2000045c    1d20 (7456)
._user_heap_stack    0x2000217c     804 (2052)
.ARM.attributes      0x00000000      28 (40)
.RAMVectorTable      0x20000000      c0 (192)
.comment             0x00000000      ae (174)
.debug_info          0x00000000   3e238 (254520)
.debug_abbrev        0x00000000    c081 (49281)
.debug_loc           0x00000000   1be46 (114246)
.debug_aranges       0x00000000     e38 (3640)
.debug_ranges        0x00000000    1348 (4936)
.debug_line          0x00000000    b3fe (46078)
.debug_str           0x00000000    4c24 (19492)
.debug_frame         0x00000000    18c0 (6336)

text	   data	    bss	    dec	    hex	filename
119632	    936	   9960	 130528	  1fde0	mpmd_marlin_1.1.x-119r00-10Alimit.elf
```
