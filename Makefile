### simple(-ish) makefile to build Marlin images

PROJECT = mpmd_marlin_1.1.x
VERSION = 119
RELEASE = 00

STM32CUBE = STM32Cube-1.10.1
MARLIN11X = Marlin-1.1.x
MARLINCHG = marlin_changes

DEFINES += -DMAKE_PROJECT='"${PROJECT}"'
DEFINES += -DMAKE_VERSION='"${VERSION}"'
DEFINES += -DMAKE_RELEASE='"${RELEASE}"'

_05A : DEFINES += -DMAKE_05ALIMIT
_10A : DEFINES += -DMAKE_10ALIMIT

### DIRECTORY ABBREVIATIONS

# use the path to the Makefile as root 
ZD := $(dir $(realpath $(lastword $(MAKEFILE_LIST))))

BUILD = ${ZD}build

CMSIS   = ${ZD}${STM32CUBE}/Drivers/CMSIS
DEVICE  = ${ZD}${STM32CUBE}/Drivers/CMSIS/Device/ST/STM32F0xx
HAL_SRC = ${ZD}${STM32CUBE}/Drivers/STM32F0xx_HAL_Driver/Src
USBDEVICE = Middlewares/ST/STM32_USB_Device_Library
CDC_SRC = ${ZD}${STM32CUBE}/${USBDEVICE}/Class/CDC/Src
USB_SRC = ${ZD}${STM32CUBE}/${USBDEVICE}/Core/Src
STM32F0 = ${ZD}stm32f0xx
MARLIN  = ${ZD}Marlin
MARLIN_ = ${ZD}${MARLINCHG}
ARDUINO = ${ZD}${MARLINCHG}/arduino_fakes

### SOURCE FILES

LINKER_LD = ${STM32F0}/LinkerScript.ld

PRJ_INCLUDE_H = ${MARLIN}/HAL_stm32.h
BSP_INCLUDE_H = ${STM32F0}/stm32f0xx_hal_conf.h

PRJ_SOURCES = \
	$(wildcard ${MARLIN}/*.c) \
	$(wildcard ${MARLIN}/*.cpp)

PRJ_EXCLUDE = \
	${MARLIN}/configuration_store.cpp \
	${MARLIN}/MarlinSerial.cpp \
	${MARLIN}/Sd2Card.cpp \
	${MARLIN}/watchdog.cpp \
	${MARLIN}/malyanlcd.cpp 

BSP_SOURCES  = \
	$(wildcard ${STM32F0}/*.s) \
	$(wildcard ${HAL_SRC}/*.c) \
	$(wildcard ${USB_SRC}/*.c) \
	$(wildcard ${CDC_SRC}/*.c) \
	$(wildcard ${STM32F0}/*.c)

BSP_EXCLUDE = \
	$(wildcard ${HAL_SRC}/*_template.c) \
	$(wildcard ${USB_SRC}/*_template.c) \
	$(wildcard ${CDC_SRC}/*_template.c)

### TARGET LISTS

# variants
_05A = ${PROJECT}-${VERSION}r${RELEASE}-05Alimit
_10A = ${PROJECT}-${VERSION}r${RELEASE}-10Alimit
PRJ  = ${PROJECT}-${VERSION}r${RELEASE}

PRJ_SRCS = $(filter-out $(PRJ_EXCLUDE),$(PRJ_SOURCES))
PRJ_OBJS = $(addsuffix .o,$(basename $(notdir $(PRJ_SRCS))))
PRJ_DEPS = $(PRJ_OBJS:.o=.d)

BSP_SRCS = $(filter-out $(BSP_EXCLUDE),$(BSP_SOURCES))
BSP_OBJS = $(addsuffix .o,$(basename $(notdir $(BSP_SRCS))))
BSP_DEPS = $(BSP_OBJS:.o=.d)

# number of dependency (.d) files to expect
D_COUNT = $(words $(PRJ_DEPS) $(BSP_DEPS))


### SOURCE FILE SEARCH PATHS

VPATH = $(sort $(dir $(PRJ_SRCS) $(BSP_SRCS)))


### INCLUDE FILE SEARCH PATHS

INCLUDE = \
	-isystem ${DEVICE}/Include \
	-isystem ${CMSIS}/Include  \
	-isystem ${CMSIS}/DSP/Include \
	-isystem ${HAL_SRC}/../Inc \
	-isystem ${USB_SRC}/../Inc \
	-isystem ${CDC_SRC}/../Inc \
	-isystem ${ARDUINO} \
	-I${STM32F0} \
	-I${MARLIN}


### BUILD FLAGS

$(BSP_OBJS) $(BSP_DEPS) : CFLAGS += -include ${BSP_INCLUDE_H}
$(PRJ_OBJS) $(PRJ_DEPS) : CFLAGS += -include ${PRJ_INCLUDE_H}

LDFLAGS  += -L${CMSIS}/Lib/GCC
LDLIBS   += -larm_cortexM0l_math

CFLAGS   += ${DEFINES}
CPPFLAGS += -fsingle-precision-constant -fmerge-all-constants 
CFLAGS   += -Os -fdata-sections -ffunction-sections -flto $(INCLUDE)
CXXFLAGS += $(CFLAGS) -fno-exceptions -fno-rtti
LDFLAGS  += -specs=nano.specs -u _printf_float -Wl,--gc-sections -flto
ASFLAGS  += -I${DEVICE}/Include


### BUILD TOOLS

CXX = arm-none-eabi-g++ -mcpu=cortex-m0 -mthumb -mfloat-abi=soft
CC  = arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft
AS  = arm-none-eabi-as  -mcpu=cortex-m0 -mthumb -mfloat-abi=soft
AR  = arm-none-eabi-ar

OBJCOPY = arm-none-eabi-objcopy
BINSIZE = arm-none-eabi-size


### MAKE RULES

.PHONY : one all clean realclean distclean depends PRJ _05A _10A

.PRECIOUS : %.elf

one :
	$(MAKE) -C ${ZD} -f ${ZD}Makefile ${MARLIN}
	$(MAKE) -C ${ZD} -f ${ZD}Makefile ${BUILD}
ifneq (${D_COUNT},$(words $(wildcard ${BUILD}/*.d)))
#	# something's missing, re-create dependencies
	$(MAKE) -C ${BUILD} -f ${ZD}Makefile depends
endif
	$(MAKE) -C ${BUILD} -f ${ZD}Makefile PRJ

all :  distclean ${MARLIN} ${BUILD}
	$(MAKE) -C ${ZD}    -f ${ZD}Makefile realclean
	$(MAKE) -C ${BUILD} -f ${ZD}Makefile depends
	$(MAKE) -C ${BUILD} -f ${ZD}Makefile realclean
	$(MAKE) -C ${BUILD} -f ${ZD}Makefile _05A
	$(MAKE) -C ${BUILD} -f ${ZD}Makefile depends
	$(MAKE) -C ${BUILD} -f ${ZD}Makefile realclean
	$(MAKE) -C ${BUILD} -f ${ZD}Makefile _10A

${MARLIN} : 
	mkdir -p $@
	cd $@ && cp -sf ../${MARLIN11X}/Marlin/*.* .
	cd $@ && cp -sf ../${MARLINCHG}/*.* .

${BUILD} :
	mkdir -p $@

distclean : clean
	$(MAKE) -C ${MARLIN_} -f ${ZD}Makefile realclean
	$(MAKE) -C ${STM32F0} -f ${ZD}Makefile realclean
	rm -fR ${BUILD}
#	# safely (conditionally) remove the Marlin directory
	-find -P ${MARLIN} -type l -delete
	-rmdir ${MARLIN}

depends : $(BSP_DEPS) $(PRJ_DEPS)

_10A : ${_10A}.map ${_10A}.bin ${_10A}.MAP
	@cp -u $^ ${ZD} && cat $<

_05A : ${_05A}.map ${_05A}.bin ${_05A}.MAP
	@cp -u $^ ${ZD} && cat $<

PRJ : ${PRJ}.map ${PRJ}.bin ${PRJ}.MAP
	@cp -u $^ ${ZD} && cat $<

%.elf : ${LINKER_LD} $(BSP_OBJS) $(PRJ_OBJS)
	$(CXX) $(LDFLAGS) -Wl,-Map=$(@:.elf=.MAP) -o $@ -T$^ $(LDLIBS)

%.map : %.elf
	$(BINSIZE) -A -B $< >$@

%.bin : %.elf
	$(OBJCOPY) -O binary $< $@

%.MAP : %.elf

realclean : clean
	rm -f *.elf *.MAP *.map *.bin

clean :
	rm -f .*~ *~ *.o *.d *.a

### AUTOMATIC PREREQUISITES
# ignore this stuff if our target is clean, realclean, or distclean
ifeq (,$(findstring ${MAKECMDGOALS},clean realclean distclean)) 

%.d : %.s
	echo "$(@:.d=.o) $@: $<" >$@                

%.d : %.c
	$(CC) -MM -MF $@ -MT '$(@:.d=.o) $@' $(CPPFLAGS) $(CFLAGS) $<

%.d : %.cpp
	$(CXX) -MM -MF $@ -MT '$(@:.d=.o) $@' $(CPPFLAGS) $(CXXFLAGS) $<

ifeq (${MAKECMDGOALS},depends)

define depends_rule =
$(basename $(notdir $(1))).d : $(1)
endef

# for depends, automatically create a rule for each source file
$(foreach f,$(BSP_SRCS) $(PRJ_SRCS),$(eval $(call depends_rule,$(f))))

else

# otherwise, include our rule for each source file (if it exists)
include $(notdir $(realpath $(BSP_DEPS) $(PRJ_DEPS)))

endif
endif
