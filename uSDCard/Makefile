# uSDCard - 

PROJECT = mpmd_marlin_1.1.x-uSDCard
VERSION = 00 
RELEASE = 01

MPMD_MARLIN = mpmd_marlin_1.1.x-119r??-05Alimit.bin

# use the path to the Makefile as our reference 
ZD := $(dir $(realpath $(lastword $(MAKEFILE_LIST))))

# firmware to include
FIRMWARE := $(realpath ${ZD}..)/${MPMD_MARLIN}
VERSION := **_$(basename $(notdir $(wildcard ${FIRMWARE})))_**

# root directory of card image
CONTENTS = ${ZD}contents

.PHONY : all clean distclean card deploy

card : distclean
#	# create gcode utilities
	${ZD}util-setup-gcode.sh
#	# copy models
	mkdir -p ${CONTENTS}/models
	cp ${ZD}models/*.stl ${CONTENTS}/models/
	cp ${ZD}models/*.gcode ${CONTENTS}/
#	# files overwritten when "zipped"
	cp README.md ${CONTENTS}/
	touch ${CONTENTS}/firmware.bin
	touch ${CONTENTS}/fcupdate.flg

all : card deploy

deploy : ${ZD}${PROJECT}.zip
	mv $< ..

${ZD}${PROJECT}.zip : ${CONTENTS} ${FIRMWARE} README.md 
	sed -i '1s/^/${VERSION}\n/' $</README.md
	cp ${FIRMWARE} $</firmware.bin
	rm -f $@
	cd $^ && zip -r $@ . 

${CONTENTS} : card

distclean : clean
	rm -fR ${CONTENTS}

clean :
	rm -f .*~ *~ *.o *.d *.a
