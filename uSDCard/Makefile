# uSDCard
PROJECT = mpmd_marlin_1.1.x-uSDCard

# pull some version info from the firmware Makefile 
__FM = ../Makefile
__PROJECT := $(shell grep -m1 '^PROJECT' ${__FM} | grep -o '[^ ]*$$')
__VERSION := $(shell grep -m1 '^VERSION' ${__FM} | grep -o '[^ ]*$$')
__RELEASE := $(shell grep -m1 '^RELEASE' ${__FM} | grep -o '[^ ]*$$')
VERSION = ${__PROJECT}-${__VERSION}r${__RELEASE}

# use the path to this Makefile as our reference 
ZD := $(dir $(realpath $(lastword $(MAKEFILE_LIST))))

# root directory of card image
CONTENTS  = ${ZD}contents
MODELS    = ${CONTENTS}/models
MISCSTUFF = ${CONTENTS}/miscellany
OCTOPRINT = ${MISCSTUFF}/OctoPrint
FIRMWARE  = ${CONTENTS}/firmware

ZIPFILE = ${ZD}${VERSION}-SDCard.zip

.PHONY : all clean distclean card deploy firmware

card : distclean
	@echo "_${VERSION}_"
#	# create gcode utilities
	${ZD}util-setup-gcode.sh
#	# copy models, stl and gcode
	mkdir -p ${MODELS}
	cp ${ZD}models/*.stl ${MODELS}/
	cp ${ZD}models/*.gcode ${MODELS}/
#	# copy the octoprint plugin
	mkdir -p ${OCTOPRINT}
	cp ${ZD}ao_m990_upload_to_sdcard.py ${OCTOPRINT}/
#	# copy the ao_m990.py file
	cp ${ZD}ao_m990.py ${MISCSTUFF}/
#	# files overwritten when "zipped"
	cp ${ZD}README.md ${CONTENTS}/
	touch ${CONTENTS}/FCUPDATE.FLG

all : card firmware deploy

firmware :
	mkdir -p ${FIRMWARE}
	cp ../${VERSION}*.bin ${FIRMWARE}/

deploy : ${ZIPFILE}
	mv $< ..

${ZIPFILE} : ${CONTENTS} README.md 
	sed -i '1s/^.*$$/_${VERSION}_/' $</README.md
	rm -f $@
	cd $^ && zip -r $@ . 

${CONTENTS} : card

distclean : clean
	rm -fR ${CONTENTS}

clean :
	rm -f .*~ *~ *.o *.d *.a
