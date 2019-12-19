#/bin/sh

# IMPORTANT! make sure that the path to the
# card is correct. DOUBLE-CHECK!

VOLUME=/dev/mmcblk0

# MONOPRICE MINI DELTA
# format a micro sd card in a way that the
# Monoprice Mini Delta may find acceptable

sudo mkfs.fat "$VOLUME" -F 32 -s 1 -S 512 -I
