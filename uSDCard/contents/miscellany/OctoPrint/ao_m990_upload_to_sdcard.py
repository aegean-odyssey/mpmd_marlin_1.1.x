### AO_M990_UPLOAD_TO_SDCARD - simple upload protocol
### Copyright (C) 2019, 2020 Aegean Associates, Inc.
#

### IMPORTANT: This plugin exploits the custom m990 command found in the
# mpmd_marlin_1.1.x firmware (Marlin firmware for the Monoprice MP Mini
# Delta 3d printer) to upload a file to the printer's sd card. Though not
# a speed demon, it is significantly faster than Marlin's M28 command, and
# it transfers the gcode file intact (unaltered). PLEASE NOTE: If you are
# not using mpmd_marlin_1.1.x firmware in an MP Mini Delta printer, then
# this plugin serves no purpose, and will likely break OctoPrint's upload
# to sd card function for your printer.
# 

### see the docs
# https://github.com/aegean-odyssey/mpmd_marlin_1.1.x.wiki/OctoPrint-plugin

### The SIMPLE UPLOAD protocol transfers a file in blocks with a fixed-
# length of 512 bytes. The transmitter always sends a 512-byte block
# to the receiver with the transmitter padding the last block with an
# end-of-file (EOF) character (i.e. nul, '\0'). At least one EOF (nul)
# character must be sent. The receiver initiates the transfer by sending
# the token, "BEGIN\n", and acknowleges each block received with a
# newline, "\n". The receiver identifies the end of the file transfer
# by examining the last character of each block. If the last character
# of the block is an EOF (nul) character, then the block should be the
# final block from the transmitter. The bytes preceeding the first EOF
# (nul) character in the block are the remaining data bytes of the file
# being transmitted. This simple scheme REQUIRES that the EOF (nul)
# character is NEVER part of the file's actual data -- not generally a
# problem when tranferring text files.
#
### NOTE: Our receiver implementation (in Marlin) also sets the variable,
# "card.saving = true" to prevent Marlin from executing any extra or stray
# commands that the sending program may transmit *after* our receiving
# process terminates (such as after an abort or error). The sending program
# should always transmit an "M29" (close file) M-code command a few seconds
# after it has sent its final data packet to return Marlin to its normal
# command process state (i.e. "set card.saving = false").
#

from __future__ import absolute_import

import os, sys, time, logging, threading, serial
import octoprint.util as util

def ao_m990_upload_to_sdcard(printer, filename, path,
                             started_f, success_f, failure_f,
                             *args, **kwargs):

    logger = logging.getLogger(__name__)

    target = util.get_dos_filename(filename, None, 'gco', ['g', 'gc'])
    if not target: target = 'CACHE.GCO'

    logger.info("Uploading {} to {}.".format(filename, target))
    started_f(filename, target)

    def ao_set_progress(pct):
        # FIXME! yet to be implemented
        pass

    def ao_upload_protocol():

        TIMEOUT = 3  # time out, seconds

        # tiny hack for python 2/3 compatibility
        ORD = (lambda c: ord(c), lambda c: c)[sys.hexversion < 0x3000000]

        def ao_waitfor(sio, pattern):
            t = time.time() + TIMEOUT
            n = len(pattern)
            i = 0
            while time.time() < t:
                c = sio.read(1)
                if not c: continue
                i = i + 1 if ORD(c) == pattern[i] else 0
                if i < n: continue
                return False  # success
            return True       # timeout

        x, port, rate, prof = printer.get_current_connection()
        printer.disconnect()

        ERROR = 0
        try:
            sio = serial.Serial(port, rate, timeout = TIMEOUT)
            inp = open(path, "rb")
            
            inp.seek(0, os.SEEK_END)
            fz = inp.tell()  # file size, bytes
            inp.seek(0, os.SEEK_SET)

            ao_set_progress(0);
        
            dT = time.time()
            N = 0
            sio.write("\nM990 S{:d} /{:s}\n".format(fz, target).encode())
            if not ao_waitfor(sio, b'BEGIN\n'):
                BLKSZ = 0x200  # block size, bytes
                pkt = bytearray(BLKSZ)
                while True:
                    u = inp.readinto(pkt)
                    if u < BLKSZ:
                        pkt[u:] = b'\0' *(BLKSZ - u)
                    if sio.write(pkt) < BLKSZ: u = 0
                    if ao_waitfor(sio, b'\n'): u = 0
                    N += u
                    if u < BLKSZ:
                        break
                    # update progress every 128 blocks
                    if (N & 0xffff) == 0:
                        ao_set_progress(N/fz)

            time.sleep(TIMEOUT)
            sio.write(b'\nM29\n')
            sio.flush()

            ao_set_progress(100)
            
            dT = time.time() - dT
            ERROR = int(N < fz)
            S = "{}. Sent {:d} of {:d} B in {:d} s ({:.0f} B/s)."
            logger.info(S.format(("SUCCESS", "FAILED")[ERROR],
                                 N, fz, int(dT), N/dT))

        except serial.SerialException as e:
            logger.exception("{}".format(e))

        except IOError as e:
            logger.exception("{}".format(e))

        finally:
            if inp: inp.close
            if sio: sio.close

        printer.connect(port=port, baudrate=rate, profile=prof)
        # call the appropriate success or failure callback function
        (success_f, failure_f)[ERROR](filename, target, int(dT))

    thread = threading.Thread(target = ao_upload_protocol)
    thread.daemon = True
    thread.start()
    return target


__plugin_name__          = "AO M990 Upload to SDCard"
__plugin_description__   = "simple upload protocol"
__plugin_version__       = "0.0.1"
__plugin_author__        = "Aegean Odyssey"
__plugin_license__       = "AGPLv3"
__plugin_pythoncompat__  = ">=2.7,<4"

__plugin_hooks__ = {
    "octoprint.printer.sdcardupload": ao_m990_upload_to_sdcard
}
