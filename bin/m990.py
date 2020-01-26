#!/usr/bin/env python3

## M990.PY - custom send file for mpmd_marlin_1.1.x
# Copyright (c) 2019 Aegean Associates, Inc. All rights reserved.
#

## The SIMPLE UPLOAD protocol transfers a file in blocks with a fixed-
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
## NOTE: Our implementation also sets "card.saving = true" to prevent
# Marlin from executing any extra or stray commands that the sending
# program may transmit *after* our receiving process terminates (such
# as after an abort or error). The sending program should always
# transmit an "M29" (close file) M-code command a few seconds after
# it has sent its final data packet to return Marlin to its normal
# command process state (i.e. "set card.saving = false").
#


import os, sys, getopt, time
import serial


def print_stderr(message):
    sys.stderr.write(message)
    sys.stderr.write('\n')


def usage(message):
    '''Usage: m990.py -d|--device=port -s|--speed=bps -q|--quiet file'''
    print_stderr(message)
    print_stderr(usage.__doc__)
    sys.exit(3)


def shorten(path):
    '''Return a "DOS8.3-like" basename from a file path.'''
    s, e = os.path.splitext(os.path.basename(path))
    s = s.replace(".", "")[:8].upper()
    return ("CACHE.GCO", s + ".GCO")[int(len(s)>0)]


class Serial(serial.Serial):

    def __init__(self, device, speed):
        super().__init__(device, speed, timeout=0)

    def waitfor(self, pattern, timeout):
        t = time.time() + timeout
        n = len(pattern)
        i = 0
        while time.time() < t:
            c = self.read(1)
            if not c: continue
            i = i + 1 if ord(c) == pattern[i] else 0
            if i < n: continue
            return False  # success
        return True       # timeout


def main(argv):
    '''Simple Upload Protocol, transfer a file through a serial port.'''

    dbg = 1
    dev = '/dev/ttyACM0'
    bps = 250000

    TMO = 3
    sio = None
    gcf = None

    try:
        opts, args = getopt.getopt(
            argv, 'hqvs:d:', ['help','quiet','verbose','speed=','device='])

    except getopt.GetoptError as e:
        usage(str(e))

    for opt, arg in opts:
        if opt in ('-h', '--help'):
            usage(main.__doc__)
        elif opt in ('-q', '--quiet'):
            dbg = 0
        elif opt in ('-v', '--verbose'):
            dbg += 1
        elif opt in ('-d', '--device'):
            dev = arg
        elif opt in ('-s', '--speed'):
            bps = arg

    if len(args) != 1:
        usage("Please supply ONE file argument.")

    arg = args[0]  # file name

    try:
        sio = Serial(dev, bps)
        gcf = open(arg, "rb")

        fn = shorten(arg) # file name
        gcf.seek(0, os.SEEK_END)
        fz = gcf.tell()   # file size
        gcf.seek(0, os.SEEK_SET)

        dT = time.time()
        N = 0

        sio.write("\nM990 S{:d} /{:s}\n".format(fz, fn).encode())
        if sio.waitfor(b'BEGIN\n', TMO):
            if dbg: print_stderr("Unable to sync up")

        else:
            BLKSZ = 0x200
            pkt = bytearray(BLKSZ)
            while True:
                u = gcf.readinto(pkt)
                if u < BLKSZ:
                    pkt[u:] = b'\0' * (BLKSZ - u)
                if sio.write(pkt) < BLKSZ: u = 0
                if sio.waitfor(b'\n',TMO): u = 0
                N += u
                if u < BLKSZ:
                    break

        time.sleep(TMO)
        sio.write(b'\nM29\n')
        sio.flush()

        dT = time.time() - dT
        if dbg:
            S = "{:s}. Sent {:d} of {:d} B in {:d} s ({:.0f} B/s)."
            R = ("SUCCESS", "FAILED")[int(N < fz)]
            print_stderr(S.format(R, N, fz, int(dT), float(N)/dT))

        sys.exit(int(N < fz))

    except serial.SerialException as e:
        if dbg: print_stderr(str(e))
        sys.exit(2)

    except IOError as e:
        if dbg: print_stderr(str(e))
        sys.exit(2)

    finally:
        if gcf: gcf.close
        if sio: sio.close


if __name__ == "__main__":
    main(sys.argv[1:])
