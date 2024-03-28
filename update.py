#!/usr/bin/python

import sys
import serial
import time
import hashlib
import os

# short feedback how to use this script
print("Usage: <portname> <image name>")

# check if we have sufficient parameters & save for later
if len(sys.argv) < 3:
	print("Error: too less arguments")
	exit()
port = sys.argv[1]
image =  sys.argv[2]
print("Active port: " + port)
print("Update file: " + image)

# open the binary input file, bytewise reading
ifile = open(image,'rb')

# open the serial port (baudrate doesn't matter, USB-CDC)
ser = serial.Serial(port, 500000, timeout=0.5)

# avoid a Windows quirk: the Arduino does not respond to any data, if the DTR line is not set
if os.name == 'nt':
    ser.dtr = True

# totally written bytes
datalen = 0

# as long as there is something available in the input file
with open(image, "rb") as input:
	while True:
        # read in 128B chunks (256B would be faster, but don't work on Arduino ProMicro -> FABI)
		data = input.read(128)
		datalen += len(data)
		print(datalen)
		ser.write(data)
		
        # if this is the last block (less than 128B)
		if len(data) < 128:
            # finished...
			print("Finished, last chunk size: " + str(len(data)))
			break
		#sleep for 10ms each packet. Faster would be better of course, but the Mega32U4 does not handle data fast enough.
		time.sleep(10 / 1000) 
print("Update finished!")
