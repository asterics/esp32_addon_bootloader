#!/usr/bin/python

import sys
import serial
import time
import hashlib

print("Usage: <portname> <image name>")

if len(sys.argv) < 3:
	print("Error: too less arguments")
	exit()
port = sys.argv[1]
image =  sys.argv[2]
print("Active port: " + port)
print("Update file: " + image)

ifile = open(image,'rb')

hash_func = hashlib.sha256()

ser = serial.Serial(port, 115200, timeout=0.5)

datalen = 0

with open(image, "rb") as input:
	while True:
		data = input.read(256)
		datalen += len(data)
		print(datalen)
		ser.write(data)
		hash_func.update(data)
		
		if len(data) < 256:
			print("Finished, last chunk size: " + str(len(data)))
			#print("SHA256: " + str(hash_func.hexdigest().encode("ascii")))
			break
		#sleep 10ms
		time.sleep(10 / 1000) 

#print("Writing SHA (HARDCODED!, because SHA256 hash is not over the whole file, but a part of it. Must be fixed to use the hash of the partition itself)")
#ser.write(hash_func.hexdigest().encode("ascii"))
#ser.write(str("a614de3ffab661b2c91824fc53db933349040bc4d0040258f2e8479c09f6f87b").encode("ascii"))
#print("Please write hash manually in a different terminal program.")
print("Update finished!")
