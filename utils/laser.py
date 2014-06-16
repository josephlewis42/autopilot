'''

laser.py - a monitor for the Renishaw plc ilm-500-hr

This program reads data from the laser altimeter.

Copyright 2013 Joseph Lewis <joseph@josephlewis.net>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the following disclaimer
  in the documentation and/or other materials provided with the
  distribution.
* Neither the name of the  nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''


from __future__ import print_function
import serial
import sys

import random

baud = 38400
resolution = 10 # resolution in cm
averaged = 100

def forward(src):
	print("Reading %s" % (src))
	
	source_ser = serial.Serial(src, int(baud))
	
	errors = 0
	avg = 0
	avgct = 0
	while True:
		header = ord(source_ser.read()) # read a byte
		value = ord(source_ser.read()) # read the value
		

		if avgct < averaged:
			if header == 0xbf:
				errors += 1			
			else:
				header = (header & 0b1111)
				#print(header)
				header *= 100
				avg += value #+ header
			avgct += 1
		else:

			if averaged - errors == 0:
				distance = 0.0			
			else:			
				distance = ((float(avg) / (averaged - errors)) * resolution)
			errorstr = "" if errors == 0 else " (%s%% error readings)" % (errors * 100 / averaged)
			print("%sm (%s cm)%s" % (distance / 100, distance, errorstr))
			avg = 0
			avgct = 0
			errors = 0

	
	source_ser.close()

if __name__ == "__main__":
	if len(sys.argv) < 2:
		print("Usage: %s from to baud [logfile]")
		exit()
	
	forward(sys.argv[1])
