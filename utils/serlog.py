'''

serlog.py - a serial port logger and retransmitter.

This program reads data on a serial port, logs it to screen and optionally to a
file and retransmits it on another port.

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



def forward(src, dest, baud, logfile):
	print("Forwarding %s to %s" % (src, dest))
	
	source_ser = serial.Serial(src, int(baud))
	dest_ser = serial.Serial(dest, int(baud))
	
	if logfile != None:
		logfile = open(logfile, 'wb')
	
	while True:
		data = source_ser.read() # read a byte
		
		if logfile != None:
			logfile.write(data)
		print(data, end='')

		dest_ser.write(data) # write the byte
	
	source_ser.close()
	dest_ser.close()
	if logfile != None:
		logfile.close()


if __name__ == "__main__":
	if len(sys.argv) < 4:
		print("Usage: %s from to baud [logfile]")
		exit()
	
	if len(sys.argv) >= 5:
		logfile = sys.argv[4]
	else:
		logfile = None
	
	forward(sys.argv[1], sys.argv[2], sys.argv[3], logfile)
