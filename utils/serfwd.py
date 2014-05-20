#!/usr/bin/env python
'''

serfwd.py - a serial port forwarder.

This program reads data on a serial port and transmits it over a TCP connection
to a serfwd service running on another device.

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


import serial
import sys
import socket


def send_loop(serdevice, host, port):
	print "Forwarding %s to %s:%s" % (serdevice, host, port)
	ser = serial.Serial(serdevice, 9600)
	print ser.name
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sock.connect((host, port))
	
	fd = open("send.bin", 'wb')


	while True:
		data = ser.read() # read one byte

		print data
		fd.write(data)
		sock.send(data)  # echo
	sock.close()
	ser.close()
	
	
def recv_loop(serdevice, port):
	
	try:
		ser = serial.Serial(serdevice, 9600)
	except serial.SerialException:
		print "Using %s as file" % serdevice
		ser = open(serdevice, 'wb')
	
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sock.bind(('', port))
	sock.listen(1)
	conn, addr = sock.accept()
	print 'Connection address:', addr
	while True:
		data = conn.recv(1)
		if not data: break
		print("%s" % hex(ord(data)))
		ser.write(data)
	
	conn.close()
	ser.close()

if __name__ == "__main__":
	if len(sys.argv) != 4:
		print "Usage: %s 'send' host:port serial" % sys.argv[0]
		print "       %s 'recv' portnum serial" % sys.argv[0]
		exit()
		
	if sys.argv[1] == 'send':
		host, port = sys.argv[2].split(":")
		port = int(port)
		serport = sys.argv[3]
		send_loop(serport, host, port)
	else:
		port = int(sys.argv[2])
		serport = sys.argv[3]
		recv_loop(serport, port)
	
