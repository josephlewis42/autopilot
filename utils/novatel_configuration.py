'''
A configuration script for the NovAtel stations.

Used to auto-configure them without the need to keep doing it manually.
'''


import serial
import time

def send_command(serial, command, human, expected_response=None, crlf=True, waittime=1000):
	'''Sends a command to the device.'''

	# flush the read buffer
	serial.read(serial.inWaiting())
	
	print human
	print "<<< " + command
	serial.write(command)
	if crlf:
		serial.write("\r\n")

	time.sleep(waittime / 1000)

	if expected_response:
		n = max([serial.inWaiting(), len(expected_response)])
		#print n
		data = serial.read(n)
		#print data
		if expected_response in data:
			print ">>> OK"
		else:
			print "<<< " + command
			print ">>> ERROR: " + data

def question(question, default):
	ans = raw_input(question + " (default: " + default + ") ")
	if ans == "":
		ans = default
	return ans

def configure_base_station():
	print "=" * 80
	print "Now configuring the base station..."
	path = question("Path?", "/dev/ttyS0")
	print "Base Station At: " + path 
	baud = question("Baud?", "9600")
	print "Using baud " + baud

	source_ser = serial.Serial(path, int(baud))

	send_command(source_ser, "FRESET", "Resetting Device Firmware", "<OK", waittime=3000)
	send_command(source_ser, "SERIALCONFIG COM2 9600 N 8 1 N OFF", "Setting up COM2", "[COM1]", waittime=3000)
	send_command(source_ser, "INTERFACEMODE COM2 NONE RTCA OFF", "Setting up COM2 DGPS", "<OK")
	send_command(source_ser, "POSAVE 0.1 0.05 0.05", "Setting up COM2 DGPS Position", "<OK")
	send_command(source_ser, "LOG COM2 RTCAOBS ONTIME 2", "Setting up COM2 DGPS RTCAOBS", "<OK")
	send_command(source_ser, "LOG COM2 RTCAREF ONTIME 10", "Setting up COM2 DGPS RTCAREF", "<OK")
	send_command(source_ser, "LOG COM2 RTCA1 ONTIME 10 3", "Setting up COM2 DGPS RTCA1", "<OK")
	send_command(source_ser, "LOG COM2 RTCAEPHEM ONTIME 10 7", "Setting up COM2 DGPS RTCAEPHEM", "<OK")
	send_command(source_ser, "SAVECONFIG", "Saving Configuration", "<OK", waittime = 3000)
	source_ser.close()

def configure_rover():
	print "=" * 80
	print "Now configuring the rover..."
	path = question("Path?", "/dev/ttyS0")
	print "Base Station At: " + path 
	baud = question("Baud?", "9600")
	print "Using baud " + baud

	source_ser = serial.Serial(path, int(baud))

	send_command(source_ser, "FRESET", "Resetting Device Firmware", "<OK", waittime=3000)
	send_command(source_ser, "SERIALCONFIG COM2 9600 N 8 1 N OFF", "Setting up COM2", "[COM1]", waittime=3000)
	send_command(source_ser, "INTERFACEMODE COM2 RTCA NONE OFF", "Setting up COM2 DGPS", "<OK")
	send_command(source_ser, "SAVECONFIG", "Saving", waittime = 3000)
	send_command(source_ser, "COM COM1 38400 N 8 1 N OFF ON", "Setting new baud for com1", waittime=3000)
	source_ser.close()
	
	source_ser = serial.Serial(path, 38400)
	send_command(source_ser, "SAVECONFIG", "Saving configuration", "<OK")	
	source_ser.close()

if __name__ == "__main__":
	if "y" in question("Configure Base Station?", "Yes").lower():
		configure_base_station()
	if "y" in question("Configure Rover?", "Yes").lower():
		configure_rover()
