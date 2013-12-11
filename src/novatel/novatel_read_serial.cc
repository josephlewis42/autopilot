/*******************************************************************************
 * Copyright 2012 Bryan Godbolt
 * Copyright 2013 Joseph Lewis <joehms22@gmail.com>
 *
 * This file is part of ANCL Autopilot.
 *
 *     ANCL Autopilot is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     ANCL Autopilot is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License
 *     along with ANCL Autopilot.  If not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************/

#include "novatel_read_serial.h"
#include "Configuration.h"

/* File Handling Headers */
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/* STL Headers */
#include <bitset>

/* C Headers */
#include <termios.h>
#include <math.h>
#include <stdint.h>

/* Boost Headers */
#include <boost/thread.hpp>

/* Project Headers */
#include "Debug.h"
#include "init_failure.h"
#include "heli.h"
#include "MainApp.h"
#include "qnx2linux.h"


//#define NDEBUG

// Constant definitions
const double GPS::ReadSerial::OEM6_LOG_20_HZ = 0.05;
const double GPS::ReadSerial::OEM6_LOG_10_HZ = 0.1;
const double GPS::ReadSerial::OEM6_LOG_5_HZ = 0.2;
const double GPS::ReadSerial::OEM6_LOG_4_HZ = .25;
const double GPS::ReadSerial::OEM6_LOG_2_HZ = 0.5;
const uint8_t GPS::ReadSerial::HEADER_SYNC_BYTES[] = {0xAA, 0x44, 0x12};
const uint8_t GPS::ReadSerial::HEADER_SYNC_BYTES_LENGTH = 3;
const uint8_t GPS::ReadSerial::HEADER_LENGTH_BYTES = 25;


int GPS::ReadSerial::readSerialBytes(int fd, void * buf, int n)
{
#ifdef __QNX__
	return readcond(fd, buf, n, n, 10, 10);
#else
	return QNX2Linux::readcond(fd, buf, n, n, 10,10);
#endif
}

/* read_serial functions */
GPS::ReadSerial::ReadSerial()
{
}

void GPS::ReadSerial::operator()()
{
	if(!Configuration::getInstance()->getb(GPS::GPS_ENABLED, GPS::GPS_ENABLED_DEFAULT))
	{
		warning() << "NovAtel disabled!";
		return;
	}

	debug() << "Initialize the NovAtel serial port";
	if(initPort())
	{
		boost::this_thread::sleep(boost::posix_time::seconds(1));
		//	send_log_command();
		readPort();
	}
	else
	{
		warning() << "Could not init NovAtel";
		MainApp::terminate();
	}

	send_unlog_command();
}

bool GPS::ReadSerial::initPort()
{
	std::string serial_port = Configuration::getInstance()->gets(GPS::GPS_SERIAL_PORT_CONFIGURATION_NAME, GPS::GPS_SERIAL_PORT_CONFIGURATION_DEFAULT);
	fd_ser = open(serial_port.c_str(), O_RDWR | O_NOCTTY);

	if(-1 == fd_ser)
	{
		return false;
	}


	// Set up the terminal configuration for the given port.
	struct termios port_config;

	tcgetattr(fd_ser, &port_config);                  // get the current port settings

	// Set the baud rate
	cfsetospeed(&port_config, B38400);
	cfsetispeed(&port_config, B38400);

	// Set the number of data bits
	port_config.c_cflag &= ~(CSIZE);                  // Set terminal data length.
	port_config.c_cflag |=  CS8;                      // 8 data bits

	// Set the number of stop bits to 1
	port_config.c_cflag &= ~(CSTOPB);                 // clear for one stop bit

	// Set parity to none
	port_config.c_cflag &= ~(PARENB );        // Set terminal parity.

	//set for non-canonical (raw processing, no echo, etc.)
	port_config.c_iflag = IGNPAR; // ignore parity check close_port(int
	port_config.c_oflag = 0; // raw output
	port_config.c_lflag = 0; // raw input


	port_config.c_cflag |= (CLOCAL | CREAD);          // Enable the receiver and set local mode...

	// Clear terminal output flow control.
	if (tcsetattr(fd_ser, TCSADRAIN, &port_config) != 0)
	{
		critical() << "NovAtel could not set serial port attributes";
	}

	if(tcflush(fd_ser, TCIOFLUSH) == -1)
	{
		critical() << "could not purge the NovAtel serial port";
		throw init_failure("Could not purge the NovAtel serial port");
	}

#ifndef NDEBUG
	debug() << "NovAtel initialized on " << serial_port;
#endif

	return true;
}

bool GPS::ReadSerial::synchronize()
{
	uint8_t currentByte;
	int position = 0;

	last_data = boost::posix_time::second_clock::local_time();

	while(!GPS::getInstance()->terminateRequested())
	{
		// Check for overall timeout.
		if ((boost::posix_time::second_clock::local_time() - last_data).total_seconds() > 10)
		{
			warning() << "NovAtel: Stopped receiving data, attempting restart.";
			last_data = boost::posix_time::second_clock::local_time();
			send_unlog_command();
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
			setupLogging();
		}

		// Check for local timeout
		if(readSerialBytes(fd_ser, &currentByte, 1) < 1)
		{
			continue; // timed out reading bytes.
		}

		// Check for byte matches.
		if(currentByte == HEADER_SYNC_BYTES[position])
		{
			position++;

			if(position == HEADER_SYNC_BYTES_LENGTH)
			{
				return true;
			}

			continue;
		}

		// not a matching byte, start looking for a new header
		position = 0;
	}

	return false;

}


void GPS::ReadSerial::readPort()
{
	setupLogging();
	while(synchronize())
	{
		// Read the header
		std::vector<uint8_t> header(HEADER_LENGTH_BYTES);
		int bytes = readSerialBytes(fd_ser, &header[0], HEADER_LENGTH_BYTES);
		if (bytes < HEADER_LENGTH_BYTES)
		{
			warning() << "NovAtel: Received valid sync bytes, but could not read header";
			continue;
		}

#ifndef NDEBUG
		debug() << "NovAtel: Received Header: " << header;
#endif
		int data_size = raw_to_int<uint16_t>(header.begin() + 5);
		std::vector<uint8_t> log_data(data_size);
		bytes = readSerialBytes(fd_ser, &log_data[0], data_size);
		if (bytes < data_size)
		{
			warning() << "NovAtel: Received header, but could not receive data log.";
			continue;
		}
#ifndef NDEBUG
		debug() << "NovAtel: Received Message: " << log_data;
#endif
		int checksum_size = 4;
		std::vector<uint8_t> checksum(checksum_size);
		bytes = readSerialBytes(fd_ser, &checksum[0], checksum_size);
		if (bytes < checksum_size)
		{
			warning() << "NovAtel: received log data but could not receive checksum.";
			continue;
		}

		std::vector<uint8_t> whole_message;

		for(int i = 0; i < HEADER_SYNC_BYTES_LENGTH; i++)
		{
			whole_message += HEADER_SYNC_BYTES[i];
		}

		whole_message.insert(whole_message.end(), header.begin(), header.end());
		whole_message.insert(whole_message.end(), log_data.begin(), log_data.end());

		std::vector<uint8_t> computed_checksum(compute_checksum(whole_message));
		if (checksum != computed_checksum)
		{
			warning() << "NovAtel: received complete message but checksum was invalid";
#ifndef NDEBUG
			debug() << "NovAtel: checksum: " << checksum << ", computed checksum: " << computed_checksum;
#endif
			continue;
		}

		uint16_t message_id = raw_to_int<uint16_t>(header.begin() + 1);
		if (is_response(header))
		{
			debug() << "NovAtel: response message: " << std::string(log_data.begin() + 4, log_data.end());
		}
		switch (message_id)
		{
		case OEM6_COMMAND_LOG: // log command (response)
			if (is_response(header))
			{
				switch(parse_enum(log_data))
				{
				case OEM6_OK:
					message() << "NovAtel: data logging successfully initialized";
					break;
				case OEM6_INVALID_CHECKSUM:
					warning() << "NovAtel: checksum failure";
					break;
				default:
					warning() << "NovAtel: RX Message: \"" << std::string(log_data.begin() + 4, log_data.end()) << '"';
				}
			}
		case OEM6_LOG_RTKXYZ:  // RTKXYZ
			if (!is_response(header))
			{
#ifndef NDEBUG
				debug() << "NovAtel: Received data";
#endif
				std::vector<double> log;
				parse_header(header, log);
				parse_log(log_data, log);
				last_data = boost::posix_time::second_clock::local_time();
				GPS::getInstance()->gps_updated();
				LogFile::getInstance()->logData(heli::LOG_NOVATEL_GPS, log);
			}
			break;

		default:
			warning() << "NovAtel: Received unexpected message id: " << message_id;
			continue;
		}
	}
}

bool GPS::ReadSerial::is_response(const std::vector<uint8_t>& header)
{
	std::bitset<8> message_type(header[3]);
	return message_type[7];
}

void GPS::ReadSerial::parse_header(const std::vector<uint8_t>& header, std::vector<double>& log)
{
	std::vector<uint8_t>::const_iterator it = header.begin() + 10;
	uint32_t time_status = *it;
#ifndef NDEBUG
	debug() << "NovAtel: time status: " << time_status;
#endif
	log += time_status;
	it += 1;
	uint16_t week = raw_to_int<uint16_t>(it);
	log += week;
	it += 2;
	uint32_t milliseconds = raw_to_int<uint32_t>(it);
	log += milliseconds;
	GPS::getInstance()->set_gps_time(gps_time(week, milliseconds, static_cast<gps_time::TIME_STATUS>(time_status)));
}

void GPS::ReadSerial::parse_log(const std::vector<uint8_t>& data, std::vector<double>& log)
{
	GPS& gps = *GPS::getInstance();

	uint pos_status = parse_enum(data);
	log += pos_status;
	gps.set_position_status(pos_status);

	uint pos_type = parse_enum(data, 4);
	log += pos_type;
	gps.set_position_type(pos_type);

#ifndef NDEBUG
	debug() << "NovAtel: Pos type: " << pos_type;
#endif

	blas::vector<double> position(parse_3floats<double>(data, 8));

	#ifndef NDEBUG
	debug() << "NovAtel: ecef position: " << position;
#endif
	log.insert(log.end(), position.begin(), position.end());
	blas::vector<double> llh(ecef_to_llh(position));
	gps.set_llh_position(llh);

#ifndef NDEBUG
	debug() << "NovAtel: llh: " << llh;
#endif
	blas::vector<float> position_error(parse_3floats<float>(data, 32));
	log.insert(log.end(), position_error.begin(), position_error.end());
	gps.set_pos_sigma(ecef_to_ned(position_error, llh));

	uint vel_status = parse_enum(data, 44);
	log += vel_status;
	gps.set_velocity_status(vel_status);

	uint vel_type = parse_enum(data, 48);
	log += vel_type;
	gps.set_velocity_type(vel_type);

	blas::vector<double> velocity(parse_3floats<double>(data, 52));
	log.insert(log.end(), velocity.begin(), velocity.end());
	gps.set_ned_velocity(ecef_to_ned(velocity, llh));

	blas::vector<float> velocity_error(parse_3floats<float>(data, 76));
	log.insert(log.end(), velocity_error.begin(), velocity_error.end());
	gps.set_vel_sigma(ecef_to_ned(velocity_error, llh));

	uint8_t num_sats = data[104];
	log += num_sats;
	gps.set_num_sats(num_sats);
}

uint GPS::ReadSerial::parse_enum(const std::vector<uint8_t>& log, int offset)
{
	return raw_to_int<uint32_t>(log.begin() + offset);
}

blas::vector<double> GPS::ReadSerial::ecef_to_llh(const blas::vector<double>& ecef)
{
	/* Transformation from ECEF [x,y,z] to geodetic [phi,lamda,h] coordinates using Jay A. Farrel algorithm p.34 */
	// ECEF2GEO Parameters
	blas::vector<double> llh(3);
	llh.clear();

	const double a = 6378137.0;
	const double f = 1.0/298.257223563;
	const double e = sqrt(f*(2-f));

	// Initialization
	double RN = a;
	double p=sqrt(pow(ecef[0],2)+pow(ecef[1],2));
	double prev_h = 0;
	double sin_phi = 0;
	double phi = 0;

	// Iteration
	do
	{
		prev_h = llh[2];
		sin_phi = ecef[2]/((1 - pow(e,2))*RN + llh[2]);
		phi = atan((ecef[2] + pow(e,2)*RN*sin_phi)/p);
		RN = a/sqrt(1 - pow(e,2)*pow(sin(phi),2));
		llh[2] = p/cos(phi)-RN;
	}
	while(abs(llh[2] - prev_h)>0.000001);

	// Saving results (origin coordinates)
	llh[0] = phi; //latitude
	llh[1] = atan2(ecef[1], ecef[0]); //longitude

	return llh;
}

void GPS::ReadSerial::_genericLog(OEM6_PORT_IDENTIFIER port, OEM6_LOG message, OEM6_LOG_TRIGGERS trigger, double period)
{
#ifndef NDEBUG
	debug() << "NovAtel: [Log msg: " << message << ", trigger: " << trigger << ", period: " << (1/period) << "HZ, on: " << port << "]";
#endif
	// generate header
	std::vector<uint8_t> command(generate_header(OEM6_COMMAND_LOG, 32));

	// append port number
	std::vector<uint8_t> portBuffer(int_to_raw(port));
	command.insert(command.end(), portBuffer.begin(), portBuffer.end());

	// append message id
	std::vector<uint8_t> idBuffer(int_to_raw(static_cast<uint16_t>(message)));
	command.insert(command.end(), idBuffer.begin(), idBuffer.end());

	// append
	command += 0, 0;

	// append trigger
	std::vector<uint8_t> triggerBuffer(int_to_raw(trigger));
	command.insert(command.end(), triggerBuffer.begin(), triggerBuffer.end());

	// append period
	std::vector<uint8_t> periodBuffer(float_to_raw(period)); // send every 1/4 second
	command.insert(command.end(), periodBuffer.begin(), periodBuffer.end());

	// append offset
	std::vector<uint8_t> offset(float_to_raw(0.0));
	command.insert(command.end(), offset.begin(), offset.end());

	// four empty bytes
	command.insert(command.end(), 4, 0);

	std::vector<uint8_t> checksum(compute_checksum(command));
	command.insert(command.end(), checksum.begin(), checksum.end());

#ifndef NDEBUG
	debug() << "NovAtel: sent log command: " << std::hex << command;
#endif

	/* Send out the command */
	write(fd_ser, &command[0], command.size());
}

void GPS::ReadSerial::send_unlog_command()
{
	_genericUnlog(OEM6_LOG_RTKXYZ);
}

void GPS::ReadSerial::_genericUnlog(OEM6_LOG message)
{
#ifndef NDEBUG
	debug() << "NovAtel: Unlogging message: " << message;
#endif
	std::vector<uint8_t> command(generate_header(OEM6_COMMAND_UNLOG, 8));

	std::vector<uint8_t> port(int_to_raw(OEM6_PORT_THISPORT));
	command.insert(command.end(), port.begin(), port.end());

	std::vector<uint8_t> id(int_to_raw(static_cast<uint16_t>(message)));
	command.insert(command.end(), id.begin(), id.end());

	command+= 0,0;

	std::vector<uint8_t> checksum(compute_checksum(command));
	command.insert(command.end(), checksum.begin(), checksum.end());

	write(fd_ser, &command[0], command.size());
}

std::vector<uint8_t> GPS::ReadSerial::generate_header(uint16_t message_id, uint16_t message_length)
{
	std::vector<uint8_t> header;
	for(int i = 0; i < HEADER_SYNC_BYTES_LENGTH; i++)
	{
		header += HEADER_SYNC_BYTES[i];
	}

	header += 0x00;
	std::vector<uint8_t> id(int_to_raw(message_id));
	header.insert(header.end(), id.begin(), id.end());
	header += 0x00, 192;

	std::vector<uint8_t> length(int_to_raw(message_length));
	header.insert(header.end(), length.begin(), length.end());

	header.insert(header.end(), 18, 0);
	header[3] = header.size();
	return header;
}

void GPS::ReadSerial::setupLogging()
{
	_genericLog(OEM6_PORT_THISPORT, OEM6_LOG_RTKXYZ, ONTIME, OEM6_LOG_4_HZ);
}

std::vector<uint8_t> GPS::ReadSerial::compute_checksum(const std::vector<uint8_t>& message)
{
	unsigned long ulTemp1;
	unsigned long ulTemp2;
	unsigned long ulCRC = 0;

	for (size_t i=0; i < message.size(); i++)
	{
		ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
		ulTemp2 = CRC32Value( ((int) ulCRC ^ message[i] ) & 0xff );
		ulCRC = ulTemp1 ^ ulTemp2;
	}

	return(int_to_raw(ulCRC));
}

unsigned long GPS::ReadSerial::CRC32Value(int i)
{
	int j;
	unsigned long ulCRC;

	ulCRC = i;
	for ( j = 8 ; j > 0; j-- )
	{
		if ( ulCRC & 1 )
		{
			ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
		}
		else
		{
			ulCRC >>= 1;
		}
	}
	return ulCRC;
}
