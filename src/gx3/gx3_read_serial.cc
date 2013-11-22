/**************************************************************************
 * Copyright 2012 Bryan Godbolt
 * Copyright 2013 Joseph Lewis <joehms22@gmail.com> | <joseph@josephlewis.net>
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
 *************************************************************************/

#include "gx3_read_serial.h"

/* STL Headers */
#include <vector>

/* C headers */
#include <stdint.h>

/* Project Headers */
#include "Debug.h"
#include "qnx2linux.h"

/**
 * Constants
 */
static const uint8_t FIRST_SYNC_BYTE = 0x75;
static const uint8_t SECOND_SYNC_BYTE = 0x65;
static const int SECONDS_UNTIL_ASSUMED_DEAD = 3;
static const int HEADER_LENGTH_BYTES = 4;
static const int MAX_PAYLOAD_SIZE_BYTES = 256 + HEADER_LENGTH_BYTES; // the gx3 specs a 1 byte length field so this should be enough for max packet length including header
static const int CHECKSUM_LENGTH_BYTES = 2;

int IMU::read_serial::read_ser(int fd, void * buf, int n)
{
#ifdef __QNX__
	return readcond(fd, buf, n, n, 10, 10);
#else
	return QNX2Linux::readcond(fd, buf, n, n, 10,10);
#endif
}


void IMU::read_serial::check_alive()
{
	IMU* imu = IMU::getInstance();
	// check to see if the sensor is "dead"
	if (imu->seconds_since_last_data() < SECONDS_UNTIL_ASSUMED_DEAD)
	{
		return;
	}

	warning() << "Stopped receiving data from GX3.  Attempting to reinitialize serial connection";
	imu->initialize_imu();
	imu->set_last_data();
}


bool IMU::read_serial::sync()
{
	uint8_t last_byte = 0, curr_byte = 0;
	IMU* imu = IMU::getInstance();
	int fd_ser = imu->fd_ser;

	while(true)
	{
		check_alive();

		// Check to see if we have been requested to terminate
		if(imu->terminateRequested())
		{
			return false;
		}

		if (read_ser(fd_ser, &curr_byte, 1) < 1)
		{
			continue;
		}

		if(last_byte == FIRST_SYNC_BYTE && curr_byte == SECOND_SYNC_BYTE)
		{
			imu->set_last_data();
			return true;
		}
		else
		{
			//debug() << "IMU::read_serial: got useless bytes: " << std::hex << last_byte << ", " << curr_byte;
		}
		last_byte = curr_byte;
	}
}

void IMU::read_serial::operator()()
{
	IMU* imu = IMU::getInstance();
	std::vector<uint8_t> header(HEADER_LENGTH_BYTES,0);
	header[0] = FIRST_SYNC_BYTE;
	header[1] = SECOND_SYNC_BYTE;
	std::vector<uint8_t> buffer;
	buffer.reserve(MAX_PAYLOAD_SIZE_BYTES);
	std::vector<uint8_t> checksum(CHECKSUM_LENGTH_BYTES, 0);
	const int fd_ser = IMU::getInstance()->fd_ser;
	imu->set_last_data();
	while (true)
	{
			// if a serious error has happened (can't sync), kill the thread.
			if(sync() == false)
			{
				return;
			}

			// got both sync bytes - get message
			uint8_t descriptor = 0, length = 0;
			if (read_ser(fd_ser, &descriptor, 1) < 1)
			{
				continue;
			}

			if (read_ser(fd_ser, &length, 1) < 1)
			{
				continue;
			}

			buffer.resize(length);
			if (read_ser(fd_ser, &buffer[0], length) < length)
			{
				warning() << "Received valid message header from IMU but did not receive payload.";
				continue;
			}

			if (read_ser(fd_ser, &checksum[0], CHECKSUM_LENGTH_BYTES) < CHECKSUM_LENGTH_BYTES)
			{
				continue;
			}

			// successfully received entire message - compare checksum
			header[2] = descriptor;
			header[3] = length;
			buffer.insert(buffer.begin(), header.begin(), header.end());
			if (checksum != IMU::compute_checksum(buffer))
			{
				warning() << "IMU checksum failure.  message checksum: " << std::hex << checksum[0] << checksum[1] << "computed checksum: " << IMU::compute_checksum(buffer) ;
				continue;
			}

			// got message, checksum passed, queue it up
			switch (descriptor)
			{
			case COMMAND_BASE:
			case COMMAND_3DM:
			case COMMAND_NAV_FILT:
			case COMMAND_SYS:
			{
				//debug() << "Received command message.";
				imu->command_queue_push(buffer);
				break;
			}
			case DATA_AHRS:
			{
				//debug() << "Received ahrs message";
				imu->ahrs_queue_push(buffer);
				break;
			}
			case DATA_GPS:
			{
				//debug() << "Received GPS message";
				imu->gps_queue_push(buffer);
				break;
			}
			case DATA_NAV:
			{
				//debug() << "Got Nav Message";
				imu->nav_queue_push(buffer);
				break;
			}
			default:
				warning() << "Unknown command received from GX3.  Cannot add it to a queue";
				break;
			}
	}
}
