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

#include "GPS.h"

// Project Headers
#include "novatel_read_serial.h"
#include "MainApp.h"

// Boost Headers
#include <boost/bind.hpp>

GPS* GPS::_instance = NULL;
boost::mutex GPS::_instance_lock;

/// path to serial device connected to Novatel
std::string GPS::GPS_SERIAL_PORT_CONFIGURATION_NAME = "gps_serial";
std::string GPS::GPS_SERIAL_PORT_CONFIGURATION_DEFAULT = "/dev/ser1";

const std::string GPS_LOGFILE_HEADER = "Time_Status Week Milliseconds P-sol_status pos_type P-X P-Y P-Z P-X_stddev P-Y_stddev P-Z_stddev "
		"V-sol_status vel_type V-X V-Y V-Z V-X_stddev V-Y_stddev V-Z_stddev "
		"#obs";

const std::string READ_GPS_THREAD_NAME = "Novatel GPS";


GPS* GPS::getInstance()
{
	boost::mutex::scoped_lock lock(_instance_lock);

	if (!_instance)
	{
		_instance = new GPS();
	}

	return _instance;
}


GPS::GPS()
: read_serial_thread(ReadSerial())
{
	LogFile::getInstance()->logHeader(heli::LOG_NOVATEL_GPS, GPS_LOGFILE_HEADER);

	MainApp::add_thread(&read_serial_thread, READ_GPS_THREAD_NAME);
}


GPS::~GPS()
{
}
