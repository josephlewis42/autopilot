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
#include "LogFile.h"

#include <mutex>


// Boost Headers
#include <boost/bind.hpp>

GPS* GPS::_instance = NULL;
std::mutex GPS::_instance_lock;

/// path to serial device connected to Novatel
std::string GPS::GPS_SERIAL_PORT_CONFIGURATION_NAME = "novatel.serial_port";
std::string GPS::GPS_SERIAL_PORT_CONFIGURATION_DEFAULT = "/dev/ser1";
std::string GPS::GPS_ENABLED = "novatel.enabled";

std::string GPS::LOG_NOVATEL_GPS = "Novatel GPS (Invalid Solutions Removed)";
std::string GPS::LOG_NOVATEL_GPS_ALL = "Novatel GPS (All Measurements)";


bool GPS::GPS_ENABLED_DEFAULT = true;

const std::string GPS_LOGFILE_HEADER = "Time_Status Week Milliseconds P-sol_status pos_type P-X P-Y P-Z P-X_stddev P-Y_stddev P-Z_stddev "
		"V-sol_status vel_type V-X V-Y V-Z V-X_stddev V-Y_stddev V-Z_stddev "
		"#obs";

const std::string READ_GPS_THREAD_NAME = "Novatel GPS";


GPS* GPS::getInstance()
{
	std::lock_guard<std::mutex> lock(_instance_lock);

	if (!_instance)
	{
		_instance = new GPS();
	}

	return _instance;
}


GPS::GPS()
:Driver("NovAtel GPS","novatel"),
 read_serial_thread(ReadSerial()),
 llh_position(blas::vector<double>(0,3)),
 ned_velocity(blas::vector<double>(0,3)),
 pos_sigma(blas::vector<double>(0,3)),
 vel_sigma(blas::vector<double>(0,3))
{
	trace() << "Generating log headers";
	LogFile::getInstance()->logHeader(LOG_NOVATEL_GPS, GPS_LOGFILE_HEADER);

	trace() << "Adding read serial thread";
	MainApp::add_thread(&read_serial_thread, READ_GPS_THREAD_NAME);
}


GPS::~GPS()
{
}
