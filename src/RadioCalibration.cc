/**************************************************************************
 * Copyright 2012 Bryan Godbolt, Hasitha Senanayake, Aakash Vasudevan
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

#include "RadioCalibration.h"
#include "Configuration.h"


RadioCalibration::RadioCalibration()
{
  gyro[0] = 1050;
  gyro[1] = 1800;

  aileron[0] = 1000;
  aileron[1] = 1500;
  aileron[2] = 2000;

  elevator[0] = 1000;
  elevator[1] = 1500;
  elevator[2] = 2000;

  rudder[0] = 1000;
  rudder[1] = 1500;
  rudder[2] = 2000;

  throttle[0] = 1000;
  throttle[1] = 1250;
  throttle[2] = 1500;
  throttle[3] = 1750;
  throttle[4] = 2000;

  pitch[0] = 1000;
  pitch[1] = 1250;
  pitch[2] = 1500;
  pitch[3] = 1750;
  pitch[4] = 2000;

  flightMode[0] = 1000;
  flightMode[1] = 1500;
  flightMode[2] = 1620;

  loadFile();
}

RadioCalibration* RadioCalibration::_instance = NULL;

RadioCalibration* RadioCalibration::getInstance()
{
  if(!_instance)
      _instance = new RadioCalibration;

  return _instance;
}


// FIXME See if we can replace the indexes here with the values from Channel in heli.h - Joseph
void RadioCalibration::setCalibration(const std::vector<std::vector<uint16_t> >& calibration_data)
{
	setAileron(calibration_data[0]);
	setElevator(calibration_data[1]);
	setThrottle(calibration_data[2]);
	setRudder(calibration_data[3]);
	setGyro(calibration_data[4]);
	setPitch(calibration_data[5]);

	saveFile();
}


// FIXME, there are lots of subtly different methods here perhaps there should be a template. - Joseph
boost::array<uint16_t, 3> RadioCalibration::getAileron()
{
	calibration_lock.lock();
	boost::array<uint16_t, 3> ail(aileron);
	calibration_lock.unlock();
	return ail;
}

boost::array<uint16_t, 3> RadioCalibration::getElevator()
{
	calibration_lock.lock();
	boost::array<uint16_t, 3> ele(elevator);
	calibration_lock.unlock();
	return ele;
}

boost::array<uint16_t, 3> RadioCalibration::getRudder()
{
	calibration_lock.lock();
	boost::array<uint16_t, 3> rud(rudder);
	calibration_lock.unlock();
	return rud;
}


boost::array<uint16_t, 5> RadioCalibration::getThrottle()
{
	calibration_lock.lock();
	boost::array<uint16_t, 5> thr(throttle);
	calibration_lock.unlock();
	return thr;
}

boost::array<uint16_t, 5> RadioCalibration::getPitch()
{
	calibration_lock.lock();
	boost::array<uint16_t, 5> pit(pitch);
	calibration_lock.unlock();
	return pit;
}

boost::array<uint16_t, 2> RadioCalibration::getGyro()
{
	calibration_lock.lock();
	boost::array<uint16_t, 2> gyr(gyro);
	calibration_lock.unlock();
	return gyr;
}

boost::array<uint16_t, 3> RadioCalibration::getFlightMode()
{
        calibration_lock.lock();
        boost::array<uint16_t, 3> flight_Mode(flightMode);
        calibration_lock.unlock();
        return flight_Mode;
}

void RadioCalibration::setAileron(const boost::array<uint16_t, 3>& setpoints)
{
	calibration_lock.lock();
	aileron = setpoints;
	calibration_lock.unlock();
}

void RadioCalibration::setAileron(const std::vector<uint16_t>& setpoints)
{
	calibration_lock.lock();
	for (int i=0; i<3; i++)
		aileron[i] = setpoints[i];
	calibration_lock.unlock();
}

void RadioCalibration::setElevator(const boost::array<uint16_t, 3>& setpoints)
{
	calibration_lock.lock();
	elevator = setpoints;
	calibration_lock.unlock();
}

void RadioCalibration::setElevator(const std::vector<uint16_t>& setpoints)
{
	calibration_lock.lock();
	for (int i=0; i<3; i++)
		elevator[i] = setpoints[i];
	calibration_lock.unlock();
}

void RadioCalibration::setThrottle(const boost::array<uint16_t, 5>& setpoints)
{
	calibration_lock.lock();
	throttle = setpoints;
	calibration_lock.unlock();
}

void RadioCalibration::setThrottle(const std::vector<uint16_t>& setpoints)
{
	calibration_lock.lock();
	for(int i=0; i<5; i++)
	{
		throttle[i]=setpoints[i];
	}
	calibration_lock.unlock();
}

void RadioCalibration::setRudder(const boost::array<uint16_t, 3>& setpoints)
{
	calibration_lock.lock();
	rudder = setpoints;
	calibration_lock.unlock();
}

void RadioCalibration::setRudder(const std::vector<uint16_t>& setpoints)
{
	calibration_lock.lock();
	for (int i=0; i<3; i++)
		rudder[i] = setpoints[i];
	calibration_lock.unlock();
}

void RadioCalibration::setGyro(const boost::array<uint16_t, 2>& setpoints)
{
	calibration_lock.lock();
	gyro = setpoints;
	calibration_lock.unlock();
}

void RadioCalibration::setGyro(const std::vector<uint16_t>& setpoints)
{
	calibration_lock.lock();
	for (int i=0; i<2; i++)
		gyro[i]=setpoints[i];
	calibration_lock.unlock();
}

void RadioCalibration::setPitch(const boost::array<uint16_t, 5>& setpoints)
{
	calibration_lock.lock();
	pitch = setpoints;
	calibration_lock.unlock();
}

void RadioCalibration::setPitch(const std::vector<uint16_t>& setpoints)
{
	calibration_lock.lock();
	for(int i=0; i<5; i++)
	{
		pitch[i]=setpoints[i];
	}
	calibration_lock.unlock();
}

void RadioCalibration::loadFile()
{
	Configuration* config = Configuration::getInstance();
    std::vector<uint16_t> setpoints;

    // set the gyro
    parseDelimiter(config->gets("channels.two.gyro.value","1050, 1800"), setpoints);
    setGyro(setpoints);
    setpoints.clear();

    // set the aileron
    parseDelimiter(config->gets("channels.three.aileron.value","1000, 1500, 2000"), setpoints);
	setAileron(setpoints);
	setpoints.clear();

	// set the elevator
	parseDelimiter(config->gets("channels.three.elevator.value","1000, 1500, 2000"), setpoints);
	setElevator(setpoints);
	setpoints.clear();

	// set the rudder
	parseDelimiter(config->gets("channels.three.rudder.value","1000, 1500, 2000"), setpoints);
	setRudder(setpoints);
	setpoints.clear();

	// set the throttle
	parseDelimiter(config->gets("channels.five.throttle.value","1000, 1250, 1500, 1750, 2000"), setpoints);
	setThrottle(setpoints);
	setpoints.clear();

	// set the pitch
	parseDelimiter(config->gets("channels.five.pitch.value","1000, 1250, 1500, 1750, 2000"), setpoints);
	setPitch(setpoints);
	setpoints.clear();
}

/**
 * Parses a string delimited by commas in to a uint16_t array
 */
void RadioCalibration::parseDelimiter(std::string input, std::vector<uint16_t>& output)
{
	std::vector<std::string> strs;
	boost::split(strs, input, boost::is_any_of(","));
	for (size_t i = 0; i < strs.size(); i++)
	{
		boost::algorithm::trim(strs[i]);
		output.push_back(boost::lexical_cast<uint16_t>(strs[i]));
	}
}

void RadioCalibration::saveFile()
{
	Configuration* config = Configuration::getInstance();

	// Aileron
	config->set("channels.three.aileron.value", toString(getAileron()));
	config->set("channels.three.aileron.number", "1");

	// Elevator
	config->set("channels.three.elevator.value", toString(getElevator()));
	config->set("channels.three.elevator.number", "2");

	// Throttle
	config->set("channels.five.throttle.value", toString(getThrottle()));
	config->set("channels.five.throttle.number", "3");

	// Rudder
	config->set("channels.three.rudder.value", toString(getRudder()));
	config->set("channels.three.rudder.number", "4");

	// Gyro
	config->set("channels.two.gyro.value",toString(getGyro()));
	config->set("channels.two.gyro.number","5");

	// Pitch
	config->set("channels.five.pitch.value", toString(getGyro()));
	config->set("channels.five.pitch.value", "6");
}

