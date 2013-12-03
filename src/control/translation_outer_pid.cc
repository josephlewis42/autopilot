/**************************************************************************
 * Copyright 2012 Bryan Godbolt
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

#include "translation_outer_pid.h"

/* Project Headers */
#include "IMU.h"
#include "Control.h"
#include "Configuration.h"

// constants
std::string translation_outer_pid::XML_TRANSLATION_X_PROPORTIONAL = "controller_params.translation_outer_pid.x.proportional";
std::string translation_outer_pid::XML_TRANSLATION_Y_PROPORTIONAL = "controller_params.translation_outer_pid.y.proportional";
std::string translation_outer_pid::XML_TRANSLATION_X_DERIVATIVE = "controller_params.translation_outer_pid.x.derivative";
std::string translation_outer_pid::XML_TRANSLATION_Y_DERIVATIVE = "controller_params.translation_outer_pid.y.derivative";
std::string translation_outer_pid::XML_TRANSLATION_X_INTEGRAL = "controller_params.translation_outer_pid.x.integral";
std::string translation_outer_pid::XML_TRANSLATION_Y_INTEGRAL = "controller_params.translation_outer_pid.y.integral";
std::string translation_outer_pid::XML_TRAVEL = "controller_params.translation_outer_pid.travel";



translation_outer_pid::translation_outer_pid()
: x(10),
  y(10),
  scaled_travel(15)
{
	x.name() = "X";
	y.name() = "Y";
}

translation_outer_pid::translation_outer_pid(const translation_outer_pid& other)
{
	{
		boost::mutex::scoped_lock lock(other.x_lock);
		x = other.x;
	}
	{
		boost::mutex::scoped_lock lock(other.y_lock);
		y = other.y;
	}
	{
		boost::mutex::scoped_lock lock(other.scaled_travel_lock);
		scaled_travel = other.scaled_travel;
	}
}



void translation_outer_pid::operator()(const blas::vector<double>& reference) throw(bad_control)
{
	// get attitude measurement
	IMU* imu = IMU::getInstance();
	blas::vector<double> euler(imu->get_euler());

	// get ned position/velocity
	blas::vector<double> position(imu->get_ned_position());
	blas::matrix<double> body_rotation(trans(IMU::euler_to_rotation(euler)));
	blas::vector<double> body_position_error(blas::prod(body_rotation, position - reference));
	blas::vector<double> body_velocity_error(blas::prod(body_rotation, imu->get_ned_velocity()));

	// roll pitch reference
	blas::vector<double> attitude_reference(2);
	attitude_reference.clear();
	std::vector<double> error_states;
	{
		boost::mutex::scoped_lock lock(x_lock);
		error_states.push_back(x.error().proportional() = body_position_error[0]);
		error_states.push_back(x.error().derivative() = body_velocity_error[0]);
		error_states.push_back(++(x.error()));
		attitude_reference[1] = -x.compute_pid();
	}
	{
		boost::mutex::scoped_lock lock(y_lock);
		error_states.push_back(y.error().proportional() = body_position_error[1]);
		error_states.push_back(y.error().derivative() = body_velocity_error[1]);
		error_states.push_back(++(y.error()));
		attitude_reference[0] = y.compute_pid();
	}

	LogFile::getInstance()->logData(heli::LOG_TRANS_PID_ERROR_STATES, error_states);

	Control::saturate(attitude_reference, scaled_travel_radians());

	// set the reference to a roll pitch orientation in radians
	set_control_effort(attitude_reference);
}

void translation_outer_pid::reset()
{
	for (boost::array<GPS_Filter, 3>::iterator it = pos_filters.begin(); it != pos_filters.end(); ++it)
		(*it).reset();
	for (boost::array<GPS_Filter, 3>::iterator it = vel_filters.begin(); it != vel_filters.end(); ++it)
		(*it).reset();

	x.reset();
	y.reset();
}

bool translation_outer_pid::runnable() const
{
	return true;
}

const std::string translation_outer_pid::PARAM_X_KP = "PID_X_KP";
const std::string translation_outer_pid::PARAM_X_KD = "PID_X_KD";
const std::string translation_outer_pid::PARAM_X_KI = "PID_X_KI";

const std::string translation_outer_pid::PARAM_Y_KP = "PID_Y_KP";
const std::string translation_outer_pid::PARAM_Y_KD = "PID_Y_KD";
const std::string translation_outer_pid::PARAM_Y_KI = "PID_Y_KI";

const std::string translation_outer_pid::PARAM_TRAVEL = "PID_TRAVEL";

std::vector<Parameter> translation_outer_pid::getParameters()
{
	std::vector<Parameter> plist;

	{
		boost::mutex::scoped_lock lock(x_lock);
		plist.push_back(Parameter(PARAM_X_KP, x.gains().proportional(), heli::CONTROLLER_ID));
		plist.push_back(Parameter(PARAM_X_KD, x.gains().derivative(), heli::CONTROLLER_ID));
		plist.push_back(Parameter(PARAM_X_KI, x.gains().integral(), heli::CONTROLLER_ID));
	}

	{
		boost::mutex::scoped_lock lock(y_lock);
		plist.push_back(Parameter(PARAM_Y_KP, y.gains().proportional(), heli::CONTROLLER_ID));
		plist.push_back(Parameter(PARAM_Y_KD, y.gains().derivative(), heli::CONTROLLER_ID));
		plist.push_back(Parameter(PARAM_Y_KI, y.gains().integral(), heli::CONTROLLER_ID));
	}
	scaled_travel_lock.lock();
	plist.push_back(Parameter(PARAM_TRAVEL, scaled_travel, heli::CONTROLLER_ID));
	scaled_travel_lock.unlock();
	return plist;
}

void translation_outer_pid::set_x_proportional(double kp)
{
	{
		boost::mutex::scoped_lock lock(x_lock);
		x.gains().proportional() = kp;
	}
	message() << "Set PID x proportional gain to: " << kp;
}

void translation_outer_pid::set_x_derivative(double kd)
{
	{
		boost::mutex::scoped_lock lock(x_lock);
		x.gains().derivative() = kd;
	}
	message() << "Set PID x derivative gain to: " << kd;
}

void translation_outer_pid::set_x_integral(double ki)
{
	{
		boost::mutex::scoped_lock lock(x_lock);
		x.gains().integral() = ki;
	}
	message() << "Set PID x integral gain to: " << ki;
}

void translation_outer_pid::set_y_proportional(double kp)
{
	{
		boost::mutex::scoped_lock lock(y_lock);
		y.gains().proportional() = kp;
	}
	message() << "Set PID y proportional gain to: " << kp;
}

void translation_outer_pid::set_y_derivative(double kd)
{
	{
		boost::mutex::scoped_lock lock(y_lock);
		y.gains().derivative() = kd;
	}
	message() << "Set PID y derivative gain to: " << kd;
}

void translation_outer_pid::set_y_integral(double ki)
{
	{
		boost::mutex::scoped_lock lock(y_lock);
		y.gains().integral() = ki;
	}
	message() << "Set PID y integral gain to: " << ki;
}

void translation_outer_pid::set_scaled_travel(double travel)
{
	{
		boost::mutex::scoped_lock lock(scaled_travel_lock);
		scaled_travel = travel;
	}
	message() << "Set travel to: " << travel;
}

double translation_outer_pid::get_x_proportional() const
{
	boost::mutex::scoped_lock lock(x_lock);
	return x.gains().proportional();
}

double translation_outer_pid::get_x_derivative() const
{
	boost::mutex::scoped_lock lock(x_lock);
	return x.gains().derivative();
}

double translation_outer_pid::get_x_integral() const
{
	boost::mutex::scoped_lock lock(x_lock);
	return x.gains().integral();
}

double translation_outer_pid::get_y_proportional() const
{
	boost::mutex::scoped_lock lock(y_lock);
	return y.gains().proportional();
}

double translation_outer_pid::get_y_derivative() const
{
	boost::mutex::scoped_lock lock(y_lock);
	return y.gains().derivative();
}

double translation_outer_pid::get_y_integral() const
{
	boost::mutex::scoped_lock lock(y_lock);
	return y.gains().integral();
}


void translation_outer_pid::get_xml_node()
{
	Configuration* cfg = Configuration::getInstance();

	cfg->setd(XML_TRANSLATION_X_PROPORTIONAL, get_x_proportional());
	cfg->setd(XML_TRANSLATION_Y_PROPORTIONAL, get_y_proportional());
	cfg->setd(XML_TRANSLATION_X_DERIVATIVE, get_x_derivative());
	cfg->setd(XML_TRANSLATION_Y_DERIVATIVE, get_y_derivative());
	cfg->setd(XML_TRANSLATION_X_INTEGRAL, get_x_integral());
	cfg->setd(XML_TRANSLATION_Y_INTEGRAL, get_y_integral());
	cfg->setd(XML_TRAVEL, scaled_travel_degrees());
}

void translation_outer_pid::parse_xml_node()
{
	Configuration* cfg = Configuration::getInstance();

	set_x_proportional(cfg->getd(XML_TRANSLATION_X_PROPORTIONAL, get_x_proportional()));
	set_y_proportional(cfg->getd(XML_TRANSLATION_Y_PROPORTIONAL, get_y_proportional()));
	set_x_derivative(cfg->getd(XML_TRANSLATION_X_DERIVATIVE, get_x_derivative()));
	set_y_derivative(cfg->getd(XML_TRANSLATION_Y_DERIVATIVE, get_y_derivative()));
	set_x_integral(cfg->getd(XML_TRANSLATION_X_INTEGRAL, get_x_integral()));
	set_y_integral(cfg->getd(XML_TRANSLATION_Y_INTEGRAL, get_y_integral()));
	set_scaled_travel_degrees(cfg->getd(XML_TRAVEL, scaled_travel_degrees()));
}


