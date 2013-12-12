/**************************************************************************
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
 *************************************************************************/

#include "attitude_pid.h"

/* Project Headers */
#include "RCTrans.h"
#include "IMU.h"
#include "Control.h"
#include "Configuration.h"

const std::string XML_ROLL_PROPORTIONAL = "controller_params.attitude_pid.roll.gain.proportional";
const std::string XML_ROLL_DERIVATIVE = "controller_params.attitude_pid.roll.gain.derivative";
const std::string XML_ROLL_INTEGRAL = "controller_params.attitude_pid.roll.gain.integral";
const std::string XML_ROLL_TRIM = "controller_params.attitude_pid.roll.trim";
const std::string XML_PITCH_PROPORTIONAL = "controller_params.attitude_pid.pitch.gain.proportional";
const std::string XML_PITCH_DERIVATIVE = "controller_params.attitude_pid.pitch.gain.derivative";
const std::string XML_PITCH_INTEGRAL = "controller_params.attitude_pid.pitch.gain.integral";
const std::string XML_PITCH_TRIM = "controller_params.attitude_pid.pitch.trim";

const std::string attitude_pid::PARAM_ROLL_KP = "PID_ROLL_KP";
const std::string attitude_pid::PARAM_ROLL_KD = "PID_ROLL_KD";
const std::string attitude_pid::PARAM_ROLL_KI = "PID_ROLL_KI";
const std::string attitude_pid::PARAM_PITCH_KP = "PID_PITCH_KP";
const std::string attitude_pid::PARAM_PITCH_KD = "PID_PITCH_KD";
const std::string attitude_pid::PARAM_PITCH_KI = "PID_PITCH_KI";
const std::string attitude_pid::PARAM_ROLL_TRIM = "TRIM_ROLL";
const std::string attitude_pid::PARAM_PITCH_TRIM = "TRIM_PITCH";

attitude_pid::attitude_pid()
:roll(5),
 pitch(5),
 control_effort(blas::zero_vector<double>(2)),
 roll_trim(0),
 pitch_trim(0),
 _runnable(true)
{
	roll.name() = "Roll";
	pitch.name() = "Pitch";

	LogFile *log = LogFile::getInstance();
	log->logHeader(heli::LOG_ATTITUDE_ERROR, "Roll_Proportional Roll_Derivative Roll_Integral Pitch_Proportional Pitch_Derivative Pitch_Integral");
	log->logData(heli::LOG_ATTITUDE_ERROR, std::vector<double>());
	log->logHeader(heli::LOG_ATTITUDE_REFERENCE, "Roll Pitch");
	log->logData(heli::LOG_ATTITUDE_REFERENCE, std::vector<double>());

}

attitude_pid::attitude_pid(const attitude_pid& other)
{

	{
		boost::mutex::scoped_lock lock(other.control_effort_lock);
		control_effort = other.control_effort;
	}
	{
		boost::mutex::scoped_lock lock(other.roll_lock);
		roll = other.roll;
	}
	{
		boost::mutex::scoped_lock lock(other.pitch_lock);
		pitch = other.pitch;
	}
	{
		boost::mutex::scoped_lock lock(other.runnable_lock);
		_runnable = other._runnable;
	}

	{
		boost::mutex::scoped_lock lock(other.roll_trim_lock);
		roll_trim = other.roll_trim;
	}

	{
		boost::mutex::scoped_lock lock(other.pitch_trim_lock);
		pitch_trim = other.pitch_trim;
	}
}

void attitude_pid::reset()
{
	roll.reset();
	pitch.reset();
}

void attitude_pid::operator()(const blas::vector<double>& reference) throw(bad_control)
{
	if (!runnable())
		throw bad_control("attempted to compute attitude_pid, but it wasn't runnable.");

	if (reference.size() < 2)
		throw bad_control("Attitude control received less than two references (roll pitch)");

	blas::vector<double> roll_pitch_reference(reference);
	roll_pitch_reference.resize(2);

	IMU* imu = IMU::getInstance();
	blas::vector<double> euler(imu->get_euler());
	blas::vector<double> euler_rate(imu->get_euler_rate());
	// resize to perform vector subtraction
	euler.resize(2);


	blas::vector<double> euler_error(euler - roll_pitch_reference);

	std::vector<double> log(euler_error.begin(), euler_error.end());
	log.insert(log.end(), euler_rate.begin(), euler_rate.end()-1);
	LogFile::getInstance()->logData("Attitude PID error", log);
	blas::vector<double> control_effort(2);
	control_effort.clear();

	std::vector<double> error_states;
	roll_lock.lock();
	error_states.push_back(roll.error().proportional() = euler_error[0]);
	error_states.push_back(roll.error().derivative() = euler_rate[0]);
	error_states.push_back(++roll.error());
	control_effort[0] = roll.compute_pid();
	roll_lock.unlock();

	pitch_lock.lock();
	error_states.push_back(pitch.error().proportional() = euler_error[1]);
	error_states.push_back(pitch.error().derivative() = euler_rate[1]);
	error_states.push_back(++pitch.error());

	LogFile::getInstance()->logData(heli::LOG_ATTITUDE_ERROR, error_states);
	control_effort[1] = pitch.compute_pid();
	pitch_lock.unlock();

	// saturate the controls to [-1, 1]
	Control::saturate(control_effort);
	set_control_effort(control_effort);

	LogFile::getInstance()->logData(heli::LOG_ATTITUDE_CONTROL_EFFORT, control_effort);
//	debug() << "Attitude PID control effort: " << control_effort;
}

std::vector<Parameter> attitude_pid::getParameters()
{
//	boost::mutex::scoped_lock lock(read_params_lock);
	std::vector<Parameter> plist;

	roll_lock.lock();
	plist.push_back(Parameter(PARAM_ROLL_KP, roll.gains().proportional(), heli::CONTROLLER_ID));
	plist.push_back(Parameter(PARAM_ROLL_KD, roll.gains().derivative(), heli::CONTROLLER_ID));
	plist.push_back(Parameter(PARAM_ROLL_KI, roll.gains().integral(), heli::CONTROLLER_ID));
	roll_lock.unlock();

	pitch_lock.lock();
	plist.push_back(Parameter(PARAM_PITCH_KP, pitch.gains().proportional(), heli::CONTROLLER_ID));
	plist.push_back(Parameter(PARAM_PITCH_KD, pitch.gains().derivative(), heli::CONTROLLER_ID));
	plist.push_back(Parameter(PARAM_PITCH_KI, pitch.gains().integral(), heli::CONTROLLER_ID));
	pitch_lock.unlock();

	plist.push_back(Parameter(PARAM_ROLL_TRIM, get_roll_trim_degrees(), heli::CONTROLLER_ID));
	plist.push_back(Parameter(PARAM_PITCH_TRIM, get_pitch_trim_degrees(), heli::CONTROLLER_ID));

	return plist;
}

void attitude_pid::set_roll_proportional(double kp)
{
	{
		boost::mutex::scoped_lock lock(roll_lock);
		roll.gains().proportional() = kp;
	}
	message() << "Set roll proportional gain to: " << kp;
}

double attitude_pid::get_pitch_proportional()
{
	boost::mutex::scoped_lock lock(pitch_lock);
	return pitch.gains().proportional();
}

double attitude_pid::get_pitch_derivative()
{
	boost::mutex::scoped_lock lock(pitch_lock);
	return pitch.gains().derivative();
}

double attitude_pid::get_pitch_integral()
{
	boost::mutex::scoped_lock lock(pitch_lock);
	return pitch.gains().integral();
}


double attitude_pid::get_roll_proportional()
{
	boost::mutex::scoped_lock lock(roll_lock);
	return roll.gains().proportional();
}

double attitude_pid::get_roll_derivative()
{
	boost::mutex::scoped_lock lock(roll_lock);
	return roll.gains().derivative();
}

double attitude_pid::get_roll_integral()
{
	boost::mutex::scoped_lock lock(roll_lock);
	return roll.gains().integral();
}

void attitude_pid::set_roll_derivative(double kd)
{
	{
		boost::mutex::scoped_lock lock(roll_lock);
		roll.gains().derivative() = kd;
	}
	message() << "Set roll derivative gain to: " << kd;
}

void attitude_pid::set_roll_integral(double ki)
{
	{
		boost::mutex::scoped_lock lock(roll_lock);
		roll.gains().integral() = ki;
	}
	message() << "Set roll integral gain to: " << ki;
}

void attitude_pid::set_pitch_proportional(double kp)
{
	{
		boost::mutex::scoped_lock lock(pitch_lock);
		pitch.gains().proportional() = kp;
	}
	message() << "Set pitch proportional gain to: " << kp;
}

void attitude_pid::set_pitch_derivative(double kd)
{
	{
		boost::mutex::scoped_lock lock(pitch_lock);
		pitch.gains().derivative() = kd;
	}
	message() << "Set pitch derivative gain to: " << kd;
}

void attitude_pid::set_pitch_integral(double ki)
{
	{
		boost::mutex::scoped_lock lock(pitch_lock);
		pitch.gains().integral() = ki;
	}
	message() << "Set pitch integral gain to: " << ki;
}
void attitude_pid::set_roll_trim_degrees(double trim_degrees)
{
	{
		boost::mutex::scoped_lock lock(roll_trim_lock);
		roll_trim = trim_degrees * boost::math::constants::pi<double>()/180;
	}
	message() << "Set roll trim to " << trim_degrees << " deg.";
}

void attitude_pid::set_pitch_trim_degrees(double trim_degrees)
{
	{
		boost::mutex::scoped_lock lock(pitch_trim_lock);
		pitch_trim = trim_degrees * boost::math::constants::pi<double>()/180;
	}
	message() << "Set pitch trim to " << trim_degrees << " deg.";
}


void attitude_pid::get_xml_node()
{

	Configuration* cfg = Configuration::getInstance();

	cfg->setd(XML_ROLL_PROPORTIONAL, get_roll_proportional());
	cfg->setd(XML_ROLL_DERIVATIVE, get_roll_derivative());
	cfg->setd(XML_ROLL_INTEGRAL, get_roll_integral());
	cfg->setd(XML_ROLL_TRIM, get_roll_trim_degrees());

	cfg->setd(XML_PITCH_PROPORTIONAL, get_pitch_proportional());
	cfg->setd(XML_PITCH_DERIVATIVE, get_pitch_derivative());
	cfg->setd(XML_PITCH_INTEGRAL, get_pitch_integral());
	cfg->setd(XML_PITCH_TRIM, get_pitch_trim_degrees());
}


void attitude_pid::parse_pid()
{
	Configuration* cfg = Configuration::getInstance();

	set_roll_proportional(cfg->getd(XML_ROLL_PROPORTIONAL, get_roll_proportional()));
	set_roll_derivative(cfg->getd(XML_ROLL_DERIVATIVE, get_roll_derivative()));
	set_roll_integral(cfg->getd(XML_ROLL_INTEGRAL, get_roll_integral()));
	set_roll_trim_degrees(cfg->getd(XML_ROLL_TRIM, get_roll_trim_degrees()));

	set_pitch_proportional(cfg->getd(XML_PITCH_PROPORTIONAL, get_pitch_proportional()));
	set_pitch_derivative(cfg->getd(XML_PITCH_DERIVATIVE, get_pitch_derivative()));
	set_pitch_integral(cfg->getd(XML_PITCH_INTEGRAL, get_pitch_integral()));
	set_pitch_trim_degrees(cfg->getd(XML_PITCH_TRIM, get_pitch_trim_degrees()));
}
