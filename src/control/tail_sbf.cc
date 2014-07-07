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

#include "tail_sbf.h"

/* Project Headers */
#include "IMU.h"
#include "tail_sbf.h"
#include "Helicopter.h"
#include "Control.h"
#include "Configuration.h"
#include "LogFile.h"

/* STL Headers */
#include <math.h>


// constants
std::string tail_sbf::XML_TRANSLATION_X_PROPORTIONAL = "controller_params.translation_outer_sbf.ned_x.proportional";
std::string tail_sbf::XML_TRANSLATION_Y_PROPORTIONAL = "controller_params.translation_outer_sbf.ned_y.proportional";
std::string tail_sbf::XML_TRANSLATION_X_DERIVATIVE = "controller_params.translation_outer_sbf.ned_x.derivative";
std::string tail_sbf::XML_TRANSLATION_Y_DERIVATIVE = "controller_params.translation_outer_sbf.ned_y.derivative";
std::string tail_sbf::XML_TRANSLATION_X_INTEGRAL = "controller_params.translation_outer_sbf.ned_x.integral";
std::string tail_sbf::XML_TRANSLATION_Y_INTEGRAL = "controller_params.translation_outer_sbf.ned_y.integral";
std::string tail_sbf::XML_TRAVEL = "controller_params.translation_outer_sbf.travel";

const std::string LOG_TRANS_SBF_ERROR_STATES = "Translation SBF Error States";



tail_sbf::tail_sbf()
    : Logger("Tail SBF"),
      ned_x(10),
      ned_y(10)
{
    scaled_travel = 0;
}

void tail_sbf::reset()
{
    ned_x.reset();
    ned_y.reset();
}

bool tail_sbf::runnable() const
{
    return true;
}

void tail_sbf::operator()(const blas::vector<double>& reference) throw(bad_control)
{

    IMU* imu = IMU::getInstance();
    blas::vector<double> ned_position_error(imu->get_ned_position() - reference);
    blas::vector<double> ned_velocity_error(imu->get_ned_velocity());

    blas::vector<double> ned_control(3);
    ned_control.clear();
    std::vector<double> error_states;
    {
        std::lock_guard<std::mutex> lock(ned_x_lock);
        error_states.push_back(ned_x.error().setProportional(ned_position_error(0)));
        error_states.push_back(ned_x.error().setDerivative(ned_velocity_error(0)));
        error_states.push_back(++(ned_x.error()));
        ned_control(0) = ned_x.compute_pid();
    }
    {
        std::lock_guard<std::mutex> lock(ned_y_lock);
        error_states.push_back(ned_y.error().setProportional(ned_position_error(1)));
        error_states.push_back(ned_y.error().setDerivative(ned_velocity_error(1)));
        error_states.push_back(++(ned_y.error()));
        ned_control(1) = ned_y.compute_pid();
    }

    LogFile::getInstance()->logData(LOG_TRANS_SBF_ERROR_STATES, error_states);

    double heading = imu->get_euler()(2);
    blas::matrix<double> Rz(3,3);
    Rz.clear();
    Rz(0,0) = cos(heading);
    Rz(0,1) = -sin(heading);
    Rz(1,0) = sin(heading);
    Rz(1,1) = cos(heading);
    Rz(2,2) = 1;

    Helicopter* bergen = Helicopter::getInstance();
    double m = bergen->get_mass();
    double g = bergen->get_gravity();
    ned_control(2) = -g;
    blas::vector<double> body_control(m*prod(trans(Rz),ned_control));
    double alpham = 0.04; // countertorque approximate slope
    double xt = abs(bergen->get_tail_hub_offset()(0));

    double theta_ref = atan(body_control(0)/body_control(2));
    double phi_ref = -atan((alpham*sqrt(pow(body_control(0),2)+pow(body_control(2),2))+xt*body_control(1))/(alpham*body_control(1) - xt*sqrt(pow(body_control(0),2)+pow(body_control(2),2))));

    blas::vector<double> attitude_reference(2);
    attitude_reference(0) = phi_ref;
    attitude_reference(1) = theta_ref;

    Control::saturate(attitude_reference, scaled_travel_radians());

    set_control_effort(attitude_reference);
}

const std::string tail_sbf::PARAM_X_KP = "SBF_X_KP";
const std::string tail_sbf::PARAM_X_KD = "SBF_X_KD";
const std::string tail_sbf::PARAM_X_KI = "SBF_X_KI";

const std::string tail_sbf::PARAM_Y_KP = "SBF_Y_KP";
const std::string tail_sbf::PARAM_Y_KD = "SBF_Y_KD";
const std::string tail_sbf::PARAM_Y_KI = "SBF_Y_KI";

const std::string tail_sbf::PARAM_TRAVEL = "SBF_TRAVEL";

std::vector<Parameter> tail_sbf::getParameters() const
{
    std::vector<Parameter> plist;

    {
        std::lock_guard<std::mutex> lock(ned_x_lock);
        plist.push_back(Parameter(PARAM_X_KP, ned_x.gains().getProportional(), heli::CONTROLLER_ID));
        plist.push_back(Parameter(PARAM_X_KD, ned_x.gains().getDerivative(), heli::CONTROLLER_ID));
        plist.push_back(Parameter(PARAM_X_KI, ned_x.gains().getIntegral(), heli::CONTROLLER_ID));
    }

    {
        std::lock_guard<std::mutex> lock(ned_y_lock);
        plist.push_back(Parameter(PARAM_Y_KP, ned_y.gains().getProportional(), heli::CONTROLLER_ID));
        plist.push_back(Parameter(PARAM_Y_KD, ned_y.gains().getDerivative(), heli::CONTROLLER_ID));
        plist.push_back(Parameter(PARAM_Y_KI, ned_y.gains().getIntegral(), heli::CONTROLLER_ID));
    }

    plist.push_back(Parameter(PARAM_TRAVEL, scaled_travel_degrees(), heli::CONTROLLER_ID));

    return plist;
}

void tail_sbf::set_x_proportional(double kp)
{
    {
        std::lock_guard<std::mutex> lock(ned_x_lock);
        ned_x.gains().setProportional(kp);
    }
    message() << "Set SBF x proportional gain to: " << kp;
}

void tail_sbf::set_x_derivative(double kd)
{
    {
        std::lock_guard<std::mutex> lock(ned_x_lock);
        ned_x.gains().setDerivative(kd);
    }
    message() << "Set SBF x derivative gain to: " << kd;
}

void tail_sbf::set_x_integral(double ki)
{
    {
        std::lock_guard<std::mutex> lock(ned_x_lock);
        ned_x.gains().setIntegral(ki);
    }
    message() << "Set SBF x integral gain to: " << ki;
}

void tail_sbf::set_y_proportional(double kp)
{
    {
        std::lock_guard<std::mutex> lock(ned_y_lock);
        ned_y.gains().setProportional(kp);
    }
    message() << "Set SBF y proportional gain to: " << kp;
}

void tail_sbf::set_y_derivative(double kd)
{
    {
        std::lock_guard<std::mutex> lock(ned_y_lock);
        ned_y.gains().setDerivative(kd);
    }
    message() << "Set SBF y derivative gain to: " << kd;
}

void tail_sbf::set_y_integral(double ki)
{
    {
        std::lock_guard<std::mutex> lock(ned_y_lock);
        ned_y.gains().setIntegral(ki);
    }
    message() << "Set SBF y integral gain to: " << ki;
}

void tail_sbf::set_scaled_travel(double travel)
{
    scaled_travel = travel;
    message() << "Set travel to: " << travel;
}

double tail_sbf::get_x_proportional() const
{
    std::lock_guard<std::mutex> lock(ned_x_lock);
    return ned_x.gains().getProportional();
}

double tail_sbf::get_y_proportional() const
{
    std::lock_guard<std::mutex> lock(ned_y_lock);
    return ned_y.gains().getProportional();
}

double tail_sbf::get_x_derivative() const
{
    std::lock_guard<std::mutex> lock(ned_x_lock);
    return ned_x.gains().getDerivative();
}

double tail_sbf::get_y_derivative() const
{
    std::lock_guard<std::mutex> lock(ned_y_lock);
    return ned_y.gains().getDerivative();
}

double tail_sbf::get_x_integral() const
{
    std::lock_guard<std::mutex> lock(ned_x_lock);
    return ned_x.gains().getIntegral();
}

double tail_sbf::get_y_integral() const
{
    std::lock_guard<std::mutex> lock(ned_y_lock);
    return ned_y.gains().getIntegral();
}



void tail_sbf::get_xml_node()
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


void tail_sbf::parse_xml_node()
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
