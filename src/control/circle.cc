/**************************************************************************
 * Copyright 2012 Bryan Godbolt
 * Copyright 2013-14 Joseph Lewis <joehms22@gmail.com>
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

#include "circle.h"

/* STL Headers */
#include <math.h>

/* Project Headers */
#include "IMU.h"
#include "Configuration.h"
#include "util/AutopilotMath.hpp"


// Constants
const std::string XML_RADIUS_PARAM = "controller_params.circle.radius";
const std::string XML_HOVER_PARAM  = "controller_params.circle.hover";
const std::string XML_SPEED_PARAM  = "controller_params.circle.speed";

const double XML_RADIUS_PARAM_DEFAULT = 10.0;
const double XML_HOVER_PARAM_DEFAULT  = 7.0;
const double XML_SPEED_PARAM_DEFAULT  = 0.30000001192092896;

circle::circle()
    : radius(0),
      start_location(blas::zero_vector<double>(3)),
      center_location(blas::zero_vector<double>(3)),
      speed(XML_SPEED_PARAM_DEFAULT),
      hover_time(XML_HOVER_PARAM_DEFAULT),
      initial_angle(0)
{


}

const std::string circle::PARAM_HOVER_TIME = "CIR_HOVER_TIME";
const std::string circle::PARAM_RADIUS = "CIR_RADIUS";
const std::string circle::PARAM_SPEED = "CIR_SPEED";

void circle::reset()
{
    set_start_location(IMU::getInstance()->get_ned_position());
    set_start_time();
    set_center_location();

    blas::vector<double> initial_vector(get_start_location() - get_center_location());
    set_initial_angle(atan2(initial_vector(1), initial_vector(0)));
}

std::vector<Parameter> circle::getParameters() const
{
    std::vector<Parameter> plist;
    plist.push_back(Parameter(PARAM_HOVER_TIME, get_hover_time(), heli::CONTROLLER_ID));
    plist.push_back(Parameter(PARAM_RADIUS, get_radius(), heli::CONTROLLER_ID));
    plist.push_back(Parameter(PARAM_SPEED, get_speed(), heli::CONTROLLER_ID));
    return plist;
}

blas::vector<double> circle::get_reference_position() const
{
    double elapsed_time = (boost::posix_time::microsec_clock::local_time() - get_start_time()).total_milliseconds()/1000.0;
    double period = (get_speed() > 0 ? get_circumference()/get_speed() : 0);
    double hover_time = get_hover_time();
    if (period == 0 || elapsed_time <= hover_time)
    {
        return get_start_location();
    }
    else
    {
        elapsed_time -= hover_time;
        blas::vector<double> reference_position(get_center_location());
        reference_position(0) += get_radius()*cos(2*AutopilotMath::PI*elapsed_time/period + get_initial_angle());
        reference_position(1) += get_radius()*sin(2*AutopilotMath::PI*elapsed_time/period + get_initial_angle());
        return reference_position;
    }
}

double circle::get_circumference() const
{
    return 2*AutopilotMath::PI*get_radius();
}

void circle::set_center_location()
{
    blas::vector<double> center(blas::zero_vector<double>(3));
    center(0) = get_radius();  // vector in body frame with origin at heli
    {
        std::lock_guard<std::mutex> lock(center_location_lock);
        center_location = prod(IMU::getInstance()->get_heading_rotation(), center) + get_start_location();
    }
    message() << "Circle: center_location set to: " << center_location;
}

void circle::get_xml_node()
{
    Configuration* config = Configuration::getInstance();

    config->setd(XML_RADIUS_PARAM, get_radius());
    config->setd(XML_HOVER_PARAM, get_hover_time());
    config->setd(XML_SPEED_PARAM, get_speed());
}

void circle::parse_xml_node()
{
    Configuration* config = Configuration::getInstance();

    set_radius(config->getd(XML_RADIUS_PARAM, get_radius()));
    set_hover_time(config->getd(XML_HOVER_PARAM, get_hover_time()));
    set_speed(config->getd(XML_SPEED_PARAM, get_speed()));
}


void circle::set_initial_angle(const double angle)
{
    initial_angle = angle;
    message() << "Circle: Initial angle set to: " << initial_angle;
}

double circle::get_initial_angle() const
{
    return initial_angle;
}




void circle::set_speed(const double newSpeed)
{
    speed = newSpeed;
    message() << "Circle: speed set to " << newSpeed;
}

double circle::get_speed() const
{
    return speed;
}

void circle::set_radius(const double newRadius)
{
    radius = newRadius;
}

double circle::get_radius() const
{
    return radius;
}

void circle::set_hover_time(const double newHoverTime)
{
    hover_time = newHoverTime;
    message() << "Circle: Hover time set to " << newHoverTime;
}

double circle::get_hover_time() const
{
    return hover_time;
}
