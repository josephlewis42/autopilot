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

#include "line.h"

/* STL Headers */
#include <math.h>

/* Project Headers */
#include "IMU.h"
#include "heli.h"
#include "Configuration.h"


// Constants
const std::string XML_X_TRAVEL_PARAM = "controller_params.line.xtravel";
const std::string XML_Y_TRAVEL_PARAM = "controller_params.line.ytravel";
const std::string XML_HOVER_PARAM = "controller_params.line.hover";
const std::string XML_SPEED_PARAM = "controller_params.line.speed";


line::line()
    : Logger("Line"),
      start_location(blas::zero_vector<double>(3)),
      end_location(blas::zero_vector<double>(3)),
      x_travel(0),
      y_travel(0),
      speed(0),
      hover_time(0)
{
}

const std::string line::PARAM_HOVER_TIME = "LIN_HOVER_TIME";
const std::string line::PARAM_X_TRAVEL = "LIN_X_TRAVEL";
const std::string line::PARAM_Y_TRAVEL = "LIN_Y_TRAVEL";
const std::string line::PARAM_SPEED = "LIN_SPEED";

void line::reset()
{
    set_start_location(IMU::getInstance()->get_ned_position());
    set_start_time();
    blas::vector<double> body_travel(blas::zero_vector<double>(3));
    body_travel(0) = get_x_travel();
    body_travel(1) = get_y_travel();
    set_end_location(get_start_location() + prod(IMU::getInstance()->get_heading_rotation(), body_travel));
}

std::vector<Parameter> line::getParameters() const
{
    std::vector<Parameter> plist;
    plist.push_back(Parameter(PARAM_HOVER_TIME, get_hover_time(), heli::CONTROLLER_ID));
    plist.push_back(Parameter(PARAM_X_TRAVEL, get_x_travel(), heli::CONTROLLER_ID));
    plist.push_back(Parameter(PARAM_Y_TRAVEL, get_y_travel(), heli::CONTROLLER_ID));
    plist.push_back(Parameter(PARAM_SPEED, get_speed(), heli::CONTROLLER_ID));
    return plist;
}

blas::vector<double> line::get_reference_position() const
{
    double elapsed_time = (boost::posix_time::microsec_clock::local_time() - get_start_time()).total_milliseconds()/1000.0;
    double flight_time = (get_speed() > 0 ? get_distance()/get_speed() : 0);
    double hover_time = get_hover_time();
    if (flight_time == 0 || elapsed_time <= hover_time)
    {
        return get_start_location();
    }
    else if ((elapsed_time - hover_time) <= flight_time)
    {
        elapsed_time -= hover_time;
        blas::vector<double> ned_velocity((get_end_location() - get_start_location())/flight_time);
        return get_start_location() + ned_velocity*elapsed_time;
    }
    else
    {
        return get_end_location();
    }
}

double line::get_distance() const
{
    return norm_2(get_end_location() - get_start_location());
}

void line::get_xml_node()
{
    Configuration* cfg = Configuration::getInstance();

    cfg->setd(XML_X_TRAVEL_PARAM, get_x_travel());
    cfg->setd(XML_Y_TRAVEL_PARAM, get_y_travel());
    cfg->setd(XML_HOVER_PARAM, get_hover_time());
    cfg->setd(XML_SPEED_PARAM, get_speed());
}

void line::parse_xml_node()
{
    Configuration* cfg = Configuration::getInstance();

    set_x_travel(cfg->getd(XML_X_TRAVEL_PARAM, get_x_travel()));
    set_y_travel(cfg->getd(XML_Y_TRAVEL_PARAM, get_y_travel()));
    set_hover_time(cfg->getd(XML_HOVER_PARAM, get_hover_time()));
    set_speed(cfg->getd(XML_SPEED_PARAM, get_speed()));
}
