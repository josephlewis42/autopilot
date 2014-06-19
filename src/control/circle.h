/*******************************************************************************
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
 ******************************************************************************/

#ifndef CIRCLE_H_
#define CIRCLE_H_

/* Boost Headers */
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
namespace blas = boost::numeric::ublas;
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>

/* STL Headers */
#include <string>
#include <vector>
#include <mutex>
#include <atomic>

/* Project Headers */
#include "Debug.h"
#include "Parameter.h"
#include "heli.h"

/**
 * This class defines a circular reference trajectory.  The helicopter
 * is assumed to start on the circle with the nose facing the center.
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @date October 25, 2012: Class creation
 */
class circle
{
public:
    circle();
    /// return the reference position for the current time
    blas::vector<double> get_reference_position() const;
    /// reset the trajectory to begin from the current location
    void reset();

    /// set the speed
    void set_speed(const double speed);
    /// get the speed
    double get_speed() const;

    /// set the radius
    void set_radius(const double radius);
    /// get the radius
    double get_radius() const;

    /// set the initial hover time before trajectory begins
    void set_hover_time(const double hover_time);
    /// get the hover time
    double get_hover_time() const;

    /// return the parameter list to send to qgc
    std::vector<Parameter> getParameters() const;

    /// save the controller parameters
    void get_xml_node();
    /// populate the values based on the config
    void parse_xml_node();

    static const std::string PARAM_RADIUS;
    static const std::string PARAM_HOVER_TIME;
    static const std::string PARAM_SPEED;
protected:
    /// radius of circular trajectory
    std::atomic<double> radius;

    /// position to begin trajectory in NED frame
    blas::vector<double> start_location;
    /// serialzie access to start_location
    mutable std::mutex start_location_lock;
    /// set the start_location
    void set_start_location(const blas::vector<double>& start_location)
    {
        {
            std::lock_guard<std::mutex>  lock(start_location_lock);
            this->start_location = start_location;
        }
        message() << "Circle: start location set to " << start_location;
    }
    /// get the start_location
    blas::vector<double> get_start_location() const
    {
        std::lock_guard<std::mutex>  lock(start_location_lock);
        return start_location;
    }

    /// position of center of circle
    blas::vector<double> center_location;
    /// serialize access to center_location
    mutable std::mutex center_location_lock;
    /// set the center location based on the current start_location, radius, and heading
    void set_center_location();
    /// get the center location
    blas::vector<double> get_center_location() const
    {
        std::lock_guard<std::mutex> lock(center_location_lock);
        return center_location;
    }

    /// average speed to fly trajectory in m/s
    std::atomic<double> speed;

    /// time to hover before manouever in seconds
    std::atomic<double> hover_time;

    /// time the trajectory started
    boost::posix_time::ptime start_time;
    /// serialize access to start_time
    mutable std::mutex start_time_lock;
    /// set the start_time to the current time
    void set_start_time()
    {
        {
            std::lock_guard<std::mutex>  lock(start_time_lock);
            start_time = boost::posix_time::microsec_clock::local_time();
        }
        message() << "Circle Trajectory Started";
    }
    /// get the start_Time
    boost::posix_time::ptime get_start_time() const
    {
        std::lock_guard<std::mutex>  lock(start_time_lock);
        return start_time;
    }

    /// initial angle (where the helicopter started in polar)
    std::atomic<double> initial_angle;

    /// set the initial angle
    void set_initial_angle(const double angle);
    /// get the initial angle
    double get_initial_angle() const;

    /// return circumference of the circular trajectory
    double get_circumference() const;
};

#endif /* CIRCLE_H_ */
