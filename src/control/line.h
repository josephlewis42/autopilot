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
#ifndef LINE_H_
#define LINE_H_

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

/**
 * Reference line trajectory generator
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @date October 24, 2012: Class creation
 */
class line
{
public:
	line();
	/// return the reference position for the current time
	blas::vector<double> get_reference_position() const;

	/// reset the trajectory to begin from the current location
	void reset();

	/// set the x_travel
	void set_x_travel(const double newXTravel)
	{
		x_travel = newXTravel;
		message() << "Line: x travel set to " << newXTravel;
	}
	/// get the x_travel
	double get_x_travel() const {return x_travel;}

	/// set y_travel
	void set_y_travel(const double newYTravel)
	{
		y_travel = newYTravel;
		message() << "Line: y travel set to " << newYTravel;
	}

	/// get y_travel
	double get_y_travel() const {return y_travel;}

	/// set the initial hover time before trajectory begins
	void set_hover_time(const double newHoverTime) {
		hover_time = newHoverTime;
		message() << "Line: hover time set to " << newHoverTime;
	}

	/// get the hover time
	double get_hover_time() const {return hover_time;}

	/// set the speed
	void set_speed(const double newSpeed) {
		speed = newSpeed;
		message() << "Line: speed set to " << newSpeed;
	}
	/// get the speed
	double get_speed() const {return speed;}

	/// return the parameter list to send to qgc
	std::vector<Parameter> getParameters() const;

	/// saves the controller parameters
	void get_xml_node();
	/// loads the parameters for the function and populate the values
	void parse_xml_node();

	static const std::string PARAM_X_TRAVEL;
	static const std::string PARAM_Y_TRAVEL;
	static const std::string PARAM_HOVER_TIME;
	static const std::string PARAM_SPEED;

protected:
	/// position to begin trajectory in NED frame
	blas::vector<double> start_location;
	/// serialzie access to start_location
	mutable std::mutex start_location_lock;
	/// set the start_location
	void set_start_location(const blas::vector<double>& start_location) {{std::lock_guard<std::mutex> lock(start_location_lock); this->start_location = start_location;} message() << "Line: Start Location set to " << start_location;}
	/// get the start_location
	blas::vector<double> get_start_location() const {std::lock_guard<std::mutex> lock(start_location_lock); return start_location;}

	/// end location in NED frame
	blas::vector<double> end_location;
	/// serialize access to end_location
	mutable std::mutex end_location_lock;
	/// set the end_location
	void set_end_location(const blas::vector<double>& end_location) {{std::lock_guard<std::mutex> lock(end_location_lock); this->end_location = end_location;} message() << "Line: End location set to " << end_location;}
	/// get the end_location
	blas::vector<double> get_end_location() const {std::lock_guard<std::mutex> lock(end_location_lock); return end_location;}


	std::atomic<double> x_travel;  /// distance to travel in body x direction in m
	std::atomic<double> y_travel;  /// distance to travel in body y direction in m
	std::atomic<double> speed;     /// average speed to fly trajectory in m/s
	std::atomic<double> hover_time;  // time to hover before manouever in seconds

	/// time the trajectory started
	boost::posix_time::ptime start_time;
	/// serialize access to start_time
	mutable std::mutex start_time_lock;
	/// set the start_time to the current time
	void set_start_time() {{std::lock_guard<std::mutex> lock(start_time_lock); start_time = boost::posix_time::microsec_clock::local_time();} message() << "Line trajectory started";}
	/// get the start_Time
	boost::posix_time::ptime get_start_time() const {std::lock_guard<std::mutex> lock(start_time_lock); return start_time;}

	/// return trajectory length
	double get_distance() const;
};

#endif /* LINE_H_ */
