/*******************************************************************************
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

#ifndef DRIVER_H_
#define DRIVER_H_

#include <boost/thread.hpp>
#include <string.h>
#include "Debug.h"


/**
 * The driver class is the base class for all the extension points in the program.
 *
 * @author Joseph Lewis <joehms22@gmail.com>
 */
class Driver
{
private:
	/// store whether to terminate the thread
	bool _terminate;
	/// serialize access to _terminate
	boost::mutex _terminate_lock;

	// Keeps a list of all drivers so we can terminate them later.
	static boost::mutex _all_drivers_lock;
	static std::list<Driver*> all_drivers;

	// Keeps the human readable name for the current driver.
	std::string _name;

public:
	Driver(std::string name);
	virtual ~Driver();
	static void terminateAll();
	inline std::string getName(){return _name;};
	inline bool terminateRequested() {boost::mutex::scoped_lock(_terminate_lock); return _terminate;};
	inline void terminate() {
		boost::mutex::scoped_lock(_terminate_lock);
		_terminate = true;
		debug() << "Driver Terminating: " << getName();
	};
};

#endif /* DRIVER_H_ */
