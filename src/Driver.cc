/*
 * Driver.cpp
 *
 *  Created on: Aug 26, 2013
 *      Author: joseph
 */

#include "Driver.h"
#include <boost/foreach.hpp>

boost::mutex Driver::_all_drivers_lock;
std::list<Driver*> Driver::all_drivers;


Driver::Driver(std::string name)
:_name(name),
 _terminate(false)
{
	boost::mutex::scoped_lock(_all_drivers_lock);
	all_drivers.push_front(this);
}

Driver::~Driver()
{
	boost::mutex::scoped_lock(_all_drivers_lock);
	all_drivers.remove(this);
}

void Driver::terminateAll()
{
	boost::mutex::scoped_lock(_all_drivers_lock);
	BOOST_FOREACH(Driver* d, all_drivers)
	{
		d->terminate();
	}

	// wait for all threads to terminate
	sleep(3);
}
