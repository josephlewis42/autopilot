/*
 * Driver.cpp
 *
 *  Created on: Aug 26, 2013
 *      Author: joseph
 */

#include "Driver.h"
//#include <boost/foreach.hpp>

#include "Configuration.h"

std::mutex Driver::_all_drivers_lock;
std::list<Driver*> Driver::all_drivers;


Driver::Driver(std::string name, std::string config_prefix)
: Logger(name + ": "),
  ConfigurationSubTree(config_prefix),
  _terminate(false),
  _config_prefix(config_prefix),
  _name(name)
{
	debug() << "Driver: Setting up " << name;

	std::lock_guard<std::mutex> lock(_all_drivers_lock);
	all_drivers.push_front(this);


	Configuration* config = Configuration::getInstance();
	_debug = config->getb(config_prefix + ".debug", false);
}

Driver::~Driver()
{
	std::lock_guard<std::mutex> lock(_all_drivers_lock);
	all_drivers.remove(this);
}

void Driver::terminateAll()
{
	{
		std::lock_guard<std::mutex> lock(_all_drivers_lock);
		for(Driver* d : all_drivers)
		{
			d->terminate();
		}
	}

	// wait for all threads to terminate
	sleep(3);
}

Debug Driver::trace()
{
	if(_debug == false)
	{
		return ignore();
	}

	return debug();
}
