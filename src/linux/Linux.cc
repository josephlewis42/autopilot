/**************************************************************************
 * Copyright 2014 Joseph Lewis <joseph@josephlewis.net>
 * 
 * This file is part of University of Denver Autopilot.
 * 
 *     UDenver Autopilot is free software: you can redistribute it 
 * 	   and/or modify it under the terms of the GNU General Public 
 *     License as published by the Free Software Foundation, either 
 *     version 3 of the License, or (at your option) any later version.
 * 
 *     UDenver Autopilot is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 * 
 *     You should have received a copy of the GNU General Public License
 *     along with UDenver Autopilot. If not, see <http://www.gnu.org/licenses/>.
 *************************************************************************/

#include "Linux.h"
#include <boost/thread.hpp>
#include <iostream>
#include <fstream>

#include "RateLimiter.h"


Linux* Linux::_instance = NULL;
std::mutex Linux::_instance_lock;

Linux* Linux::getInstance()
{
	std::lock_guard<std::mutex> lock(_instance_lock);
	if (_instance == NULL)
	{
		_instance = new Linux;
	}
	return _instance;
}

Linux::Linux()
:Driver("Linux CPU Info","linux_cpu_info")
{	
	isEnabled = configGetb("enabled", true);
	
	// If the system wants to halt, don't start running.
	if(terminateRequested())
	{
		return;
	}

	// If the user has disabled this component, don't start running
	if(! isEnabled)
	{
		return;
	}

	// Tell the user we made it up
	warning() << "starting CPU Information System";
	
	// Start our processing thread.
	boost::thread(std::bind(Linux::cpuInfo, this));
}

Linux::~Linux() {

}

void Linux::cpuInfo(Linux* instance)
{	
	RateLimiter rl(1);

	std::ifstream myfile;
	myfile.open ("/proc/loadavg");
	
	float util = 0;
	while(!instance->terminateRequested())
	{
		rl.wait(); // suspends until we're ready to go
		
		myfile >> util;
		myfile.seekg(0); // go to the beginning of the file
		instance->cpu_utilization = util; // set our utilization
		instance->debug() << "Got Load of: " << util;
		
		rl.finishedCriticalSection(); // call to yield
	}
	
	myfile.close();
}

bool Linux::sendMavlinkMsg(mavlink_message_t* msg, int uasId, int sendRateHz, int msgNumber)
{
	if(! isEnabled) return false;

	if(msgNumber % sendRateHz == 0) // do this once a second.
	{
		debug() << "Sending CPU Utilization";
		mavlink_msg_udenver_cpu_usage_pack(uasId, 40, msg, getCpuUtilization());
		return true;
	}
	return false;
};

