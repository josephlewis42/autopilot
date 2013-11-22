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

#include "QGCLink.h"

/* Project Headers */
#include "QGCReceive.h"
#include "QGCSend.h"
#include "Configuration.h"

/* Boost Headers */
#include <boost/asio.hpp>

/* Mavlink Headers */
#include <mavlink.h>


// Configuration file name of the IP address param
const std::string QGCLINK_HOST_ADDRESS_PARAM = "QGROUNDCONTROL_HOST_IP_ADDRESS";
const std::string QGCLINK_HOST_ADDRESS_DEFAULT = "0.0.0.0";

QGCLink* QGCLink::_instance = NULL;
boost::mutex QGCLink::_instance_lock;


// Function definitions

QGCLink::QGCLink()
: socket(io_service),
  heartbeat_rate(10),
  rc_channel_rate(10),
  control_output_rate(10),
  position_rate(10),
  attitude_rate(10),
  requested_rc_calibration(false),
  uasId(100)
{
	init();
	param_recv = false;
}

void QGCLink::init()
{
	try
	{
		std::string ip_addr = Configuration::getInstance()->gets(QGCLINK_HOST_ADDRESS_PARAM, QGCLINK_HOST_ADDRESS_DEFAULT);

		qgc.address(boost::asio::ip::address::from_string(ip_addr));
		debug() << "QGCLink: Opening socket to " << qgc.address().to_string();
		qgc.port(14550); // FIXME we shouldn't assume this port as firewall rules may (should) be in effect - Joseph

		// FIXME we didn't check to make sure the address is indeed IPV4 - Joseph
		socket.open(boost::asio::ip::udp::v4());

		receive_thread = boost::thread(QGCReceive());

		send_thread = boost::thread(QGCSend(this));
	}
	catch (std::exception& e)
	{
		critical() << "QGCLink: " << e.what();
		throw e;
	}
}

QGCLink* QGCLink::getInstance()
{
	boost::mutex::scoped_lock lock(_instance_lock);
	if(!_instance)
	{
		_instance = new QGCLink;
	}

	return _instance;
}
