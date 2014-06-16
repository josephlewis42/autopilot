/*
 * Driver.cpp
 *
 *  Created on: Aug 26, 2013
 *      Author: joseph
 */

#include "Driver.h"
//#include <boost/foreach.hpp>
#include <termios.h>
#include <boost/algorithm/string.hpp>

#include "Configuration.h"
#include "qnx2linux.h"

#include <mavlink.h> 



std::mutex Driver::_all_drivers_lock;
std::list<Driver*> Driver::all_drivers;


Driver::Driver(std::string name, std::string config_prefix)
: Logger(name + ": "),
  ConfigurationSubTree(config_prefix),
  _terminate(false),
  _config_prefix(config_prefix),
  _name(name)
{
	{
		std::lock_guard<std::mutex> lock(_all_drivers_lock);
		all_drivers.push_front(this);
	}

	Configuration* config = Configuration::getInstance();
	_debug = config->getb(config_prefix + ".debug", false);
	_readDeviceType = config->geti(config_prefix + ".read_style", 2);

	config->gets(config_prefix + ".read_style_COMMENT", "0:read until min, 1:readcond, 2:read(), 3:wait then read()");
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

std::vector<Driver*> Driver::getDrivers()
{
	std::vector<Driver*> drivers;
	{
		std::lock_guard<std::mutex> lock(_all_drivers_lock);
		for(Driver* d : all_drivers)
		{
			drivers.push_back(d);
		}
	}
	
	return drivers;
}

Debug Driver::trace()
{
	if(_debug == false)
	{
		return ignore();
	}

	return debug();
}


int Driver::readDevice(int fd, void * buf, int n)
{
	if(_readDeviceType == 0)
	{
		return QNX2Linux::readUntilMin(fd, buf, n, n);
	}

	if(_readDeviceType == 1)
	{
		return QNX2Linux::readcond(fd, buf, n, n, 10,10);
	}

	if(_readDeviceType == 3)
	{
		struct termios port_config;
		tcgetattr(fd, &port_config);                  // get the current port settings

		// Set the baud rate
		speed_t speed = cfgetospeed(&port_config);
		int waittimeMS = (1000 * speed) / (n * 8);
		boost::this_thread::sleep(boost::posix_time::milliseconds(waittimeMS));
	}

	return read(fd, buf, n);
}

bool Driver::namedTerminalSettings(std::string name,
								int fd,
								int baudrate,
								std::string parity,
								bool enableHwFlow,
								bool enableRawMode)
{
	Configuration* config = Configuration::getInstance();

	std::string prefix = _config_prefix + ".terminal." + name;
	baudrate = config->geti(prefix + ".baudrate", baudrate);
	parity = config->gets(prefix + ".parity", parity);
	enableHwFlow = config->getb(prefix + ".hardware_flow_control", enableHwFlow);
	enableRawMode = config->getb(prefix + ".raw_mode", enableRawMode);

	return terminalSettings(fd, baudrate, parity, enableHwFlow, enableRawMode);

}

bool Driver::terminalSettings(int fd,
								int baudrate,
								std::string parity,
								bool enableHwFlow,
								bool enableRawMode)
{
	// Set up the terminal configuration for the given port.
	struct termios options;

	tcgetattr(fd, &options);                  // get the current port settings

	// Set the baud rate
	uint32_t optbaud = 0;
	switch(baudrate)
	{
	case 9600:
		optbaud = B9600;
		break;
	case 19200:
		optbaud = B19200;
		break;
	case 38400:
		optbaud = B38400;
		break;
	case 57600:
		optbaud = B57600;
		break;
	case 115200:
		optbaud = B115200;
		break;
	default:
		warning() << "The selected baud of " << baudrate << " is not standard.";
		optbaud = baudrate;
	}


	// Set baudrates for input and output
	cfsetospeed(&options, optbaud);
	cfsetispeed(&options, optbaud);



	//parity
	boost::to_upper(parity);

	if(parity == std::string("8N1")) {	//No parity (8N1):
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS8;
	} else if(parity == std::string("7E1")) {	//Even parity (7E1):
		options.c_cflag |= PARENB;
		options.c_cflag &= ~PARODD;
		options.c_cflag &= ~CSTOPB;
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS7;
	} else if(parity == std::string("701")){
		// Odd parity (7O1):
		options.c_cflag |= PARENB;
		options.c_cflag |= PARODD;
		options.c_cflag &= ~CSTOPB;
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS7;

	} else if(parity == std::string("7M1")) {	//Mark parity
		options.c_cflag &= ~PARENB;
		options.c_cflag |= CSTOPB;
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS7;
	} else { //Space parity is setup the same as no parity (7S1):
		if(parity != std::string("7S1"))
		{
			warning() << "The parity " << parity << " is not known, use one of [8N1, 7E1, 701, 7M1, 7S1]";
		}
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS8;
    }

	// Hardware flow control
	if(enableHwFlow)
	{
		//options.c_cflag |= CNEW_RTSCTS;
	} else {
		//options.c_cflag &= ~CNEW_RTSCTS;
	}


	// raw mode
	if(enableRawMode)
	{
		//set for non-canonical (raw processing, no echo, etc.)
		/**options.c_iflag = IGNPAR; // ignore parity check close_port(int
		options.c_oflag = 0; // raw output
		options.c_lflag = 0; // raw input
		**/
		cfmakeraw(&options);
	}else{
		options.c_lflag |= (ICANON | ECHO | ECHOE);
	}

	options.c_cflag |= (CLOCAL | CREAD); // Enable the receiver and set local mode...

	// Clear terminal output flow control.
	if (tcsetattr(fd, TCSADRAIN, &options) != 0)
	{
		critical() << "could not set serial port attributes";
		return false;
	}

	if(tcflush(fd, TCIOFLUSH) == -1)
	{
		critical() << "could not purge the serial port";
		return false;
	}

	trace() << "serial set up success " << fd;

	return true;
}
