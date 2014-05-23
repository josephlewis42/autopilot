/*
 * MdlAltimiter.cpp
 *
 *  Created on: May 16, 2014
 *      Author: joseph
 */

#include "MdlAltimiter.h"
/* File Handling Headers */
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/* C Headers */
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <cstdlib>
#include <math.h>


/* Project Headers */
#include "Configuration.h"
#include "Debug.h"
#include "gx3_read_serial.h"
#include "MainApp.h"
#include "message_parser.h"
#include "ack_handler.h"
#include "gx3_send_serial.h"
#include "QGCLink.h"



const std::string ALTIMITER_ENABLED = "mdl_altimiter.enabled";
const bool ALTIMITER_ENABLED_DEFAULT = false;
const std::string ALTIMITER_PATH = "mdl_altimiter.device";
const std::string ALTIMITER_PATH_DEFAULT = "/dev/ttyS1";


MdlAltimiter::MdlAltimiter()
:Driver("MDL Altimiter", "mdl_altimiter")
{
	if(!Configuration::getInstance()->getb(ALTIMITER_ENABLED, ALTIMITER_ENABLED_DEFAULT))
	{
		warning() << "MDL Altimiter disabled!";
		return;
	}

	std::string serial_path = Configuration::getInstance()->gets(ALTIMITER_PATH, ALTIMITER_PATH_DEFAULT);

	_serialFd = open(serial_path.c_str(), O_RDWR | O_NOCTTY);

	// Set up the terminal.
	if(!namedTerminalSettings("Altimiter1", _serialFd, 38400, "8N1", false, true))
	{
		warning() << "Could not setup serial!";
	}
	else
	{
		boost::thread(mainLoop);
	}
}

MdlAltimiter::~MdlAltimiter() {
	close(_serialFd);
}

void MdlAltimiter::mainLoop() {

	char first = '\0', second = '\0';
	int multiplierCM = 10;
	int numberToAverage = 100;
	int averagedThusFar = 0;
	int errorsThusFar = 0;
	int sum = 0;

	while(! terminateRequested())
	{

		while(first >> 4 != 0x8)
		{
			readDevice(_serialFd, &first, 1);
		}

		if(readDevice(_serialFd, &second, 1) < 1)
		{
			continue;
		}

		// Average a number of results because the error on this device is huge.
		if(averagedThusFar < numberToAverage)
		{
			if(first == 0xbf)
			{
				errorsThusFar++;
			}
			else
			{
				sum += second;
			}

			averagedThusFar++;
		}
		else
		{
			float distance = (averagedThusFar - errorsThusFar == 0)? 0.0 :
									((float(sum) / (averagedThusFar - errorsThusFar)) * multiplierCM);

			debug() << "Distance: " << distance / 100 << "m " << distance << " cm (" << distance * 0.393701 << " in)";

			sum = 0;
			errorsThusFar = 0;
			averagedThusFar = 0;
		}
	}
}
