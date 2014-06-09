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
const bool ALTIMITER_ENABLED_DEFAULT = true;
const std::string ALTIMITER_PATH = "mdl_altimiter.device";
const std::string ALTIMITER_PATH_DEFAULT = "/dev/ttyUSB0";


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
		debug() << "Altimiter set up!";
		boost::thread* altimiter_thread = new boost::thread(&MdlAltimiter::mainLoop, this);
	}
}

MdlAltimiter::~MdlAltimiter() {
	close(_serialFd);
}

void MdlAltimiter::mainLoop() {

	uint8_t first = '\0', second = '\0';
	uint16_t multiplierCM = 10;
	uint16_t numberToAverage = 100;
	uint16_t averagedThusFar = 0;
	uint16_t errorsThusFar = 0;
	uint16_t sum = 0;

	debug() << "Started main Altimiter loop ";

	while(! terminateRequested())
	{
		while(((first >> 6) & 0b11) != 0x2)
		{
			readDevice(_serialFd, &first, 1);
		}
		if((readDevice(_serialFd, &second, 1) >> 6) != 0x0)
		{
			continue;
		}

		// Average a number of results because the error on this device is huge.
		if(averagedThusFar < numberToAverage)
		{	
			uint16_t first_masked = first & 0b00111111;
			uint16_t second_masked = second & 0b00111111;
			uint16_t decoded_dist = (first_masked << 6) | second_masked;
			sum += decoded_dist;
			averagedThusFar++;
		}
		else
		{
			float distance = (float(sum) / float(averagedThusFar)) * multiplierCM;

			debug() << "Distance: " << distance / 100 << "m " << distance << " cm (" << distance * 0.393701 << " in)";

			sum = 0;
			errorsThusFar = 0;
			averagedThusFar = 0;
		}
		first = '\0';
		second = '\0';
	}
}
