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

// Includes
#include "Debug.h"

#include <string.h>


// Variables
boost::mutex Debug::cerr_lock;
boost::signals2::signal<void (std::string)> Debug::warning;
boost::signals2::signal<void (std::string)> Debug::critical;
std::string Debug::last_message = "";
int Debug::message_count = 0;

const char* LOGFILE_NAME = "messages.log";
const char* ARRAY_SEPARATOR = ", ";
const char* NUMBER_SEPARATOR = " ";


Debug::Debug(DEBUG_LEVEL lvl)
:debug_level(lvl)
{
}

Debug::Debug(const Debug& other)
:debug_level(other.debug_level)
{
	ss << other.ss.rdbuf();
}

Debug::~Debug()
{
	std::string message;
	switch(debug_level)
	{
		case WARNING:
			message += "Warning: ";
			break;
		case CRITICAL:
			message += "Critical: ";
			break;
		case MESSAGE:
			message += "Message: ";
			break;

		case DEBUG:
			message += "Debug: ";
			break;
		default:
			message += "UNKNOWN: ";
	}


	message += ss.str();

#ifndef NDEBUG
	{

		boost::mutex::scoped_lock lock(cerr_lock);
		if(strcmp(last_message.c_str(), message.c_str()) == 0)
		{
			message_count++;
			std::cerr << "\r" << message << " x" << message_count;
		}
		else
		{
			last_message = message;
			message_count = 1;
			std::cerr << std::endl << message;
		}
	}
#endif

	switch(debug_level)
	{
	case WARNING:
		LogFile::getInstance()->logMessage(LOGFILE_NAME, message);
		Debug::warning(message);
		break;

	case CRITICAL:
		LogFile::getInstance()->logMessage(LOGFILE_NAME, message);
		Debug::critical(message);
		break;

	case MESSAGE:
		LogFile::getInstance()->logMessage(LOGFILE_NAME, message);
		break;

	case DEBUG:
		// do nothing for debug messages.
		break;
	}
}



Debug& Debug::operator<<(const std::string& s)
{
	ss << s;
	return *this;
}

Debug& Debug::operator<<(const char* c)
{
	ss << c;
	return *this;
}

Debug& Debug::operator<<(const int i)
{
	ss << NUMBER_SEPARATOR << i << NUMBER_SEPARATOR;
	return *this;
}

Debug& Debug::operator<<(const unsigned int i)
{
	ss << NUMBER_SEPARATOR << i << NUMBER_SEPARATOR;
	return *this;
}

Debug& Debug::operator<<(const unsigned long i)
{
	ss << NUMBER_SEPARATOR << i << NUMBER_SEPARATOR;
	return *this;
}

Debug& Debug::operator<<(const double d)
{
	ss << NUMBER_SEPARATOR << d << NUMBER_SEPARATOR;
	return *this;
}

Debug& Debug::operator<<(std::ios_base& (*pf)(std::ios_base&))
{
	ss << pf;
	return *this;
}

Debug& Debug::operator<<(std::ostream& (*pf)(std::ostream&))
{
	ss << pf;
	return *this;
}


Debug& Debug::operator<<(const std::vector<uint8_t>& v)
{
	ss << "[";
	for (size_t i = 0; i<v.size(); i++)
	{
		// TODO check to see if the cast to an int rather than a uint ever gets to the range of errors. - Joseph
		ss << static_cast<int>(v[i]);
		if (i < v.size() - 1)
		{
			ss << ARRAY_SEPARATOR;
		}
	}
	ss << "]";
	return *this;
}
