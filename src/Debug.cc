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

#include <string.h>
#include <mutex> // c++11

/* Project Headers */
#include "Debug.h"
#include "LogFile.h"

// Variables
std::mutex Debug::cerr_lock;
boost::signals2::signal<void (std::string)> Debug::warningSignal;
boost::signals2::signal<void (std::string)> Debug::criticalSignal;
std::string Debug::last_message = "";
int Debug::message_count = 0;

const char* LOGFILE_NAME = "messages.log";
const char* ARRAY_SEPARATOR = ", ";
const char* NUMBER_SEPARATOR = " ";


Debug::Debug(DEBUG_LEVEL lvl, std::string prefix)
:debug_level(lvl)
{
	appendLevel();
	ss << prefix;
}

Debug::Debug(const Debug& other)
:debug_level(other.debug_level)
{
	ss << other.ss.rdbuf();
}

void Debug::appendLevel()
{
	switch(debug_level)
	{
		case WARNING:
			ss << "Warning:  ";
			break;
		case CRITICAL:
			ss << "Critical: ";
			break;
		case MESSAGE:
			ss << "Message:  ";
			break;
		case DEBUG:
			ss << "Debug:    ";
			break;
		case IGNORE:
			ss << "Ignore:   ";
			break;
		default:
			ss << "UNKNOWN:  ";
	}
}

Debug::~Debug()
{
	if(debug_level == IGNORE)
	{
		return;
	}

	std::string message = ss.str();

#ifndef NDEBUG
	{

		std::lock_guard<std::mutex> lock(cerr_lock);
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
		Debug::warningSignal(message);
		break;

	case CRITICAL:
		LogFile::getInstance()->logMessage(LOGFILE_NAME, message);
		Debug::criticalSignal(message);
		break;

	case MESSAGE:
		LogFile::getInstance()->logMessage(LOGFILE_NAME, message);
		break;

	case DEBUG:
	case IGNORE:
		// do nothing for debug/ignore messages.
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

Debug& Debug::operator<<(const void* ptr)
{
	ss << std::hex << ptr << std::dec;
	return *this;
}
