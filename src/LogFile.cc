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

#include "LogFile.h"
#include "Debug.h"
/* Project headers */
#include "MainApp.h"
#include "heli.h"
#include "RateLimiter.h"

// System Headers
#include <vector>
#include <iostream>
#include <fstream>

// Boost:: headers
#include <boost/date_time/posix_time/posix_time.hpp>


#include "LogFileWriter.h"


LogFile::LogFile()
:startTime(boost::posix_time::microsec_clock::local_time())
{
	std::stringstream ss;
	ss.str("");

	boost::posix_time::time_facet *tfacet(new boost::posix_time::time_facet("%Y-%m-%d_%H:%M:%S"));
	ss.imbue(std::locale(ss.getloc(), tfacet));
	ss << startTime;
	std::string time_folder(ss.str());



	log_folder = boost::filesystem::current_path();
	log_folder /= time_folder;

	if (boost::filesystem::exists(log_folder))
	{
		// remove the contents of the directory
		for (boost::filesystem::directory_iterator it(log_folder);
				it != boost::filesystem::directory_iterator();
				it++)
		{
			boost::filesystem::remove(*it);
		}
	}
	else // Create the new directory.
	{
		try
		{
			boost::filesystem::create_directory(log_folder);
			std::cout << "Logging in: " << log_folder.c_str();
		}
		catch (boost::filesystem::filesystem_error& fserr)
		{
			std::cout << "LogFile: Could not create directory: " << log_folder.c_str() << "Error: " << fserr.what();
		}
	}

}

LogFile::~LogFile()
{
}

LogFile* LogFile::_instance = NULL;
std::mutex LogFile::_instance_lock;

LogFile* LogFile::getInstance()
{
	std::lock_guard<std::mutex> lock(_instance_lock);

	if (NULL == _instance)
	{
		_instance = new LogFile;
	}

	return _instance;
}

void LogFile::logHeader(const std::string& name, const std::string& header)
{
	LogfileWriter::getLogger(name)->setHeader(header);
}

void LogFile::logMessage(const std::string& name, const std::string& msg)
{
	std::stringstream dataStr;

	boost::posix_time::ptime time(boost::posix_time::microsec_clock::local_time());
	dataStr << ((time-startTime).total_milliseconds()) << '\t';
	dataStr << msg;
	dataStr << std::endl;

	LogfileWriter::getLogger(name)->log(dataStr.str());
}
