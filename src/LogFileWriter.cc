/*******************************************************************************
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
 ******************************************************************************/

#include "LogFileWriter.h"
#include "RateLimiter.h"

#include <exception>
#include <iostream>
#include <fstream>


// static members
std::map<std::string, LogfileWriter*> LogfileWriter::_ALL_LOGGERS;
boost::mutex LogfileWriter::_ALL_LOGGERS_MUTEX;


LogfileWriter* LogfileWriter::getLogger(const std::string& path)
{
	boost::mutex::scoped_lock lock(_ALL_LOGGERS_MUTEX);
	if(_ALL_LOGGERS.count(path) > 0)
	{
		return _ALL_LOGGERS[path];
	}

	LogfileWriter* newLogger = new LogfileWriter(path);

	_ALL_LOGGERS[path] = newLogger;

	return newLogger;
}



LogfileWriter::LogfileWriter(std::string path)
:Driver("Logger", "log")
{
	_logName = path;
	_currentBuffer = &_firstBuffer;

	debug() << "Created for " << path;

	// all logging write threads will die when the software shuts down.
	new boost::thread(boost::bind(&LogfileWriter::writeThread, this));
}

LogfileWriter::~LogfileWriter()
{
	boost::mutex::scoped_lock lock(_ALL_LOGGERS_MUTEX);
	_ALL_LOGGERS.erase(_logName);
}

boost::filesystem::path LogfileWriter::getLogPath()
{
	boost::filesystem::path filename;
	if (!(boost::filesystem::path(_logName)).has_extension())
	{
		filename = LogFile::getInstance()->getLogFolder() / (_logName + ".dat");
	}
	else
	{
		filename = LogFile::getInstance()->getLogFolder() / _logName;
	}

	return filename;
}

std::stringstream* LogfileWriter::swapBuffers()
{
	boost::mutex::scoped_lock lock(_currentBufferLock);

	std::stringstream* firstPtr = &_firstBuffer;
	std::stringstream* secondPtr = &_secondBuffer;

	if(_currentBuffer == firstPtr)
	{
		_currentBuffer = secondPtr;
		return firstPtr;
	}
	else
	{
		_currentBuffer = firstPtr;
		return secondPtr;
	}
}

void LogfileWriter::log(const std::string& message)
{
	boost::mutex::scoped_lock lock(_currentBufferLock);
	*_currentBuffer << message;
}

void LogfileWriter::writeThread()
{
	boost::filesystem::path filename = getLogPath();

	RateLimiter rl(2);

	try
	{
		bool existed = boost::filesystem::exists(filename);
		std::fstream output(filename.c_str(), std::fstream::out | std::fstream::ate | std::fstream::app);

		if(! existed)
		{
			debug() << "Creating log file " << filename.c_str();
			std::string header = _header;
			output << "Time(ms)\t" << header << std::endl;
		}

		while(! terminateRequested())
		{
			rl.wait();

			std::stringstream* writeBuffer = swapBuffers();
			output << writeBuffer->str();
			writeBuffer->str("");

			rl.finishedCriticalSection();
		}

		output.close();
	}
	catch (boost::filesystem::filesystem_error& fserr)
	{
		critical() << fserr.what();

		while(! terminateRequested())
		{
			rl.wait();
			critical() << "Discarding: " << filename.c_str() << " data b/c the file could not be opened.";

			swapBuffers()->str("");
			rl.finishedCriticalSection();
		}
	}
}
