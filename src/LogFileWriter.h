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

#ifndef LOGFILEWRITER_H_
#define LOGFILEWRITER_H_

#include "Driver.h"
#include "LogFile.h"
#include "ThreadSafeVariable.h"

#include <sstream>
#include <map>
#include <string>
#include <mutex>

#include <boost/filesystem.hpp>

/**
 * A basic file writer that accepts strings and writes them
 * to a file periodically.
 */
class LogfileWriter : public Driver
{
private:
    static std::recursive_mutex _ALL_LOGGERS_MUTEX;
    static std::map<std::string, LogfileWriter* > _ALL_LOGGERS;

    std::string _logName;
    ThreadSafeVariable<std::string> _header;

    mutable std::mutex _currentBufferLock;
    std::stringstream* _currentBuffer;

    std::stringstream _firstBuffer;
    std::stringstream _secondBuffer;

    /// swaps the buffers and returns a pointer to the one that was just swapped out.
    std::stringstream* swapBuffers();

    /// a function that writes the buffers out.
    void writeThread();

    /// gets the path to the log file
    boost::filesystem::path getLogPath();

    LogfileWriter(std::string path);
    ~LogfileWriter();
public:
    static LogfileWriter* getLogger(const std::string& path);

    void log(const std::string& message);
    void setHeader(const std::string& header)
    {
        _header = header;
    }

};

#endif /* LOGFILEWRITER_H_ */
