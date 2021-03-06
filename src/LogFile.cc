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

/* Project headers */
#include "LogFile.h"
#include "Debug.h"
#include "LogFileWriter.h"

// System Headers
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>


LogFile::LogFile()
:startTime(std::chrono::system_clock::now()),
 log_folder(),
 _checkpoint(0)
{
    setupLogFolder();
}

LogFile::~LogFile()
{
}

void LogFile::newLogPoint()
{
    _checkpoint = _checkpoint.load() + 1;
    setupLogFolder();
}


void LogFile::setupLogFolder()
{
    Path newFolder = Path();
    std::time_t now_c = std::chrono::system_clock::to_time_t(startTime);

    char time_folder[1000];
    std::strftime(time_folder, sizeof(time_folder), "%F_%T", std::localtime(&now_c));

    newFolder /= time_folder;
    newFolder /= std::to_string(_checkpoint.load());

    newFolder.remove_all();
    newFolder.create_directories();
    std::cout << "Logging in: " << log_folder.toString();

    {
        std::lock_guard<std::mutex> lg(_logFolderLock);
        log_folder = newFolder;
    }
}


void LogFile::logHeader(const std::string& name, const std::string& header)
{
    LogfileWriter::getLogger(name)->setHeader(header);
}

void LogFile::logMessage(const std::string& name, const std::string& msg)
{
    std::stringstream dataStr;

    dataStr << getMicrosSinceInit() << '\t';
    dataStr << msg;
    dataStr << std::endl;

    LogfileWriter::getLogger(name)->log(dataStr.str());
}
