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
:startTime(std::chrono::system_clock::now())
{

    std::time_t now_c = std::chrono::system_clock::to_time_t(startTime);
    //std::string time_folder(std::put_time(std::localtime(&now_c), "%F_%T")); // TODO use this once compilers support full c++11 standard

    char time_folder[1000];
    std::strftime(time_folder, sizeof(time_folder), "%F_%T", std::localtime(&now_c));

    log_folder = boost::filesystem::current_path();
    log_folder /= time_folder;

    try
    {
        boost::filesystem::remove_all(log_folder);
        boost::filesystem::create_directories(log_folder);
        std::cout << "Logging in: " << log_folder.c_str();
    }
    catch (boost::filesystem::filesystem_error& fserr)
    {
        std::cout << "LogFile: Could not create directory: " << log_folder.c_str() << "Error: " << fserr.what();
    }
}

LogFile::~LogFile()
{
}


void LogFile::logHeader(const std::string& name, const std::string& header)
{
    LogfileWriter::getLogger(name)->setHeader(header);
}

void LogFile::logMessage(const std::string& name, const std::string& msg)
{
    std::stringstream dataStr;

    dataStr << getMsSinceInit() << '\t';
    dataStr << msg;
    dataStr << std::endl;

    LogfileWriter::getLogger(name)->log(dataStr.str());
}
