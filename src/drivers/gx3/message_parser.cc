/**************************************************************************
 * Copyright 2012 Bryan Godbolt
 * Copyright 2014 Joseph Lewis <josephlewis42@gmail.com>
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

#include "message_parser.h"

/* STL Headers */
#include <bitset>
#include <thread>
#include <chrono>

/* Boost Headers */
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
namespace blas = boost::numeric::ublas;

#include "heli.h"


/* Project Headers */
#include "Debug.h"
#include "LogFile.h"

// Constants
std::string const IMU::message_parser::LOG_LLH_POS = "GX3 Estimated LLH Position";
std::string const IMU::message_parser::LOG_NED_VEL = "GX3 Estimated NED Velocity";
std::string const IMU::message_parser::LOG_ORIENTATION = "GX3 Nav Orientation Matrix";
std::string const IMU::message_parser::LOG_ANG_RATE = "GX3 Nav Angular Rates";
std::string const IMU::message_parser::LOG_ANG_RATE_FILTERED = "GX3 Nav Angular Rates Filtered";
std::string const IMU::message_parser::LOG_EULER = "GX3 Nav Euler Angles";
std::string const IMU::message_parser::Log_AHRS_Euler = "GX3 AHRS Euler Angles";
std::string const IMU::message_parser::Log_AHRS_Ang_Rate = "GX3 AHRS Angular Rates";
std::string const IMU::message_parser::Log_AHRS_Ang_Rate_Filtered = "GX3 AHRS Angular Rates Filtered";


IMU::message_parser::message_parser()
{

}

IMU::message_parser::~message_parser()
{

}

void IMU::message_parser::operator()()
{

    // set up log files
    LogFile *log = LogFile::getInstance();
    log->logHeader(LOG_LLH_POS, "Latitude Longitude Height Valid");
    log->logData(LOG_LLH_POS, std::vector<double>());

    log->logHeader(LOG_NED_VEL, "Vel_X, Vel_Y, Vel_Z Valid");
    log->logData(LOG_NED_VEL, std::vector<double>());

    log->logHeader(LOG_ORIENTATION, "R11, R12, R13, R21, R22, R23, R31, R32, R33 Valid");
    log->logData(LOG_ORIENTATION, std::vector<double>());

    log->logHeader(LOG_ANG_RATE, "X, Y, Z Valid");
    log->logData(LOG_ANG_RATE, std::vector<double>());

    log->logHeader(LOG_ANG_RATE_FILTERED, "X, Y, Z");
    log->logData(LOG_ANG_RATE_FILTERED, std::vector<double>());

    log->logHeader(Log_AHRS_Ang_Rate, "X, Y, Z");
    log->logData(Log_AHRS_Ang_Rate, std::vector<double>());

    log->logHeader(Log_AHRS_Ang_Rate_Filtered, "X, Y, Z");
    log->logData(Log_AHRS_Ang_Rate_Filtered, std::vector<double>());

    log->logHeader(LOG_EULER, "Roll Pitch Yaw Valid");
    log->logData(LOG_EULER, std::vector<double>());

    while (true)
    {
        // parse messages in order of priority
        while (!IMU::getInstance()->nav_queue.empty())
        {
            parse_nav_message(IMU::getInstance()->nav_queue.pop());
        }
        while (!IMU::getInstance()->ahrs_queue.empty())
        {
            // ahrs messages currently unhandled
            parse_ahrs_message(IMU::getInstance()->ahrs_queue.pop());
        }
        while (!IMU::getInstance()->gps_queue.empty())
        {
            // gps messages currently unhandled
            IMU::getInstance()->gps_queue.pop();
        }
        while (!IMU::getInstance()->command_queue.empty())
        {
            parse_command_message(IMU::getInstance()->command_queue.pop());
        }
        std::this_thread::sleep_for( std::chrono::milliseconds( 5 ) );// don't need precise timing, just want to yield
    }
}

void IMU::message_parser::parse_ahrs_message(const std::vector<uint8_t>& message)
{
    IMU* imu = IMU::getInstance();

    // divide message into fields
    std::vector<std::vector<uint8_t> > payload;
    {
        std::vector<uint8_t>::const_iterator it = message.begin() + 4;
        while (it != message.end())
        {
            payload.push_back(std::vector<uint8_t>(it, it + *it));
            it += *it;
        }
    }
    for (std::vector<std::vector<uint8_t> >::const_iterator it = payload.begin();
            it != payload.end(); ++it)
    {
        switch ((*it)[1])
        {
        case 0x05: // scaled gyro
        {
            blas::vector<double> ang_rate(3);
            ang_rate.clear();
            std::vector<uint8_t>::const_iterator first_data = (*it).begin() + 2;
            ang_rate[0] = raw_to_float(first_data);
            ang_rate[1] = raw_to_float(first_data + 4);
            ang_rate[2] = raw_to_float(first_data + 8);
            LogFile::getInstance()->logData(Log_AHRS_Ang_Rate, ang_rate);
            for (int i=0; i<3; ++i)
                ang_rate[i] = ahrs_filters[i](ang_rate[i]);
            LogFile::getInstance()->logData(Log_AHRS_Ang_Rate_Filtered, ang_rate);
            imu->set_ahrs_angular_rate(ang_rate);
//			debug() << "ahrs ang rage" << ang_rate;
            break;
        }
        case 0x0C: //euler angles
        {
            blas::vector<double> euler(3);
            euler.clear();
            std::vector<uint8_t>::const_iterator first_data = (*it).begin() + 2;
            euler[0] = raw_to_float(first_data);
            euler[1] = raw_to_float(first_data + 4);
            euler[2] = raw_to_float(first_data + 8);
            LogFile::getInstance()->logData(Log_AHRS_Euler, euler);
            imu->set_ahrs_euler(euler);
			imu->debug() << "AHRS Euler roll: " << euler[0] << " pitch: " << euler[1] << " yaw: " << euler[2];
            break;
        }
        default:
            imu->warning() << "Message Parser: Received unhandled AHRS message with descriptor: " << std::hex << it->at(1);
            break;
        }
    }
}

void IMU::message_parser::parse_nav_message(const std::vector<uint8_t>& message)
{
    // divide message into fields
    std::vector<std::vector<uint8_t> > payload;
    {
        std::vector<uint8_t>::const_iterator it = message.begin() + 4;
        while (it != message.end())
        {
            payload.push_back(std::vector<uint8_t>(it, it + *it));
            it += *it;
        }
    }
    for (std::vector<std::vector<uint8_t> >::const_iterator it = payload.begin();
            it != payload.end(); ++it)
    {
        switch ((*it)[1])
        {
        case 0x10: // Filter Status
        {
            uint16_t filter_state = static_cast<uint16_t>((*it)[2] << 8) + (*it)[3];
            std::bitset<16> status_flags(static_cast<uint16_t>((*it)[6] << 8) + (*it)[7]);

            IMU* imu = IMU::getInstance();
            imu->set_gx3_mode(static_cast<GX3_MODE>(filter_state));

//			if (filter_state == INIT)
            {
                std::string message;
                if (status_flags[12] != nav_status_flags[12])
                {
                    if (status_flags[12])
                        message += "Att not Init. ";
                    else
                        message += "Att Init. ";
                }
                if (status_flags[13] != nav_status_flags[13])
                {
                    if (status_flags[13])
                        message += "Pos not Init. ";
                    else
                        message += "Pos Init. ";
                }
                if (!message.empty())
                {
                    imu->set_gx3_status_message(message);
                    imu->message() << message;
                }
            }
            if (filter_state == RUNNING)
            {
                std::string message;
                if (status_flags[0] && status_flags[0] != nav_status_flags[0])
                    message += "IMU unavailable";
                if (status_flags[1] && status_flags[1] != nav_status_flags[1])
                    message += "GPS unavailable";
                if (status_flags[3] && status_flags[3] != nav_status_flags[3])
                    message += "Matrix Singularity";
                if (status_flags[4] && status_flags[4] != nav_status_flags[4])
                {
                    message += "Pos cov high";
                }
                if (status_flags[5] && status_flags[5] != nav_status_flags[5])
                {
                    message += "Vel cov high";
                }
                if (status_flags[6] && status_flags[6] != nav_status_flags[6])
                {
                    message += "Att cov high";
                }
                if (status_flags[7] && status_flags[7] != nav_status_flags[7])
                    message += "NAN in soln";

                if (!message.empty())
                {
                    imu->critical() << message;
                    imu->set_gx3_status_message(message);
                }
            }

            nav_status_flags = status_flags;
            break;
        }
        case 0x01: // LLH Position
        {
            double lat = raw_to_double((*it).begin()+2, (*it).begin()+10);
            double lon = raw_to_double((*it).begin()+10, (*it).begin()+18);
            double height = raw_to_double((*it).begin()+18, (*it).begin()+26);
            uint8_t valid = (*it)[27];

            GPSPosition pos(lat, lon, height);

            std::vector<double> log {lat, lon, height, static_cast<double>(valid)};
            LogFile::getInstance()->logData(LOG_LLH_POS, log);

            if (valid)
            {
                // update current position measurement
                IMU::getInstance()->setPosition(pos);
            }

            break;
        }
        case 0x02:
        {
            blas::vector<double> velocity(3);
            std::vector<uint8_t>::const_iterator first_data = (*it).begin() + 2;
            velocity[0] = raw_to_float(first_data, first_data + 4);
            velocity[1] = raw_to_float(first_data + 4, first_data + 8);
            velocity[2] = raw_to_float(first_data + 8, first_data + 12);
            uint8_t valid = *(first_data + 13);

            std::vector<double> log(velocity.begin(), velocity.end());
            log.push_back(static_cast<double>(valid));
            LogFile::getInstance()->logData(LOG_NED_VEL, log);
            if (valid)
            {
                IMU::getInstance()->set_velocity(velocity);
            }
            break;
        }
        case 0x04:  // rotation matrix
        {
            blas::matrix<double> rotation(3,3);
            std::vector<uint8_t>::const_iterator first_data = (*it).begin() + 2;
            rotation(0,0) = raw_to_float(first_data, first_data + 4);
            rotation(0,1) = raw_to_float(first_data + 4, first_data + 8);
            rotation(0,2) = raw_to_float(first_data + 8, first_data + 12);
            rotation(1,0) = raw_to_float(first_data + 12, first_data + 16);
            rotation(1,1) = raw_to_float(first_data + 16, first_data + 20);
            rotation(1,2) = raw_to_float(first_data + 20, first_data + 24);
            rotation(2,0) = raw_to_float(first_data + 24, first_data + 28);
            rotation(2,1) = raw_to_float(first_data + 28, first_data + 32);
            rotation(2,2) = raw_to_float(first_data + 32, first_data + 36);
            uint8_t valid = *(first_data + 37);
//			debug() << "Orientation Matrix: " << rotation;
            std::vector<double> orientation(9, 0);
            for (int i=0; i<3; i++)
            {
                for (int j=0; j<3; j++)
                {
                    orientation[i*3+j] = rotation(i,j);
                }
            }

            orientation.push_back(static_cast<double>(valid));
            LogFile::getInstance()->logData(LOG_ORIENTATION, orientation);
            if (valid)
            {
                IMU::getInstance()->set_nav_rotation(rotation);
                // also log euler if rotation valid
                blas::vector<double> euler(IMU::getInstance()->get_euler());
                LogFile::getInstance()->logData("Euler Angles (converted)", euler);
            }

            break;
        }
        case 0x05: //euler angles
        {
            blas::vector<double> euler(3);
            euler.clear();
            std::vector<uint8_t>::const_iterator first_data = (*it).begin() + 2;
            euler[0] = raw_to_float(first_data);
            euler[1] = raw_to_float(first_data + 4);
            euler[2] = raw_to_float(first_data + 8);
            uint8_t valid = *(first_data + 13);
            std::vector<double> log(euler.begin(), euler.end());
            log.push_back(static_cast<double>(valid));
            LogFile::getInstance()->logData(LOG_EULER, log);

            if (valid)
            {
                IMU::getInstance()->set_nav_euler(euler);
            }
            break;
        }
        case 0x0E:
        {
            blas::vector<double> angular_rate(3);
            std::vector<uint8_t>::const_iterator first_data = (*it).begin() + 2;
            angular_rate[0] = raw_to_float(first_data);
            angular_rate[1] = raw_to_float(first_data + 4);
            angular_rate[2] = raw_to_float(first_data + 8);
            uint8_t valid = *(first_data + 13);

            std::vector<double> log(angular_rate.begin(), angular_rate.end());
            log.push_back(valid);
            LogFile::getInstance()->logData(LOG_ANG_RATE, log);

            if (valid)
            {
                for (int i=0; i<3; ++i)
                {
                    angular_rate[i] = nav_filters[i](angular_rate[i]);
                }
                LogFile::getInstance()->logData(LOG_ANG_RATE_FILTERED, angular_rate);
                IMU::getInstance()->set_nav_angular_rate(angular_rate);
            }
            break;
        }
        default:
            IMU::getInstance()->warning() << "Message Parser: Received unhandled NAV message with descriptor: " << std::hex << it->at(1);
            break;
        }
    }
    IMU::getInstance()->writeToSystemState();
}

void IMU::message_parser::parse_command_message(const std::vector<uint8_t>& message)
{
    std::vector<std::vector<uint8_t> > payload;
    {
        std::vector<uint8_t>::const_iterator it = message.begin() + 4;
        while (it != message.end())
        {
            payload.push_back(std::vector<uint8_t>(it, it + *it));
            it += *it;
        }
    }
    for (std::vector<std::vector<uint8_t> >::const_iterator it = payload.begin();
            it != payload.end(); ++it)
    {
        switch (it->at(1))
        {
        case 0xF1: //ACK/NACK
        {
//			debug() << "Received ACK/NACK, emitting ack signal with code " << it->at(2);
            IMU::getInstance()->ack(std::vector<uint8_t>(it->begin() + 2, it->begin() + it->at(0)));
            break;
        }
        default:
            IMU::getInstance()->warning() << "Message Parser: Received unknown command message with descriptor: " << std::hex << it->at(1);
            break;
        }
    }
}
