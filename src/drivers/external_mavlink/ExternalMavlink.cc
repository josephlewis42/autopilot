/**************************************************************************
 * Copyright 2014 Joseph Lewis <joseph@josephlewis.net>
 *
 * This file is part of University of Denver Autopilot.
 *
 *     UDenver Autopilot is free software: you can redistribute it
 * 	   and/or modify it under the terms of the GNU General Public
 *     License as published by the Free Software Foundation, either
 *     version 3 of the License, or (at your option) any later version.
 *
 *     UDenver Autopilot is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License
 *     along with UDenver Autopilot. If not, see <http://www.gnu.org/licenses/>.
 *************************************************************************/

#include "ExternalMavlink.h"
#include <sys/sysinfo.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "SystemState.h"
#include "LogFile.h"
#include "mavlink.h"


const std::string GPS_LOG_FILE_NAME = "external_mavlink_gps";
const std::string GPS_LOG_FILE_FORMAT = "lat (dd)\tlon (dd)\theight (m)";
const std::string IMU_LOG_FILE_NAME = "external_mavlink_imu";
const std::string IMU_LOG_FILE_FORMAT = "roll (rad)\tpitch (rad)\tyaw (rad)\troll speed (rad/s)\tpitch speed (rad/s)\tyaw speed (rad/s)";

ExternalMavlink::ExternalMavlink()
:Plugin("External Mavlink Source","external_mavlink", 50)
{
    start(); // Start the plugin
}

bool ExternalMavlink::init()
{
    configDescribe("use_external_gps",
                   "true/false",
                   "Should we update our GPS with external sources?");
    _use_external_gps = configGetb("use_external_gps", true);

    configDescribe("use_external_imu",
                   "true/false",
                   "Should we use the external IMU?");
    _use_external_imu = configGetb("use_external_imu", true);


    configDescribe("read_path",
                   "filesystem path",
                   "The device to read from.");
    std::string serial_path = configGets("read_path", "/dev/ttyUSB0");
    
    trace() << "starting on " << serial_path;
    fd = open(serial_path.c_str(), O_RDWR | O_NOCTTY);
    trace() << "port opened";

    if(-1 == fd)
    {
        return false;
    }

    namedTerminalSettings("read_settings", fd, 57600, "8N1", false, true);
    trace() << "started";

    LogFile* lf = LogFile::getInstance();

    lf->logHeader(GPS_LOG_FILE_NAME, GPS_LOG_FILE_FORMAT);
    lf->logHeader(IMU_LOG_FILE_NAME, IMU_LOG_FILE_FORMAT);

    return true; // we setup correctly.
}

void ExternalMavlink::teardown()
{
    if(fd > 0)
    {
        close(fd);
    }
}


void ExternalMavlink::loop()
{

    // read message
    char buf[1024];
	int bytes_received = 0;
    bytes_received = readDevice(fd, buf, 1024);
    

    // parse message
	for (int i=0; i<bytes_received; i++)
	{
		if(mavlink_parse_char(MAVLINK_COMM_1, buf[i], &_msg, &_status))
		{
			switch(_msg.msgid)
			{
            case MAVLINK_MSG_ID_HEARTBEAT:
                break;
            case MAVLINK_MSG_ID_RC_CHANNELS_SCALED:
                break;
            case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
                break;
            case MAVLINK_MSG_ID_STATUSTEXT:
                {
                mavlink_statustext_t text;
                mavlink_msg_statustext_decode(&_msg, &text);
                char message[51];
                message[50] = 0; // add null terminator

                for(int i = 0; i < 50; i++) message[i] = text.text[i];
            
                
                info() << "Got status message: " << message << " severity: " << text.severity;
                
                break;
                }
            case MAVLINK_MSG_ID_ATTITUDE: // IMU
                {
                mavlink_attitude_t att;
                mavlink_msg_attitude_decode(&_msg, &att);

                std::vector<float> attitude {att.roll, att.pitch, att.yaw,
                                             att.rollspeed, att.pitchspeed,
                                             att.yawspeed};
                
                LogFile::getInstance()->logData(IMU_LOG_FILE_NAME, attitude);
                break;
                }
            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: // gps
                {
                mavlink_global_position_int_t pos;
                mavlink_msg_global_position_int_decode(&_msg, &pos);
                
                std::vector<double> position {pos.lat / (double) 1E7, pos.lon / (double) 1E7, pos.alt / 1000.0};
                LogFile::getInstance()->logData(GPS_LOG_FILE_NAME, position);

                debug() << "got position lat: " << position[0] << " lon: " << position[1] << " height: " << position[2];
                break;
                }
            default:
            // lf->logData(GPS_LOG_FILE_NAME, GPS_LOG_FILE_FORMAT);
                if( _msg.msgid >= 150 && _msg.msgid <= 240)
                {
                    trace() << "got custom message, ignoring it.";
                }
                else
                {   
                    debug() << "got unknown message with id: " << _msg.msgid;
                }
            }
        }
    }
}


/**
void ExternalMavlink::sendMavlinkMsg(std::vector<mavlink_message_t>& msgs, int uasId, int sendRateHz, int msgNumber)
{
    if(! isEnabled()) return;

    if(msgNumber % sendRateHz == 0) // do this once a second.
    {
        debug() << "Sending CPU Utilization";

        mavlink_message_t msg;
        mavlink_msg_udenver_cpu_usage_pack(uasId, 40, &msg, cpu_utilization.get(), totalram.load(), freeram.load());
        msgs.push_back(msg);
    }
};
**/

