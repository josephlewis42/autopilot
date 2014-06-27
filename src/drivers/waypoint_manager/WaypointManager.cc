/*
 * Copyright 2014 Joseph Lewis <joseph@josephlewis.net>
 *
 * This file is part of University of Denver Autopilot.
 * Dual licensed under the GPL v 3 and the Apache 2.0 License
 */

#include "WaypointManager.h"
#include "mavlink_types.h"
#include <sys/time.h>
#include <time.h>


mavlink_system_t mavlink_system;

/* 3: Define waypoint helper functions */
void mavlink_missionlib_send_message(mavlink_message_t* msg);
void mavlink_missionlib_send_gcs_string(const char* string);
uint64_t mavlink_missionlib_get_system_timestamp();
void mavlink_missionlib_current_waypoint_changed(uint16_t index, float param1,
		float param2, float param3, float param4, float param5_lat_x,
		float param6_lon_y, float param7_alt_z, uint8_t frame, uint16_t command);


/* 4: Include waypoint protocol */
#include "waypoints.h"
#include "waypoints.c"
mavlink_wpm_storage wpm;

/* Provide the interface functions for the waypoint manager */

/*
 *  @brief Sends a MAVLink message over UDP
 */

void mavlink_missionlib_send_message(mavlink_message_t* msg)
{
    mavlink_message_t internal;
    internal = *msg;

    WaypointManager::getInstance()->addMessage(internal);
}

void mavlink_missionlib_send_gcs_string(const char* string)
{
	const int len = 50;
	mavlink_statustext_t status;
	int i = 0;
	while (i < len - 1)
	{
		status.text[i] = string[i];
		if (string[i] == '\0')
			break;
		i++;
	}
	status.text[i] = '\0'; // Enforce null termination
	mavlink_message_t msg;

	mavlink_msg_statustext_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &status);
	mavlink_missionlib_send_message(&msg);
}

uint64_t mavlink_missionlib_get_system_timestamp()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
}

void mavlink_missionlib_current_waypoint_changed(uint16_t index, float param1,
		float param2, float param3, float param4, float param5_lat_x,
		float param6_lon_y, float param7_alt_z, uint8_t frame, uint16_t command)
{
    WaypointManager::getInstance()->warning("current waypoint modified");

}




/// TRUE WAYPOINT MANAGER STUFF

WaypointManager* WaypointManager::_instance = NULL;
std::mutex WaypointManager::_instance_lock;

WaypointManager* WaypointManager::getInstance()
{
    std::lock_guard<std::mutex> lock(_instance_lock);
    if (_instance == NULL)
    {
        _instance = new WaypointManager;
    }
    return _instance;
}



WaypointManager::WaypointManager()
    :Driver("Waypoint Manager","waypoint_manager")
{
    mavlink_wpm_init(&wpm);
	mavlink_system.sysid = 100; // TODO make this dynamic
	mavlink_system.compid = 20;

}

WaypointManager::~WaypointManager()
{

}

void WaypointManager::sendMavlinkMsg(std::vector<mavlink_message_t>& msgs, int uasId, int sendRateHz, int msgNumber)
{
    if(! isEnabled()) return;

    {
        std::lock_guard<std::mutex> lock(_messageQueueLock);
        for(mavlink_message_t& msg : _messageQueue)
        {
            msgs.push_back(msg);
        }

        _messageQueue.empty();
    }
};


bool WaypointManager::recvMavlinkMsg(const mavlink_message_t& msg)
{
    if(! isEnabled()) return false;

//    warning() << "got message";
    mavlink_wpm_message_handler(&msg);

    return false;
};
