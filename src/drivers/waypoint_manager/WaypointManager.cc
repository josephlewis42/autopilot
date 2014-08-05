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

   WaypointManager::getInstance()->warning() << string;
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
    WaypointManager::getInstance()->warning() << "current waypoint modified command: " << command;

}




/// TRUE WAYPOINT MANAGER STUFF

WaypointManager::WaypointManager()
    :Plugin("Waypoint Manager","waypoint_manager", 2)
{
    mavlink_wpm_init(&wpm);
	mavlink_system.sysid = 100; // TODO make this dynamic
	mavlink_system.compid = 20;

}

WaypointManager::~WaypointManager()
{

}

bool WaypointManager::init()
{
    return true;
}

void WaypointManager::loop()
{
    mavlink_wpm_loop(); // do waypoint timeouts.


    // Check to see if we are at the waypoint/have completed it.
        // on yes, goto the next point
        // on nothing left, just hover
}

void WaypointManager::teardown()
{
    // notify QGC that we are going down?
}

void WaypointManager::sendMavlinkMsg(std::vector<mavlink_message_t>& msgs, int uasId, int sendRateHz, int msgNumber)
{
    if(! isEnabled()) return;


    if(shouldSendMavlinkMessage(msgNumber, sendRateHz, 5)) // limit to 5 hz then burst all messages
    {
        std::lock_guard<std::mutex> lock(_messageQueueLock);
        for(mavlink_message_t& msg : _messageQueue)
        {
            debug() << "sending message with id: " << msg.msgid;
            msgs.push_back(msg);

        }

        _messageQueue.empty();
    }
};


bool WaypointManager::recvMavlinkMsg(const mavlink_message_t& msg)
{
    if(! isEnabled()) return false;

    info() << "got message";
    mavlink_wpm_message_handler(&msg);

    return false;
};
