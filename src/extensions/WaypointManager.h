/*
 * Copyright 2014 Joseph Lewis <joseph@josephlewis.net>
 *
 * This file is part of University of Denver Autopilot.
 * Dual licensed under the GPL v 3 and the Apache 2.0 License
 */

#pragma once
#ifndef WAYPOINT_MANAGER_H
#define WAYPOINT_MANAGER_H

#include <atomic>  // Used for atomic types
#include <mutex>   // Used for singleton design.
#include "Driver.h" // All drivers implement this.
#include <vector>

/**
 * Provides an interface to the performance of Linux.
 **/
class WaypointManager: public Driver
{
public:
    /**
    Returns the one allowed instance of this Driver
    **/
    static WaypointManager* getInstance();
    virtual void sendMavlinkMsg(std::vector<mavlink_message_t>& msgs, int uasId, int sendRateHz, int msgNumber) override;
    virtual bool recvMavlinkMsg(const mavlink_message_t& msg) override;

    void addMessage(mavlink_message_t msg)
    {
        std::lock_guard<std::mutex> lock(_messageQueueLock);
        _messageQueue.push_back(msg);
    }


private:
    static WaypointManager* _instance;
    static std::mutex _instance_lock;

    WaypointManager();
    virtual ~WaypointManager();
    std::mutex _messageQueueLock;
    std::vector<mavlink_message_t> _messageQueue;
};

#endif /* LINUX_H */
