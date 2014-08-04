/*
 * Copyright 2014 Joseph Lewis <joseph@josephlewis.net>
 *
 * This file is part of University of Denver Autopilot.
 * Dual licensed under the GPL v 3 and the Apache 2.0 License
 */

#pragma once
#ifndef HIL_H
#define HIL_H

#include <atomic>  // Used for atomic types
#include <mutex>   // Used for singleton design.
#include "Plugin.h" // All drivers implement this.
#include <vector>

#include "Singleton.h"

/**
 * Provides a mechanism for performing HIL simulation using the Mavlink v 1.0 packets
 * and QGroundControl.
 **/
class Hil: public Plugin, public Singleton<Hil>
{
    friend Singleton<Hil>;
public:
    /**
    Returns the one allowed instance of this Driver
    **/
    virtual void sendMavlinkMsg(std::vector<mavlink_message_t>& msgs, int uasId, int sendRateHz, int msgNumber) override;
    virtual bool recvMavlinkMsg(const mavlink_message_t& msg) override;


    virtual bool init() override;
    virtual void loop() override;
    virtual void teardown() override;


    void addMessage(mavlink_message_t msg)
    {
        std::lock_guard<std::mutex> lock(_messageQueueLock);
        _messageQueue.push_back(msg);
    }

private:

    Hil();
    virtual ~Hil();
    std::mutex _messageQueueLock;
    std::vector<mavlink_message_t> _messageQueue;
};

#endif /* HIL_H */
