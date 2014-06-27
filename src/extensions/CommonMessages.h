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

#ifndef COMMON_MESSAGES_H
#define COMMON_MESSAGES_H

#include <atomic>  // Used for atomic types
#include <mutex>   // Used for singleton design.
#include "Driver.h" // All drivers implement this.

/**
 * Provides an interface to the performance of Linux.
 **/
class CommonMessages: public Driver
{
public:
    /**
    Returns the one allowed instance of this Driver
    **/
    static CommonMessages* getInstance();
    virtual void sendMavlinkMsg(std::vector<mavlink_message_t>& msgs, int uasId, int sendRateHz, int msgNumber) override;
    std::atomic_bool _sendParams;

private:
    static CommonMessages* _instance;
    static std::mutex _instance_lock;

    CommonMessages();
    virtual ~CommonMessages();
    std::atomic<int> _frequencyHz; // frequency at which to send these messages.
    std::atomic_bool _sendSysStatus;
};

#endif /* LINUX_H */
