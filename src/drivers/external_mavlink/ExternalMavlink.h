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

#ifndef EXTERNAL_MAVLINK_H
#define EXTERNAL_MAVLINK_H

#include <atomic>  // Used for atomic types
#include <vector>
#include "Plugin.h"
#include "Singleton.h"
#include "SystemStateParam.hpp"


/**
 * Provides an interface for reading external mavlink packets from a UART 
 * or file with the intenet of updating local data from said packets. 
 * 
 * This will allow us to read in from an Ardupilot, or supposedly straight
 * from XPlane and such to allow emulation.
 **/
class ExternalMavlink: public Plugin, public Singleton<ExternalMavlink>
{
    friend class Singleton<ExternalMavlink>;

public:
//    virtual void sendMavlinkMsg(std::vector<mavlink_message_t>& msgs, int uasId, int sendRateHz, int msgNumber) override;
    virtual bool init();
    virtual void loop();
    virtual void teardown();

private:
    ExternalMavlink();

    /// The open file for the external mavlink to read from (file or device node)
    int fd;

    /// set if we're going to update our imu from the external file
    bool _use_external_imu;
    
    /// Set if we're going to update our GPS from the external file
    bool _use_external_gps;
    
	mavlink_message_t _msg;
	mavlink_status_t _status;
};

#endif /* EXTERNAL_MAVLINK_H */
