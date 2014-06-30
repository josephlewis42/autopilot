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

#ifndef LINUX_H
#define LINUX_H

#include <atomic>  // Used for atomic types
#include <mutex>  // Used for singleton design.
#include "Driver.h"  // All drivers implement this.
#include "Singleton.h"

/**
 * Provides an interface to the performance of Linux.
 **/
class Linux: public Driver, public Singleton<Linux>
{
    friend class Singleton<Linux>;

public:
    /**
     * Returns the CPU utilization of the whole system.
     **/
    float getCpuUtilization() const
    {
        return cpu_utilization.load();
    }

    virtual void sendMavlinkMsg(std::vector<mavlink_message_t>& msgs, int uasId, int sendRateHz, int msgNumber) override;

private:
    Linux();
    virtual ~Linux();

    static void cpuInfo(Linux* instance);

    std::atomic<float> cpu_utilization;
    std::atomic<long> uptime_seconds;
    std::atomic<long> load1, load5, load15, totalram, freeram;
    std::atomic<short> procs;
};

#endif /* LINUX_H */
