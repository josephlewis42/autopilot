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

#include "Linux.h"
#include <sys/sysinfo.h>
#include "SystemState.h"

Linux::Linux()
:Plugin("Linux CPU Info","linux_cpu_info", 1),
cpu_utilization(0)
{
    start(); // Start the plugin
    cpu_utilization.notifySet(SystemState::getInstance()->cpu_load);
}

bool Linux::init()
{
    return true; // we setup correctly.
}

void Linux::teardown()
{
}


void Linux::loop()
{
    struct sysinfo sysinf;
    sysinfo(&sysinf);

    float util = 0;

    cpu_utilization.set(sysinf.loads[0] / (float)(1 << SI_LOAD_SHIFT), 0); // set our utilization at 5 min.
    uptime_seconds = sysinf.uptime;
    totalram = (sysinf.totalram * sysinf.mem_unit) / (1024 * 1024);
    freeram = (sysinf.freeram * sysinf.mem_unit) / (1024 * 1024);
    int totalraml = (int)totalram.load();
    int freeraml = (int)freeram.load();
    trace() << "Got Load of: " << util << " memtotal: " << totalraml << " mb free: " << freeraml << " mb";
}

void Linux::sendMavlinkMsg(std::vector<mavlink_message_t>& msgs, int uasId, int sendRateHz, int msgNumber)
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

