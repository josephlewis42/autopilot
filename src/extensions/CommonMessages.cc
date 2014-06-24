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

#include "CommonMessages.h"
#include "SystemState.h"


CommonMessages* CommonMessages::_instance = NULL;
std::mutex CommonMessages::_instance_lock;

CommonMessages* CommonMessages::getInstance()
{
    std::lock_guard<std::mutex> lock(_instance_lock);
    if (_instance == NULL)
    {
        _instance = new CommonMessages;
    }
    return _instance;
}

CommonMessages::CommonMessages()
    :Driver("Mavlink Common Messages","common_messages")
{
    _sendSysStatus = configGetb("send_system_status_message", true);
    _frequencyHz = configGeti("message_send_rate_hz", 10);
    debug() << "Sending messages at: " << _frequencyHz.load();
}

CommonMessages::~CommonMessages()
{

}

void CommonMessages::sendMavlinkMsg(std::vector<mavlink_message_t>& msgs, int uasId, int sendRateHz, int msgNumber)
{
    if(! isEnabled()) return;

    if(msgNumber % (sendRateHz / _frequencyHz.load()) != 0)
    {
        return;
    }

    SystemState* state = SystemState::getInstance();

    state->batteryVoltage_mV.set(12 * 1000, 0);


    trace() << "Sending Mavlink Messages";
    /**
    mavlink_message_t msg;
    mavlink_msg_udenver_cpu_usage_pack(uasId, 40, &msg, getCpuUtilization(), totalram.load(), freeram.load());
    msgs.push_back(msg);
    **/
    // sys_status
    if(_sendSysStatus.load())
    {
        mavlink_message_t msg;
        mavlink_msg_sys_status_pack(uasId, MAV_COMP_ID_ALL, &msg,
                                    0, // uint32_t onboard_control_sensors_present,
                                    0, // uint32_t onboard_control_sensors_enabled,
                                    0, // uint32_t onboard_control_sensors_health,
                                    0, // uint16_t load,
                                    state->batteryVoltage_mV.get(), // uint16_t voltage_battery,
                                    0, // int16_t current_battery,
                                    0, // int8_t battery_remaining,
                                    0, // uint16_t drop_rate_comm,
                                    0, // uint16_t errors_comm,
                                    0, 0, 0, 0 //uint16_t errors_count1-4
                                    );
        msgs.push_back(msg);
    }


};

