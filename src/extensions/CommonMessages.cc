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
#include "Control.h"
#include "Helicopter.h"


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
    configDescribe("send_system_status_message",
                   "true/false",
                   "Enables/disables sending the status message which includes battery usage system load and enabled sensors.");
    _sendSysStatus = configGetb("send_system_status_message", true);

    configDescribe("message_send_rate_hz",
                   "0 - 200",
                   "The rate at which the set of common messages are sent.",
                   "hz");
    _frequencyHz = configGeti("message_send_rate_hz", 10);

    debug() << "Sending messages at: " << _frequencyHz.load();

    _sendParams = false; // don't send params until requested
    _sendRCCalibration = false; // don't send calibration until requested

}

CommonMessages::~CommonMessages()
{

}

void CommonMessages::sendMavlinkMsg(std::vector<mavlink_message_t>& msgs, int uasId, int sendRateHz, int msgNumber)
{
    if(! isEnabled()) return;

    if(_frequencyHz.load() <= 0 || msgNumber % (sendRateHz / _frequencyHz.load()) != 0)
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

    if(_sendParams.load())
    {
        mavlink_message_t msg;

        std::vector<std::vector<Parameter> > plist;

        plist.push_back(Control::getInstance()->getParameters());
        plist.push_back(Helicopter::getInstance()->getParameters());

        int num_params = 0;
        for (unsigned int i=0; i<plist.size(); i++)
        {
            num_params += plist[i].size();
        }
        int index = 0;
        for (unsigned int i=0; i<plist.size(); i++)
        {
            for (unsigned int j=0; j<plist.at(i).size(); j++)
            {
                mavlink_msg_param_value_pack(   uasId,
                                                (uint8_t)plist.at(i).at(j).getCompID(),
                                                &msg,
                                                (const char*)(plist.at(i).at(j).getParamID().c_str()),
                                                plist.at(i).at(j).getValue(),
                                                MAV_VAR_FLOAT,
                                                num_params,
                                                index);
                msgs.push_back(msg);
                index++;
            }
        }

        _sendParams = false;
    }

    if(!requested_params.empty())
    {
        std::lock_guard<std::mutex> lock(requested_params_lock);
        mavlink_message_t msg;
        while(!requested_params.empty())
        {
            mavlink_msg_param_value_pack(uasId,
                                         requested_params.front().getCompID(),
                                         &msg, (const char*)(requested_params.front().getParamID().c_str()),
                                         requested_params.front().getValue(),
                                         MAV_VAR_FLOAT,
                                         1,   // num of params
                                         -1); // index

            msgs.push_back(msg);
            requested_params.pop();
        }
    }

    if(_sendRCCalibration.load())
    {
        mavlink_message_t msg;
        RadioCalibration *radio = RadioCalibration::getInstance();

        mavlink_msg_radio_calibration_pack(uasId, heli::RADIO_CAL_ID, &msg,
                                           radio->getAileron().data(),
                                           radio->getElevator().data(),
                                           radio->getRudder().data(),
                                           radio->getGyro().data(),
                                           radio->getPitch().data(),
                                           radio->getThrottle().data()
                                          );
        
        msgs.push_back(msg);
        _sendRCCalibration = false;
    }

};

