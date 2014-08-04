/*******************************************************************************
 * Copyright 2012 Bryan Godbolt
 * Copyright 2013 Joseph Lewis <joehms22@gmail.com>
 *
 * This file is part of ANCL Autopilot.
 *
 *     ANCL Autopilot is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     ANCL Autopilot is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License
 *     along with ANCL Autopilot.  If not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************/

#include "GPS.h"

// Project Headers
#include "novatel_read_serial.h"
#include "MainApp.h"
#include "LogFile.h"
#include "SystemState.h"
#include "heli.h"


GPS::GPS()
    :Driver("NovAtel GPS","novatel"),
     read_serial_thread(ReadSerial()),
     llh_position(blas::vector<double>(0,3)),
     ned_velocity(blas::vector<double>(0,3)),
     pos_sigma(blas::vector<double>(0,3)),
     vel_sigma(blas::vector<double>(0,3))
{
}


GPS::~GPS()
{
}

void GPS::writeToSystemState()
{
    SystemState *state = SystemState::getInstance();

    // no need to lock for the position because it uses SystemStateObjParams
    {
        blas::vector<double> llh_position = get_llh_position();
        blas::vector<double> llh_errors = get_pos_sigma();

        // take the maximum error from the NovAtel to report as the position
        double max = -1;
        for(int i = 0; i < (int) llh_errors.size(); i++)
        {
            max = llh_errors[i] > max? llh_errors[i] : max;
        }

        GPSPosition pos(llh_position[0], llh_position[1], llh_position[2], max);
        state->position.set(pos, 0); // TODO find a suitable number for this
    }
    /**
    // TODO add the traits back in that we need.
    state->state_lock.lock();

    state->novatel_ned_velocity = ned_velocity;
    state->novatel_pos_sigma = pos_sigma;
    state->novatel_vel_sigma = vel_sigma;
    state->novatel_position_status = position_status;
    state->novatel_position_type = position_type;
    state->novatel_velocity_status = velocity_status;
    state->novatel_velocity_type = velocity_type;
    state->novatel_num_sats = num_sats;
    state->novatel_gps_time = _gps_time;
    state->state_lock.unlock();
    **/
}

void GPS::sendMavlinkMsg(std::vector<mavlink_message_t>& msgs, int uasId, int sendRateHz, int msgNumber)
{
    if(msgNumber % (sendRateHz / 10) == 0)
    {
        auto gps = GPS::getInstance();
        gps->trace() << "Sending novatel gps raw message";

        blas::vector<double> _pos_error(get_pos_sigma());
        std::vector<float> pos_error(_pos_error.begin(), _pos_error.end());
        blas::vector<double> _vel_error(get_vel_sigma());
        std::vector<float> vel_error(_vel_error.begin(), _vel_error.end());

        mavlink_message_t msg;
        mavlink_msg_novatel_gps_raw_pack(uasId,
                                         heli::NOVATEL_ID,
                                         &msg,
                                         get_position_type(),
                                         get_position_status(),
                                         get_num_sats(),
                                         &pos_error[0],
                                         get_velocity_type(),
                                         &vel_error[0],
                                         getMsSinceInit());
        msgs.push_back(msg);
    }
}
