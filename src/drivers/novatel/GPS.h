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

#ifndef GPS_H_
#define GPS_H_

/* STL Headers */
#include <string>
#include <mutex>
#include <thread>

/* Boost Headers */
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
namespace blas = boost::numeric::ublas;
#include <boost/signals2/signal.hpp>

/* Project Headers */
#include "gps_time.h"
#include "Driver.h"
#include "Singleton.h"

/**
 * @brief Read position and velocity measurements from the Novatel GPS.
 *
 * This class performs the communication with the novatel gps. It initiates logging of the, RTKXYZ RTK
 * Cartesian Position and Velocity message; which provides GPS to a unit.
 *
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @author Aakash Vasudevan <avasudev@ualberta.ca>
 * @author Nikos Vitzilaios <nvitzilaios@ualberta.ca>
 * @author Joseph Lewis <joehms22@gmail.com>
 *
 * @date January 2012: Class creation and initial implementation
 * @date February 16, 2012: Refactored into separate files
 * @date April 27, 2012: Completed work to integrate novatel with gx3
 */

class GPS : public Driver, public Singleton<GPS>
{
    friend Singleton<GPS>;

public:

    virtual void sendMavlinkMsg(std::vector<mavlink_message_t>& msgs, int uasId, int sendRateHz, int msgNumber) override;

    virtual void writeToSystemState() override;

    class ReadSerial;

    /// threadsafe get llh_position
    inline blas::vector<double> get_llh_position()
    {
        std::lock_guard<std::mutex> lock(llh_position_lock);
        return llh_position;
    }
    /// threadsafe get ned_velocity
    inline blas::vector<double> get_ned_velocity()
    {
        std::lock_guard<std::mutex> lock(ned_velocity_lock);
        return ned_velocity;
    }
    /// threadafe get position error std dev
    inline blas::vector<double> get_pos_sigma()
    {
        std::lock_guard<std::mutex> lock(pos_sigma_lock);
        return pos_sigma;
    }
    /// threadsafe get vel error std dev
    inline blas::vector<double> get_vel_sigma()
    {
        std::lock_guard<std::mutex> lock(vel_sigma_lock);
        return vel_sigma;
    }
    /// threadsafe get gps time
    inline gps_time get_gps_time()
    {
        std::lock_guard<std::mutex> lock(_gps_time_lock);
        return _gps_time;
    }

    inline uint get_position_type()
    {
        std::lock_guard<std::mutex> lock(position_type_lock);
        return position_type;
    }

    inline uint get_position_status()
    {
        std::lock_guard<std::mutex> lock(position_status_lock);
        return position_status;
    }

    inline uint get_velocity_type()
    {
        std::lock_guard<std::mutex> lock(velocity_type_lock);
        return velocity_type;
    }

    inline uint get_velocity_status()
    {
        std::lock_guard<std::mutex> lock(velocity_status_lock);
        return velocity_status;
    }

    inline uint8_t get_num_sats()
    {
        std::lock_guard<std::mutex> lock(num_sats_lock);
        return num_sats;
    }

    /// signal when gps measurement is updated
    boost::signals2::signal<void ()> gps_updated;

private:


    /// default constructor
    GPS();

    virtual ~GPS();

    /// thread used to communicate with the GPS
    std::thread read_serial_thread;

    /// container for llh_position
    blas::vector<double> llh_position;
    /// serialize access to llh_position
    std::mutex llh_position_lock;
    /// threadsafe set llh_position
    inline void set_llh_position(const blas::vector<double>& llh)
    {
        std::lock_guard<std::mutex> lock(llh_position_lock);
        llh_position = llh;
    }


    /// container for ned_velocity
    blas::vector<double> ned_velocity;
    /// serialize access to ned_velocity
    std::mutex ned_velocity_lock;
    /// threadsafe set ned_velocity
    inline void set_ned_velocity(const blas::vector<double>& ned_vel)
    {
        std::lock_guard<std::mutex> lock(ned_velocity_lock);
        ned_velocity = ned_vel;
    }

    /// container for pos error std dev
    blas::vector<double> pos_sigma;
    /// serialize access to pos_sigma
    std::mutex pos_sigma_lock;
    /// threadsafe set pos_sigma
    inline void set_pos_sigma(const blas::vector<double>& pos_error)
    {
        std::lock_guard<std::mutex> lock(pos_sigma_lock);
        pos_sigma = pos_error;
    }

    /// container for velocity error std dev
    blas::vector<double> vel_sigma;
    /// serialize access to vel_sigma
    std::mutex vel_sigma_lock;
    /// threadsafe set vel sigma
    inline void set_vel_sigma(const blas::vector<double>& vel_error)
    {
        std::lock_guard<std::mutex> lock(vel_sigma_lock);
        vel_sigma = vel_error;
    }

    /// container for gps_time
    gps_time _gps_time;
    /// serialize access to gps_time
    std::mutex _gps_time_lock;
    /// threadsafe set gps_time
    inline void set_gps_time(const gps_time& time)
    {
        std::lock_guard<std::mutex> lock(_gps_time_lock);
        _gps_time = time;
    }

    uint position_status;
    std::mutex position_status_lock;
    inline void set_position_status(const uint status)
    {
        std::lock_guard<std::mutex> lock(position_status_lock);
        position_status = status;
    }

    uint position_type;
    std::mutex position_type_lock;
    inline void set_position_type(const uint type)
    {
        std::lock_guard<std::mutex> lock(position_type_lock);
        position_type = type;
    }

    uint velocity_status;
    std::mutex velocity_status_lock;
    inline void set_velocity_status(const uint status)
    {
        std::lock_guard<std::mutex> lock(velocity_status_lock);
        velocity_status = status;
    }

    uint velocity_type;
    std::mutex velocity_type_lock;
    inline void set_velocity_type(const uint type)
    {
        std::lock_guard<std::mutex> lock(velocity_type_lock);
        velocity_type = type;
    }

    uint8_t num_sats;
    std::mutex num_sats_lock;
    inline void set_num_sats(const uint8_t num)
    {
        std::lock_guard<std::mutex> lock(num_sats_lock);
        num_sats = num;
    }
};

#endif
