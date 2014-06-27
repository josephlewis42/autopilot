/*******************************************************************************
 * Copyright 2012 Bryan Godbolt
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

#ifndef IMU_H_
#define IMU_H_

/* Boost Headers */
#include <boost/signals2.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

/* STL Headers */
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>

/* Project Headers */
#include "Driver.h"
#include "ThreadQueue.h"
#include "ThreadSafeVariable.h"

namespace blas = boost::numeric::ublas;


/**
 * @brief This class contains all the code for interacting with the 3DM-GX3 IMU.
 *
 * The data received from the imu is stored as data members of this class and can be accessed
 * using the public threadsafe accessors.
 *
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @date February 2, 2012
 */
class IMU : public Driver
{
public:
    static IMU* getInstance();
    virtual ~IMU();

    class read_serial;
    class send_serial;
    class message_parser;
    class ack_handler;

    enum FIELD_DESCRIPTOR
    {
        COMMAND_BASE = 0x01,
        COMMAND_3DM = 0x0C,
        COMMAND_NAV_FILT = 0x0D,
        COMMAND_SYS = 0x7F,
        DATA_AHRS = 0x80,
        DATA_GPS = 0x81,
        DATA_NAV = 0x82
    };

    enum GX3_MODE
    {
        STARTUP,
        INIT,
        RUNNING,
        ERROR,
        NUM_GX3_MODES
    };

    /// get position in local ned frame (origin must be set)
    blas::vector<double> get_ned_position() const;
    /// public interface to set the origin
    inline void set_ned_origin()
    {
        set_ned_origin(get_llh_position());
    }
    /// function to keep names symmetrical with position
    inline blas::vector<double> get_ned_velocity() const
    {
        return get_velocity();
    }
    /// get rotation matrix depending on use_nav_attitude
    inline blas::matrix<double> get_rotation() const
    {
        return euler_to_rotation(get_euler());
    }
    /// get the rotation matrix for the heading angle only (body->navigation)
    blas::matrix<double> get_heading_rotation() const;
    /// get the euler angles depending on use_nav_attitude
    inline blas::vector<double> get_euler() const
    {
        return (get_use_nav_attitude()?get_nav_euler():get_ahrs_euler());
    }

    /// get the euler angle derivatives
    blas::vector<double> get_euler_rate() const
    {
        // just return the angular rate since the yaw gyro measurement is not reliable
        return (get_use_nav_attitude()?get_nav_angular_rate():get_ahrs_angular_rate());
    }

    /// signal to notify when gx3 mode changes
    boost::signals2::signal<void (GX3_MODE)> gx3_mode_changed;
    
    ThreadSafeVariable<std::string> status_message;
    std::atomic_bool _newStatusMessage;
    void set_gx3_status_message(std::string in)
    {
        status_message = in;
        _newStatusMessage = true;
    }

    static blas::matrix<double> euler_to_rotation(const blas::vector<double>& euler);

	virtual void sendMavlinkMsg(std::vector<mavlink_message_t>& msgs, int uasId, int sendRateHz, int msgNumber) override;

    virtual void writeToSystemState() override;

    void setMessageSendRate(int hz)
    {
        _positionSendRateHz = hz;
    };

private:

    IMU();
    static IMU* _instance;
    static std::mutex _instance_lock;

    send_serial* _send_serial;
    /// thread to receive data on serial port
    std::thread receive_thread;
    /// thread to parse messages received from imu
    std::thread parse_thread;

    /// serial port file descriptor
    int fd_ser;
    /// initialize the serial port
    bool init_serial();

    /// compute the checksum for the imu data packet
    static std::vector<uint8_t> compute_checksum(std::vector<uint8_t> data);


    /// container for command data
    ThreadQueue<std::vector<uint8_t> > command_queue;
    /// container for ahrs data
    ThreadQueue<std::vector<uint8_t> > ahrs_queue;
    /// container for gps data
    ThreadQueue<std::vector<uint8_t> > gps_queue;
    /// container for nav data
    ThreadQueue<std::vector<uint8_t> > nav_queue;


    /// keep track of the gx3's mode
    GX3_MODE gx3_mode;
    /// serialize access to IMU::gx3_mode
    mutable std::mutex gx3_mode_lock;
    /// threadsafe mode set.  emits IMU::gx3_mode_changed if mode has changed
    void set_gx3_mode(GX3_MODE mode);
    /// threadsafe get mode
    inline GX3_MODE get_gx3_mode() const
    {
        std::lock_guard<std::mutex> lock(gx3_mode_lock);
        return gx3_mode;
    }


    /// signal when covariance of position measurement is too high (pos measurement invalid)
    boost::signals2::signal<void ()> pos_cov_high;
    /// signal when covariance of velocity measurement is too high
    boost::signals2::signal<void ()> vel_cov_high;
    /// signal when covariance of attitude measruement is too high
    boost::signals2::signal<void ()> att_cov_high;

    /// store the current position
    blas::vector<double> position;
    /// serialize access to IMU::position
    mutable std::mutex position_lock;
    /// threadsafe set position
    inline void set_position(const blas::vector<double>& position)
    {
        std::lock_guard<std::mutex> lock(position_lock);
        this->position = position;
    }
    /// threadsafe get position
    inline blas::vector<double> get_position() const
    {
        std::lock_guard<std::mutex> lock(position_lock);
        return position;
    }

    /// store the current ned origin (in llh)
    blas::vector<double> ned_origin;
    /// serialize access to ned_origin
    mutable std::mutex ned_origin_lock;
    /// threadsafe set ned origin @arg origin in llh
    void set_ned_origin(const blas::vector<double>& origin);
    /// threadsafe get ned origin (in llh)
    inline blas::vector<double> get_ned_origin() const
    {
        std::lock_guard<std::mutex> lock(ned_origin_lock);
        return ned_origin;
    }

    /// store current velocity in ned coords
    blas::vector<double> velocity;
    /// serialize access to IMU::velocity
    mutable std::mutex velocity_lock;
    /// threadsafe set velocity
    inline void set_velocity(const blas::vector<double>& velocity)
    {
        std::lock_guard<std::mutex> lock(velocity_lock);
        this->velocity = velocity;
    }

    /// use nav attitude measurements if true, ahrs if false
    std::atomic_bool use_nav_attitude;
    /// threadsafe get use_nav_attitude
    inline bool get_use_nav_attitude() const
    {
        return use_nav_attitude.load();
    }
    /// threadsafe set use_nav_attitude
    void set_use_nav_attitude(bool attitude_source);

    /// store the current euler angle estimate from the nav filter
    blas::vector<double> nav_euler;
    /// serialize access to nav_euler
    mutable std::mutex nav_euler_lock;
    /// threadsafe set nav_euler
    inline void set_nav_euler(const blas::vector<double>& euler)
    {
        std::lock_guard<std::mutex> lock(nav_euler_lock);
        nav_euler = euler;
    }

    /// store the current euler angle estimate from the ahrs filter
    blas::vector<double> ahrs_euler;
    /// serialize access to ahrs_euler
    mutable std::mutex ahrs_euler_lock;
    /// threadsafe set ahrs_euler
    inline void set_ahrs_euler(const blas::vector<double>& euler)
    {
        std::lock_guard<std::mutex> lock(ahrs_euler_lock);
        ahrs_euler = euler;
    }

    /// store the current rotation matrix between ned and body frames
    blas::matrix<double> nav_rotation;
    /// serialize access to IMU::rotation
    mutable std::mutex nav_rotation_lock;
    /// threadsafe set rotation
    inline void set_nav_rotation(const blas::matrix<double>& rotation)
    {
        std::lock_guard<std::mutex> lock(nav_rotation_lock);
        this->nav_rotation = rotation;
    }

    /// store the current angular rates
    blas::vector<double> nav_angular_rate;
    /// serialize access to IMU::angular_rate
    mutable std::mutex nav_angular_rate_lock;
    /// threadsafe set angular_rate
    inline void set_nav_angular_rate(const blas::vector<double>& angular_rate)
    {
        std::lock_guard<std::mutex> lock(nav_angular_rate_lock);
        nav_angular_rate = angular_rate;
    }


    /// store the current angular rates
    blas::vector<double> ahrs_angular_rate;
    /// serialize access to IMU::angular_rate
    mutable std::mutex ahrs_angular_rate_lock;
    /// threadsafe set angular_rate
    inline void set_ahrs_angular_rate(const blas::vector<double>& angular_rate)
    {
        std::lock_guard<std::mutex> lock(ahrs_angular_rate_lock);
        ahrs_angular_rate = angular_rate;
    }

    /// store the last time data was successfully received
    boost::posix_time::ptime last_data;
    /// serialize access to last_data
    mutable std::mutex last_data_lock;
    /// set last_data to the current time
    inline void set_last_data()
    {
        std::lock_guard<std::mutex> lock(last_data_lock);
        last_data = boost::posix_time::second_clock::local_time();
    }
    /// get the number of seconds since data was received
    inline int seconds_since_last_data() const
    {
        std::lock_guard<std::mutex> lock(last_data_lock);
        return (boost::posix_time::second_clock::local_time() - last_data).total_seconds();
    }


    /// send ack/nack signal when received from imu
    boost::signals2::signal<void (std::vector<uint8_t>)> ack;

    /// signal to notify imu it needs to reinitialize the serial connection
    boost::signals2::signal<void ()> initialize_imu;

    /// convert llh to ecef
    static blas::vector<double> llh2ecef(const blas::vector<double>& llh_deg);


    std::atomic_int _positionSendRateHz;
    std::atomic_int _attitudeSendRateHz;

    /// connection to allow use_nav_attitude to be set from qgc
    boost::signals2::scoped_connection attitude_source_connection;

        /// get llh position
    inline blas::vector<double> get_llh_position() const // 2014-06-23 -- now only used internally
    {
        return get_position();
    }
    /// get ecef position
    inline blas::vector<double> get_ecef_position() const // 2014-06-23 -- now only used internally
    {
        return llh2ecef(get_position());
    }
    /// get llh ned origin
    inline blas::vector<double> get_llh_origin() const // 2014-06-23 -- now only used internally
    {
        return get_ned_origin();
    }
    /// get ecef ned origin
    inline blas::vector<double> get_ecef_origin() const // 2014-06-23 -- now only used internally
    {
        return llh2ecef(get_ned_origin());
    }

    /// threadsafe get nav_euler
    inline blas::vector<double> get_nav_euler() const // 2014-06-23 -- now only used internally
    {
        std::lock_guard<std::mutex> lock(nav_euler_lock);
        return nav_euler;
    }
    /// threadsafe get ahrs_euler
    inline blas::vector<double> get_ahrs_euler() const // 2014-06-23 -- now only used internally
    {
        std::lock_guard<std::mutex> lock(ahrs_euler_lock);
        return ahrs_euler;
    }

    /// threadsafe get nav angular_rate
    inline blas::vector<double> get_nav_angular_rate() const  // 2014-06-23 -- now only used internally
    {
        std::lock_guard<std::mutex> lock(nav_angular_rate_lock);
        return nav_angular_rate;
    }
    /// threadsafe get ahrs angular_rate
    inline blas::vector<double> get_ahrs_angular_rate() const  // 2014-06-23 -- now only used internally
    {
        std::lock_guard<std::mutex> lock(ahrs_angular_rate_lock);
        return ahrs_angular_rate;
    }
        /// threadsafe get velocity (ned)
    inline blas::vector<double> get_velocity() const // 2014-06-23 -- now only used internally
    {
        std::lock_guard<std::mutex> lock(velocity_lock);
        return velocity;
    }
    /// threadsafe get rotation
    inline blas::matrix<double> get_nav_rotation() const // 2014-06-23 -- not used
    {
        std::lock_guard<std::mutex> lock(nav_rotation_lock);
        return nav_rotation;
    }

};

#endif /* IMU_H_ */
