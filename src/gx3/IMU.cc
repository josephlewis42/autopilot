/**************************************************************************
 * Copyright 2012 Bryan Godbolt
 * Copyright 2013-14 Joseph Lewis <joseph@josephlewis.net>
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
 *************************************************************************/

#include "IMU.h"

/* Project Headers */
#include "Debug.h"
#include "gx3_read_serial.h"
#include "MainApp.h"
#include "message_parser.h"
#include "ack_handler.h"
#include "gx3_send_serial.h"
#include "QGCLink.h"
#include "util/AutopilotMath.hpp"
#include "Control.h"
#include "SystemState.h"

/* File Handling Headers */
#include <sys/types.h>
#include <sys/stat.h>

/* C Headers */
#include <unistd.h>
#include <errno.h>
#include <cstdlib>
#include <math.h>

/* Boost Headers */
#include <boost/math/constants/constants.hpp>

IMU* IMU::_instance = NULL;
std::mutex IMU::_instance_lock;

// path to serial device connected to gx3
const std::string IMU_SERIAL_PORT_CONFIG_NAME = "serial_port";
const std::string IMU_SERIAL_PORT_CONFIG_DEFAULT = "/dev/ser2";
const std::string IMU_ENABLED = "enabled";
const bool IMU_ENABLED_DEFAULT = true;

IMU* IMU::getInstance()
{
    std::lock_guard<std::mutex> lock(_instance_lock);

    if (!_instance)
    {
        _instance = new IMU;
    }

    return _instance;
}

IMU::IMU()
    :Driver("GX3 IMU", "gx3"),
     fd_ser(-1),
     position(blas::zero_vector<double>(3)),
     ned_origin(blas::zero_vector<double>(3)),
     velocity(blas::zero_vector<double>(3)),
     use_nav_attitude(true),
     attitude_source_connection(QGCLink::getInstance()->attitude_source.connect(
                                    boost::bind(&IMU::set_use_nav_attitude, this, _1))),
     nav_euler(blas::zero_vector<double>(3)),
     ahrs_euler(blas::zero_vector<double>(3)),
     nav_rotation(blas::identity_matrix<double>(3)),
     nav_angular_rate(blas::zero_vector<double>(3)),
     ahrs_angular_rate(blas::zero_vector<double>(3))
{
    isEnabled = configGetb(IMU_ENABLED, IMU_ENABLED_DEFAULT);
    _positionSendRateHz = configGeti("position_message_rate_hz", 10);
    _attitudeSendRateHz = configGeti("attitude_message_rate_hz", 10);

    if(! isEnabled)
    {
        warning() << "GX3 disabled!";
        return;
    }

    if(!init_serial())
    {
        critical() << "Failed to open serial port for GX3.  Attempting to terminate autopilot.";
        MainApp::terminate();
        return;
    }

    receive_thread = boost::thread(read_serial());
    parse_thread = boost::thread(message_parser());
    _send_serial = new send_serial(this);
}

IMU::~IMU()
{
    close(fd_ser);
}

bool IMU::init_serial()
{
    if(fd_ser != -1)
    {
        close(fd_ser);
    }

    std::string serial_path = configGets(IMU_SERIAL_PORT_CONFIG_NAME, IMU_SERIAL_PORT_CONFIG_DEFAULT);
    trace() << "starting on " << serial_path;
    fd_ser = open(serial_path.c_str(), O_RDWR | O_NOCTTY);
    trace() << "port opened";

    if(-1 == fd_ser)
    {
        critical() << "Could not initialize imu serial port";
        return false;
    }

    namedTerminalSettings("IMU1", fd_ser, 115200, "8N1", false, true);
    trace() << "started";

    set_last_data(); // start the timer for the data timeout.

    return true;

}

std::vector<uint8_t> IMU::compute_checksum(std::vector<uint8_t> data)
{
    std::vector<uint8_t> checksum(2,0);
    for (std::vector<uint8_t>::const_iterator it = data.begin(); it != data.end(); ++it)
    {
        checksum[0] += *it;
        checksum[1] += checksum[0];
    }
    return checksum;
}

void IMU::set_gx3_mode(IMU::GX3_MODE mode)
{
    bool mode_changed = false;
    {
        std::lock_guard<std::mutex> lock(gx3_mode_lock);
        if (mode != gx3_mode)
        {
            if (mode == RUNNING && gx3_mode != ERROR) // just came out of init or startup
            {
                set_ned_origin();
            }

            gx3_mode = mode;
            mode_changed = true;
        }

    }
    if (mode_changed)
    {
        gx3_mode_changed(mode);
    }
}

blas::vector<double> IMU::get_euler_rate() const
{
    // just return the angular rate since the yaw gyro measurement is not reliable
    return get_angular_rate();
}

blas::vector<double> IMU::get_ned_position() const
{
    blas::vector<double> llh(get_llh_position());
    llh[0] = AutopilotMath::degreesToRadians(llh[0]);
    llh[1] = AutopilotMath::degreesToRadians(llh[1]);

    blas::matrix<double> rot(3,3);
    rot.clear();
    rot(0,0) = -sin(llh[0])*cos(llh[1]);
    rot(0,1) = -sin(llh[0])*sin(llh[1]);
    rot(0,2) = cos(llh[0]);
    rot(1,0) = -sin(llh[1]);
    rot(1,1) = cos(llh[1]);
    rot(2,0) = -cos(llh[0])*cos(llh[1]);
    rot(2,1) = -cos(llh[0])*sin(llh[1]);
    rot(2,2) = -sin(llh[0]);

    return prod(rot, get_ecef_position() - get_ecef_origin());  // take advantage of matrix expression type
}

blas::vector<double> IMU::llh2ecef(const blas::vector<double>& llh_deg)
{

    // wgs84 constants
    blas::vector<double> llh(llh_deg);
    llh[0] = AutopilotMath::degreesToRadians(llh[0]);
    llh[1] = AutopilotMath::degreesToRadians(llh[1]);
    static const double equatorial_radius = 6378137;
    static const double flatness = 1/298.257223563;
    static double eccentricity = sqrt(flatness*(2-flatness));

    double normal_radius = equatorial_radius/sqrt(1 - pow(eccentricity, 2)*pow(sin(llh[0]),2));

    blas::vector<double> ecef(3);
    ecef.clear();

    ecef[0] = (normal_radius + llh[2])*cos(llh[0])*cos(llh[1]);
    ecef[1] = (normal_radius + llh[2])*cos(llh[0])*sin(llh[1]);
    ecef[2] = (normal_radius*(1-pow(eccentricity,2)) + llh[2])*sin(llh[0]);

    return ecef;
}

void IMU::set_use_nav_attitude(bool attitude_source)
{

    use_nav_attitude = attitude_source;
    message() << "Attitude source changed to " << (attitude_source?"nav filter":"ahrs") << ".";
}

blas::matrix<double> IMU::euler_to_rotation(const blas::vector<double>& euler)
{
    blas::matrix<double> rot(3,3);
    rot.clear();

    double roll = euler[0], pitch = euler[1], yaw = euler[2];
    rot(0, 0) = cos(yaw)*cos(pitch);
    rot(0, 1) = sin(yaw)*cos(pitch);
    rot(0, 2) = -sin(pitch);
    rot(1, 0) = -sin(yaw)*cos(roll)+cos(yaw)*sin(pitch)*sin(roll);
    rot(1, 1) = cos(yaw)*cos(roll)+sin(yaw)*sin(pitch)*sin(roll);
    rot(1, 2) = cos(pitch)*sin(roll);
    rot(2, 0) = sin(yaw)*sin(roll)+cos(yaw)*sin(pitch)*cos(roll);
    rot(2, 1) = -cos(yaw)*sin(roll)+sin(yaw)*sin(pitch)*cos(roll);
    rot(2, 2) = cos(pitch)*cos(roll);

    return trans(rot);
}

blas::matrix<double> IMU::get_heading_rotation() const
{
    double heading = get_euler()(2);
    blas::matrix<double> Rz(3,3);
    Rz.clear();
    Rz(0,0) = cos(heading);
    Rz(0,1) = -sin(heading);
    Rz(1,0) = sin(heading);
    Rz(1,1) = cos(heading);
    Rz(2,2) = 1;

    return Rz;
}

void IMU::set_ned_origin(const blas::vector<double>& origin)
{
    {
        std::lock_guard<std::mutex> lock(ned_origin_lock);
        ned_origin = origin;
    }
    message() << "Origin set to: " << origin;
}


void IMU::sendMavlinkMsg(std::vector<mavlink_message_t>& msgs, int uasId, int sendRateHz, int msgNumber)
{
    // Global position (mavlink common message)
    {
        trace() << "Sending mavlink_msg_global_position_int_pack";

        // send the default mavlink message
        mavlink_message_t msg;
        blas::vector<double> _llh_pos(get_llh_position());

        mavlink_msg_global_position_int_pack(uasId,
                                             heli::GX3_ID,
                                             &msg,
                                             getMsSinceInit(),
                                             _llh_pos[0] * 1E7,
                                             _llh_pos[1] * 1E7,
                                             _llh_pos[2] * 1000,
                                             0,
                                             0,
                                             0,
                                             0,
                                             0);
        msgs.push_back(msg);
    }

    
    // UAlberta Position
    {
        trace() << "Sending mavlink_msg_ualberta_position_pack";

        // get llh pos
        blas::vector<double> _llh_pos(get_llh_position());
        std::vector<float> llh_pos(_llh_pos.begin(), _llh_pos.end());

        // get ned pos
        blas::vector<double> _ned_pos(get_ned_position());
        std::vector<float> ned_pos(_ned_pos.begin(), _ned_pos.end());

        // get ned vel
        blas::vector<double> _ned_vel(get_velocity());
        std::vector<float> ned_vel(_ned_vel.begin(), _ned_vel.end());

        // get ned origin
        blas::vector<double> _ned_origin(get_llh_origin());
        std::vector<float> ned_origin(_ned_origin.begin(), _ned_origin.end());

        Control *control = Control::getInstance();
        // get reference position
        blas::vector<double> _ref_pos(control->get_reference_position());
        std::vector<float> ref_pos(_ref_pos.begin(), _ref_pos.end());

        // get position error in body
        blas::vector<double> _body_error(control->get_body_postion_error());
        std::vector<float> body_error(_body_error.begin(), _body_error.end());

        // get position error in ned
        blas::vector<double> _ned_error(control->get_ned_position_error());
        std::vector<float> ned_error(_ned_error.begin(), _ned_error.end());

        mavlink_message_t msg;


        mavlink_msg_ualberta_position_pack(
            uasId,
            heli::GX3_ID,
            &msg,
            &llh_pos[0], &ned_pos[0], &ned_vel[0], &ned_origin[0],
            &ref_pos[0], &body_error[0], &ned_error[0],
            getMsSinceInit());

        msgs.push_back(msg);
    }

    // UAlberta Attitude
    {
        trace() << "Sending mavlink_msg_ualberta_attitude";

        blas::vector<double> _nav_euler(get_nav_euler());
        std::vector<float> nav_euler(_nav_euler.begin(), _nav_euler.end());

        blas::vector<double> _nav_ang_rate(get_nav_angular_rate());
        std::vector<float> nav_ang_rate(_nav_ang_rate.begin(), _nav_ang_rate.end());

        blas::vector<double> _ahrs_euler(get_ahrs_euler());
        std::vector<float> ahrs_euler(_ahrs_euler.begin(), _ahrs_euler.end());

        blas::vector<double> _ahrs_ang_rate(get_ahrs_angular_rate());
        std::vector<float> ahrs_ang_rate(_ahrs_ang_rate.begin(), _ahrs_ang_rate.end());

        blas::vector<double> _attitude_reference(Control::getInstance()->get_reference_attitude());
        std::vector<float> attitude_reference(_attitude_reference.begin(), _attitude_reference.end());

        mavlink_message_t msg;
        mavlink_msg_ualberta_attitude_pack(uasId, heli::GX3_ID, &msg,
                                           &nav_euler[0], &nav_ang_rate[0], &ahrs_euler[0], &ahrs_ang_rate[0], &attitude_reference[0],
                                           getMsSinceInit());

        msgs.push_back(msg);
    }
};

void IMU::writeToSystemState()
{
    SystemState *state = SystemState::getInstance();
    state->state_lock.lock();
    state->gx3_position = position;
    state->gx3_ned_origin = ned_origin;
    state->gx3_velocity = velocity;
    state->gx3_nav_euler = nav_euler;
    state->gx3_ahrs_euler = ahrs_euler;
    state->gx3_nav_rotation = nav_rotation;
    state->gx3_nav_angular_rate = nav_angular_rate;
    state->gx3_ahrs_angular_rate = ahrs_angular_rate;
    state->gx3_use_nav_attitude = use_nav_attitude;
    state->gx3_mode = gx3_mode;
    state->state_lock.unlock();
}
