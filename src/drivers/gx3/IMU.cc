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
#include <mavlink.h>


/* Boost Headers */
#include <boost/math/constants/constants.hpp>


// path to serial device connected to gx3
const std::string IMU_SERIAL_PORT_CONFIG_NAME = "serial_port";
const std::string IMU_SERIAL_PORT_CONFIG_DEFAULT = "/dev/ser2";


IMU::IMU()
    :Driver("GX3 IMU", "gx3"),
     fd_ser(-1),
     _position(),
     _ned_origin(),
     velocity(blas::zero_vector<double>(3)),
     use_nav_attitude(false),
     nav_euler(blas::zero_vector<double>(3)),
     ahrs_euler(blas::zero_vector<double>(3)),
     nav_rotation(blas::identity_matrix<double>(3)),
     nav_angular_rate(blas::zero_vector<double>(3)),
     ahrs_angular_rate(blas::zero_vector<double>(3)),
    attitude_source_connection(QGCLink::getInstance()->attitude_source.connect(
                                    boost::bind(&IMU::set_use_nav_attitude, this, _1)))
{
    configDescribe("position_message_rate_hz",
                   "0 - 100",
                   "Rate at which position messages are sent.",
                   "hz");
    _positionSendRateHz = configGeti("position_message_rate_hz", 10);

    configDescribe("attitude_message_rate_hz",
                   "0 - 100",
                   "Rate at which attitude messages are sent.",
                   "hz");
    _attitudeSendRateHz = configGeti("attitude_message_rate_hz", 10);

    configDescribe("use_external_gps",
                   "true/false",
                   "Defines if IMU should use external GPS data.");
    externGPS = configGetb("use_external_gps", true);

    if(! isEnabled())
    {
        warning() << "GX3 disabled!";
        return;
    }

    if(!init_serial())
    {
        initFailed("could not open serial port");
        return;
    }


    new std::thread(read_serial());
    new std::thread(message_parser());
    new send_serial(this);
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
                setNedOrigin(getPosition()); // set the origin to where we are
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


void IMU::sendMavlinkMsg(std::vector<mavlink_message_t>& msgs, int uasId, int sendRateHz, int msgNumber)
{


    // UAlberta Position
    if(shouldSendMavlinkMessage(msgNumber, sendRateHz, 20))
    {
        //trace() << "Sending mavlink_msg_ualberta_position_pack";

        // get llh pos
        std::vector<double> _llh_pos = getPosition().toLLH();
        std::vector<float> llh_pos(_llh_pos.begin(), _llh_pos.end());

        // get ned pos
        blas::vector<double> _ned_pos = get_ned_position();
        std::vector<float> ned_pos(_ned_pos.begin(), _ned_pos.end());

        // get ned vel
        blas::vector<double> _ned_vel(get_ned_velocity());
        std::vector<float> ned_vel(_ned_vel.begin(), _ned_vel.end());

        // get ned origin
        std::vector<double> _ned_origin = getNedOriginPosition().toLLH();
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
    if(shouldSendMavlinkMessage(msgNumber, sendRateHz, 20))
    {
        //trace() << "Sending mavlink_msg_ualberta_attitude";

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

    if(_newStatusMessage.load())
    {
        std::string message(status_message);
        _newStatusMessage = false;
        message.resize(49); // leave room for \0
        mavlink_message_t msg;
        mavlink_msg_ualberta_gx3_message_pack(uasId, heli::GX3_ID, &msg, message.c_str());

        msgs.push_back(msg);
    }

};

void IMU::writeToSystemState()
{

    SystemState *state = SystemState::getInstance();

    // no need to lock for the position because it uses SystemStateObjParams
    {
        double accuracy = externGPS ? 0 : 50; // high if we are using our own as it isn't very precise.
        state->position.set(getPosition(), accuracy);
    }

    state->nedOrigin.set(getNedOriginPosition(), 0);


    // set the rotation
    auto euler = get_euler();
    EulerAngles ea(euler[0], euler[1], euler[2]);
    debug() << "Roll: " << euler[0] << " Pitch: " << euler[1] << " Yaw: " << euler[2];
    state->rotation.set(ea, 0);

    // set the angular rates.
    auto eulerrate =  get_euler_rate();
    state->rollSpeed_radPerS.set(eulerrate[0], 0);
    state->pitchSpeed_radPerS.set(eulerrate[1], 0);
    state->yawSpeed_radPerS.set(eulerrate[2], 0);

    /**

    state->state_lock.lock();
    state->gx3_velocity = velocity;
    state->gx3_nav_euler = get_nav_euler();
    state->gx3_ahrs_euler = ahrs_euler;
    state->gx3_nav_rotation = get_nav_rotation();
    state->gx3_nav_angular_rate = get_nav_angular_rate();
    state->gx3_ahrs_angular_rate = get_ahrs_angular_rate();
    state->gx3_use_nav_attitude = use_nav_attitude;
    state->gx3_mode = gx3_mode;
    state->state_lock.unlock();

    **/
}
