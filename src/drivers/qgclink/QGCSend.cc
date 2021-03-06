/**************************************************************************
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
 *************************************************************************/

#include "QGCSend.h"

/* Project Headers */
#include "RCTrans.h"
#include "Control.h"
#include "MainApp.h"
#include "IMU.h"
#include "GPS.h"
#include "MdlAltimeter.h"
#include "Helicopter.h"
#include "RateLimiter.h"
#include "Debug.h"

/* MAVLink Headers */
#include <mavlink.h>

/* STL Headers */
#include <vector>
#include <exception>

/* Boost Headers */
#include <boost/bind.hpp>
#include <boost/signals2.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
namespace blas = boost::numeric::ublas;
#include <boost/algorithm/string.hpp>


#include <asio.hpp>

#define NDEBUG

QGCSend::QGCSend()
    :qgc(NULL),
     servo_source(heli::NUM_AUTOPILOT_MODES),
     pilot_mode(heli::NUM_PILOT_MODES),
     filter_state(IMU::NUM_GX3_MODES),
     control_mode(heli::Num_Controller_Modes),
     attitude_source(true)
{
    send_queue = new std::queue<std::vector<uint8_t> >();
}


QGCSend::~QGCSend()
{
    delete send_queue;
}

void QGCSend::send()
{
    int send_rate = 200;
    RateLimiter rl(200);


    if (qgc == NULL)
    {
        qgc = QGCLink::getInstance();
    }

    int loop_count = 0;

    // get initial system modes
    pilot_mode = servo_switch::getInstance()->get_pilot_mode();
    control_mode = Control::getInstance()->get_controller_mode();

    // connect signals to track system mode
    boost::signals2::scoped_connection control_mode_connection(Control::getInstance()->mode_changed.connect(
                boost::bind(&QGCSend::set_control_mode, this, _1)));
    boost::signals2::scoped_connection servo_source_connection(MainApp::mode_changed.connect(
                boost::bind(&QGCSend::set_servo_source, this, _1)));
    boost::signals2::scoped_connection pilot_mode_connection(servo_switch::getInstance()->pilot_mode_changed.connect(
                boost::bind(&QGCSend::set_pilot_mode, this, _1)));
    boost::signals2::scoped_connection gx3_state_connection(IMU::getInstance()->gx3_mode_changed.connect(
                boost::bind(&QGCSend::set_filter_state, this, _1)));
    attitude_source_connection = QGCLink::getInstance()->attitude_source.connect(
                                     boost::bind(&QGCSend::set_attitude_source, this, _1));

    while(true)
    {
        rl.wait();

        if (should_run(qgc->get_heartbeat_rate(), send_rate, loop_count))
        {
            send_heartbeat(send_queue);
            send_status(send_queue);
        }

        /* Send any queued messages to the console */
        if (!message_queue_empty()) //only send max one message per iteration
        {
            send_console_message(message_queue_pop(), send_queue);
        }

        // Do bulk allocation of messages for drivers.
        for(Driver* driver : Driver::getDrivers())
        {
        	std::vector<mavlink_message_t> msgs;
        	driver->sendMavlinkMsg(msgs, qgc->getUasId(), send_rate, loop_count);
        	for(mavlink_message_t &msg : msgs)
        	{
        		std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);
        		buf.resize(mavlink_msg_to_send_buffer(&buf[0], &msg));
        		send_queue->push(buf);
        	}
        }

        /* actually send data to qgc */
        try
        {
            while (!send_queue->empty())
            {
                uint8_t sysid = send_queue->front().at(3);
                uint8_t compid = send_queue->front().at(4);
                uint8_t msgid = send_queue->front().at(5);
                qgc->trace() << "Sending message system: " << sysid << " component: " << compid << " message: " << msgid;

                qgc->send(send_queue->front());
                send_queue->pop();
            }

        }
        catch (std::exception e)
        {
            qgc->warning() << e.what();
        }

        /* Increment loop count */
        loop_count++;

        rl.finishedCriticalSection();
    }
}

bool QGCSend::should_run(int stream_rate, int send_rate, int count)
{
    if (stream_rate == 0 || stream_rate > send_rate)
        return false;
    return count%(send_rate/stream_rate) == 0;
}

void QGCSend::send_raw_imu(std::queue<std::vector<uint8_t> > *sendq)
{
//	NavFilter *nav = NavFilter::getInstance();
//
//	NavFilter::IMU_Data data = nav->getCurrentRawIMU();
//
//	mavlink_message_t msg;
//	std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);
//
//	mavlink_msg_scaled_imu_pack(100, 200, &msg, data.timerticks(),
//			data.accel()[0]*1000, data.accel()[1]*1000, data.accel()[2]*1000,
//			data.gyro()[0]*1000, data.gyro()[1]*1000, data.gyro()[2]*1000,
//			data.mag()[0]/10, data.mag()[1]/10, data.mag()[2]/10);
//
//	buf.resize(mavlink_msg_to_send_buffer(&buf[0], &msg));
//
//
//	sendq->push(buf);

}

void QGCSend::send_heartbeat(std::queue<std::vector<uint8_t> > *sendq)
{
    int system_type = MAV_TYPE_HELICOPTER;
    int autopilot_type = MAV_AUTOPILOT_UALBERTA;

    mavlink_message_t msg;
    std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);

    mavlink_msg_heartbeat_pack(100, 200, &msg, system_type, autopilot_type, 0, 0, 0);
    buf.resize(mavlink_msg_to_send_buffer(&buf[0], &msg));

    sendq->push(buf);

}

void QGCSend::send_status(std::queue<std::vector<uint8_t> >* sendq)
{
    // get elements of the system status
    heli::AUTOPILOT_MODE servo_source = get_servo_source();
    uint8_t qgc_servo_source = 255;
    switch(servo_source)
    {
    case heli::MODE_DIRECT_MANUAL:
        qgc_servo_source = ::UALBERTA_MODE_MANUAL_DIRECT;
        break;
    case heli::MODE_SCALED_MANUAL:
        qgc_servo_source = ::UALBERTA_MODE_MANUAL_SCALED;
        break;
    case heli::MODE_AUTOMATIC_CONTROL:
        qgc_servo_source = ::UALBERTA_MODE_AUTOMATIC_CONTROL;
        break;
    default:
        break;
    }

    heli::PILOT_MODE pilot_mode = get_pilot_mode();
    if (pilot_mode == heli::NUM_PILOT_MODES)
        pilot_mode = servo_switch::getInstance()->get_pilot_mode();
    uint8_t qgc_pilot_mode = 255;
    switch(pilot_mode)
    {
    case heli::PILOT_MANUAL:
        qgc_pilot_mode = ::UALBERTA_PILOT_MANUAL;
        break;
    case heli::PILOT_AUTO:
        qgc_pilot_mode = ::UALBERTA_PILOT_AUTO;
        break;
    default:
        break;
    }

    heli::Trajectory_Type trajectory = Control::getInstance()->get_trajectory_type();
    uint8_t qgc_trajectory = 255;
    switch (trajectory)
    {
    case heli::Point_Trajectory:
        qgc_trajectory = ::UALBERTA_POINT;
        break;
    case heli::Line_Trajectory:
        qgc_trajectory = ::UALBERTA_LINE;
        break;
    case heli::Circle_Trajectory:
        qgc_trajectory = ::UALBERTA_CIRCLE;
        break;
    default:  // prevents warning
        break;
    }

    IMU::GX3_MODE filter_state = get_filter_state();
    uint8_t qgc_filter_state = 255;
    switch(filter_state)
    {
    case IMU::STARTUP:
        qgc_filter_state = ::UALBERTA_GX3_STARTUP;
        break;
    case IMU::INIT:
        qgc_filter_state = ::UALBERTA_GX3_INIT;
        break;
    case IMU::RUNNING:
        qgc_filter_state = ::UALBERTA_GX3_RUNNING_VALID;
        break;
    case IMU::ERROR:
        qgc_filter_state = ::UALBERTA_GX3_RUNNING_ERROR;
        break;
    default:
        break;
    }

    heli::Controller_Mode control_mode = get_control_mode();
    uint8_t qgc_control_mode = 255;
    switch(control_mode)
    {
    case heli::Mode_Attitude_Stabilization_PID:
        qgc_control_mode = ::UALBERTA_ATTITUDE_PID;
        break;
    case heli::Mode_Position_Hold_PID:
        qgc_control_mode = ::UALBERTA_TRANSLATION_PID;
        break;
    case heli::Mode_Position_Hold_SBF:
        qgc_control_mode = ::UALBERTA_TRANSLATION_SBF;
        break;
    default:
        break;
    }

    mavlink_message_t msg;
    std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);

    mavlink_msg_ualberta_sys_status_pack(qgc->getUasId(), 200, &msg,
                                         qgc_servo_source, qgc_filter_state, qgc_pilot_mode, qgc_control_mode,(get_attitude_source()?UALBERTA_NAV_FILTER:UALBERTA_AHRS),
                                         0, 0, Helicopter::getInstance()->get_main_collective(), 0, 0, qgc_trajectory);

    buf.resize(mavlink_msg_to_send_buffer(&buf[0], &msg));

    sendq->push(buf);
}

std::string QGCSend::message_queue_pop()
{
    std::lock_guard<std::mutex> lock(message_queue_lock);
    std::string message(message_queue.front());
    message_queue.pop();
    return message;
}

void QGCSend::send_console_message(const std::string& message, std::queue<std::vector<uint8_t> > *sendq)
{
    std::string console(message);
    console.resize(50);

    mavlink_message_t msg;
    std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);

    ::mavlink_msg_statustext_pack(qgc->getUasId(), 0, &msg, (boost::algorithm::starts_with(console, "Critical")?255:0), console.c_str());
    buf.resize(mavlink_msg_to_send_buffer(&buf[0], &msg));
    sendq->push(buf);
}
