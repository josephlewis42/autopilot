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

#ifndef QGCLINK_H_
#define QGCLINK_H_

/* Project Headers */
#include "Parameter.h"

/* Boost Headers */
#include <boost/asio.hpp>
using boost::asio::ip::udp;
using boost::asio::ip::address;
#include <boost/thread.hpp>
#include <boost/signals2/signal.hpp>
#include "heli.h"
#include "Driver.h"


/* STL Headers */
#include <vector>
#include <queue>
#include <mutex>
#include <atomic>

/**
 *  @brief Sends and receives data to QGroundControl via UDP.
 *  This class handles all Mavlink communication to QGC.  It spawns a receive thread
 *  to receive data from a UDP socket, and a send thread to send data on the same UDP
 *  socket.
 *  @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 *  @date July 8, 2011 : Created class
 *  @date October 20, 2011 : Modified send/receive calls to catch system_failure exception - ethernet cable can now be safely unplugged
 *  @date February 6, 2012: Broke class into multiple files
  */

class QGCLink : public Driver
{

public:
	/// returns the running instance of QGCLink
	static QGCLink* getInstance();

	/// emitted when a shutdown message is received
	boost::signals2::signal<void ()> shutdown;
	/// emitted when a filter reset message is received
	boost::signals2::signal<void ()> reset_filter;
	/// emitted with a filter init message is received
	boost::signals2::signal<void ()> init_filter;
	/// emitted when a set origin message is received
	boost::signals2::signal<void ()> set_origin;
	/** emitted when a message changes the attitude
	 * true means use nav attitude, false means use ahrs
	 */
	boost::signals2::signal<void (bool)> attitude_source;
	/// emitted when qgc requests a servo source change
	boost::signals2::signal<void (heli::AUTOPILOT_MODE)> servo_source;
	/// emitted when qgc requests a control mode change
	boost::signals2::signal<void (heli::Controller_Mode)> control_mode;

private:

	class QGCReceive;
	class QGCSend;

	/// Construct QGCLink class
	QGCLink();


	// delete the copy and equality constructors.
	QGCLink(const QGCLink &) = delete;
	QGCLink& operator = (const QGCLink &) = delete;

	/// Pointer to instance of QGCLink
	static QGCLink* _instance;
	/// Mutex to make class instantiation threadsafe
	static std::mutex _instance_lock;
	/// creates the udp socket to qgc and spawns the receive and send threads
	void init();

	/// mutex to make access to send_queue threadsafe
//	std::mutex send_queue_lock;

	udp::endpoint qgc;
	boost::asio::io_service io_service;
	udp::socket socket;

	/// thread to receive data from qgc see QGCLink::QGCReceive
	boost::thread receive_thread;
	/// thread to send data to qgc see QGCLink::QGCSend
	boost::thread send_thread;

	/// frequency to send heartbeat messages.  also used for system status messages
	std::atomic_int heartbeat_rate;
	inline int get_heartbeat_rate() const {return heartbeat_rate;}
	inline void set_heartbeat_reate(int rate) {heartbeat_rate = rate;}

	/// frequency to send rc channel measurements
	std::atomic_int rc_channel_rate;
	inline int get_rc_channel_rate() const {return rc_channel_rate;}
	inline void set_rc_channel_rate(int rate) {rc_channel_rate = rate;}

	/// frequency to send control output -- threadsafe
	std::atomic_int control_output_rate;
	inline int get_control_output_rate() const {return control_output_rate;}
	inline void set_control_output_rate(int rate) {control_output_rate = rate;}

	/// store frequency to send position measurement rate
	std::atomic_int position_rate;
	inline int get_position_rate() const {return position_rate;}
	inline void set_position_rate(int rate) {position_rate = rate;}

	/// Store desired transmission rate for attitude state estimate
	std::atomic<int> attitude_rate;
	inline void set_attitude_rate(int rate) {attitude_rate = rate;}
	inline int get_attitude_rate() const {return attitude_rate;}

	mutable std::mutex param_recv_lock;
	bool param_recv;

	mutable std::mutex requested_params_lock;
	std::queue<Parameter> requested_params;

	/// store whether rc_calibration packet has been requested
	std::atomic_bool requested_rc_calibration;
	/// threadsafe set requested rc calibration (could be used as slot)
	inline void set_requested_rc_calibration() {requested_rc_calibration = true;}
	/// threadsafe clear requested rc calibration
	inline void clear_requested_rc_calibration() {requested_rc_calibration = false;}
	/// threadsafe get requested rc calibration
	inline bool get_requested_rc_calibration() const {return requested_rc_calibration;}

	int uasId;
};

#endif
