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

#include "gx3_send_serial.h"

/* Project Headers */
#include "Debug.h"
#include "ack_handler.h"
#include "QGCLink.h"
#include "GPS.h"
#include "gps_time.h"

/* Boost Headers */
#include <boost/bind.hpp>
#include <boost/signals2.hpp>
#include <boost/thread.hpp>
#include <boost/ref.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/assign.hpp>
// this scope only pollutes the global namespace in a minimal way consistent with the stl global operators
using namespace boost::assign;

IMU::send_serial::send_serial(IMU* parent)
: reset_connection(QGCLink::getInstance()->reset_filter.connect(
		boost::bind(&IMU::send_serial::start_send_thread<boost::function<void ()> >,
				this, boost::function<void ()>(boost::bind(&IMU::send_serial::reset_filter, this))))),
  init_filter_connection(QGCLink::getInstance()->init_filter.connect(
		  boost::bind(&IMU::send_serial::start_send_thread<boost::function<void ()> >,
				  this, boost::function<void ()>(boost::bind(&IMU::send_serial::init_filter, this))))),
  gps_update_connection(GPS::getInstance()->gps_updated.connect(
		  boost::bind(&IMU::send_serial::start_send_thread<boost::function<void ()> >,
				  this, boost::function<void ()>(boost::bind(&IMU::send_serial::external_gps_update, this))))),
  initialize_imu_connection(parent->initialize_imu.connect(
		  boost::bind(&IMU::send_serial::start_send_thread<boost::function<void ()> >,
				  this, boost::function<void ()>(boost::bind(&IMU::send_serial::init_imu, this)))))

{
	boost::thread t(boost::bind(&IMU::send_serial::init_imu, this));
}


uint8_t IMU::send_serial::finish_packet(std::vector<uint8_t> &vec, uint8_t ack_command)
{
	std::vector<uint8_t> checksum = compute_checksum(vec);
	vec.insert(vec.end(), checksum.begin(), checksum.end());

	ack_handler ack(ack_command);

	send_lock.lock();
	write(IMU::getInstance()->fd_ser, &vec[0], vec.size());
	send_lock.unlock();

	ack.wait_for_ack();

	return ack.get_error_code();
}


void IMU::send_serial::init_imu()
{
	IMU* imu = IMU::getInstance();

	imu->debug("Setting to idle");
	set_to_idle();
	imu->debug("Setting AHRS message format");
	ahrs_message_format();
	imu->debug("Setting NAV message format");
	nav_message_format();
	imu->debug("Setting filter params");
	set_filter_parameters();
	imu->debug("resetting filter");
	reset_filter();
	imu->debug("enabling messages");
	enable_messages();
}


void IMU::send_serial::set_to_idle()
{
	std::vector<uint8_t> set_to_idle;
	set_to_idle += 0x75, 0x65, 0x01, 0x02, 0x02, 0x02;

	if(finish_packet(set_to_idle, 0x02) == 0x00)
		IMU::getInstance()->message() << "Successfully sent set to idle command";
	else
		IMU::getInstance()->debug() << "Received NACK after sending set to idle";
}

void IMU::send_serial::ping()
{
	IMU* imu = IMU::getInstance();

	std::vector<uint8_t> ping;
	ping += 0x75, 0x65, 0x01, 0x02, 0x02, 0x01;

	if(finish_packet(ping, 0x01) == 0x00)
		imu->debug("Successfully sent ping");
	else
		imu->warning("Received NACK after sending ping command");
}

void IMU::send_serial::ahrs_message_format()
{
	IMU* imu = IMU::getInstance();

	std::vector<uint8_t> ahrs_format;
	ahrs_format += 0x75, 0x65, 0x0C, 0x0A, 0x0A, 0x08, 0x01, 0x02, 0x0C, 0, 0x01, 0x05, 0, 0x01;

	uint8_t error_code = finish_packet(ahrs_format, 0x08);
	if(error_code == 0x00)
		imu->message("Successfully sent AHRS format message");
	else
		imu->message() << "Received NACK after sending AHRS format message with error code: " << std::hex << static_cast<int>(error_code);
}



void IMU::send_serial::nav_message_format()
{
	IMU* imu = IMU::getInstance();

	std::vector<uint8_t> nav_format;
	nav_format += 0x75, 0x65, 0x0C, 0x0, 0x0, 0x0A, 0x01, 0x05, 0x10, 0, 0x0A, 0x01, 0, 0x05, 0x02, 0, 0x05, 0x0E, 0, 0x01, 0x05, 0, 0x01;
	nav_format[3] = nav_format[4] = nav_format.size() - 4;

	uint8_t error_code = finish_packet(nav_format, 0x0A);
	if(error_code == 0x00)
		imu->message("Successfully sent nav message format command");
	else
		imu->warning() << "Received NACK after sending nav message format command with error code: " << std::hex << static_cast<int>(error_code);
}


void IMU::send_serial::enable_messages()
{
	std::vector<uint8_t> enable;
	enable += 0x75, 0x65, 0x0C, 0x0A, 0x05, 0x11, 0x01, 0x03, 0x01, 0x05, 0x11, 0x01, 0x01, 0x01;
	std::vector<uint8_t> checksum = compute_checksum(enable);
	enable.insert(enable.end(), checksum.begin(), checksum.end());

	ack_handler nav_enable_ack(0x11);
	ack_handler ahrs_enable_ack(0x11);

	send_lock.lock();
	write(IMU::getInstance()->fd_ser, &enable[0], enable.size());
	send_lock.unlock();

	nav_enable_ack.wait_for_ack();
	ahrs_enable_ack.wait_for_ack();
	if (nav_enable_ack.get_error_code() == 0x00 && ahrs_enable_ack.get_error_code() == 0x00)
		IMU::getInstance()->message() << "Successfully sent enable messages command";
	else
		IMU::getInstance()->warning() << "Received NACK after sending enable message with error codes: "
		<< std::hex << static_cast<int>(nav_enable_ack.get_error_code())
		<< " and " << static_cast<int>(ahrs_enable_ack.get_error_code());
}

void IMU::send_serial::set_filter_parameters()
{
	// set airborne dynamics
	std::vector<uint8_t> dynamics;
	dynamics += 4, 0x10, 0x01, 0x03; // airborne
	ack_handler dynamics_ack(0x10);

	// set vehicle frame offset
	std::vector<uint8_t> vehicle;
	vehicle += 0x0F, 0x12, 0x01;
	{
		std::vector<uint8_t> x(float_to_raw(0.052f)), y(float_to_raw(0.0f)), z(float_to_raw(-0.30f));
		vehicle.insert(vehicle.end(), x.begin(), x.end());
		vehicle.insert(vehicle.end(), y.begin(), y.end());
		vehicle.insert(vehicle.end(), z.begin(), z.end());
	}
	ack_handler vehicle_ack(0x12);

	// set antenna offset
	std::vector<uint8_t> antenna;
	antenna += 0x0F, 0x13, 0x01;
	{
		std::vector<uint8_t> x(float_to_raw(-0.24f)), y(float_to_raw(-0.05f)), z(float_to_raw(-0.473f));
		antenna.insert(antenna.end(), x.begin(), x.end());
		antenna.insert(antenna.end(), y.begin(), y.end());
		antenna.insert(antenna.end(), z.begin(), z.end());
	}
	ack_handler antenna_ack(0x13);

	// heading update control
	std::vector<uint8_t> heading;
	heading += 0x04, 0x18, 0x01, 0x01;
	ack_handler heading_ack(0x18);

	// gps source control
	std::vector<uint8_t> gps;
	gps += 0x04, 0x15, 1, 2;
	ack_handler gps_ack(0x15);

	// auto initialization
	std::vector<uint8_t> init;
	init += 0x04, 0x19, 0x01, 0x0;
	ack_handler init_ack(0x19);

	// create message header
	std::vector<uint8_t> nav_params;
	nav_params += 0x75, 0x65, 0x0D, 0;

	// add fields
	nav_params.insert(nav_params.end(), dynamics.begin(), dynamics.end());
	nav_params.insert(nav_params.end(), vehicle.begin(), vehicle.end());
	nav_params.insert(nav_params.end(), antenna.begin(), antenna.end());
	nav_params.insert(nav_params.end(), heading.begin(), heading.end());
	nav_params.insert(nav_params.end(), gps.begin(), gps.end());
	nav_params.insert(nav_params.end(), init.begin(), init.end());

	// get final size
	nav_params[3] = nav_params.size() - 4;
	std::vector<uint8_t> checksum = compute_checksum(nav_params);
	nav_params.insert(nav_params.end(), checksum.begin(), checksum.end());

	IMU::getInstance()->message("Sending GX3 Nav Filter Parameters.");

	send_lock.lock();
	write(IMU::getInstance()->fd_ser, &nav_params[0], nav_params.size());
	send_lock.unlock();

	dynamics_ack.wait_for_ack();
	if (dynamics_ack.get_error_code() == 0x00)
		IMU::getInstance()->message() << "Successfully set vehicle dynamics mode.";
	else
	{
		std::string message = "Error setting vehicle dynamics mode";
		::message() << message;
		IMU::getInstance()->gx3_status_message(message);
	}

	vehicle_ack.wait_for_ack();
	if (dynamics_ack.get_error_code() == 0x00)
		IMU::getInstance()->message("Successfully set vehicle frame offset");
	else
	{
		std::string message = "Error setting vehicle frame offset";
		::message() << message;
		IMU::getInstance()->gx3_status_message(message);
	}

	antenna_ack.wait_for_ack();
	if (antenna_ack.get_error_code() == 0x00)
		IMU::getInstance()->message() << "Successfully set antenna offset.";
	else
	{
		std::string message = "Error setting antenna offset";
		::message() << message;
		IMU::getInstance()->gx3_status_message(message);
	}

	heading_ack.wait_for_ack();
	if (heading_ack.get_error_code() == 0x00)
		IMU::getInstance()->message() << "Successfully set heading update source";
	else
	{
		std::string message = "Error setting antenna offset";
		::message() << message;
		IMU::getInstance()->gx3_status_message(message);
	}

	gps_ack.wait_for_ack();
	if (gps_ack.get_error_code() == 0x00)
		IMU::getInstance()->message("Successfully set gps source");
	else
	{
		std::string message = "Error setting gps source";
		::message() << message;
		IMU::getInstance()->gx3_status_message(message);
	}

	init_ack.wait_for_ack();
	if (init_ack.get_error_code() == 0x00)
		IMU::getInstance()->message("Successfully set auto-initialization control");
	else
	{
		std::string message = "Error setting auto-initialization";
		::message() << message;
		IMU::getInstance()->gx3_status_message(message);
	}
}

void IMU::send_serial::reset_filter()
{
	std::vector<uint8_t> reset;
	reset += 0x75, 0x65, 0x0D, 0x02, 0x02, 0x01;

	if(finish_packet(reset, 0x01) == 0x00)
		IMU::getInstance()->message() << "Successfully reset navigation filter";
	else
	{
		std::string message = "Error resetting nav filter";
		::message() << message;
		IMU::getInstance()->gx3_status_message(message);
	}
}

void IMU::send_serial::init_filter()
{
	IMU* imu = IMU::getInstance();

	std::vector<uint8_t> declination(float_to_raw(0.0f));
	std::vector<uint8_t> init;
	init += 0x75, 0x65, 0x0D, 0x06, 0x06, 0x04;
	init.insert(init.end(), declination.begin(), declination.end());

	uint8_t error_code = finish_packet(init,0x04);
	if(error_code == 0x00)
		imu->message("Successfully sent Set Initial Attitude from AHRS");
	else
		imu->message() << "Error sending Set Initial Attitude from AHRS with error code: " << static_cast<int>(error_code);
}

void IMU::send_serial::external_gps_update()
{
	IMU* imu = IMU::getInstance();

//	if (IMU::getInstance()->get_gx3_mode() == IMU::RUNNING)
	{
		// get gps data
		GPS* gps = GPS::getInstance();
		blas::vector<double> llh(gps->get_llh_position());
		llh[0]*=180.0/boost::math::constants::pi<double>();
		llh[1]*=180.0/boost::math::constants::pi<double>();
		blas::vector<float> vel(gps->get_ned_velocity());
		blas::vector<float> pos_error(gps->get_pos_sigma());
		blas::vector<float> vel_error(gps->get_vel_sigma());
		gps_time time(gps->get_gps_time());


		// create message
		std::vector<uint8_t> gps_update;
		gps_update += 0x75, 0x65, 0x0d, 0x48, 0x48, 0x16;

		pack_float(time.get_seconds(), gps_update);
		pack_int(time.get_week(), gps_update);

		for (int i=0; i<3; i++)
			pack_float(llh[i], gps_update);

		for (int i=0; i<3; i++)
			pack_float(vel[i], gps_update);

		for (int i=0; i<3; i++)
			pack_float(pos_error[i], gps_update);

		for (int i=0; i<3; i++)
			pack_float(vel_error[i], gps_update);


		uint8_t error_code = finish_packet(gps_update, 0x16);

		if (error_code != 0x00)
			imu->warning() << "Error sending External GPS Update with code: " << static_cast<int>(error_code);
	}
}
