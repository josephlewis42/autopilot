/*
 * SystemState.h
 *
 * Copyright 2014 Andrew Hannum
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 */
#ifndef SYSTEMSTATE_H_
#define SYSTEMSTATE_H_

#include <vector>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <atomic>
#include <mutex>
#include <string.h>
#include "IMU.h"
#include <GPS.h>
#include <servo_switch.h>
#include <MdlAltimeter.h>
#include "Helicopter.h"
#include "RadioCalibration.h"
#include <Control.h>
namespace blas = boost::numeric::ublas;
#include <SystemStateParam.hpp>

class SystemState
{

public:
    // Instance manager
    static SystemState* getInstance();

    // System state lock
    std::mutex state_lock;

    // gx3 data
    blas::vector<double> 	gx3_position;
    blas::vector<double> 	gx3_ned_origin;
    blas::vector<double> 	gx3_velocity;
    blas::vector<double> 	gx3_nav_euler;
    blas::vector<double> 	gx3_ahrs_euler;
    blas::matrix<double> 	gx3_nav_rotation;
    blas::vector<double> 	gx3_nav_angular_rate;
    blas::vector<double> 	gx3_ahrs_angular_rate;
    bool 					gx3_use_nav_attitude;
    std::string 			gx3_status_message;
    IMU::GX3_MODE 			gx3_mode;

    // novatel data
    blas::vector<double> 	novatel_llh_position;
    blas::vector<double> 	novatel_ned_velocity;
    blas::vector<double> 	novatel_pos_sigma;
    blas::vector<double> 	novatel_vel_sigma;
    uint 					novatel_position_status;
    uint 					novatel_position_type;
    uint 					novatel_velocity_status;
    uint 					novatel_velocity_type;
    uint8_t 				novatel_num_sats;
    gps_time 				novatel_gps_time;

    // servo data
    std::vector<uint16_t> 			servo_raw_inputs;
    std::vector<uint16_t> 			servo_raw_outputs;
    std::atomic<heli::PILOT_MODE> 	servo_pilot_mode;
    std::atomic<double> 			servo_engine_speed;

    // altimeter data
    float altimeter_height;

    // radio calibration data
    std::array<uint16_t, 2> radio_calibration_gyro;
    std::array<uint16_t, 3> radio_calibration_aileron;
    std::array<uint16_t, 3> radio_calibration_elevator;
    std::array<uint16_t, 3> radio_calibration_rudder;
    std::array<uint16_t, 5> radio_calibration_throttle;
    std::array<uint16_t, 5> radio_calibration_pitch;
    std::array<uint16_t, 3> radio_calibration_flightMode;

    // helicopter param data
    double 						helicopter_mass;
    double 						helicopter_gravity;
    blas::vector<double> 		helicopter_main_hub_offset;
    blas::vector<double> 		helicopter_tail_hub_offset;
    blas::banded_matrix<double> helicopter_inertia;

    // control data
    heli::Controller_Mode 	control_mode;
    blas::vector<double> 	control_reference_position;
    blas::vector<double> 	control_reference_attitude;
    blas::vector<double> 	control_effort;
    heli::Trajectory_Type 	control_trajectory_type;
    std::vector<double> 	control_pilot_mix;

    // Data from the helicopter.
    SystemStateParam<uint16_t> batteryVoltage_mV;

private:
    SystemState();
    static SystemState* _instance;
};

#endif //SYSTEMSTATE_H_
