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

// System imports
#include <vector>
#include <atomic>
#include <mutex>
#include <string.h>

// Project imports
#include "IMU.h"
#include "Parameter.h"
#include "SystemStateParam.hpp"
#include "SystemStateObjParam.hpp"
#include "GPSPosition.h"
#include "heli.h"
#include "gps_time.h"
#include "Singleton.h"
#include "EulerAngles.h"

/**
 * The SystemState keeps track of variables that multiple drivers wish to manipulate
 * and implements a location where controllers can get their data without being
 * tied to a specific driver implementation or set of hardware. This class facilitates
 * the following use cases:
 *
 * - a new driver is created and must be tested
 * - a HIL simulator needs to take over with all sensors offline
 * - we want to try a patch using logs from a prior flight.
 * - we build in redundant systems of which we only want the most accurate one to be reporting
 *   until it dies when a fallback comes in.
 * - we can attach/remove filters between a device and the output
 * - we can run the autopilot with missing components and it will know what to do rather than
 *   being explicitly configured
 * - the autopilot can be moved to platforms with a different set of sensors with no code change
 * - sensors can be removed and replaced using an entirely different system e.g. running our
 *   autopilot by plugging in an Ardupilot which we use sensor data from.
 *
 * The creators of MAVLink believe the following fields to be mostly sufficient for performing
 * autopilot calculations:
 *
 * - attitude (roll, pitch, yaw)
 * - roll/pitc/yaw speeds
 * - lat/lon/altitude
 * - ground x,y,z speed
 * - indicated airspeed
 * - true airspeed
 * - x,y,z acceleration
 * - servo/radio inputs/outputs
 *
 * These should be considered the minimal set of requirements for running a functioning
 * autopilot system. Additionally we keep:
 * - main loop load (we should consider choosing easier manuveurs if calculations are taking too long)
 * - battery (could be used in decision making)
 * - NED origin (used in some autopilot calculations)
 *
 **/
class SystemState : public Singleton<SystemState>
{
friend class Singleton<SystemState>;

public:

    // System state lock
    std::mutex state_lock;

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

    // Data from the helicopter.
    SystemStateParam<uint16_t> batteryVoltage_mV;

    /// The position of the helicopter on Earth, defaults to accepting a less precise position in one second.
    SystemStateObjParam<GPSPosition> position;

    /// The origin of the ned reference frame.
    SystemStateObjParam<GPSPosition> nedOrigin;

    /// The CPU load as a proportion
    SystemStateParam<float> cpu_load;

    /// The mainloop load as a proportion
    SystemStateParam<float> main_loop_load;

    /// The rate of change in roll
    SystemStateParam<float> rollSpeed_radPerS;
    /// The rate of change in pitch
    SystemStateParam<float> pitchSpeed_radPerS;
    /// The rate of change in yaw
    SystemStateParam<float> yawSpeed_radPerS;
    /// The rotation of the system
    SystemStateObjParam<EulerAngles> rotation;


private:
    SystemState();
};

#endif //SYSTEMSTATE_H_
