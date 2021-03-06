/*
 * SystemState.cc
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

#include "SystemState.h"

SystemState::SystemState()
:batteryVoltage_mV(500),
 position(1000 , GPSPosition(0,0,0,500)),
 nedOrigin(2000, GPSPosition(0,0,0,500)),
 cpu_load(0),
 main_loop_load(500),
 rollSpeed_radPerS(500),
 pitchSpeed_radPerS(500),
 yawSpeed_radPerS(500),
 rotation(500, EulerAngles(0,0,0)),
 servoRawInputs(3000, std::array<uint16_t, 8>()) // wait 3 seconds before defaulting.
{
}
