/*******************************************************************************
 * Copyright 2012 Bryan Godbolt
 * Copyright 2014 Joseph Lewis <joseph@josephlewis.net>
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

#ifndef HELI_H_
#define HELI_H_


/* STL Headers */
#include <string>

namespace heli
{

enum PILOT_MODE
{
	PILOT_MANUAL,
	PILOT_AUTO,
	PILOT_UNKNOWN,
	NUM_PILOT_MODES
};

const std::string PILOT_MODE_DESCRIPTOR[] = {
	"manual",
	"auto",
	"unknown"
};

// Used by QGCLink receive to detect a parameter change.
enum COMPONENT_ID
{
	NAVFILTER_ID = 10,
	CONTROLLER_ID = 20,
	GX3_ID = 30,
	SERVO_SWITCH_ID = 40,
	RADIO_CAL_ID = 50,
	NOVATEL_ID = 60,
	HELICOPTER_ID = 70,
	ALTIMETER_ID = 80,
	NUM_COMPONENT_IDS
};

/// Enumeration constant to represent autopilot mode
enum AUTOPILOT_MODE
{
	MODE_DIRECT_MANUAL = 0,
	MODE_SCALED_MANUAL,
	MODE_AUTOMATIC_CONTROL,
	NUM_AUTOPILOT_MODES
};

const std::string AUTOPILOT_MODE_DESCRIPTOR[] =
{
	"Direct Manual Mode",
	"Scaled Manual Mode",
	"Automatic Control Mode",
	"Unknown Mode"
};

enum Controller_Mode
{
	Mode_Attitude_Stabilization_PID,
	Mode_Position_Hold_PID,
	Mode_Position_Hold_SBF,
	Num_Controller_Modes
};

enum Trajectory_Type
{
	Point_Trajectory,
	Line_Trajectory,
	Circle_Trajectory,
	Num_Trajectories
};

/**
 *  \enum Channel
 */
enum Channel
{
	/** Aileron (Roll) */
    CH1=0,
	/** Elevator (Pitch) */
    CH2,
    /** Throttle */
    CH3,
    /** Rudder (Yaw) */
    CH4,
    /** Gyro */
    CH5,
    /** Collective Pitch */
    CH6,
    /** AUX */
    CH7,
    /** Ready Signal */
    CH8,
    /** Not connected*/
    CH9
};

}
#endif
