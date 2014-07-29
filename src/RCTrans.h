/*******************************************************************************
 * Copyright 2012 Bryan Godbolt, Hasitha Senanayake
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

#ifndef RCTRANS_H
#define RCTRANS_H

/* Project Headers */
#include "servo_switch.h"
#include "RadioCalibration.h"

/**
    \brief This class handles input pulse scaling to normalized values
    based on calibration data from RadioCalibration.

    \author Hasitha Senanayake <senanaya@ualberta.ca>
    \author Bryan Godbolt <godbolt@ece.ualberta.ca>
    \date July 14, 2011 : class creation
	@date November 8, 2011: Fixed class so functions which do not access data members are static
*/
class RCTrans
{
public:
    RCTrans();
    /** returns a vector of scaled valued for all channels */
    static std::vector<double> getScaledVector();
    /// List provides index to channel mapping for the RCTrans::getScaled function.
    enum RadioElement
    {
        AILERON = 0,
        ELEVATOR,
        THROTTLE,
        RUDDER,
        GYRO,
        PITCH
    };

private:
    /** Scales a pulse value to a normalized value 0 or 1
        @param pulse the received pulse to be scaled
        @param setpoint an array that stores the calibrated end point pulse values of the Radio
        @return normalizedPulse a scaled value of the pulse with respect to end points */
    static double pulse2norm(uint16_t pulse, std::array<uint16_t, 2> setpoint);
    /** Scales a pulse value to a normalized value between -1 and 1, based on calibrated end points
        @param pulse the received pulse to be scaled
        @param setpoint an array that stores the calibrated end point pulse values of the Radio
        @return normalizedPulse a scaled value of the pulse with respect to end points */
    static double pulse2norm(uint16_t pulse, std::array<uint16_t, 3> setpoint);
    /** Scales a pulse value to a normalized value 0 or 1, based on calibrated set points
        @param pulse the received pulse to be scaled
        @param setpoint an array that stores the calibrated end point pulse values of the Radio
        @return normalizedPulse a scaled value of the pulse with respect to end points */
    static double pulse2norm(uint16_t pulse, std::array<uint16_t, 5> setpoint);
    /** Determine state of Pilot ready signal
        @param pulse CH8 pulse from TX
        @param setpoint set points to interpret mode state.
        @return flightMode returns 0 = Manual, 1 = Autonomous, 2 = Rotomotion. */
    static int flightMode(uint16_t pulse, std::array<uint16_t, 3> setpoint);

    /** function to determine state of Pilot ready signal */
    static inline int getFlightMode()
    {
        return flightMode(servo_switch::getInstance()->getRaw(heli::CH8),
                          RadioCalibration::getInstance()->getFlightMode());
    }

    /// Resolves Gyro mode to scaled value in RCTrans::pulse2norm.
    enum TailGyro
    {
        AVCS_HEADING_HOLD_MODE = 0,
        NORMAL_GYRO_MODE
    };

};
#endif // RCTRANS_H
