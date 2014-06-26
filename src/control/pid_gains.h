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

#ifndef PID_GAINS_H_
#define PID_GAINS_H_

/* Project Headers */
#include "Debug.h"

#include <atomic>
/**
 * @brief Store the gains for PID control
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @date October 27, 2011: Class creation
 * @date February 10, 2012: Refactored out of class control
 */
class pid_gains
{
public:
    /**
     * Initializes the gains to 1
     */
    pid_gains()
    {
        _proportional = 1;
        _derivative = 1;
        _integral = 1;
    }

    /** copy constructor */
    pid_gains(const pid_gains& other)
    {
        _proportional = other.getProportional();
        _derivative = other.getDerivative();
        _integral = other.getIntegral();
    }

    pid_gains& operator=(const pid_gains& other)
    {
        setProportional(other.getProportional());
        setDerivative(other.getDerivative());
        setIntegral(other.getIntegral());

        return *this;
    }


    double getProportional() const
    {
        return _proportional;
    }
    void setProportional(double value)
    {
        _proportional = value;
    }


    double getDerivative() const
    {
        return _derivative;
    }
    void setDerivative(double value)
    {
        _derivative = value;
    }

    double getIntegral() const
    {
        return _integral;
    }
    void setIntegral(double value)
    {
        _integral = value;
    }


    /** proportional gain as lvalue */
    //double& proportional() {return gains[0];}
    /** proportional gain as rvalue */
    //const double& proportional() const {return gains[0];}
    //double& derivative() {return gains[1];}
    //const double& derivative() const {return gains[1];}
    //double& integral() {return gains[2];}
    //const double& integral() const {return gains[2];}
    /**
     * Stream insertion for Debug object
     */
    friend Debug& operator<<(Debug& dbg, const pid_gains& gains);
private:
    //std::array<double, 3> gains;

    std::atomic<double> _proportional;
    std::atomic<double> _derivative;
    std::atomic<double> _integral;
};

Debug& operator<<(Debug& dbg, const pid_gains& gains);
#endif
