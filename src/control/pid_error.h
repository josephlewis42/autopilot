/*******************************************************************************
 * Copyright 2012 Bryan Godbolt
 * Copyright 2013 Joseph Lewis <joehms22@gmail.com>
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

#ifndef PID_ERROR_H_
#define PID_ERROR_H_

/* STL Headers */
#include <iostream>
#include <atomic>

/* Project Headers */
#include "Debug.h"

/**
 * @brief Store the error for PID control. (Integral error reset for all two channels if its magnitude exceeds a limit)
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @date October 27, 2011: Class creation
 * @date February 10, 2012: Refactored out of class Control
 */
class pid_error
{
public:
    /**
     * Initializes error to all zeros
     */
    pid_error(double integral_error_limit = 1);

    /** copy constructor */
    pid_error(const pid_error& other)
    {
        _integral_error_limit = other._integral_error_limit;
        _proportional = other.getProportional();
        _derivative = other.getDerivative();
        _integral = other.getIntegral();
    };

    pid_error& operator=(const pid_error& other)
    {
        _integral_error_limit = other._integral_error_limit;
        setProportional(other.getProportional());
        setDerivative(other.getDerivative());
        setIntegral(other.getIntegral());

        return *this;
    }


    /**
     * @returns proportional error as lvalue
     */
    double getProportional() const
    {
        return _proportional;
    }
    double getDerivative() const
    {
        return _derivative;
    }
    double getIntegral() const
    {
        return _integral;
    }
    double setProportional(double np)
    {
        _proportional = np;
        return np;
    };
    double setDerivative(double nd)
    {
        _derivative = nd;
        return nd;
    }
    double setIntegral(double ni)
    {
        _integral = ni;
        return ni;
    }

    /**
     * Easy way to increment integral error
     * When the integral is computed, if it's absolute value is greater than GPS::Error::_integral_error_limit
     * the integral state is reset.
     * @returns result integral error state state
     */
    double operator++();

    /**
     * Stream insertion for std::ostream (cerr, cout)
     */
    friend std::ostream& operator<<(std::ostream& os, const pid_error& error);

    /**
     * Stream insertion operator for Debug object
     */
    friend Debug& operator<<(Debug& dbg, const pid_error& error);

    /// zero all the errors
    void reset();

private:
    double _integral_error_limit;

    std::atomic<double> _proportional;
    std::atomic<double> _derivative;
    std::atomic<double> _integral;
};

Debug& operator<<(Debug& dbg, const pid_error& error);
#endif
