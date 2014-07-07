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

#ifndef TRANSLATION_OUTER_PID_H_
#define TRANSLATION_OUTER_PID_H_

/* STL Headers */
#include <vector>
#include <string>
#include <mutex>
#include <atomic>

/* Project Headers */
#include "Parameter.h"
#include "pid_channel.h"
#include "ControllerInterface.h"
#include "AutopilotMath.hpp"
#include "Debug.h"

/* Boost Headers */
#include <boost/math/constants/constants.hpp>

/**
 * @brief outer loop pid controller for body frame x-y control
 * This controller is outer loop in the sense that it provides
 * an attitude reference (roll pitch) which must be regulated by
 * an inner loop attitude controller
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @author Nikos Vitzilaios <nvitzilaios@ualberta.ca>
 * @date January 2012: Class Creation
 * @date February 10, 2012: Refactored into separate file
 */
class translation_outer_pid : public ControllerInterface, public Logger
{
public:
    translation_outer_pid();
    translation_outer_pid(const translation_outer_pid& other);
    /** Set the x channel proportional gain
     * @param kp new proportional gain value
     * This function is threadsafe.
     */
    void set_x_proportional(double kp);
    /** set the x channel derivative gain
     * @param kd new derivative gain value
     * This function is threadsafe.
     */
    void set_x_derivative(double kd);
    /** set the x channel integral gain
     * @param ki new integral channel gain value
     * This function is threadsafe.
     */
    void set_x_integral(double ki);
    /** Set the y channel proportional gain
     * @param kp new proportional gain value
     * This function is threadsafe.
     */
    void set_y_proportional(double kp);
    /** Set the y channel derivative gain
     * @param kd new derivative gain value
     * This function is threadsafe.
     */
    void set_y_derivative(double kd);
    /**
     * Set the y channel integral gain
     * @param ki new integral gain value
     * This function is threadsafe.
     */
    void set_y_integral(double ki);
    /**
     * Perform the pid control computation and return
     * the roll pitch reference
     */
    void operator()(const blas::vector<double>& reference) throw(bad_control);
    /// @returns the roll pitch reference in radians (threadsafe)
    inline blas::vector<double> get_control_effort() const
    {
        std::lock_guard<std::mutex> lock(control_effort_lock);
        return control_effort;
    }

    /// @returns the list of parameters for the translational pid outer loop
    std::vector<Parameter> getParameters();

    /// x kp parameter representation
    static const std::string PARAM_X_KP;
    /// x kd parameter string representation
    static const std::string PARAM_X_KD;
    /// x ki parameter string representation
    static const std::string PARAM_X_KI;
    /// y kp parameter string representation
    static const std::string PARAM_Y_KP;
    /// y kd parameter string representation
    static const std::string PARAM_Y_KD;
    /// y ki parameter string representation
    static const std::string PARAM_Y_KI;
    static const std::string PARAM_TRAVEL;
    /// save the controller parameters
    void get_xml_node();
    /// load parameters for the function and populate the values
    void parse_xml_node();
    /// resets the controller
    void reset();
    /// test is controller is runnable
    bool runnable() const;

    inline double scaled_travel_degrees()
    {
        return scaled_travel;
    }
    inline double scaled_travel_radians()
    {
        return AutopilotMath::degreesToRadians(scaled_travel);
    }
    inline void set_scaled_travel_degrees(double travel)
    {
        set_scaled_travel(travel);
    }
    inline void set_scaled_travel_radians(double travel)
    {
        set_scaled_travel(AutopilotMath::radiansToDegrees(travel));
    }

    // various getters to go with the setters
    double get_x_proportional() const;
    double get_x_derivative() const;
    double get_x_integral() const;
    double get_y_proportional() const;
    double get_y_derivative() const;
    double get_y_integral() const;

private:

    // XML config references.
    static std::string XML_TRANSLATION_X_PROPORTIONAL;
    static std::string XML_TRANSLATION_Y_PROPORTIONAL;
    static std::string XML_TRANSLATION_X_DERIVATIVE;
    static std::string XML_TRANSLATION_Y_DERIVATIVE;
    static std::string XML_TRANSLATION_X_INTEGRAL;
    static std::string XML_TRANSLATION_Y_INTEGRAL;
    static std::string XML_TRAVEL;

    pid_channel x;
    mutable std::mutex x_lock;
    pid_channel y;
    mutable std::mutex y_lock;

    /// store the current control effort
    blas::vector<double> control_effort;
    /// serialize access to control_effort
    mutable std::mutex control_effort_lock;
    /// threadsafe set control_effort
    inline void set_control_effort(const blas::vector<double>& control_effort)
    {
        std::lock_guard<std::mutex> lock(control_effort_lock);
        this->control_effort = control_effort;
    }

    /**
     * Stores the rotational travel which is used to map the output of the
     * translational control into an angle reference.  This value is stored in degrees.
     * @note Roll and pitch use the same value.
     */
    std::atomic<double> scaled_travel;

    /**
     * Set the scaled travel used to represent the pilot stick position
     * @param travel new travel value in degrees
     * This function is threadsafe.
     */
    void set_scaled_travel(double travel);

};
#endif
