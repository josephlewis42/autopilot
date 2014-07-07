/**************************************************************************
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
 *************************************************************************/

/**
 * @brief Implement translation control with tail rotor sbf compensation as described in ACC2013 Godbolt/Lynch
 *
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @date: September 26, 2012: Class creation
 */
#ifndef TAIL_SBF_H_
#define TAIL_SBF_H_

/* Boost Headers */
#include <boost/math/constants/constants.hpp>
#include <boost/numeric/ublas/vector.hpp>
namespace blas = boost::numeric::ublas;

/* Project Headers */
#include "pid_channel.h"
#include "ControllerInterface.h"
#include "Parameter.h"
#include "Debug.h"

/* STL Headers */
#include <vector>
#include <string>
#include <mutex>
#include <atomic>


class tail_sbf : public ControllerInterface, public Logger
{
public:
    tail_sbf();

    /// reset the integrator states
    void reset();

    /// test if the controller is safe to run ** NOT IMPLEMENTED **
    bool runnable() const;

    /// integrate position error, and compute the resulting control effort
    void operator()(const blas::vector<double>& reference) throw(bad_control);

    /// @returns the roll pitch reference in radians (threadsafe)
    inline blas::vector<double> get_control_effort() const
    {
        std::lock_guard<std::mutex> lock(control_effort_lock);
        return control_effort;
    }

    /// return the scaled travel in degrees
    inline double scaled_travel_degrees() const
    {
        return scaled_travel;
    }
    /// return the scaled travel in radians
    inline double scaled_travel_radians() const
    {
        return scaled_travel_degrees()*boost::math::constants::pi<double>()/180;
    }
    /// set the scaled travel in degrees
    inline void set_scaled_travel_degrees(double travel)
    {
        set_scaled_travel(travel);
    }
    /// set the scaled travel in radians
    inline void set_scaled_travel_radians(double travel)
    {
        set_scaled_travel(travel*boost::math::constants::pi<double>()/180);
    }

    /// get the parameter list
    std::vector<Parameter> getParameters() const;

    static const std::string PARAM_X_KP;
    static const std::string PARAM_X_KD;
    static const std::string PARAM_X_KI;

    static const std::string PARAM_Y_KP;
    static const std::string PARAM_Y_KD;
    static const std::string PARAM_Y_KI;

    static const std::string PARAM_TRAVEL;

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

    /// saves the controller parameters
    void get_xml_node();
    /// load the parameters for the function and populate the values
    void parse_xml_node();

    double get_x_proportional() const;
    double get_y_proportional() const;
    double get_x_derivative() const;
    double get_y_derivative() const;
    double get_x_integral() const;
    double get_y_integral() const;

private:

    // constants for accessing the XML config
    static std::string XML_TRANSLATION_X_PROPORTIONAL;
    static std::string XML_TRANSLATION_Y_PROPORTIONAL;
    static std::string XML_TRANSLATION_X_DERIVATIVE;
    static std::string XML_TRANSLATION_Y_DERIVATIVE;
    static std::string XML_TRANSLATION_X_INTEGRAL;
    static std::string XML_TRANSLATION_Y_INTEGRAL;
    static std::string XML_TRAVEL;


    /// error states in ned x,y directions
    pid_channel ned_x, ned_y;
    /// serialize access to error states
    mutable std::mutex ned_x_lock, ned_y_lock;

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

#endif /* TAIL_SBF_H_ */
