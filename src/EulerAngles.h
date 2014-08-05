/**************************************************************************
 * Copyright 2014 Joseph Lewis <joseph@josephlewis.net>
 *
 * This file is part of University of Denver Autopilot.
 *
 *     UDenver Autopilot is free software: you can redistribute it
 * 	   and/or modify it under the terms of the GNU General Public
 *     License as published by the Free Software Foundation, either
 *     version 3 of the License, or (at your option) any later version.
 *
 *     UDenver Autopilot is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License
 *     along with UDenver Autopilot. If not, see <http://www.gnu.org/licenses/>.
 *************************************************************************/

#ifndef EULER_H
#define EULER_H

#include <boost/numeric/ublas/fwd.hpp>

/** A representation of a set of Euler Angles (roll, pitch, yaw)
 *
 **/
class EulerAngles
{
    public:
        EulerAngles(double rollRad, double pitchRad, double yawRad);

        /** Returns the rotation matrix that reperesents these angles.
         *
         **/
        boost::numeric::ublas::matrix<double> toRotation() const;

        static EulerAngles fromQuaternion(double w, double x, double y, double z);

        double getRollRad()
        {
            return _rollRad;
        }


        double getPitchRad()
        {
            return _pitchRad;
        }


        double getYawRad()
        {
            return _yawRad;
        }

    private:
        double _rollRad, _pitchRad, _yawRad;
};

#endif
