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

#ifndef GPS_POSITION_H
#define GPS_POSITION_H

#include <boost/numeric/ublas/vector.hpp>
#include <vector>

using namespace boost::numeric;

/** GPSPosition represents a single point in the geographic reference system
for Earth. It provides utilities for converting this reference to other formats
as well.

We use the WGS84 Standard For Coordinates, although we could optionally use
anything else.
 */
class GPSPosition
{
    public:
        /** Constructs a new GPSPosition.
         *
         * @param latitudeDD - the latitude in decimal degrees
         * @param longitudeDD - the longitude in decimal degrees
         * @param heightM - the height in meters
         * @param accuracyM - the accuracy of the position in meters, default is 100. Numbers
         * less than 1 can represent cm, but numbers less than or equal to 0 should not be used.
         *
         * @note The algorithms in this class have not yet been verified for their correctness.
         */
        GPSPosition(double latitudeDD, double longitudeDD, double heightM, double accuracyM=100);

        /// Empty constructor.
        GPSPosition();
        /**
         * Convert lat/lon/height coordinates to Earth-Centered
         * Earth-Fixed (ECEF) coordinates (WGS84).
         *
         * @return a vector of x,y,z (in meters)
         * TODO - add tests for this to ensure correctness of calculations.
         **/
        ublas::vector<double> ecef() const;

        /**
         * Converts the lat/long/height coordinates to North East Down (NED)
         * position.
         *
         * @param origin - the origin for the ned position
         * @return a vector of n,e,d
         **/
        ublas::vector<double> ned(GPSPosition &origin) const;

        /// Returns the latitude in decimal degrees
        const double getLatitudeDD(){return _latitudeDD;};

        /// Returns the longitude in decimal degrees
        const double getLongitudeDD(){return _longitudeDD;};

        /// Returns the height in meters
        const double getHeightM(){return _heightM;};

        /// Returns the acuracy in meters
        const double getAccuracyM(){return _accuracyM;};

        /** Calculates and returns the distance between the two points in meters
         * uses the haversine formula.
         * @param other - the point to get the distance to
         * @param useCurrentAltitude - whether or not to add in the current altitude
         * to the radius of Earth.
         *
         * @return the number of meters between the two points.
         **/
        double distanceTo(GPSPosition* other, bool useCurrentAltitude=true);

        /**
         * Returns a string representation of this position.
         **/
        std::string toString() const;

        /** Returns the latitude, longitude, height representation of this position.
         */
        std::vector<double> toLLH() const;

        /** Equality test on two positions, ignoring the accuracy **/
        bool operator==(const GPSPosition& rhs) const;

        /** inequality test on two positions, ignoring the accuracy **/
        bool operator!=(const GPSPosition& rhs) const;

    private:
        double _latitudeDD, _longitudeDD, _heightM, _accuracyM;
};

#endif

