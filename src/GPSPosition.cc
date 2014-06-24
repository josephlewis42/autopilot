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

#include "GPSPosition.h"
#include "AutopilotMath.hpp"
#include <cmath>

GPSPosition::GPSPosition(double latitudeDD, double longitudeDD, double heightM, double accuracyM)
:_latitudeDD(latitudeDD),
_longitudeDD(longitudeDD),
_heightM(heightM),
_accuracyM(accuracyM)
{

}

ublas::vector<double> GPSPosition::ecef() const
{
    double lat = AutopilotMath::degreesToRadians(_latitudeDD);
    double lon = AutopilotMath::degreesToRadians(_longitudeDD);
    double height = _heightM;


    // wgs84 constants
    static const double equatorial_radius = 6378137;
    static const double flatness = 1/298.257223563;
    static double eccentricity = sqrt(flatness*(2-flatness));

    double normal_radius = equatorial_radius/sqrt(1 - pow(eccentricity, 2)*pow(sin(lat),2));

    ublas::vector<double> ecef(3);
    ecef.clear();

    ecef[0] = (normal_radius + height)*cos(lat)*cos(lon);
    ecef[1] = (normal_radius + height)*cos(lat)*sin(lon);
    ecef[2] = (normal_radius*(1-pow(eccentricity,2)) + height)*sin(lat);

    return ecef;
}

double GPSPosition::distanceTo(GPSPosition* other, bool useCurrentAltitude)
{
    double earthRadius = 6378137;

    if(useCurrentAltitude)
    {
        earthRadius += _heightM;
    }

    double lat1Rad = AutopilotMath::degreesToRadians(_latitudeDD);
    double lat2Rad = AutopilotMath::degreesToRadians(other->_latitudeDD);
    double deltaLat = AutopilotMath::degreesToRadians(other->_latitudeDD - _latitudeDD);
    double deltaLon = AutopilotMath::degreesToRadians(other->_longitudeDD - _longitudeDD);

    double a = sin(deltaLat / 2) * sin(deltaLat / 2) +
                cos(lat1Rad) * cos(lat2Rad) *
                sin(deltaLon / 2) * sin(deltaLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return earthRadius * c;
    /**

    http://www.movable-type.co.uk/scripts/latlong.html

    var Δλ = (lon2-lon1).toRadians();

    var a = Math.sin(Δφ/2) * Math.sin(Δφ/2) +
            Math.cos(φ1) * Math.cos(φ2) *
            Math.sin(Δλ/2) * Math.sin(Δλ/2);
    var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));

    var d = R * c;
    **/
}

