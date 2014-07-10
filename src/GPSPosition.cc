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
#include <sstream>      // std::stringstream

// GeoLib
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>


#include <boost/numeric/ublas/matrix.hpp>


GPSPosition::GPSPosition()
:_latitudeDD(0),
_longitudeDD(0),
_heightM(0),
_accuracyM(1000)
{
}

GPSPosition::GPSPosition(double latitudeDD, double longitudeDD, double heightM, double accuracyM)
:_latitudeDD(latitudeDD),
_longitudeDD(longitudeDD),
_heightM(heightM),
_accuracyM(accuracyM)
{

}

ublas::vector<double> GPSPosition::ecef() const
{
    double x=0, y=0, z=0;
    GeographicLib::Geocentric::WGS84.Forward(_latitudeDD, _longitudeDD, _heightM, x, y, z);

    ublas::vector<double> ecef(3);
    ecef.clear();

    ecef[0] = x;
    ecef[1] = y;
    ecef[2] = z;

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



ublas::vector<double> GPSPosition::ned(GPSPosition &origin) const
{
    auto originCoords = GeographicLib::LocalCartesian::LocalCartesian(origin._latitudeDD,
                                                                      origin._longitudeDD,
                                                                      origin._heightM);


    double x=0, y=0, z=0;
    originCoords.Forward(_latitudeDD, _longitudeDD, _heightM, x, y, z);

    ublas::vector<double> ned(3);
    ned.clear();
    ned[0] = x;
    ned[1] = y;
    ned[2] = -z;

    return ned;
}


std::string GPSPosition::toString() const
{
    std::stringstream ss;
    ss << "[Lat: " << _latitudeDD << " dd, ";
    ss << "Lon: " << _longitudeDD << " dd, ";
    ss << "Height: " << _heightM << " m]";

    return ss.str();
}

std::vector<double> GPSPosition::toLLH() const
{
    std::vector<double> llh_pos {_latitudeDD, _longitudeDD, _heightM};
    return llh_pos;
}


bool GPSPosition::operator==(const GPSPosition& rhs) const
{
    // ignore the accuracy.
    return _latitudeDD == rhs._latitudeDD && _longitudeDD == rhs._longitudeDD && _heightM == rhs._heightM;
}

bool GPSPosition::operator!=(const GPSPosition& rhs) const
{
    // ignore the accuracy.
    return _latitudeDD != rhs._latitudeDD || _longitudeDD != rhs._longitudeDD || _heightM != rhs._heightM;
}

