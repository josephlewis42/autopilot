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
