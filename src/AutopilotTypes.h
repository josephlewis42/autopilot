/*
 * AutopilotTypes.h -- basic types for the autopilot, should
 * be used to replace the boost:: blas stuff.
 *
 *  Created on: Jun 10, 2014
 *      Author: Joseph Lewis III <joseph@josephlewis.net>
 */

#ifndef AUTOPILOTTYPES_H_
#define AUTOPILOTTYPES_H_


struct LatLonHeight{
	double latDD;
	double lonDD;
	double heightM;
};

struct EulerAngles{
	double rollRad;
	double pitchRad;
	double yawRad;
};



#endif /* AUTOPILOTTYPES_H_ */
