/*
 * AutopilotMath.hpp
 *
 *  Created on: Jun 12, 2014
 *      Author: joseph
 */

#ifndef AUTOPILOTMATH_HPP_
#define AUTOPILOTMATH_HPP_

namespace AutopilotMath{

	const double PI = 3.141592653589793238462643383279502884197169399375105820974944f;

	inline double radiansToDegrees(double radians)
	{
		return radians * 180.0 / PI;
	};

	inline double degreesToRadians(double degrees)
	{
		return degrees * PI / 180.0 ;
	};
}


#endif /* AUTOPILOTMATH_HPP_ */
