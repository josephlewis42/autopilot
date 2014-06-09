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

#include "pid_error.h"

/* c headers */
#include <math.h>


pid_error::pid_error(double integral_error_limit)
:_integral_error_limit(integral_error_limit)
{
	reset();
}

double pid_error::operator++()
{
	_integral = _integral + _proportional * 0.01; // multiply by the timestep

	if(fabs(getIntegral()) > _integral_error_limit)
	{
		_integral = 0;
	}

	return _integral;
}

/// zero all the errors
void pid_error::reset()
{
	_integral = 0;
	_derivative = 0;
	_proportional = 0;
}


/* Global functions */
std::ostream& operator<<(std::ostream& os, const pid_error& error)
{
	return os << error._integral << ", " << error._derivative << ", " << error._proportional;
}

Debug& operator<<(Debug& dbg, const pid_error& error)
{
	return dbg << error._integral << ", " << error._derivative << ", " << error._proportional;
}
