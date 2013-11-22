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

#include "Parameter.h"

const int MAX_PARAM_ID_LENGTH = 14;

Parameter::Parameter(std::string name, float value, int compid)
{
	setParamID(name);
	setValue(value);
	setCompID(compid);
}

void Parameter::setParamID(std::string name)
{
	const int len = name.length();

	if(len == 0)
	{
		std::cout << "Empty parameter name received. Please enter a parameter name" << std::endl;
	}

	if(len > MAX_PARAM_ID_LENGTH)
	{
		// truncate string to 14 chars
		name = name.substr(0, MAX_PARAM_ID_LENGTH);
	}
	else if(len < MAX_PARAM_ID_LENGTH)
	{
		// pad with spaces
		int strlen = MAX_PARAM_ID_LENGTH - len;
		name = name.append(strlen, ' ');
	}
	id = name;
}

Debug& operator<<(Debug& dbg, const Parameter& p)
{
	return dbg << "Parameter ID: " << p.getParamID() << ", Value: " << p.getValue();
}
