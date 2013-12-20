/*******************************************************************************
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
 ******************************************************************************/


#ifndef THREADSAFEVARIABLE_H_
#define THREADSAFEVARIABLE_H_

template<typename T>
class ThreadSafeVariable
{
private:
	mutable boost::mutex _mutex;
	T _value;
public:
	operator T() const {boost::mutex::scoped_lock(_mutex); return _value;}

	ThreadSafeVariable<T>& operator =(const T& newValue)
	{
		boost::mutex::scoped_lock(_mutex);
		_value = newValue;
		return *this;
	}
};

#endif /* THREADSAFEVARIABLE_H_ */
