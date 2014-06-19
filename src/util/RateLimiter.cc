/**************************************************************************
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

#include "RateLimiter.h"

#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include "Debug.h"

#define NDEBUG

RateLimiter::RateLimiter(int hz)
    :_msToWait(1000 / hz),
     _io(),
     _timer(_io),
     _nextTime()
{
    _nextTime = boost::posix_time::microsec_clock::universal_time();
    _nextTime = _nextTime + _msToWait;
    _timer.expires_at(_nextTime);
    _io.run();
}

RateLimiter::~RateLimiter()
{
    _io.stop();
}

void RateLimiter::wait()
{
    try
    {
        _timer.wait();
    }
    catch(boost::system::system_error err)
    {
        return;
    }

    _nextTime += _msToWait;

#ifndef NDEBUG
    if(boost::posix_time::microsec_clock::universal_time() > _nextTime)
    {
        debug() << "RateLimiter: Fallen behind!";
    }
#endif

    _timer.expires_at(_nextTime);
}

void RateLimiter::finishedCriticalSection()
{
    // yield the thread so others can execute now.
    boost::thread::yield();
}
