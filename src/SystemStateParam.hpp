/*
 * Copyright 2014 Joseph Lewis <joseph@josephlewis.net>
 *
 * This file is part of University of Denver Autopilot.
 * Dual licensed under the GPL v 3 and the Apache 2.0 License
 */

#pragma once
#ifndef SYSTEM_STATE_PARAM_H
#define SYSTEM_STATE_PARAM_H

#include <type_traits>
#include <atomic>
#include <limits>
#include <stdint.h>
#include <chrono>


/** SystemStateParam represents a system state parameter.

Parameters automatically become invalid after a certain time, have a value a min and max value
and only update themselves if the old value is invalid or the new value has less than or equal
to the same error as the old value.

Therefore, multiple devices can connect to a SystemStateParam to submit the same data. For
example say we have three altimeter, one that works for short ranges, one that works for long
ranges and one in software that interpolates based on previous height and vertical acceleration.

Near the ground the short range altimeter would report less of an error than the long range
altimeter, while in the sky the long range would report less and it's value would be chosen.

Now let's say that both are knocked out due to a power outage on the craft, once the precise
value beceomes invalid because of the timeout the software version can take over even while
reporting a high error. As soon as a more reliable source comes in to play its values will
be discarded however.

**/
template <class T>
class SystemStateParam
{
    static_assert(std::is_fundamental<T>::value, "Type must be primitive in SystemStateParam!");

public:
    /**
    @param invalidationTimeMS - the number of milliseconds until the parameter becomes invalid, if you
    want the param to be constantly updating set to 0.
    @param minValue - the minimum value of this param, anything greater is ignored
    @param maxValue - the maximum value of this param, anything greater is ignored

    \note If minValue is less than maxValue, the limits of the data type are used i.e. for uint8_t 0 and 255
    **/
    SystemStateParam(uint64_t invalidationTimeMS, T minValue=1, T maxValue=0)
    {
        if(minValue > maxValue)
        {
            _min = std::numeric_limits<T>::min();
            _max = std::numeric_limits<T>::max();
        }
        else
        {
            _min = minValue;
            _max = maxValue;
        }

        _lastTime = std::chrono::system_clock::now();
        _currentError = std::numeric_limits<double>::max();
    }

    /** Sets the value of this parameter.
    @param value - the value to set
    @param error - the +/- error of the value.

    @return false if the value was not set, true if it was.
    **/
    bool set(T value, double error)
    {
        if(error < 0)
        {
            error *= -1;
        }

        if(value < _min || value > _max)
        {
            return false;
        }

        auto currtime = std::chrono::system_clock::now();
        uint64_t elapsed_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(currtime-_lastTime).count();

        if( error <= _currentError ||
            elapsed_milliseconds > _invalidationTimeMS)
        {
            _value = value;
            _currentError = error;
            _lastTime = currtime;
            return true;
        }

        return false;
    }

    /** Returns the value associated with this system state parameter.
    **/
    T get()
    {
        return _value.load();
    }

private:
    T _min;
    T _max;
    std::atomic<T> _value;
    double _currentError;
    uint64_t _invalidationTimeMS;
    std::chrono::time_point<std::chrono::system_clock> _lastTime;
};

#endif
