/*
 * RateLimiter.h
 *
 *  Created on: Sep 27, 2013
 *      Author: Joseph Lewis <joehms22@gmail.com>
 */

#ifndef RATELIMITER_H_
#define RATELIMITER_H_

#include <thread>
#include <chrono>

class RateLimiter
{
private:
    std::chrono::milliseconds _msToWait;
    std::chrono::time_point<std::chrono::high_resolution_clock> _nextTime;

public:
    /**
     * Provides a limiting mechanism to functions
     *
     * @param hz - the number of hertz to run this function.
     */
    RateLimiter(int hz);
    virtual ~RateLimiter();


    /**
     * This function should be called within loops to slow them down if necessary
     * it will return when it is time to "wake up"
     */
    void wait();

    /**
     * This function is called when the loop is finished with one iteration,
     * it gives the OS an opportunity to do some cleanup and go about doing other
     * things.
     */
    void finishedCriticalSection();
};

#endif /* RATELIMITER_H_ */
