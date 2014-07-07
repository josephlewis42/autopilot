/**
 * Copyright 2014 Joseph Lewis <joseph@josephlewis.net>
 *
 * This file is part of University of Denver Autopilot.
 * Dual licensed under the GPL v 3 and the Apache 2.0 License
 *
**/

#include "RateLimiter.h"
#include <gtest/gtest.h>
#include <thread>
#include "Timer.hpp"

// TESTS
TEST(RateLimiter, Frequency_Acceptable_100HZ)
{
    int a = 0;

    Timer t;

    t.click();
    RateLimiter rl(100);
    for(int i = 0; i < 100; i++)
    {
        rl.wait();
        a++;
    }
    long ms = t.click();
    long error = (ms > 1000) ? ms - 1000 : 1000 - ms;

    EXPECT_LT(error, 10);
}

TEST(RateLimiter, Frequency_Acceptable_10HZ)
{
    int a = 0;

    Timer t;

    t.click();
    RateLimiter rl(10);
    for(int i = 0; i < 10; i++)
    {
        rl.wait();
        a++;
    }
    long ms = t.click();
    long error = (ms > 1000) ? ms - 1000 : 1000 - ms;

    EXPECT_LT(error, 10);
}
