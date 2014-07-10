/*
 * Copyright 2014 Joseph Lewis <joseph@josephlewis.net>
 *
 * This file is part of University of Denver Autopilot.
 * Dual licensed under the GPL v 3 and the Apache 2.0 License
 */
#include "SystemStateParam.hpp"
#include <gtest/gtest.h>



// TESTS
TEST(SystemStateParam, Toss_OOR)
{
    SystemStateParam<double> ssp(0, 1, 2);
    EXPECT_EQ(ssp.set(0, 0), false); // can't set value to be 0, out of range
    EXPECT_EQ(ssp.set(1, 0), true); // edge case
    EXPECT_EQ(ssp.set(2, 0), true); // edge case
    EXPECT_EQ(ssp.set(1.5, 0), true); // normal case
}

TEST(SystemStateParam, Toss_Worse)
{
    SystemStateParam<double> ssp(100 * 1000); // as long as test takes less than 100 seconds we're good.
    EXPECT_EQ(ssp.set(100, 0), true); // initial set should work
    EXPECT_EQ(ssp.set(200, 1), false); // test positive out of range
    EXPECT_EQ(ssp.set(200, -1), false); // test negative
    EXPECT_EQ(ssp.set(200, 0), true); // test equal
}

TEST(SystemStateParam, TimeoutInit)
{
    SystemStateParam<double> ssp(100 * 1000); // as long as test takes less than 100 seconds we're good.
    EXPECT_EQ(ssp.set(100, 0), true); // initial set should work
}

TEST(SystemStateParam, TossWorsePositive)
{
    SystemStateParam<double> ssp(100 * 1000); // as long as test takes less than 100 seconds we're good.
    EXPECT_EQ(ssp.set(100, 0), true); // initial set should work
    EXPECT_EQ(ssp.set(200, 1), false); // test positive out of range

}

TEST(SystemStateParam, TossWorseNegative)
{
    SystemStateParam<double> ssp(100 * 1000); // as long as test takes less than 100 seconds we're good.
    EXPECT_EQ(ssp.set(100, 0), true); // initial set should work
    EXPECT_EQ(ssp.set(200, -1), false); // test negative

}

TEST(SystemStateParam, AcceptEqualBeforeTimeout)
{
    SystemStateParam<double> ssp(100 * 1000); // as long as test takes less than 100 seconds we're good.
    EXPECT_EQ(ssp.set(100, 0), true); // initial set should work
    EXPECT_EQ(ssp.set(200, 0), true); // test equal

}



TEST(SystemStateParam, observer)
{
    SystemStateParam<double> ssp(0);
    SystemStateParam<double> ssp2(0);
    ssp2.set(400, 0);

    ssp.notifySet(ssp2);
    ssp.set(100, 0);

    EXPECT_EQ(ssp2.get(), 100);
}

