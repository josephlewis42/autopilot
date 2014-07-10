/*
 * Copyright 2014 Joseph Lewis <joseph@josephlewis.net>
 *
 * This file is part of University of Denver Autopilot.
 * Dual licensed under the GPL v 3 and the Apache 2.0 License
 */
#include "SystemStateObjParam.hpp"
#include <gtest/gtest.h>
#include <vector>


// TESTS
TEST(SystemStateObjParam, CheckDefault)
{
    std::vector<int> in = {1,2,3};
    SystemStateObjParam<std::vector<int> > ssp(0, in);

    std::vector<int> other;
    other = ssp.get();
    EXPECT_EQ(other.size(), in.size());
    EXPECT_EQ(other[0], in[0]);
    EXPECT_EQ(other[1], in[1]);
    EXPECT_EQ(other[2], in[2]);
}

TEST(SystemStateObjParam, Toss_Worse)
{
    std::vector<int> one = {1,1,1};
    std::vector<int> two = {2,2,2};
    SystemStateObjParam<std::vector<int> > ssp(100 * 1000, one); // as long as test takes less than 100 seconds we're good.
    EXPECT_EQ(ssp.set(one, 0), true); // initial set should work
    EXPECT_EQ(ssp.set(two, 1), false); // test positive out of range
    EXPECT_EQ(ssp.set(two, -1), false); // test negative
    EXPECT_EQ(ssp.set(two, 0), true); // test equal
}


TEST(SystemStateObjParam, observer)
{
    std::vector<int> in = {1,1,1};
    std::vector<int> two = {2,2,2};
    SystemStateObjParam<std::vector<int> > ssp(0, in);
    SystemStateObjParam<std::vector<int> > ssp2(0, in);

    ssp.notifySet(ssp2);
    ssp.set(two, 0);

    auto other = ssp2.get();
    EXPECT_EQ(other.size(), two.size());
    EXPECT_EQ(other[0], two[0]);
    EXPECT_EQ(other[1], two[1]);
    EXPECT_EQ(other[2], two[2]);
}
