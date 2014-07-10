/*
 * Copyright 2014 Joseph Lewis <joseph@josephlewis.net>
 *
 * This file is part of University of Denver Autopilot.
 * Dual licensed under the GPL v 3 and the Apache 2.0 License
 */
#include "GPSPosition.h"
#include <gtest/gtest.h>
#include <cmath>
#include <array>

// TESTS
TEST(GPSPosition, SET_POSITION)
{
    GPSPosition gps(1,2,3);
    EXPECT_EQ(gps.getLatitudeDD(), 1);
    EXPECT_EQ(gps.getLongitudeDD(), 2);
    EXPECT_EQ(gps.getHeightM(), 3);
}

TEST(GPSPosition, LLH_POSITION_TO_ECEF)
{
    GPSPosition gps(1,2,3);

    boost::numeric::ublas::vector<double> exp(3);
    exp[0] = 6373.29 * 1000;
    exp[1] = 222.56 * 1000;
    exp[2] = 110.569 * 1000;


    auto res = gps.ecef();
    EXPECT_LT(fabs(res(0) - exp(0)), 1);
    EXPECT_LT(fabs(res(1) - exp(1)), 1);
    EXPECT_LT(fabs(res(2) - exp(2)), 1);
}


TEST(GPSPosition, NED_OFFSET_NONE)
{
    GPSPosition gps_origin(1,1,1);
    GPSPosition gps(1,1,1);

    boost::numeric::ublas::vector<double> exp(3);
    exp[0] = 6373.29 * 1000;
    exp[1] = 222.56 * 1000;
    exp[2] = 110.569 * 1000;


    auto res = gps.ned(gps_origin);
    EXPECT_EQ(res(0), 0);
    EXPECT_EQ(res(1), 0);
    EXPECT_EQ(res(2), 0);
}

TEST(GPSPosition, NED_OFFSET_LAT_ONE)
{
    GPSPosition gps_origin(0,0,0);
    GPSPosition gps(0.001,0,0);

    auto res = gps.ned(gps_origin);
    EXPECT_GT(res(1), 0); // ensure we are positive for going north.


    // Wikipedia lists 1 dd at the equator to be 111.32 km
    //https://en.wikipedia.org/wiki/Decimal_degrees
    EXPECT_EQ(res(0), 0);
    EXPECT_LT(fabs(res(1) - 111.320), 10); // +/- 10m is okay
    EXPECT_LT(fabs(res(2)), .01); // +/- 1 cm is okay
}


TEST(GPSPosition, NED_OFFSET_LON_ONE)
{
    GPSPosition gps_origin(0,0,0);
    GPSPosition gps(0,0.001,0);

    auto res = gps.ned(gps_origin);

    EXPECT_GT(res(0), 0); // ensure we are positive for going east.

    // Wikipedia lists 1 dd at the equator to be 111.32 km
    //https://en.wikipedia.org/wiki/Decimal_degrees
    EXPECT_LT(fabs(res(0) - 111.320), 10); // +/- 10m is okay
    EXPECT_EQ(res(1), 0);
    EXPECT_LT(fabs(res(2)), .01); // +/- 1 cm is okay
}


TEST(GPSPosition, NED_OFFSET_HEIGHT_ONE)
{
    GPSPosition gps_origin(0,0,0);
    GPSPosition gps(0,0,1);

    auto res = gps.ned(gps_origin);

    EXPECT_LT(res(2), 0); // ensure we are negative for going up.

    EXPECT_EQ(res(0), 0);
    EXPECT_EQ(res(1), 0);
    EXPECT_EQ(res(2), -1);
}



TEST(GPSPosition, toLLH)
{
    GPSPosition zero(0,0,0);
    GPSPosition one(0,0,1);
    GPSPosition two(0,1,0);
    GPSPosition four(1,0,0);

    EXPECT_EQ(zero.toLLH()[0], 0);
    EXPECT_EQ(zero.toLLH()[1], 0);
    EXPECT_EQ(zero.toLLH()[2], 0);

    EXPECT_EQ(one.toLLH()[0], 0);
    EXPECT_EQ(one.toLLH()[1], 0);
    EXPECT_EQ(one.toLLH()[2], 1);

    EXPECT_EQ(two.toLLH()[0], 0);
    EXPECT_EQ(two.toLLH()[1], 1);
    EXPECT_EQ(two.toLLH()[2], 0);

    EXPECT_EQ(four.toLLH()[0], 1);
    EXPECT_EQ(four.toLLH()[1], 0);
    EXPECT_EQ(four.toLLH()[2], 0);
}


TEST(GPSPosition, equality_operator)
{
    GPSPosition zero(0,0,0);
    GPSPosition one(0,0,1);
    GPSPosition two(0,1,0);
    GPSPosition three(0,1,1);
    GPSPosition four(1,0,0);
    GPSPosition five(1,0,1);
    GPSPosition six(1,1,0);
    GPSPosition seven(1,1,1);

    EXPECT_EQ(one, one);
    EXPECT_NE(one, two);
    EXPECT_NE(one, zero);
    EXPECT_NE(one, three);
    EXPECT_NE(one, four);
    EXPECT_NE(one, five);
    EXPECT_NE(one, six);
    EXPECT_NE(one, seven);
    EXPECT_EQ(two, two);
    EXPECT_NE(two, zero);
    EXPECT_NE(two, three);
    EXPECT_NE(two, four);
    EXPECT_NE(two, five);
    EXPECT_NE(two, six);
    EXPECT_NE(two, seven);
    EXPECT_EQ(zero, zero);
    EXPECT_NE(zero, three);
    EXPECT_NE(zero, four);
    EXPECT_NE(zero, five);
    EXPECT_NE(zero, six);
    EXPECT_NE(zero, seven);
    EXPECT_EQ(three, three);
    EXPECT_NE(three, four);
    EXPECT_NE(three, five);
    EXPECT_NE(three, six);
    EXPECT_NE(three, seven);
    EXPECT_EQ(four, four);
    EXPECT_NE(four, five);
    EXPECT_NE(four, six);
    EXPECT_NE(four, seven);
    EXPECT_EQ(five, five);
    EXPECT_NE(five, six);
    EXPECT_NE(five, seven);
    EXPECT_EQ(six, six);
    EXPECT_NE(six, seven);
    EXPECT_EQ(seven, seven);

    // test ignore of accuracy

    GPSPosition zero_perfect(0, 0, 0, 0);
    GPSPosition zero_imperfect(0, 0, 0, 100);

    EXPECT_TRUE(zero_perfect == zero_imperfect); // == operator
    EXPECT_FALSE(zero_perfect != zero_imperfect); // != operator
}
