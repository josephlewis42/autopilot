/*
 * Copyright 2014 Joseph Lewis <joseph@josephlewis.net>
 *
 * This file is part of University of Denver Autopilot.
 * Dual licensed under the GPL v 3 and the Apache 2.0 License
 */
#include "IMU.h"
#include "Parameter.h"
#include <gtest/gtest.h>
#include <cmath>
#include <array>

// TESTS
TEST(IMU, SET_POSITION)
{
    boost::numeric::ublas::vector<double> llh(3);
    llh[0] = 1;
    llh[1] = 2;
    llh[2] = 3;

    IMU* imu = IMU::getInstance();
    imu->set_position(llh);

    for(int i = 0; i < 3; i++)
        EXPECT_EQ(imu->get_position()[i], llh[i]);
}

TEST(IMU, SET_NED_ORIGIN)
{
    boost::numeric::ublas::vector<double> llh(3);
    llh[0] = 1;
    llh[1] = 2;
    llh[2] = 3;

    IMU* imu = IMU::getInstance();
    imu->set_ned_origin(llh);

    for(int i = 0; i < 3; i++)
        EXPECT_EQ(imu->get_ned_origin()[i], llh[i]);
}


TEST(IMU, LLH_POSITION_ECEF)
{
    boost::numeric::ublas::vector<double> llh(3);
    llh[0] = 1;
    llh[1] = 2;
    llh[2] = 3;

    boost::numeric::ublas::vector<double> exp(3);
    exp[0] = 6373.29 * 1000;
    exp[1] = 222.56 * 1000;
    exp[2] = 110.569 * 1000;

    IMU* imu = IMU::getInstance();
    imu->set_position(llh);

    for(int i = 0; i < 3; i++)
        printf("llh ecef %d : %f\n", i, imu->get_ecef_position()[i]);

    auto res = imu->get_ecef_position();
    EXPECT_LT(fabs(res(0) - exp(0)), 1);
    EXPECT_LT(fabs(res(1) - exp(1)), 1);
    EXPECT_LT(fabs(res(2) - exp(2)), 1);
}

TEST(IMU, ORIGIN_POSITION_ECEF)
{
    boost::numeric::ublas::vector<double> llh(3);
    llh[0] = 1;
    llh[1] = 2;
    llh[2] = 3;

    boost::numeric::ublas::vector<double> exp(3);
    exp[0] = 6373.29 * 1000;
    exp[1] = 222.56 * 1000;
    exp[2] = 110.569 * 1000;

    IMU* imu = IMU::getInstance();
    imu->set_ned_origin(llh);

    auto res = imu->get_ecef_position();
    EXPECT_LT(fabs(res(0) - exp(0)), 1);
    EXPECT_LT(fabs(res(1) - exp(1)), 1);
    EXPECT_LT(fabs(res(2) - exp(2)), 1);
}
