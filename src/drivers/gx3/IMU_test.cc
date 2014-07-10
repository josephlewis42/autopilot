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

    GPSPosition llh(1,2,3);

    IMU* imu = IMU::getInstance();
    imu->setPosition(llh);

    EXPECT_EQ(imu->getPosition(), llh);
}

TEST(IMU, SET_NED_ORIGIN)
{
    GPSPosition llh(1,2,3);

    IMU* imu = IMU::getInstance();
    imu->setNedOrigin(llh);

    EXPECT_EQ(imu->getNedOriginPosition(), llh);
}
