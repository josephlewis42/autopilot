/**
 * Copyright 2014 Joseph Lewis <joseph@josephlewis.net>
 *
 * This file is part of University of Denver Autopilot.
 * Dual licensed under the GPL v 3 and the Apache 2.0 License
 *
**/

#include "LogFileWriter.h"
#include "Path.h"
#include <gtest/gtest.h>
#include <thread>
#include <chrono>

// TESTS
TEST(LogfileWriter, getLogger)
{
    auto one = LogfileWriter::getLogger("test_one");
    auto two = LogfileWriter::getLogger("test_two");
    auto three = LogfileWriter::getLogger("test_one");

    EXPECT_EQ(one, three);
    EXPECT_NE(one, two);
}

TEST(LogfileWriter, getLogPath)
{
    auto one = LogfileWriter::getLogger("test_one");
    auto two = LogfileWriter::getLogger("test_two");
    auto three = LogfileWriter::getLogger("test_one");

    one->log("test");
    two->log("test2");
    three->log("test2");

    /// should sync within 1 second.
    std::chrono::milliseconds dura( 1000 );
    std::this_thread::sleep_for(dura);

    EXPECT_TRUE(one->getLogPath().exists());
    EXPECT_TRUE(two->getLogPath().exists());
    EXPECT_TRUE(three->getLogPath().exists());
}
