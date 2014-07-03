/**
 * Copyright 2014 Joseph Lewis <joseph@josephlewis.net>
 *
 * This file is part of University of Denver Autopilot.
 * Dual licensed under the GPL v 3 and the Apache 2.0 License
 *
**/

#include "Singleton.h"
#include <gtest/gtest.h>

class SingletonTestClass : public Singleton<SingletonTestClass>
{
    friend class Singleton<SingletonTestClass>;
    public:
        int value;

    private:
        SingletonTestClass()
        :value(0){}
};


// TESTS
TEST(Singleton, Same_Instance)
{
    EXPECT_EQ(SingletonTestClass::getInstance(), SingletonTestClass::getInstance());
}

TEST(Singleton, Diff_After_Delete)
{
    SingletonTestClass::getInstance()->value = 100;
    EXPECT_EQ(SingletonTestClass::getInstance()->value, 100);
    SingletonTestClass::destroyInstance();
    EXPECT_EQ(SingletonTestClass::getInstance()->value, 0);
}

TEST(Singleton, Instance_if_constructed)
{
    SingletonTestClass::destroyInstance();
    auto c = SingletonTestClass::getInstanceIfConstructed();
    EXPECT_EQ(c, nullptr);
}

TEST(Singleton, Instance_if_constructed_equal)
{
    auto a = SingletonTestClass::getInstance();
    auto b = SingletonTestClass::getInstanceIfConstructed();

    EXPECT_EQ(a, b);
}

