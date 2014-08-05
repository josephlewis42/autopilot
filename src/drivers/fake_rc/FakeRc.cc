/**
 * Copyright 2014 Joseph Lewis <joseph@josephlewis.net>
 *
 * This file is part of University of Denver Autopilot.
 * Dual licensed under the GPL v 3 and the Apache 2.0 License
**/

#include "FakeRc.h"
#include "SystemState.h"
#include <array>
#include <string>

FakeRc::FakeRc()
:Plugin("Fake RC", "fake_rc", 5),
step(0)
{
    start(); // Start the plugin
}

bool FakeRc::init()
{
    period = configGeti("period_seconds", 2);

    if(period <= 0)
        period = 2;

    for(int i = 0; i < 8; i++)
    {
        lows[i] = configGeti("channel_" + std::to_string(i + 1) + "_low", 1000);
        highs[i] = configGeti("channel_" + std::to_string(i + 1) + "_high", 2000);
    }

    auto state = SystemState::getInstance();
    state->servoRawInputs.onSet.connect([&](std::array<uint16_t, 8> val, double err){debug() << "Got servo positions" << val;});


    return true;
}

void FakeRc::loop()
{
    int stepsPerPeriod = period * 5;
    int stepInPeriod = step % stepsPerPeriod;

    std::array<uint16_t,8> servoInputs;

    for(int i = 0; i < 8; i++)
    {
        int amount = (highs[i] - lows[i]) / stepsPerPeriod;
        servoInputs[i] = stepInPeriod * amount + lows[i];

        debug() << "Channel " << i << " is " << servoInputs[i];
    }

    auto state = SystemState::getInstance();
    state->servoRawInputs.set(servoInputs, 0);

    step++;
}

void FakeRc::teardown()
{
}
