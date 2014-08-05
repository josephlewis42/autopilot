/**
 * Copyright 2014 Joseph Lewis <joseph@josephlewis.net>
 *
 * This file is part of University of Denver Autopilot.
 * Dual licensed under the GPL v 3 and the Apache 2.0 License
**/

#pragma once

#ifndef FAKE_RC_H
#define FAKE_RC_H

#include "Plugin.h"
#include "Singleton.h"

/**
The FakeRc system fakes pilot inputs.
**/
class FakeRc : public Plugin, public Singleton<FakeRc>
{
    friend Singleton<FakeRc>;
    public:
        virtual bool init();
        virtual void loop();
        virtual void teardown();

    private:
        FakeRc();
        int lows[8];
        int highs[8];
        int step;
        int period;
};




#endif //FAKE_RC_H
