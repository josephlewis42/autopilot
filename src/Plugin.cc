/**
 * Copyright 2014 Joseph Lewis <joseph@josephlewis.net>
 *
 * This file is part of University of Denver Autopilot.
 * Dual licensed under the GPL v 3 and the Apache 2.0 License
**/

#include "Plugin.h"
#include "RateLimiter.h"

Plugin::Plugin(std::string humanReadableName,
               std::string machineReadableName,
               int requestedLoopFrequencyHZ
              )
:Driver(humanReadableName, machineReadableName),
loopRateHz(requestedLoopFrequencyHZ)
{


}

void Plugin::start()
{
     // Tell the user we made it up
    warning() << "Calling Init";
    bool initResult = init();

    if(initResult == false)
    {
        initFailed("Could not init plugin, init() returned false");
        return;
    }

    // If the system wants to halt, don't start running.
    if(terminateRequested())
    {
        return;
    }

    // If the user has disabled this component, don't start running
    if(! isEnabled())
    {
        return;
    }

    // Tell the user we made it up
    warning() << "Starting main loop";


    if(loopRateHz != 0)
    {
        // Start our processing thread.
        new std::thread(Plugin::internalLoop, this);
    }
}


void Plugin::internalLoop(Plugin* inst)
{
    if(inst->loopRateHz > 0)
    {
        RateLimiter rl(inst->loopRateHz);
        while(! inst->terminateRequested())
        {
            rl.wait();
            inst->loop();
            rl.finishedCriticalSection();
        }
    }
    else
    {
        while(! inst->terminateRequested())
        {
            inst->loop();
        }
    }

    inst->teardown();
}
