/**
 * A plugin is a simplified Driver that takes care of the threading, init, stopping,
 * and message retrieval and sending by itself.
 *
 * To make a plugin, simply override this class and implement the given methods.
 * Copyright 2014 Joseph Lewis <joseph@josephlewis.net>
 *
 * This file is part of University of Denver Autopilot.
 * Dual licensed under the GPL v 3 and the Apache 2.0 License
**/

#pragma once
#ifndef PLUGIN_H
#define PLUGIN_H

#include "Driver.h"
#include <string>
#include <mavlink.h>

class Plugin : public Driver
{
    public:
        /**
        Constructs a new Plugin

        @param humanReadableName - a human readable name that the plugin can use to
        send messages out as.
        @param machineReadableName - a valid XML tag that can be used to store the
        configuration of this plugin.
        @param requestedLoopFrequencyHZ - the requested frequency at which loop will
        called in hertz. A negative frequency means as fast as possible while a
        zero frequency will never call loop()
        **/
        Plugin(std::string humanReadableName,
               std::string machineReadableName,
               int requestedLoopFrequencyHZ
              );

        /**
        Override this method to start your plugin. Don't do any processing here,
        rather gather all of your configuration information.

        @return true if setup was a success, false if it failed
        **/
        virtual bool init() = 0;

        /**
        Override this method to do one step of the processing needed, it will be
        called at the requested rate (or as close as possible)
        **/
        virtual void loop() = 0;

        /**
        Use this method to perform cleanup before terminating (close files, etc.)
        **/
        virtual void teardown() = 0;

        /**
        Call this method once you are done with your constructor.
        **/
        void start();

    private:
        static void internalLoop(Plugin* inst);
        int loopRateHz;
};


#endif // PLUGIN_H
