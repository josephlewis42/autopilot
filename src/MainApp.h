/*******************************************************************************
 * Copyright 2012 Bryan Godbolt
 * Copyright 2014 Joseph Lewis <joseph@josephlewis.net>
 *
 * This file is part of ANCL Autopilot.
 *
 *     ANCL Autopilot is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     ANCL Autopilot is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License
 *     along with ANCL Autopilot.  If not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************/

#ifndef MAINAPP_H_
#define MAINAPP_H_

/* STL Headers */
#include <string>
#include <mutex>
#include <atomic>

/* Project Headers */
#include "heli.h"
#include "Debug.h"

/* Boost Headers */
#include <boost/signals2.hpp>

/**
 * \brief This class replaces the standard main function and implements the main program logic.
 * \author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * \author Joseph Lewis <joseph@josephlewis.net>
 * \date June 15, 2011 Class created
 * @date January 20, 2012 Added pilot mode switch from new takeover
 * \date 2014-06-19 - Removed the main signaling and thread tracking done
 * by this class as it now is handled internally by all drivers
 */

class MainApp : public Logger
{
public:

    /**
    Returns the one allowed instance of this Driver
    **/
    static MainApp* getInstance();

    /// Function in which to place main program logic (replaces main()).
    void run();

    /// signal send by main app to notify other threads of a mode change (in particular qgclink::qgcsend)
    static boost::signals2::signal<void (heli::AUTOPILOT_MODE)> mode_changed;

    /// signal to request a mode change from other threads
    static boost::signals2::signal<void (heli::AUTOPILOT_MODE)> request_mode;

    /// terminates the signal
    static void terminate()
    {
        MainApp::getInstance()->_terminate = true;
    };


private:
    static const std::string LOG_SCALED_INPUTS ;
    static MainApp* _instance;
    static std::mutex _instance_lock;


    /// default constructor (initializes terminate to false)
    MainApp();

    /// thread safe copy constructor
    MainApp(const MainApp& other) = delete;

    /// thread safe assignment operator
    const MainApp& operator=(const MainApp& other) = delete;

    /// controls whether the main loop continues to execute
    std::atomic_bool _terminate;

    /// stores the current operating mode of the autopilot
    std::atomic<heli::AUTOPILOT_MODE> autopilot_mode;

    /// @returns the string representation of the current mode
    std::string getModeString();

    /// slot connected to MainApp::request_mode to change the value of MainApp::autopilot_mode
    void change_mode(heli::AUTOPILOT_MODE mode);

    /** Changes the pilot mode and sends a signal out that it has been changed.
     *
     **/
    void change_pilot_mode(heli::PILOT_MODE mode);


};

#endif /* MAINAPP_H_ */
