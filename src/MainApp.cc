/**************************************************************************
 * Copyright 2012 Bryan Godbolt
 * Copyright 2013 Joseph Lewis <joehms22@gmail.com>
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
 *************************************************************************/

#include "MainApp.h"

/* Project Headers */
#include "servo_switch.h"
#include "heli.h"
#include "LogFile.h"
#include "QGCLink.h"
#include "RCTrans.h"
#include "Helicopter.h"
#include "Control.h"
#include "Debug.h"
#include "bad_control.h"
#include "IMU.h"
#include "GPS.h"
#include "Driver.h"
#include "MdlAltimeter.h"
#include "RateLimiter.h"
#include "TCPSerial.h"
#include "Linux.h"
#include "SystemState.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time.hpp>


const std::string MainApp::LOG_SCALED_INPUTS = "Scaled Inputs";


MainApp* MainApp::_instance = NULL;
std::mutex MainApp::_instance_lock;

MainApp* MainApp::getInstance()
{
    std::lock_guard<std::mutex> lock(_instance_lock);
    if (_instance == NULL)
    {
        _instance = new MainApp;
    }
    return _instance;
}


MainApp::MainApp()
    :autopilot_mode(heli::MODE_AUTOMATIC_CONTROL)
{
    this->_terminate = false;
}


void MainApp::run()
{
    boost::posix_time::ptime startTime(boost::posix_time::microsec_clock::local_time());          // Used during timer-based schedg tests, to create a program start time stamp.

    signal(SIGINT, [](int signum)
    {
        MainApp::getInstance()->terminate();
    }); // Shutdown program by sending a SIGINT.

    request_mode.connect([](heli::AUTOPILOT_MODE mode)
    {
        MainApp::getInstance()->change_mode(mode);
    });

    /* Construct components of the autopilot */
    message() << "Setting up system state object";
    SystemState::getInstance();

    message() << "Setting up servo board";
    servo_switch* servo_board = servo_switch::getInstance();

    message() << "Setting up LogFile";
    LogFile *log = LogFile::getInstance();

    message() << "Setting up IMU";
    IMU::getInstance();

    message() << "Setting up Altimeter";
    MdlAltimeter::getInstance();

    message() << "Setting up TCP";
    new TCPSerial();

    message() << "Setting up QGCLink";
    QGCLink* qgc = QGCLink::getInstance();
    qgc->shutdown.connect(MainApp::terminate);
    qgc->servo_source.connect(this->request_mode);

    message() << "Setting up Helicopter";
    Helicopter* bergen = Helicopter::getInstance();

    message() << "Setting up control";
    Control* control = Control::getInstance();

    message() << "Setting up Linux CPU Reader";
    Linux::getInstance();


    // broadcast the controller mode
    control->mode_changed(control->get_controller_mode());
    GPS::getInstance();

    using std::vector;
    vector<uint16_t> inputMicros(6);
    vector<double> inputScaled(6);

    // Set default autopilot mode
    autopilot_mode = heli::MODE_AUTOMATIC_CONTROL;
    mode_changed(autopilot_mode);

    uint16_t ch7PulseWidthLast = 1000;
    uint16_t ch7PulseWidth = 1000;

    boost::signals2::scoped_connection pilot_connection(servo_board->pilot_mode_changed.connect(
                boost::bind(&MainApp::change_pilot_mode, this, _1)));

    message() << "Started main loop";
    RateLimiter rl(100);

    while(! _terminate.load())
    {

        /* Dequeue messages & pulses on a channel with MsgReceivev(). Threads Receive-block & queue on channel for a msg/pulse to arrive.  */
        rl.wait();

        // Pilot Flight log marker.
        ch7PulseWidth = servo_board->getRaw(heli::CH7);
        if(ch7PulseWidth - ch7PulseWidthLast > 500)
        {
            log->logData("Flight log marker", std::vector<uint16_t>());
            ch7PulseWidthLast = ch7PulseWidth;
        }
        else if (ch7PulseWidth - ch7PulseWidthLast < -500)
        {
            ch7PulseWidthLast = ch7PulseWidth;
        }


        inputScaled = RCTrans::getScaledVector();
        log->logHeader(LOG_SCALED_INPUTS, "CH1 CH2 CH3 CH4 CH5 CH6");
        log->logData(LOG_SCALED_INPUTS, inputScaled);

        switch(autopilot_mode.load())
        {
        case heli::MODE_DIRECT_MANUAL:
            inputMicros = servo_board->getRaw();
            servo_board->setRaw(inputMicros);
            break;

        case heli::MODE_SCALED_MANUAL:
            bergen->setScaled(inputScaled);
            break;

        case heli::MODE_AUTOMATIC_CONTROL:
        {
            if (control->runnable())
            {
                try
                {
                    (*control)();
                    bergen->setScaled(control->get_control_effort());
                }
                catch (bad_control& b)
                {
                    critical() << "MainApp: Caught control error exception.";
                    critical() << "Exception Message: " << b;
                    critical() << "MainApp: Switching to Direct Manual Mode.";
                    request_mode(heli::MODE_DIRECT_MANUAL);
                }
            }
            else
            {
                critical() << "MainApp: Controller reports that it is not runnable.";
                critical() << "MainApp: Switching to Direct Manual Mode.";
                request_mode(heli::MODE_DIRECT_MANUAL);
            }
            break;
        }

        default:
            critical() << "MainApp: The given mode is not defined!";
            critical() << "MainApp: Switching to Direct Manual Mode.";
            request_mode(heli::MODE_DIRECT_MANUAL);
            break;
        }

        rl.finishedCriticalSection();
    }

    Driver::terminateAll();
}

boost::signals2::signal<void (heli::AUTOPILOT_MODE)> MainApp::mode_changed;
boost::signals2::signal<void (heli::AUTOPILOT_MODE)> MainApp::request_mode;

void MainApp::change_mode(heli::AUTOPILOT_MODE mode)
{
    debug() << "Switching autopilot mode out of " << MainApp::getModeString();
    autopilot_mode = mode;
    message() << "Switched autopilot mode into " << MainApp::getModeString();
    MainApp::mode_changed(mode);
}

std::string MainApp::getModeString()
{
    return heli::AUTOPILOT_MODE_DESCRIPTOR[autopilot_mode.load()];
}

void MainApp::change_pilot_mode(heli::PILOT_MODE mode)
{
    if (mode == heli::PILOT_AUTO)
    {
        warning() << "Pilot engaged autopilot. Recording position setpoint";
        Control::getInstance()->reset();
        Control::getInstance()->set_reference_position();
    }
}

