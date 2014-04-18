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

#ifdef __QNX__
#include <sched.h>      // for setting thread priorities.

#endif
#include "RateLimiter.h"

MainApp::MainApp()
{
	this->_terminate = false;
}

MainApp::MainApp(const MainApp& other)
{
	{
		std::lock_guard<std::mutex> scoped_lock(other.terminate_lock);
		_terminate = other._terminate;
	}
	{
		std::lock_guard<std::mutex> scoped_lock(other.autopilot_mode_lock);
		autopilot_mode = other.autopilot_mode;
	}
}

const MainApp& MainApp::operator=(const MainApp& other)
{
	if (this == &other)
		return *this;

	// use addresses to ensure mutexes are always locked in same order
	std::lock_guard<std::mutex> lock1(&terminate_lock < &other.terminate_lock? terminate_lock : other.terminate_lock);
	std::lock_guard<std::mutex> lock2(&terminate_lock > &other.terminate_lock? terminate_lock : other.terminate_lock);
	_terminate = other._terminate;

	return *this;
}


#if defined(LOGFILE_TEST)
#include "tests/logfile_test.cc"
#elif defined(QGCLINK_TEST)
#include "tests/qgclink_test.cc"
#elif defined(GX3_TEST)
#include "tests/gx3_test.cc"
#else

void MainApp::run()
{
	boost::posix_time::ptime startTime(boost::posix_time::microsec_clock::local_time());          // Used during timer-based schedg tests, to create a program start time stamp.
	boost::this_thread::at_thread_exit(cleanup());


	do_terminate terminate_slot(this);
	terminate.connect(terminate_slot);

	request_mode.connect(change_mode(this));

	signal(SIGINT, heli::shutdown);             // Shutdown program by sending a SIGINT.

	/* Construct components of the autopilot */
	message() << "Settnig up servo board";
	servo_switch* servo_board = servo_switch::getInstance();

	message() << "Setting up LogFile";
	LogFile *log = LogFile::getInstance();

	message() << "Setting up IMU";
	IMU::getInstance();

	message() << "Setting up QGCLink";
	QGCLink* qgc = QGCLink::getInstance();
	qgc->shutdown.connect(this->terminate);
	qgc->servo_source.connect(this->request_mode);

	message() << "Setting up Helicopter";
	Helicopter* bergen = Helicopter::getInstance();

	message() << "Setting up control";
	Control* control = Control::getInstance();


	// broadcast the controller mode
	control->mode_changed(control->get_controller_mode());
	GPS::getInstance();

	using std::vector;
	vector<uint16_t> inputMicros(6);
	vector<double> inputScaled(6);

	// Set default autopilot mode
	autopilot_mode_lock.lock();
	autopilot_mode = heli::MODE_AUTOMATIC_CONTROL;
	mode_changed(autopilot_mode);
	autopilot_mode_lock.unlock();

	uint16_t ch7PulseWidthLast = 1000;
	uint16_t ch7PulseWidth = 1000;

	boost::signals2::scoped_connection pilot_connection(servo_board->pilot_mode_changed.connect(
			boost::bind(&MainApp::change_pilot_mode, this, _1)));

	message() << "Started main loop";
	RateLimiter rl(100);

	while(check_terminate())
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
		log->logHeader(heli::LOG_SCALED_INPUTS, "CH1 CH2 CH3 CH4 CH5 CH6");
		log->logData(heli::LOG_SCALED_INPUTS, inputScaled);

		switch(getMode())
		{
		case heli::MODE_DIRECT_MANUAL:
		{
			inputMicros = servo_board->getRaw();
			servo_board->setRaw(inputMicros);
			break;
		}
		case heli::MODE_SCALED_MANUAL:
		{
			bergen->setScaled(inputScaled);
			break;
		}
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
		}

		rl.finishedCriticalSection();
	}
}

#endif

std::vector<MainApp::ThreadName> MainApp::threads;

void MainApp::add_thread(boost::thread *thread, std::string name)
{
	threads.push_back(MainApp::ThreadName(thread, name));
}

MainApp::ThreadName::ThreadName(boost::thread *thread, std::string name)
{
	this->thread = thread;
	this->name = name;
}

bool MainApp::check_terminate()
{
	terminate_lock.lock();
	bool terminate = this->_terminate;
	terminate_lock.unlock();
	return !terminate;
}

boost::signals2::signal<void ()> MainApp::terminate;
boost::signals2::signal<void (heli::AUTOPILOT_MODE)> MainApp::mode_changed;
boost::signals2::signal<void (heli::AUTOPILOT_MODE)> MainApp::request_mode;

void MainApp::cleanup::operator()()
{
	Driver::terminateAll();

    int n = (int)boost::posix_time::time_duration::ticks_per_second() * 1;
    boost::posix_time::time_duration delay(0,0,0,n);

	BOOST_FOREACH(ThreadName t, MainApp::threads)
	{
		debug() << "MainApp: Waiting for " << (t.name.empty()?"Unknown Thread":t.name);
		t.thread->timed_join(delay);
	}
	message() << "All registered threads ended.";
}

void MainApp::do_terminate::operator()()
{

	parent->terminate_lock.lock();
	if (!parent->_terminate)
	{
		parent->_terminate = true;
		message() << "MainApp: Received terminate signal.  Shutting down.";
	}
	parent->terminate_lock.unlock();
}

void MainApp::change_mode::operator()(heli::AUTOPILOT_MODE mode)
{
	std::lock_guard<std::mutex> lock(parent->autopilot_mode_lock);
	debug() << "Switching autopilot mode out of " << MainApp::getModeString(parent->autopilot_mode);
	parent->autopilot_mode = mode;
	message() << "Switched autopilot mode into " << MainApp::getModeString(parent->autopilot_mode);
	MainApp::mode_changed(mode);
}

int MainApp::getMode()
{
	std::lock_guard<std::mutex> lock(autopilot_mode_lock);
	return autopilot_mode;
}

std::string MainApp::getModeString()
{
	return heli::AUTOPILOT_MODE_DESCRIPTOR[getMode()];
}

std::string MainApp::getModeString(heli::AUTOPILOT_MODE mode)
{
	return heli::AUTOPILOT_MODE_DESCRIPTOR[mode];
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

