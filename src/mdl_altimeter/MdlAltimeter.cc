/*
 * MdlAltimeter.cpp
 *
 *  Created on: May 16, 2014
 *      Author: joseph
 */

#include "MdlAltimeter.h"
/* File Handling Headers */
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/* C Headers */
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <cstdlib>
#include <math.h>


/* Project Headers */
#include "Configuration.h"
#include "Debug.h"
#include "gx3_read_serial.h"
#include "MainApp.h"
#include "message_parser.h"
#include "ack_handler.h"
#include "gx3_send_serial.h"
#include "QGCLink.h"
#include "SystemState.h"



const std::string ALTIMETER_ENABLED = "mdl_altimeter.enabled";
const bool ALTIMETER_ENABLED_DEFAULT = true;
const std::string ALTIMETER_PATH = "mdl_altimeter.device";
const std::string ALTIMETER_PATH_DEFAULT = "/dev/ttyUSB0";

MdlAltimeter* MdlAltimeter::_instance = NULL;
std::mutex MdlAltimeter::_instance_lock;

MdlAltimeter* MdlAltimeter::getInstance()
{
    std::lock_guard<std::mutex> lock(_instance_lock);

    if (!_instance)
    {
        _instance = new MdlAltimeter();
    }

    return _instance;
}

MdlAltimeter::MdlAltimeter()
    :Driver("MDL Altimeter", "mdl_altimeter")
{
    isEnabled = configGetb("enabled", true);

    if(terminateRequested())
    {
        return;
    }

    if(!isEnabled)
    {
        warning() << "MDL Altimeter disabled!";
        return;
    }

    // Open up the serial port
    std::string serial_path = Configuration::getInstance()->gets(ALTIMETER_PATH, ALTIMETER_PATH_DEFAULT);
    _serialFd = open(serial_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

    // Set up the terminal.
    if(!namedTerminalSettings("Altimeter1", _serialFd, 38400, "8N1", false, true))
    {
        warning() << "Could not setup serial!";
    }
    else
    {
        debug() << "Altimeter set up!";
        distance = 0;
        has_new_distance = false;
        new boost::thread(&MdlAltimeter::mainLoop, this);
    }
}

MdlAltimeter::~MdlAltimeter()
{
    close(_serialFd);
}

void MdlAltimeter::mainLoop()
{

    uint8_t first = '\0', second = '\0';
    uint16_t multiplierCM = 10;
    uint16_t numberToAverage = 100;
    uint16_t averagedThusFar = 0;
    uint16_t sum = 0;

    debug() << "Started main Altimeter loop ";

    while(! terminateRequested())
    {
        while(((first >> 6) & 0b11) != 0x2)
        {
            readDevice(_serialFd, &first, 1);
        }
        if((readDevice(_serialFd, &second, 1) >> 6) != 0x0)
        {
            continue;
        }

        // Average a number of results because the error on this device is huge.
        if(averagedThusFar < numberToAverage)
        {
            uint16_t first_masked = first & 0b00111111; // last six bits of first byte
            uint16_t second_masked = second & 0b00111111; // last six bits of second byte
            uint16_t decoded_dist = (first_masked << 6) | second_masked; // combine them to decode distance
            sum += decoded_dist;
            averagedThusFar++;
        }
        else
        {
            distance = (float(sum) / float(averagedThusFar)) * multiplierCM;
            has_new_distance = true;
            sum = 0;
            averagedThusFar = 0;
        }
        first = '\0';
        second = '\0';
    }
}

bool MdlAltimeter::sendMavlinkMsg(mavlink_message_t* msg, int uasId, int sendRateHz, int msgNumber)
{
    if(! isEnabled) return false;

    if(has_new_distance) // only send message for new distance
    {
        mavlink_msg_ualberta_altimeter_pack(uasId, heli::ALTIMETER_ID, msg, distance);
        //debug() << "Sending altimeter distance: " << distance << "\n";
        has_new_distance = false;
        return true;
    }
    return false;
};

void MdlAltimeter::writeToSystemState()
{
    SystemState *state = SystemState::getInstance();
    state->state_lock.lock();
    state->altimeter_height = distance;
    state->state_lock.unlock();
};