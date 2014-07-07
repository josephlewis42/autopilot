/*
 * Driver.cpp
 *
 *  Created on: Aug 26, 2013
 *      Author: joseph
 */

#include "Driver.h"
#include <termios.h>

#include "Configuration.h"
#include "qnx2linux.h"

#include <mavlink.h>
#include "MainApp.h"

#include "Debug.h"
#include "Configuration.h"
#include <thread>
#include <cstdio>
#include <fcntl.h>
#include <algorithm>


std::mutex Driver::_all_drivers_lock;
std::list<Driver*> Driver::all_drivers;
std::atomic_bool Driver::_all_drivers_terminate(false);



Driver::Driver(std::string name, std::string config_prefix)
    : Logger(name + ": "),
      ConfigurationSubTree(config_prefix),
      _terminate(_all_drivers_terminate.load()),
      _config_prefix(config_prefix),
      _name(name),
      _savePathFd(-1), // by default don't save anything
      _driverInit(std::chrono::system_clock::now())
{
    {
        std::lock_guard<std::mutex> lock(_all_drivers_lock);
        all_drivers.push_front(this);
    }

    configDescribe("logging_level",
                  "0-5",
                  "Sets the level of messages you want logged in this driver. 0-trace+, 1-debug+, 2-info+, 3-warning+, 4-critical+, 5-none");
    setLoggingLevel(configGeti("logging_level", 2));

    configDescribe("read_style",
                   "0 - 3",
                   "The style of serial port read to use for this driver.");
    _readDeviceType = configGeti("read_style", 2);

    configDescribe("enable",
                   "true/false",
                   "Enables/disables this driver.");
    _enabled = configGetb("enable", true);

    configDescribe("terminate_if_init_failed",
                   "true/false",
                   "Defines whether the autopilot should terminate if this driver is enabled but fails to initialize.");
    _terminate_if_init_failed = configGetb("terminate_if_init_failed", true);

    configDescribe("read_style_COMMENT",
                   "string",
                   "Description for the functionality of different serial read styles.");
    configGets("read_style_COMMENT", "0:read until min, 1:readcond, 2:read(), 3:wait then read()");

    configDescribe("read_save_path",
                  "path to a file or blank to not save",
                  "A location on the filesystem where all of the incoming data passed through the driver "
                   "`readDevice` call will be appended.");
    _savePath = configGets("read_save_path", "");

    if(!_savePath.empty())
    {
        _savePathFd = open(_savePath.c_str(), O_CREAT | O_WRONLY | O_APPEND);
    }
}

Driver::~Driver()
{
    std::lock_guard<std::mutex> lock(_all_drivers_lock);
    all_drivers.remove(this);
}

void Driver::terminateAll()
{
    _all_drivers_terminate = true;

    {
        std::lock_guard<std::mutex> lock(_all_drivers_lock);
        for(Driver* d : all_drivers)
        {
            d->terminate();
        }
    }

    // wait for all threads to terminate
    sleep(3);
}

std::vector<Driver*> Driver::getDrivers()
{
    std::vector<Driver*> drivers;
    {
        std::lock_guard<std::mutex> lock(_all_drivers_lock);
        for(Driver* d : all_drivers)
        {
            drivers.push_back(d);
        }
    }

    return drivers;
}

int Driver::readDevice(int fd, void * buf, int n)
{
    int amt = 0;

    switch(_readDeviceType)
    {
        case 0:
            amt = QNX2Linux::readUntilMin(fd, buf, n, n);
            break;
        case 1:
            amt = QNX2Linux::readcond(fd, buf, n, n, 10,10);
            break;
        case 3:
        {
            struct termios port_config;
            tcgetattr(fd, &port_config);                  // get the current port settings

            // Set the baud rate
            speed_t speed = cfgetospeed(&port_config);
            int waittimeMS = (1000 * speed) / (n * 8);

            std::this_thread::sleep_for( std::chrono::milliseconds( waittimeMS ) );
            amt = read(fd, buf, n);
            break;
        }
        default:
            amt = read(fd, buf, n);
    }

    if(_savePathFd > 0)
    {
        write(_savePathFd, buf, amt);
    }

    return amt;
}

bool Driver::namedTerminalSettings(std::string name,
                                   int fd,
                                   int baudrate,
                                   std::string parity,
                                   bool enableHwFlow,
                                   bool enableRawMode)
{
    Configuration* config = Configuration::getInstance();

    std::string prefix = _config_prefix + ".terminal." + name;
    baudrate = config->geti(prefix + ".baudrate", baudrate);
    parity = config->gets(prefix + ".parity", parity);
    enableHwFlow = config->getb(prefix + ".hardware_flow_control", enableHwFlow);
    enableRawMode = config->getb(prefix + ".raw_mode", enableRawMode);

    return terminalSettings(fd, baudrate, parity, enableHwFlow, enableRawMode);

}

bool Driver::terminalSettings(int fd,
                              int baudrate,
                              std::string parity,
                              bool enableHwFlow,
                              bool enableRawMode)
{
    // Set up the terminal configuration for the given port.
    struct termios options;

    tcgetattr(fd, &options);                  // get the current port settings

    // Set the baud rate
    uint32_t optbaud = 0;
    switch(baudrate)
    {
    case 9600:
        optbaud = B9600;
        break;
    case 19200:
        optbaud = B19200;
        break;
    case 38400:
        optbaud = B38400;
        break;
    case 57600:
        optbaud = B57600;
        break;
    case 115200:
        optbaud = B115200;
        break;
    default:
        warning() << "The selected baud of " << baudrate << " is not standard.";
        optbaud = baudrate;
    }


    // Set baudrates for input and output
    cfsetospeed(&options, optbaud);
    cfsetispeed(&options, optbaud);



    //parity
    std::transform(parity.begin(), parity.end(), parity.begin(), ::toupper);


    if(parity == std::string("8N1"))  	//No parity (8N1):
    {
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
    }
    else if(parity == std::string("7E1"))  	//Even parity (7E1):
    {
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS7;
    }
    else if(parity == std::string("701"))
    {
        // Odd parity (7O1):
        options.c_cflag |= PARENB;
        options.c_cflag |= PARODD;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS7;

    }
    else if(parity == std::string("7M1"))  	//Mark parity
    {
        options.c_cflag &= ~PARENB;
        options.c_cflag |= CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS7;
    }
    else     //Space parity is setup the same as no parity (7S1):
    {
        if(parity != std::string("7S1"))
        {
            warning() << "The parity " << parity << " is not known, use one of [8N1, 7E1, 701, 7M1, 7S1]";
        }
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
    }

    // Hardware flow control
    if(enableHwFlow)
    {
        //options.c_cflag |= CNEW_RTSCTS;
    }
    else
    {
        //options.c_cflag &= ~CNEW_RTSCTS;
    }


    // raw mode
    if(enableRawMode)
    {
        //set for non-canonical (raw processing, no echo, etc.)
        /**options.c_iflag = IGNPAR; // ignore parity check close_port(int
        options.c_oflag = 0; // raw output
        options.c_lflag = 0; // raw input
        **/
        cfmakeraw(&options);
    }
    else
    {
        options.c_lflag |= (ICANON | ECHO | ECHOE);
    }

    options.c_cflag |= (CLOCAL | CREAD); // Enable the receiver and set local mode...

    // Clear terminal output flow control.
    if (tcsetattr(fd, TCSADRAIN, &options) != 0)
    {
        critical() << "could not set serial port attributes";
        return false;
    }

    if(tcflush(fd, TCIOFLUSH) == -1)
    {
        critical() << "could not purge the serial port";
        return false;
    }

    trace() << "serial set up success " << fd;

    return true;
}



long Driver::getMsSinceInit() const
{
    std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - _driverInit;
    return (long)(elapsed_seconds.count() * 1000);
}


 /**
    * Alerts the user and system that the initialization of this driver failed, if
    * terminate_on_init_fail is set to true, terminates the platform.
    *
    * @param why - the reason for the initialization failure.
    **/
void Driver::initFailed(std::string why)
{
    std::string msg =  getName() + " initialization failed with message: " + why;

    if(_terminate_if_init_failed.load() == true)
    {
        critical() << msg;
        critical() << "CALLING TERMIANTE";
        _all_drivers_terminate = true;
        MainApp::terminate();
    }
    else
    {
        warning() << msg;
    }
}


bool Driver::isEnabled()
{
    return _enabled.load();
}
