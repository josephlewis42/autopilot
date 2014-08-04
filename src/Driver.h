/*******************************************************************************
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
 ******************************************************************************/

#ifndef DRIVER_H_
#define DRIVER_H_

#include <mutex>
#include <atomic>
#include <vector>
#include <chrono>
#include <mavlink.h>
#include <list>

#include "Debug.h"
#include "Configuration.h"


/**
 * The driver class is the base class for all the extension points in the program.
 *
 * @author Joseph Lewis <joehms22@gmail.com>
 */
class Driver : public Logger, public ConfigurationSubTree
{
private:
    /// store whether to terminate the thread
    std::atomic_bool _terminate;

    /// stores the prefix for the driver
    std::string _config_prefix;

    /// Keeps the human readable name for the current driver.
    std::string _name;

    /// Locks the all_drivers list
    static std::mutex _all_drivers_lock;

    /// Keeps a list of all drivers so we can iterate over them later.
    static std::list<Driver*> all_drivers;

    /// Keeps the global value of terminate
    static std::atomic_bool _all_drivers_terminate;

    /// Keeps the value of the read property on a particular device.
    int _readDeviceType;

    /// Keeps the value of the save path for reading (a tee location where the raw data can be dumped)
    std::string _savePath;

    /// The file descriptor of the save path.
    int _savePathFd;

    /// Keeps the time that the driver was initiated
    std::chrono::time_point<std::chrono::system_clock> _driverInit;

    /// True if the driver is enabled, false if it is not.
    std::atomic_bool _enabled;

    /// Holds the property of whether or not to terminate if init failed.
    std::atomic_bool _terminate_if_init_failed;

public:
    Driver(std::string name, std::string config_prefix);
    virtual ~Driver();

    /// calls terminate() on all drivers
    static void terminateAll();

    /// Returns the human-readable name of this driver
    inline const std::string getName()
    {
        return _name;
    };

    /** Checks if terminate() has been called for this driver
     *
     * @return true if terminate() has been called, false otherwise
     **/
    inline bool terminateRequested()
    {
        return _terminate;
    };

    inline void terminate()
    {
        debug() << "Driver Terminating: " << getName();
        _terminate = true;
    };


    /**
     * Gets a vector with all drivers contained in it.
     **/
    static std::vector<Driver*> getDrivers();


    /**
     * Reads a fd in to the given buffer with a minimum of n bytes
     **/
    int readDevice(int fd, void * buf, int n);

    /**
     * Sets the given terminal configuration on the given fd and saves them
     * with name. If name already exists in the configuration file, those
     * settings will overwrite these.
     */
    bool namedTerminalSettings(	std::string name,
                                int fd,
                                int baudrate,
                                std::string parity,
                                bool enableHwFlow,
                                bool enableRawMode);


    /**
     * Sets up the terminal at the given fd with the given baudrate,
     * parity (one of: [8N1, 7E1, 701, 7M1, 7S1])
     * hw flow control
     * and raw mode
     */
    bool terminalSettings(	int fd,
                            int baudrate,
                            std::string parity,
                            bool enableHwFlow,
                            bool enableRawMode);

    /**
     * Override this method to send out your own MavLink messages from
     * your driver.
     *
     * @param msgs - append your message to this vector to send it
     * @param uasId - the identifier of this system
     * @param sendRateHz - the number of messages sent/sec.
     * @param msgNumber - the number of messages sent thus far
     *
     **/
    virtual void sendMavlinkMsg(std::vector<mavlink_message_t>& msgs, int uasId, int sendRateHz, int msgNumber)
    {
    };

    /**
     * Utility function for checking if a message should be sent.
     *
     * @param messageNumber - the number of times the channel has transmitted so far
     * @param channelRateHz - the rate at which the channel is sending messages
     * @param desiredRateHz - the rate at which we would like to send messages,
     * numbers <= 0 mean that the function will never return true.
     *
     * @return true if it is time to send a message, false if it is not.
     **/
    bool shouldSendMavlinkMessage(int messageNumber, int channelRateHz, int desiredRateHz)
    {
        if(desiredRateHz <= 0)
        {
            return false;
        }

        if(desiredRateHz > channelRateHz)
        {
            return true;
        }

        return (messageNumber % (channelRateHz / desiredRateHz) == 0);
    }


    /**
     * Override this method to get a copy of incoming mavlink messages.
     *
     * @param msg - the message to send
     * @return - true if you processed this message, false otherwise.
     **/
    virtual bool recvMavlinkMsg(const mavlink_message_t& msg)
    {
        return false;
    };

    /**
     * Override this method to write any relevent values to the
     * SystemState object
    **/
    virtual void writeToSystemState() {};

    /**
     * Gets the number of milliseconds since this driver was instnatiated.
     **/
    long getMsSinceInit() const;

    /**
     * Gets the number of microseconds since the driver was insantitated.
     **/
    long getMicrosSinceInit() const;

    /**
    * Alerts the user and system that the initialization of this driver failed, if
    * terminate_on_init_fail is set to true, terminates the platform.
    *
    * @param why - the reason for the initialization failure.
    **/
    void initFailed(std::string why="");

    /**
     * Checks to see if this driver should be run.
     *
     * @return true if the driver is enabled, false if it is not.
     **/
    bool isEnabled();


};

#endif /* DRIVER_H_ */
