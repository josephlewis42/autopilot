/*******************************************************************************
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
 ******************************************************************************/

#ifndef SERVO_SWITCH_H_
#define SERVO_SWITCH_H_

/* STL Headers */
#include <vector>
#include <sys/types.h>
#include <mutex>
#include <atomic>
#include <thread>

#include <boost/signals2.hpp>

/* Project Headers */
#include "Driver.h"
#include "heli.h"
#include "Singleton.h"



/**
 * @brief Contains hardware specific code for Microbotics Servo Switch.
 * @author Bryan Godbolt <godbolt@ece.ualberta.ca>
 * @author Nikos Vitzilaios <nvitzilaios@ualberta.ca>
 *
 * This class runs two threads.  One to send data on the serial port (/dev/ser3) and one to receive data.
 * The sending thread runs at 50 Hz and is used to transmit the new pulse widths commanded by the control.
 * The receive thread is used to get the current pilot inputs and the status message which indicates the state of the
 * control channel (pilot manual or pilot auto).
 * @date February 2012: Class creation
 * @date May 1, 2012: Added auxiliary input for engine speed
 */
class servo_switch : public Driver, public Singleton<servo_switch>
{
    friend Singleton<servo_switch>;
public:

    virtual void writeToSystemState() override;

    class read_serial
    {
    public:
        void operator()()
        {
            read_data();
        }

    private:

        void read_data();
        void sync();
        void parse_message(uint8_t id, const std::vector<uint8_t>& payload);
        void parse_pulse_inputs(const std::vector<uint8_t>& payload);
        void parse_aux_inputs(const std::vector<uint8_t>& payload);
        void find_next_header();
        int readSerialBytes(int fd, void * buf, int n);
    };

    class send_serial
    {
    public:
        void operator()();
    };

    /** get the current pilot inputs
     */
    std::vector<uint16_t> getRaw()
    {
        return get_raw_inputs();
    }
    uint16_t getRaw(heli::Channel ch)
    {
        return get_raw_inputs()[ch];
    }
    /// set the value of the servo outputs
    void setRaw(const std::vector<uint16_t>& raw_outputs)
    {
        set_raw_outputs(raw_outputs);
        writeToSystemState();
    }
    inline void setRaw(heli::Channel ch, uint16_t pulse_width)
    {
        std::lock_guard<std::mutex> lock(raw_outputs_lock);
        raw_outputs[ch] = pulse_width;
        writeToSystemState();
    }

    /// signal with new mode as argument
    boost::signals2::signal<void (heli::PILOT_MODE)> pilot_mode_changed;
    inline heli::PILOT_MODE get_pilot_mode()
    {
        return pilot_mode;
    }

    inline double get_engine_speed()
    {
        return engine_speed;
    }
    inline double get_engine_rpm()
    {
        return engine_speed * 60;
    }
    inline double get_main_rotor_speed()
    {
        return engine_speed * 13.0 / 90.0;
    }
    inline double get_main_rotor_rpm()
    {
        return get_engine_rpm() * 13.0 / 90.0;
    }

private:
    servo_switch();

    static const std::string LOG_INPUT_PULSE_WIDTHS ;
    static const std::string LOG_OUTPUT_PULSE_WIDTHS ;
    static const std::string LOG_INPUT_RPM ;


    /// @returns true if the port was successfully set up, false otherwise
    bool init_port();

    std::atomic<int> fd_ser1;

    std::thread receive;
    std::thread send;

    static std::vector<uint8_t> compute_checksum(uint8_t id, uint8_t count, const std::vector<uint8_t>& payload);

    std::vector<uint16_t> raw_inputs;
    std::mutex raw_inputs_lock;
    inline std::vector<uint16_t> get_raw_inputs()
    {
        std::lock_guard<std::mutex> lock(raw_inputs_lock);
        return raw_inputs;
    }
    inline void set_raw_inputs(const std::vector<uint16_t>& raw_inputs)
    {
        std::lock_guard<std::mutex> lock(raw_inputs_lock);
        this->raw_inputs = raw_inputs;
    }

    std::vector<uint16_t> raw_outputs;
    std::mutex raw_outputs_lock;
    inline std::vector<uint16_t> get_raw_outputs()
    {
        std::lock_guard<std::mutex> lock(raw_outputs_lock);
        return raw_outputs;
    }
    inline void set_raw_outputs(const std::vector<uint16_t>& raw_outputs)
    {
        std::lock_guard<std::mutex> lock(raw_outputs_lock);
        this->raw_outputs = raw_outputs;
    }

    std::atomic<heli::PILOT_MODE> pilot_mode;
    void set_pilot_mode(heli::PILOT_MODE mode);

    std::atomic<double> engine_speed;
    inline void set_engine_speed(double speed)
    {
        engine_speed = speed;
    }
};

#endif
