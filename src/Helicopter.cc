/**************************************************************************
 * Copyright 2012 Bryan Godbolt, Hasitha Senanayake
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

#include "Helicopter.h"
#include "Configuration.h"
#include "SystemState.h"

/* STL Headers */
#include <fstream>
#include <string>

// Configuration Headers
const std::string XML_ROOT = "physical_params.";
const std::string XML_MASS = XML_ROOT + "mass";
const std::string XML_MAIN_HUB_OFFSET_X = XML_ROOT + "main_hub_offset.x";
const std::string XML_MAIN_HUB_OFFSET_Y = XML_ROOT + "main_hub_offset.y";
const std::string XML_MAIN_HUB_OFFSET_Z = XML_ROOT + "main_hub_offset.z";
const std::string XML_TAIL_HUB_OFFSET_X = XML_ROOT + "tail_hub_offset.x";
const std::string XML_TAIL_HUB_OFFSET_Y = XML_ROOT + "tail_hub_offset.y";
const std::string XML_TAIL_HUB_OFFSET_Z = XML_ROOT + "tail_hub_offset.z";
const std::string XML_INERTIA_X = XML_ROOT + "inertia.x";
const std::string XML_INERTIA_Y = XML_ROOT + "inertia.y";
const std::string XML_INERTIA_Z = XML_ROOT + "inertia.z";




Helicopter::Helicopter()
    :Logger("Helicopter"),
     radio_cal_data(RadioCalibration::getInstance()),
     out(servo_switch::getInstance()),
     mass(13.65),
     gravity(9.8),
     main_hub_offset(3),
     tail_hub_offset(3),
     inertia(3,3)
{
    main_hub_offset.clear();
    main_hub_offset(2) = -0.32;

    tail_hub_offset.clear();
    tail_hub_offset(0) = -1.06;

    inertia(0,0) = 0.36;
    inertia(1,1) = 1.48;
    inertia(2,2) = 1.21;

    Configuration* config = Configuration::getInstance();

    set_mass(config->getd(XML_MASS, mass));

    set_main_hub_offset_x(config->getd(XML_MAIN_HUB_OFFSET_X, 0.0));
    set_main_hub_offset_y(config->getd(XML_MAIN_HUB_OFFSET_Y, 0.0));
    set_main_hub_offset_z(config->getd(XML_MAIN_HUB_OFFSET_Z, -0.32));

    set_tail_hub_offset_x(config->getd(XML_TAIL_HUB_OFFSET_X, -1.06));
    set_tail_hub_offset_y(config->getd(XML_TAIL_HUB_OFFSET_Y, 0.0));
    set_tail_hub_offset_z(config->getd(XML_TAIL_HUB_OFFSET_Z, 0.0));

    set_inertia_x(config->getd(XML_INERTIA_X, 0.36));
    set_inertia_y(config->getd(XML_INERTIA_Y, 1.48));
    set_inertia_z(config->getd(XML_INERTIA_Z, 1.21));

    writeToSystemState();
}

Helicopter* Helicopter::_instance = NULL;

Helicopter* Helicopter::getInstance()
{
    if(!_instance)
        _instance = new Helicopter;

    return _instance;
}

const std::string Helicopter::PARAM_MASS = "Mass";

const std::string Helicopter::PARAM_MAIN_OFFSET_X = "Main_X";
const std::string Helicopter::PARAM_MAIN_OFFSET_Y = "Main_Y";
const std::string Helicopter::PARAM_MAIN_OFFSET_Z = "Main_Z";

const std::string Helicopter::PARAM_TAIL_OFFSET_X = "Tail_X";
const std::string Helicopter::PARAM_TAIL_OFFSET_Y = "Tail_Y";
const std::string Helicopter::PARAM_TAIL_OFFSET_Z = "Tail_Z";

const std::string Helicopter::PARAM_INERTIA_X = "J_X";
const std::string Helicopter::PARAM_INERTIA_Y = "J_Y";
const std::string Helicopter::PARAM_INERTIA_Z = "J_Z";

std::vector<Parameter> Helicopter::getParameters()
{
    std::vector<Parameter> plist;

    plist.push_back(Parameter(PARAM_MASS, get_mass(), heli::HELICOPTER_ID));

    plist.push_back(Parameter(PARAM_MAIN_OFFSET_X, get_main_hub_offset()(0), heli::HELICOPTER_ID));
    plist.push_back(Parameter(PARAM_MAIN_OFFSET_Y, get_main_hub_offset()(1), heli::HELICOPTER_ID));
    plist.push_back(Parameter(PARAM_MAIN_OFFSET_Z, get_main_hub_offset()(2), heli::HELICOPTER_ID));

    plist.push_back(Parameter(PARAM_TAIL_OFFSET_X, get_tail_hub_offset()(0), heli::HELICOPTER_ID));
    plist.push_back(Parameter(PARAM_TAIL_OFFSET_Y, get_tail_hub_offset()(1), heli::HELICOPTER_ID));
    plist.push_back(Parameter(PARAM_TAIL_OFFSET_Z, get_tail_hub_offset()(2), heli::HELICOPTER_ID));

    plist.push_back(Parameter(PARAM_INERTIA_X, get_inertia()(0,0), heli::HELICOPTER_ID));
    plist.push_back(Parameter(PARAM_INERTIA_Y, get_inertia()(1,1), heli::HELICOPTER_ID));
    plist.push_back(Parameter(PARAM_INERTIA_Z, get_inertia()(2,2), heli::HELICOPTER_ID));

    return plist;
}

void Helicopter::setParameter(Parameter p)
{
    std::string param_id(p.getParamID());
    boost::trim(param_id);

    if (param_id == PARAM_MASS)
        set_mass(p.getValue());

    else if (param_id == PARAM_MAIN_OFFSET_X)
        set_main_hub_offset_x(p.getValue());
    else if (param_id == PARAM_MAIN_OFFSET_Y)
        set_main_hub_offset_y(p.getValue());
    else if (param_id == PARAM_MAIN_OFFSET_Z)
        set_main_hub_offset_z(p.getValue());

    else if (param_id == PARAM_TAIL_OFFSET_X)
        set_tail_hub_offset_x(p.getValue());
    else if (param_id == PARAM_TAIL_OFFSET_Y)
        set_tail_hub_offset_y(p.getValue());
    else if (param_id == PARAM_TAIL_OFFSET_Z)
        set_tail_hub_offset_z(p.getValue());

    else if (param_id == PARAM_INERTIA_X)
        set_inertia_x(p.getValue());
    else if (param_id == PARAM_INERTIA_Y)
        set_inertia_y(p.getValue());
    else if (param_id == PARAM_INERTIA_Z)
        set_inertia_z(p.getValue());
    else
        debug() << "Helicopter: Received unknown parameter.";

    saveFile();
    writeToSystemState();
}

void Helicopter::saveFile()
{
    Configuration* config = Configuration::getInstance();

    config->set(XML_MASS, std::to_string(get_mass()));

    blas::vector<double> hub(get_main_hub_offset());
    config->set(XML_MAIN_HUB_OFFSET_X, std::to_string(hub(0)));
    config->set(XML_MAIN_HUB_OFFSET_Y, std::to_string(hub(1)));
    config->set(XML_MAIN_HUB_OFFSET_Z, std::to_string(hub(2)));

    blas::vector<double> tail(get_tail_hub_offset());
    config->set(XML_TAIL_HUB_OFFSET_X, std::to_string(tail(0)));
    config->set(XML_TAIL_HUB_OFFSET_Y, std::to_string(tail(1)));
    config->set(XML_TAIL_HUB_OFFSET_Z, std::to_string(tail(2)));

    blas::banded_matrix<double> inertia(get_inertia());
    config->set(XML_INERTIA_X, std::to_string(inertia(0,0)));
    config->set(XML_INERTIA_Y, std::to_string(inertia(1,1)));
    config->set(XML_INERTIA_Z, std::to_string(inertia(2,2)));
}

void Helicopter::writeToSystemState()
{
    SystemState *state = SystemState::getInstance();
    state->state_lock.lock();
    state->helicopter_gravity = gravity;
    state->helicopter_params = getParameters();
    state->state_lock.unlock();
}

uint16_t Helicopter::norm2pulse(double norm, std::array<uint16_t, 3> setpoint)
{
    uint16_t pulse = 0;
    if(setpoint[2] > setpoint[0])
    {
        if(norm >= 0)
            pulse = setpoint[1] + (norm * (setpoint[2] - setpoint[1]));
        else
            pulse = setpoint[1] - (norm * (setpoint[0] - setpoint[1]));
    }
    else if(setpoint[2] < setpoint[0])
    {
        if(norm >= 0)
            pulse = setpoint[1] - (norm * (setpoint[1] - setpoint[2]));
        else
            pulse = setpoint[1] + (norm * (setpoint[1] - setpoint[0]));
    }
    return pulse;
}

uint16_t Helicopter::norm2pulse(double norm, std::array<uint16_t, 2> setpoint)
{
    uint16_t pulse = 0;
    {
        if(norm == 0)
            pulse = setpoint[0];
        else
            pulse = setpoint[1];
    }
    return pulse;
}

// FIXME - the complexity of this is huge, the magic numbers  don't help. - Joseph
uint16_t Helicopter::norm2pulse(double norm, std::array<uint16_t, 5> setpoint)
{
    uint16_t pulse = 1;
    if(setpoint[4] > setpoint[0])
    {
        if(norm <= 0.25 && norm >= 0)
            pulse = (norm/0.25) * (setpoint[1] - setpoint[0]) + setpoint[0];
        else if(norm <= 0.50 && norm > 0.25)
            pulse = ((norm - 0.25)/0.25) * (setpoint[2] - setpoint[1]) + setpoint[1];
        else if(norm <= 0.75 && norm > 0.50)
            pulse = ((norm - 0.50)/0.25) * (setpoint[3] - setpoint[2]) + setpoint[2];
        else if(norm <= 1.0 && norm > 0.75)
            pulse = ((norm - 0.75)/0.25) * (setpoint[4] - setpoint[3]) + setpoint[3];
    }
    else if(setpoint[4] < setpoint[0])
    {
        if(norm <= 0.25 && norm >= 0)
            pulse = setpoint[0] - (norm/0.25) * (setpoint[0] - setpoint[1]);
        else if(norm <= 0.50 && norm > 0.25)
            pulse = setpoint[1] - ((norm - 0.25)/0.25) * (setpoint[1] - setpoint[2]);
        else if(norm <= 0.75 && norm > 0.50)
            pulse = setpoint[2] - ((norm - 0.50)/0.25) * (setpoint[2] - setpoint[3]);
        else if(norm <= 1.0 && norm > 0.75)
            pulse = setpoint[3] - ((norm - 0.75)/0.25) * (setpoint[3] - setpoint[4]);
    }
    return pulse;
}

std::array<uint16_t, 6> Helicopter::setScaled(std::array<double, 6> norm)
{
    std::array<uint16_t, 6> pulse;

    pulse[AILERON] = setAileron(norm[0]);
    pulse[ELEVATOR] = setElevator(norm[1]);
    pulse[THROTTLE] = setThrottle(norm[2]);
    pulse[RUDDER] = setRudder(norm[3]);
    pulse[GYRO] = setGyro(norm[4]);
    pulse[PITCH] = setPitch(norm[5]);

    return pulse;
}

std::vector<uint16_t> Helicopter::setScaled(std::vector<double> norm)
{
    std::vector<uint16_t> pulse(6);

    pulse[AILERON] = setAileron(norm[0]);
    pulse[ELEVATOR] = setElevator(norm[1]);
    pulse[THROTTLE] = setThrottle(norm[2]);
    pulse[RUDDER] = setRudder(norm[3]);
    pulse[GYRO] = setGyro(norm[4]);
    pulse[PITCH] = setPitch(norm[5]);

    return pulse;
}

// FIXME these look like they might be able to be made in to templates - Joseph
uint16_t Helicopter::setAileron(double norm)
{
    uint16_t pulse = norm2pulse(norm, radio_cal_data->getAileron());
    out->setRaw(heli::CH1, pulse);
    return pulse;
}

uint16_t Helicopter::setElevator(double norm)
{
    uint16_t pulse = norm2pulse(norm, radio_cal_data->getElevator());
    out->setRaw(heli::CH2, pulse);
    return pulse;
}

uint16_t Helicopter::setThrottle(double norm)
{
    uint16_t pulse = norm2pulse(norm, radio_cal_data->getThrottle());
    out->setRaw(heli::CH3, pulse);
    return pulse;
}

uint16_t Helicopter::setRudder(double norm)
{
    uint16_t pulse = norm2pulse(norm, radio_cal_data->getRudder());
    out->setRaw(heli::CH4, pulse);
    return pulse;
}

uint16_t Helicopter::setGyro(double norm)
{
    uint16_t pulse = norm2pulse(norm, radio_cal_data->getGyro());
    out->setRaw(heli::CH5, pulse);
    return pulse;
}

uint16_t Helicopter::setPitch(double norm)
{
    uint16_t pulse = norm2pulse(norm, radio_cal_data->getPitch());
    out->setRaw(heli::CH6, pulse);
    return pulse;
}

double Helicopter::get_main_collective() const
{
    uint16_t pitch_servo = servo_switch::getInstance()->getRaw(heli::CH6);

    const uint16_t pitch_neutral = 1880;

    if (pitch_servo < pitch_neutral)
        return -0.017113*pitch_servo + 31.97;
    else
        return -0.0082034*pitch_servo + 15.349;
}
