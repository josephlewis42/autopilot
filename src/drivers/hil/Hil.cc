/*
 * Copyright 2014 Joseph Lewis <joseph@josephlewis.net>
 *
 * This file is part of University of Denver Autopilot.
 * Dual licensed under the GPL v 3 and the Apache 2.0 License
 */

#include "Hil.h"
#include "mavlink_types.h"
#include <sys/time.h>
#include <time.h>
#include "mavlink.h"
#include "SystemState.h"

Hil::Hil()
:Plugin("Hardware in the Loop","hil", 2)
{
}

Hil::~Hil()
{

}

bool Hil::init()
{
    return true;
}

void Hil::loop()
{

}

void Hil::teardown()
{
}

void Hil::sendMavlinkMsg(std::vector<mavlink_message_t>& msgs, int uasId, int sendRateHz, int msgNumber)
{
    if(! isEnabled()) return;


    if(shouldSendMavlinkMessage(msgNumber, sendRateHz, 20)) // limit to 10 hz then burst all messages
    {
        std::lock_guard<std::mutex> lock(_messageQueueLock);
        for(mavlink_message_t& msg : _messageQueue)
        {
            debug() << "sending message with id: " << msg.msgid;
            msgs.push_back(msg);
        }

        _messageQueue.empty();
    }

    // HIL_CONTROLS

    //
};


bool Hil::recvMavlinkMsg(const mavlink_message_t& msg)
{
    if(! isEnabled()) return false;

    // check to see if is a HIL message
    switch(msg.msgid)
    {
        case MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW:
            /**
               <field type="uint64_t" name="time_usec">Timestamp (microseconds since UNIX epoch or microseconds since system boot)</field>
               <field type="uint16_t" name="chan1_raw">RC channel 1 value, in microseconds</field>
               <field type="uint16_t" name="chan2_raw">RC channel 2 value, in microseconds</field>
               <field type="uint16_t" name="chan3_raw">RC channel 3 value, in microseconds</field>
               <field type="uint16_t" name="chan4_raw">RC channel 4 value, in microseconds</field>
               <field type="uint16_t" name="chan5_raw">RC channel 5 value, in microseconds</field>
               <field type="uint16_t" name="chan6_raw">RC channel 6 value, in microseconds</field>
               <field type="uint16_t" name="chan7_raw">RC channel 7 value, in microseconds</field>
               <field type="uint16_t" name="chan8_raw">RC channel 8 value, in microseconds</field>
               <field type="uint16_t" name="chan9_raw">RC channel 9 value, in microseconds</field>
               <field type="uint16_t" name="chan10_raw">RC channel 10 value, in microseconds</field>
               <field type="uint16_t" name="chan11_raw">RC channel 11 value, in microseconds</field>
               <field type="uint16_t" name="chan12_raw">RC channel 12 value, in microseconds</field>
               <field type="uint8_t" name="rssi">Receive signal strength indicator, 0: 0%, 255: 100%</field>
               **/
            break;

        case MAVLINK_MSG_ID_HIL_SENSOR:
            // ignore the message because we don't use this information yet..
            debug() << "got sensor message";
            break;

        case MAVLINK_MSG_ID_HIL_GPS:
            // ignore the gps message because we get the info we need from hil state.
            debug() << "got gps message";
            break;

        case MAVLINK_MSG_ID_HIL_OPTICAL_FLOW:
            // ignore the optical flow message because we don't use it.
            debug() << "got optical flow message";
            break;

        case MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
            {
                debug() << "got new state message";

                mavlink_hil_state_quaternion_t pkt;
                mavlink_msg_hil_state_quaternion_decode(&msg, &pkt);

                double lat = ((double)pkt.lat) / 1E7f; // dd lat (wgs87)
                double lon = ((double)pkt.lon) / 1E7f; // long (wgs87)
                double alt = ((double)pkt.alt) / 1000; // alt in m

                GPSPosition gps(lat, lon, alt, 0);

                float* quat = pkt.attitude_quaternion; // in format w,x,y,z
                EulerAngles ea = EulerAngles::fromQuaternion(quat[0], quat[1], quat[2], quat[3]);


                auto ss = SystemState::getInstance();

                ss->rotation.set(ea,0);
                ss->position.set(gps,0);
                ss->rollSpeed_radPerS.set(pkt.rollspeed,0);
                ss->pitchSpeed_radPerS.set(pkt.pitchspeed,0);
                ss->yawSpeed_radPerS.set(pkt.yawspeed,0);

            /* REMAINING FIELDS:
             <field type="int16_t" name="vx">Ground X Speed (Latitude), expressed as m/s * 100</field>
             <field type="int16_t" name="vy">Ground Y Speed (Longitude), expressed as m/s * 100</field>
             <field type="int16_t" name="vz">Ground Z Speed (Altitude), expressed as m/s * 100</field>
             <field type="uint16_t" name="ind_airspeed">Indicated airspeed, expressed as m/s * 100</field>
             <field type="uint16_t" name="true_airspeed">True airspeed, expressed as m/s * 100</field>
             <field type="int16_t" name="xacc">X acceleration (mg)</field>
             <field type="int16_t" name="yacc">Y acceleration (mg)</field>
             <field type="int16_t" name="zacc">Z acceleration (mg)</field>
            */
            }
            break;

        // warning, this message is deprecated but is provided here because
        // it may still be used by some QGC protocols and we would not wish
        // to ignore those.
        case MAVLINK_MSG_ID_HIL_STATE:
            warning() << "Got deprecated message in HIL operation.  MAVLINK_MSG_ID_HIL_STATE";
            break;


        default: // if we haven't processed the message.
            return false;
    }

    return true;
};
