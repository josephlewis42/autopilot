/*
 * TCPSerial.h - allows the forwarding of a serial port to the Autopilot or
 * back over TCP
 *
 *  Created on: May 23, 2014
 *      Author: Joseph Lewis <josephlewis42@gmail.com>
 */

#ifndef TCPSERIAL_H_
#define TCPSERIAL_H_

#include "Driver.h"
#include <atomic>


class TCPSerial: public Driver
{
public:
    TCPSerial();
    virtual ~TCPSerial();

private:
    void tcpToSer();
    void serToTcp();
    static void tcpListen(TCPSerial*);

    std::atomic_int tcp_fd;
    std::atomic_int ser_fd;
};

#endif /* TCPSERIAL_H_ */
