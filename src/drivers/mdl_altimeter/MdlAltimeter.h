/*
 * MdlAltimeter.h
 *
 *  Created on: May 16, 2014
 *      Author: Andrew Hannum
 */

#ifndef MDLALTIMETER_H_
#define MDLALTIMETER_H_

#include "Driver.h"

class MdlAltimeter: public Driver
{
public:
    MdlAltimeter();
    virtual ~MdlAltimeter();
    static MdlAltimeter* getInstance();
    float distance;
    void mainLoop();
    virtual void sendMavlinkMsg(std::vector<mavlink_message_t>& msgs, int uasId, int sendRateHz, int msgNumber) override;
    virtual void writeToSystemState() override;
private:
    static MdlAltimeter* _instance; /// pointer to the instance of Alitimiter
    static std::mutex _instance_lock; /// serialize access to _instance
    int _serialFd;
    bool has_new_distance;

};

#endif /* MDLALTIMETER_H_ */
