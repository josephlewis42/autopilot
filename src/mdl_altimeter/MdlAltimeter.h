/*
 * MdlAltimeter.h
 *
 *  Created on: May 16, 2014
 *      Author: joseph
 */

#ifndef MDLALTIMETER_H_
#define MDLALTIMETER_H_

#include "Driver.h"
#include <boost/signals2.hpp>

class MdlAltimeter: public Driver {
public:
	MdlAltimeter();
	virtual ~MdlAltimeter();
	static MdlAltimeter* getInstance();
	float distance;
	void mainLoop();
	virtual bool sendMavlinkMsg (mavlink_message_t* msg, int uasId, int sendRateHz, int msgNumber) override;

private:
	static MdlAltimeter* _instance; /// pointer to the instance of Alitimiter
	static std::mutex _instance_lock; /// serialize access to _instance
	int _serialFd;
	bool isEnabled;
	bool has_new_distance;

};

#endif /* MDLALTIMETER_H_ */
