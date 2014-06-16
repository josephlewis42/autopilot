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
	bool hasNewDistance();

private:
	/// pointer to the instance of Alitimiter
	static MdlAltimeter* _instance;
	/// serialize access to _instance
	static std::mutex _instance_lock;

	int _serialFd;
	bool new_distance;

};

#endif /* MDLALTIMETER_H_ */
