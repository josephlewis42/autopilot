/*
 * MdlAltimiter.h
 *
 *  Created on: May 16, 2014
 *      Author: joseph
 */

#ifndef MDLALTIMITER_H_
#define MDLALTIMITER_H_

#include "Driver.h"
#include <boost/signals2.hpp>

class MdlAltimiter: public Driver {
public:
	MdlAltimiter();
	virtual ~MdlAltimiter();
	static MdlAltimiter* getInstance();
	float distance;
	void mainLoop();
	bool hasNewDistance();

private:
	/// pointer to the instance of Alitimiter
	static MdlAltimiter* _instance;
	/// serialize access to _instance
	static std::mutex _instance_lock;

	int _serialFd;
	bool new_distance;

};

#endif /* MDLALTIMITER_H_ */
