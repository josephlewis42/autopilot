/*
 * MdlAltimiter.h
 *
 *  Created on: May 16, 2014
 *      Author: joseph
 */

#ifndef MDLALTIMITER_H_
#define MDLALTIMITER_H_

#include "Driver.h"

class MdlAltimiter: public Driver {
public:
	MdlAltimiter();
	virtual ~MdlAltimiter();

	void mainLoop();
private:
	int _serialFd;
};

#endif /* MDLALTIMITER_H_ */
