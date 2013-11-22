/*
 * qnx2linux.h - a set of conversion libraries for some of the QNX sources.
 *
 *  Created on: Nov 15, 2013
 *      Author: Joseph Lewis <joehms22@gmail.com>
 */

#ifndef QNX2LINUX_H_
#define QNX2LINUX_H_


namespace QNX2Linux
{
#ifndef __QNX__

	int readcond(int fd, void * buf, int n, int min, int time, int timeout);
#endif
}

#endif /* QNX2LINUX_H_ */
