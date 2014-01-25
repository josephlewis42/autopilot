/*
 * qnx2linux.h - a set of conversion libraries for some of the QNX sources.
 *
 *  Created on: Nov 15, 2013
 *      Author: Joseph Lewis <joehms22@gmail.com>
 */

#ifndef QNX2LINUX_H_
#define QNX2LINUX_H_

/**
 * @brief Provides analogous system calls as some of those that QNX supplies.
 */
namespace QNX2Linux
{
#ifndef __QNX__

	/**
	 * An alternate to the read() system call, that provides a timeout and amount
	 * of time to read.
	 *
	 * @param fd - the file to read from
	 * @param buf - the buffer to read in to
	 * @param n - the number of bytes possible to read
	 * @param min - the minimum number of bytes to read
	 * @param time - in tenths of a second to wait for data
	 * @param timeout - in tenths of a second to time out multiple reades
	 */
	int readcond(int fd, void * buf, int n, int min, int time, int timeout);

	/**
	 * Reads until min, on a blocking fd.
	 */
	int readUntilMin(int fd, void * buf, int n, int min);

#endif
}

#endif /* QNX2LINUX_H_ */
