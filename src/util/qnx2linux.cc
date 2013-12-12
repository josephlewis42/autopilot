/*
 * qnx2linux.cpp
 *
 *  Created on: Nov 15, 2013
 *      Author: joseph
 */

#ifndef __QNX__

#include "qnx2linux.h"
#include <termios.h>
#include <unistd.h>
#include <boost/thread.hpp>
#include "Debug.h"

int QNX2Linux::readcond(int fd, void * buf, int n, int min, int time, int timeout) {
	struct termios orig;
	struct termios termios;

	tcgetattr(fd, &termios);
	orig = termios;
	termios.c_lflag &= ~ICANON; /* Set non-canonical mode */
	termios.c_cc[VTIME] = time;
	termios.c_cc[VMIN] = min;
	tcsetattr(fd, TCSANOW, &termios);

	char* pointer = (char*)buf; // convert to a pointer of bytes

	int totalBytesRead = 0;
	if(timeout == 0)
	{
		totalBytesRead = read(fd, pointer, n);
	}
	else
	{
		for(int i = 0; i < timeout; i++)
		{
			int bytesRead = read(fd, pointer, n - totalBytesRead);
			totalBytesRead += bytesRead;
			pointer += bytesRead;

			// if we've read enough, stop
			if(totalBytesRead == n)
			{
				break;
			}

			// QNX specification says this is 1/10th of a second.
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));

			debug() << "read: " << bytesRead << " of " << n << " ptr is: " << (unsigned int) pointer << "orig: " << (unsigned int) buf;

		}
	}

	tcsetattr(fd, TCSANOW, &orig); // return to the original state.
	return totalBytesRead;
}
#endif
