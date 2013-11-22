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

int QNX2Linux::readcond(int fd, void * buf, int n, int min, int time, int timeout) {
	struct termios orig;
	struct termios termios;

	tcgetattr(fd, &termios);
	orig = termios;
	termios.c_lflag &= ~ICANON; /* Set non-canonical mode */
	termios.c_cc[VTIME] = time;
	termios.c_cc[VMIN] = min;
	tcsetattr(fd, TCSANOW, &termios);

	int bytesRead = read(fd, buf, n);

	tcsetattr(fd, TCSANOW, &orig); // return to the original state.

	return bytesRead;
}
#endif
