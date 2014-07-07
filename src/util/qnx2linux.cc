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
#include <thread>
#include "Debug.h"
#include <stdint.h>
#include <errno.h>
#include <chrono>
#include <cstring>

Logger qnx2LinuxLogger("QNX2Linux");


int QNX2Linux::readcond(int fd, void * buf, int n, int min, int time, int timeout)
{

    if(n == 0)
    {
        return 0;
    }

    assert(min >= 0);
    assert(n >= 0);
    assert(fd >= 0);


    //debug() << "readcond " << n << std::hex << buf;
    struct termios orig;
    struct termios modified;

    tcgetattr(fd, &orig);
    modified = orig;
    modified.c_lflag &= ~ICANON; /* Set non-canonical mode */
    modified.c_cc[VTIME] = time;
    modified.c_cc[VMIN] = min;
    tcsetattr(fd, TCSANOW, &modified);

    uint8_t buffer[n];

    int totalBytesRead = 0;
    if(timeout == 0)
    {
        totalBytesRead = read(fd, buffer, n);
    }
    else
    {
        for(int i = 0; i < timeout; i++)
        {
            assert(totalBytesRead < n);

            int bytesRead = read(fd, &buffer[totalBytesRead], n - totalBytesRead);

            if(bytesRead < 0)
            {
                bytesRead = 0;
                if(errno == EAGAIN || errno == EWOULDBLOCK)
                    qnx2LinuxLogger.trace() << "Tried to do a blocking read from a nonblocking socket";
                if(errno == EBADF)
                    qnx2LinuxLogger.warning() << "Bad fd";
                if(errno == EFAULT)
                    qnx2LinuxLogger.warning() << "buf is outside your address space";
                if(errno == EINTR)
                    qnx2LinuxLogger.warning() << "Call interrupted before data read";
                if(errno == EINVAL)
                    qnx2LinuxLogger.warning() << "object is not suitable for reading";
                if(errno == EIO)
                    qnx2LinuxLogger.warning() << "I/O error";
                if(errno == EISDIR)
                    qnx2LinuxLogger.warning() << "fd is a directory";
                continue;
            }
            totalBytesRead += bytesRead;

            //debug() << "bytes read" <<  totalBytesRead;
            assert(totalBytesRead <= n);

            // if we've read enough, stop
            if(totalBytesRead == n)
            {
                break;
            }

            // QNX specification says this is 1/10th of a second.
            std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );

        }
    }

    memcpy(buf, buffer, n);

    tcsetattr(fd, TCSANOW, &orig); // return to the original state.
    return totalBytesRead;
}

int QNX2Linux::readUntilMin(int fd, void* buf, int n, int min)
{
    uint8_t buffer[n];
    int totalBytesRead = 0;

    while(totalBytesRead < min)
    {
        assert(totalBytesRead < n);

        int bytesRead = read(fd, &buffer[totalBytesRead], n - totalBytesRead);

        if(bytesRead < 0)
        {
            bytesRead = 0;
            if(errno == EAGAIN || errno == EWOULDBLOCK)
                qnx2LinuxLogger.trace() << "Tried to do a blocking read from a nonblocking socket";
            if(errno == EBADF)
                qnx2LinuxLogger.warning() << "Bad fd";
            if(errno == EFAULT)
                qnx2LinuxLogger.warning() << "buf is outside your address space";
            if(errno == EINTR)
                qnx2LinuxLogger.warning() << "Call interrupted before data read";
            if(errno == EINVAL)
                qnx2LinuxLogger.warning() << "object is not suitable for reading";
            if(errno == EIO)
                qnx2LinuxLogger.warning() << "I/O error";
            if(errno == EISDIR)
                qnx2LinuxLogger.warning() << "fd is a directory";
            continue;
        }

        totalBytesRead += bytesRead;

        assert(totalBytesRead <= n);

        // if we've read enough, stop
        if(totalBytesRead >= min)
        {
            break;
        }

        std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
    }

    memcpy(buf, buffer, totalBytesRead);
    return totalBytesRead;
}

#endif
