/**

This program reads data from a serial port and forwards it to a TCP
server listening somewhere.

Portions of this code were adapted from beej's networking guide.

Copyright 2014 Joseph Lewis <joseph@josephlewis.net>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the following disclaimer
  in the documentation and/or other materials provided with the
  distribution.
* Neither the name of the  nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#include <algorithm>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>


bool terminalSettings(int fd, int baudrate, std::string parity)
{
	// Set up the terminal configuration for the given port.
	struct termios options;

	tcgetattr(fd, &options);                  // get the current port settings

	// Set the baud rate
	uint32_t optbaud = 0;
	switch(baudrate)
	{
	case 9600:
		optbaud = B9600;
		break;
	case 19200:
		optbaud = B19200;
		break;
	case 38400:
		optbaud = B38400;
		break;
	case 57600:
		optbaud = B57600;
		break;
	case 115200:
		optbaud = B115200;
		break;
	default:
		printf("The selected baud of %d is not standard.\n", baudrate);
		optbaud = baudrate;
	}


	// Set baudrates for input and output
	cfsetospeed(&options, optbaud);
	cfsetispeed(&options, optbaud);

	// parity
	std::transform(parity.begin(), parity.end(),parity.begin(), ::toupper);

	if(parity == std::string("8N1") || parity == std::string("7S1")) {	//No parity (8N1):
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS8;
	} else if(parity == std::string("7E1")) {	//Even parity (7E1):
		options.c_cflag |= PARENB;
		options.c_cflag &= ~PARODD;
		options.c_cflag &= ~CSTOPB;
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS7;
	} else if(parity == std::string("701")){
		// Odd parity (7O1):
		options.c_cflag |= PARENB;
		options.c_cflag |= PARODD;
		options.c_cflag &= ~CSTOPB;
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS7;

	} else if(parity == std::string("7M1")) {	//Mark parity
		options.c_cflag &= ~PARENB;
		options.c_cflag |= CSTOPB;
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS7;
	} else { //Space parity is setup the same as no parity (7S1):
		printf("The parity %s is not known, use one of [8N1, 7E1, 701, 7M1, 7S1]\n", parity.c_str());
		return false;
    }

	//set for non-canonical (raw processing, no echo, etc.)
	cfmakeraw(&options);

	options.c_cflag |= (CLOCAL | CREAD); // Enable the receiver and set local mode...

	// Clear terminal output flow control.
	if (tcsetattr(fd, TCSADRAIN, &options) != 0)
	{
		printf("could not set serial port attributes\n");
		return false;
	}

	if(tcflush(fd, TCIOFLUSH) == -1)
	{
		printf("could not purge the serial port\n");
		return false;
	}

	printf("serial set up success\n");

	return true;
}



// get sockaddr, IPv4 or IPv6:
void *get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }

    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}



int main(int argc, char* argv[])
{
	if(argc < 6)
	{
		printf("Usage: %s /ser/port baudrate parity remote_host remote_port\n", argv[0]);
		printf("\t ex. /dev/ttyS0 9600 8N1 localhost 8000\n");
	
		return 1;
	}
	
	char* serial_port = argv[1];
	int baudrate = atoi(argv[2]);
	char* parity = argv[3];
	char* remote_host = argv[4];
	char* remote_port = argv[5];
	
	
	////////////////////////////////////////////////////////////////////
	// Setup Serial
	////////////////////////////////////////////////////////////////////
	
	
	int ser_fd = open(serial_port, O_RDWR | O_NOCTTY | O_NDELAY); // nonblocking is important here
	
	if(ser_fd == -1 || ! terminalSettings(ser_fd, baudrate, std::string(parity)) )
	{
		printf("Error setting terminal settings\n");
		return 1;
	}
	
	////////////////////////////////////////////////////////////////////
	// Setup TCP
	////////////////////////////////////////////////////////////////////
	
    int sockfd, numbytes;  
    struct addrinfo hints, *servinfo, *p;
    int rv;
    char s[INET6_ADDRSTRLEN];

 
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    if ((rv = getaddrinfo(remote_host, remote_port, &hints, &servinfo)) != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return 1;
    }

    // loop through all the results and connect to the first we can
    for(p = servinfo; p != NULL; p = p->ai_next) {
        if ((sockfd = socket(p->ai_family, p->ai_socktype,
                p->ai_protocol)) == -1) {
            perror("client: socket");
            continue;
        }

        if (connect(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
            close(sockfd);
            perror("client: connect");
            continue;
        }

        break;
    }

    if (p == NULL) {
        fprintf(stderr, "client: failed to connect\n");
        return 2;
    }

    inet_ntop(p->ai_family, get_in_addr((struct sockaddr *)p->ai_addr),
            s, sizeof s);
    printf("client: connecting to %s\n", s);

    freeaddrinfo(servinfo); // all done with this structure
	fcntl(sockfd, F_SETFL, O_NONBLOCK);  // set to non-blocking

	////////////////////////////////////////////////////////////////////
	// Main software loop
	////////////////////////////////////////////////////////////////////
	while(true)
	{
		char buffer[1024];

		int serin = read(ser_fd, buffer, 1024);
		if(serin > 0)
		{
			if(send(sockfd, buffer, serin, 0) == -1)
			{
				break; // problem sending
			}
		}

		int tcpin = recv(sockfd, buffer, 1024, 0);
		if(tcpin > 0)
		{
			if(write(ser_fd, buffer, tcpin) == -1)
			{
				break; // problem writing
			}
		}
	}

    close(sockfd);
    close(ser_fd);

    return 0;
}
