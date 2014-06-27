/*
 * TCPSerial.cc - allows the forwarding of a serial port to the Autopilot or
 * back over TCP
 *
 *  Created on: May 23, 2014
 *      Author: Joseph Lewis <josephlewis42@gmail.com>
 */

#include "TCPSerial.h"

#include <thread>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <signal.h>

#define BUFFERSIZE 4096


// get sockaddr, IPv4 or IPv6:
void *get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET)
    {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }

    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

void sigchld_handler(int s)
{
    while(waitpid(-1, NULL, WNOHANG) > 0);
}



TCPSerial::TCPSerial()
    :Driver("TCP Serial","tcpserial")
{
    if(terminateRequested())
    {
        return;
    }

    if(! isEnabled())
    {
        return;
    }

    // init serial port
    configDescribe("serial_path",
                   "file path",
                   "File path for serial port.");
    std::string port = configGets("serial_path", "/dev/ttyS0");
    ser_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY); // nonblocking is important here
    namedTerminalSettings("serial_settings", ser_fd, 9600, "8N1", false, true);

    if(ser_fd == -1)
    {
        initFailed("could not open serial port");
    }


    // init TCP port
    configDescribe("TCP_Port",
                   "port number",
                   "Port number for TCP port.");
    std::string PORT = configGets("TCP_Port", "5000");

    configDescribe("TCP_backlog",
                   "0 - 100",
                   "TCP backlog size.");
    int BACKLOG = configGeti("TCP_backlog", 10);
    debug() << "Trying to start tcp on port " << PORT;
    struct addrinfo hints, *servinfo, *p;
    struct sigaction sa;
    int yes=1;
    int rv;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE; // use my IP

    if ((rv = getaddrinfo(NULL, PORT.c_str(), &hints, &servinfo)) != 0)
    {
        warning() << "getaddrinfo: " << gai_strerror(rv);
        return;
    }



    // loop through all the results and bind to the first we can
    for(p = servinfo; p != NULL; p = p->ai_next)
    {
        if ((tcp_fd = socket(p->ai_family, p->ai_socktype,
                             p->ai_protocol)) == -1)
        {
            warning() << "server: socket";
            continue;
        }

        if (setsockopt(tcp_fd.load(), SOL_SOCKET, SO_REUSEADDR, &yes,
                       sizeof(int)) == -1)
        {
            warning() << "setsockopt";
            return;
        }

        if (bind(tcp_fd.load(), p->ai_addr, p->ai_addrlen) == -1)
        {
            close(tcp_fd.load());
            warning("server: bind");
            continue;
        }

        break;
    }

    if (p == NULL)
    {
        warning("server: failed to bind\n");
        return;
    }

    freeaddrinfo(servinfo); // all done with this structure

    if (listen(tcp_fd.load(), BACKLOG) == -1)
    {
        warning("listen");
        return;
    }

    sa.sa_handler = sigchld_handler; // reap all dead processes
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    if (sigaction(SIGCHLD, &sa, NULL) == -1)
    {
        warning("sigaction");
        return;
    }



    debug() << "starting tcp";
    new std::thread(std::bind(TCPSerial::tcpListen, this));
}

TCPSerial::~TCPSerial()
{

}




void TCPSerial::tcpListen(TCPSerial* instance)
{
    struct sockaddr_storage their_addr; // connector's address information
    char s[INET6_ADDRSTRLEN];

    instance->debug() << "server: waiting for connections on" << instance->tcp_fd;

    while(! instance->terminateRequested())    // main accept() loop
    {
        socklen_t sin_size = sizeof their_addr;
        int tcp_client_fd = accept(instance->tcp_fd, (struct sockaddr *)&their_addr, &sin_size);
        if (tcp_client_fd == -1)
        {
            instance->warning("accept");
            continue;
        }

        inet_ntop(their_addr.ss_family,	get_in_addr((struct sockaddr *)&their_addr), s, sizeof s);
        instance->debug() << "server: got connection from " << s;

        fcntl(tcp_client_fd, F_SETFL, O_NONBLOCK);  // set to non-blocking

        // loop this section until close
        while(! instance->terminateRequested())
        {
            char buffer[BUFFERSIZE];

            int serin = read(instance->ser_fd, buffer, BUFFERSIZE);
            if(serin > 0)
            {
                if(send(tcp_client_fd, buffer, serin, 0) == -1)
                {
                    instance->warning() << "Error sendings: " + errno;
                    break; // problem sending
                }
            }

            int tcpin = recv(tcp_client_fd, buffer, BUFFERSIZE, 0);
            if(tcpin > 0)
            {
                if(write(instance->ser_fd, buffer, tcpin) == -1)
                {
                    instance->warning() << "Error recvingr: " + errno;
                    break; // problem writing
                }
            }
        }

        close(tcp_client_fd);
        // end loop
    }
}
