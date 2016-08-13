//
//  UDPServer.c
//  SerialPortSample
//
//  Created by Juhani Vehvilainen on 10/06/16.
//
//

#include <stdio.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include "Datagram.h"
#include <netdb.h>

#define BUFLEN 512
#define NPACK 10
#define PORT_IN 49000
#define PORT_OUT 40011

int socketIn = -1, socketOut = -1;

void debugnote(char *msg)
{
    printf("// DEBUG : %s\n", msg);
    fflush(stdout);
}

int udpServerInit(void)
{
    if ((socketIn = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
        return 1;
    
    int flags = fcntl(socketIn, F_GETFL, 0);
    
    if(flags > -1)
        fcntl(socketIn, F_SETFL, flags | O_NONBLOCK);
    
    const char* hostname=0; /* wildcard */

    struct addrinfo hints;
    memset(&hints,0,sizeof(hints));
    hints.ai_family=AF_UNSPEC;
    hints.ai_socktype=SOCK_DGRAM;
    hints.ai_protocol=0;
    hints.ai_flags=AI_PASSIVE|AI_ADDRCONFIG;
    struct addrinfo* res=0;
    int err=getaddrinfo(hostname,"40010",&hints,&res);
    if (err!=0) {
        debugnote(strerror(errno));
        return 10;
    }
    
    if (bind(socketIn, res->ai_addr, res->ai_addrlen)==-1) {
        debugnote(strerror(errno));
        return 2;
    }
    
    if ((socketOut = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
        return 1;
    
    flags = fcntl(socketOut, F_GETFL, 0);
    
    if(flags > -1)
        fcntl(socketOut, F_SETFL, flags | O_NONBLOCK);

    return 0;
}

ssize_t udpClient(const char *buf, ssize_t size)
{
    struct sockaddr_in si_other;
    
    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT_OUT);
    if (inet_aton("127.0.0.1", &si_other.sin_addr)==0) {
        fprintf(stderr, "inet_aton() failed\n");
        return 0;
    }

    return sendto(socketOut, buf, size, 0, (const struct sockaddr*) &si_other, sizeof(si_other));
}

void udpServerCleanup(void)
{
    if(socketIn > -1)
        close(socketIn);
    if(socketOut > -1)
        close(socketOut);
}

int udpServerInput(bool ready)
{
    struct SimLinkSensor buf;
    long packetSize = 0;
    
    packetSize = read(socketIn, &buf, sizeof(buf));

    if(ready && packetSize == sizeof(buf)) {
        datagramTxStart(DG_SIMLINK);
        datagramTxOut((const uint8_t*) &buf, (int) sizeof(buf));
        datagramTxEnd();
        
        return 1;
    }
    
    return 0;
}


