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
#include "Datagram.h"

#define BUFLEN 512
#define NPACK 10
#define PORT_IN 40010
#define PORT_OUT 40011

int socketIn = -1, socketOut = -1;

int udpServerInit(void)
{
    struct sockaddr_in si_me;
    
    if ((socketIn = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
        return 1;
    
    memset((char *) &si_me, 0, sizeof(si_me));
    
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(PORT_IN);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);
    
    if (bind(socketIn, (struct sockaddr*) &si_me, sizeof(si_me))==-1)
        return 2;

    unsigned long flags;
    fcntl(socketIn, F_GETFL, &flags);
    flags |= O_NONBLOCK;
    fcntl(socketIn, F_SETFL, flags);
    
    if ((socketOut = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
        return 1;
    
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
    struct sockaddr_in si_other;
    socklen_t sockLen = sizeof(si_other);
    char buf[BUFLEN];
    
    ssize_t packetSize = 0;
    
    if ((packetSize = recvfrom(socketIn, buf, BUFLEN, 0, (struct sockaddr*) &si_other, &sockLen)) != sizeof(struct SensorData))
        return 0;
    
    if(ready) {
        datagramTxStart(DG_SENSOR);
        datagramTxOut((const uint8_t*) buf, (int) packetSize);
        datagramTxEnd();
    }
    
//     static int count;
    
//     printf("Received packet num %d size %d\n", count++, packetSize);

    return 1;
}


