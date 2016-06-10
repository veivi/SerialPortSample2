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

#define BUFLEN 512
#define NPACK 10
#define PORT 40010

void diep(char *s)
{
    perror(s);
    exit(1);
}

int socketIn = -1, socketOut = -1;

int udpServerInit(void)
{
    struct sockaddr_in si_me;
    
    if ((socketIn = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
        return 1;
    
    memset((char *) &si_me, 0, sizeof(si_me));
    
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(PORT);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);
    
    if (bind(socketIn, (struct sockaddr*) &si_me, sizeof(si_me))==-1)
        return 2;

    return 0;
}

void udpServerCleanup(void)
{
    if(socketIn > -1)
        close(socketIn);
    if(socketOut > -1)
        close(socketOut);
}

int udpServerInput(void)
{
    struct sockaddr_in si_other;
    int slen=sizeof(si_other);
    char buf[BUFLEN];
    
    int packetSize = 0;
    
    if ((packetSize = recvfrom(socketIn, buf, BUFLEN, 0, &si_other, &slen)) < 0)
        return 1;
    
    printf("Received packet from %s:%d\nSize: %d\n\n",
            inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port), packetSize);

    return 0;
}


