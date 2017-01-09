/* 
 * File:   sendUDP.c
 * Author: turnej04
 *
 * Created on 15 March 2016, 17:27
 * UDP transmit - Generates UDP traffic
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <arpa/inet.h>


int sendUDP(char txData[], char destIPAddress[], int destPort) {
    printf("sendUDP() arg supplied: %s\n",txData);
    //char txData[] = "String";
    int txDataSize = (strlen (txData))+1; //Get size of string plus null char
    printf("txDataSize: %d\n", txDataSize);

    //char buf[BUFLEN]; //tx buffer
    socklen_t slen; //Special typdef. Holds size of sockaddr_in struct
    //Request a new socket from the OS and get a reference to it
    int handle = socket(AF_INET, SOCK_DGRAM, 0);
    if (handle < 0) {
        perror("socket");
        exit(1);
    }

    //Create an ip address/port structure for the remote end
    //Use htons() to convert a decimal port no to a special binary type
    //Use inet_aton() to parse SRV_IP dotted decimal notation
    struct sockaddr_in servaddr;
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(destPort);
    if (inet_aton(destIPAddress, &servaddr.sin_addr.s_addr) == 0) {
        perror("inet_aton() failed");
        exit(1);
    }
    slen = sizeof (servaddr);
    //Now send the data
    if (sendto(handle, txData, txDataSize, 0, (struct sockaddr*) &servaddr, slen) == -1)
        perror("sendto()");

    close(handle);
    return 0;
}

