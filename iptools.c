/*
 * Useful IP functions
 */

#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <ifaddrs.h>

#include "camera_control.h"


int getLocalIPaddr(char interfaceList[][2][20], int maxEntries){
    //This function takes a pointer to a 3D char array of the form 
    //char interfaceList[maxEntries][2][20]; //Will store results of getifaddrs())
    //and will populate the array with the system name and address of the 
    //network interfaces that have an ip address.
    //The function returns the no. of active network interfaces found.
    //maxEntries is the no. of maximum network cards info that can be stored
    
    //Interface 0 will always be the loopback, so you'd always expect this
    //function to return a value of 1 or greater
    
    struct ifaddrs *ifap, *ifa;
    struct sockaddr_in *sa;
    char *addr;
    char interfaceNo=0;
    
    getifaddrs (&ifap);
    for (ifa = ifap; ifa; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr->sa_family==AF_INET) {
            sa = (struct sockaddr_in *) ifa->ifa_addr;
            addr = inet_ntoa(sa->sin_addr);
            strcpy(interfaceList[interfaceNo][0],ifa->ifa_name); //Store name
            strcpy(interfaceList[interfaceNo][1],addr);          //Store address
            
            //printf("Interface: %s\tAddress: %s\n", ifa->ifa_name, addr);
        if(interfaceNo<maxEntries) interfaceNo++;    //increment index    
        }
    }
    freeifaddrs(ifap);
    /*
    printf("No of interfaces found:%d\n",(int)interfaceNo);
    for(int n=0;n<(int)interfaceNo;n++)
        printf("%d: %s %s\n",
                n,
                interfaceList[n][0],
                interfaceList[n][1]);
    */
    return (int)interfaceNo;    //Return no. of interfaces found
}

int sendUDP(char txData[], char destIPAddress[], int destPort) {
    //Sends a string (txData) via UDPto specifed addr:port
    //printf("sendUDP() arg supplied: %s\n",txData);
    //char txData[] = "String";
    int txDataSize = (strlen (txData))+1; //Get size of string plus null char
    //printf("txDataSize: %d\n", txDataSize);

    //char buf[BUFLEN]; //tx buffer
    socklen_t slen; //Special typdef. Holds size of sockaddr_in struct
    //Request a new socket from the OS and get a reference to it
    int handle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (handle < 0) {
        perror("iptools.c:sendUDP():socket");
        return -1;
    }

    //Create an ip address/port structure for the remote end
    //Use htons() to convert a decimal port no to a special binary type
    //Use inet_aton() to parse SRV_IP dotted decimal notation
    struct sockaddr_in servaddr;
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(destPort);
    if (inet_aton(destIPAddress, &servaddr.sin_addr.s_addr) == 0) {
        perror("iptools.c:sendUDP():inet_aton() failed");
        return -1;
    }
    slen = sizeof (servaddr);
    //Now send the data
    if (sendto(handle, txData, txDataSize, 0, (struct sockaddr*) &servaddr, slen) == -1)
        perror("iptools.c:sendUDP():sendto()");

    close(handle);
    return 0;
}

int sendBroadcastUDP(char txData[], int destPort) {
    //Sends a string (txData) via UDP to the broadcast address on port destPort
    //printf("sendUDP() arg supplied: %s\n",txData);
    //char txData[] = "String";
    int txDataSize = (strlen(txData)) + 1; //Get size of string plus null char
    //printf("txDataSize: %d\n", txDataSize);
    int broadcast = 1;
    //char buf[BUFLEN]; //tx buffer
    socklen_t slen; //Special typdef. Holds size of sockaddr_in struct
    //Request a new socket from the OS and get a reference to it
    int handle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (handle < 0) {
        perror("iptools.c:sendUDP():socket");
        return -1;
    }
    //Now set socket options to allow UDP broadcast
    if ((setsockopt(handle, SOL_SOCKET, SO_BROADCAST,
            &broadcast, sizeof broadcast)) == -1) {
        perror("setsockopt - SO_SOCKET ");
        exit(1);
    }

    //Create an ip address/port structure for the remote end
    //Use htons() to convert a decimal port no to a special binary type
    //Use inet_aton() to parse SRV_IP dotted decimal notation
    struct sockaddr_in servaddr;
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(destPort);
    servaddr.sin_addr.s_addr = INADDR_BROADCAST;
    /*
    if (inet_aton(destIPAddress, &servaddr.sin_addr.s_addr) == 0) {
        perror("iptools.c:sendUDP():inet_aton() failed");
        return -1;
    }
     */
    slen = sizeof (servaddr);
    //Now send the data
    if (sendto(handle, txData, txDataSize, 0, (struct sockaddr*) &servaddr, slen) == -1)
        perror("iptools.c:sendUDP():sendto()");

    close(handle);
    return 0;
}

int receiveUDP(char *rxBuffer, int rxBufferSize, int udpPort, char *srcIP, int *srcPort) {
    //Listens for UDP data on specified port
    //Fills supplied buffer (rxBuffer) with received data
    //The supplied pointers srcIP and srcPort will be populated with the source address
    //and port of the incoming message. *srcIP should be a char array of 20 characters
    //so that it can easily fit an IPv4 address in dot notation 000.000.000.000
    
    int handle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (handle < 0) {
        perror("iptools.c:receiveUDP():socket");
        return -1;
    }
    
    //printf("iptools.c:receiveUDP():supplied handle: %d\n",handle);
    struct sockaddr_in servaddr;
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(udpPort);
    servaddr.sin_addr.s_addr = INADDR_ANY;

    if (bind(handle, (struct sockaddr*) &servaddr, sizeof (servaddr)) < 0) {
        perror("iptools.c:receiveUDP():bind");
        return -1;
    }
    struct sockaddr_in cliaddr; //Address of remote end


    //char rxBuffer[1024];

    socklen_t len = sizeof (cliaddr);
    int received_bytes = recvfrom(handle, rxBuffer, rxBufferSize, 0, (struct sockaddr*) &cliaddr, &len);
    if (received_bytes > 0) {
        //    printf("Here is the message: %s\n", rxBuffer);
        //printf("From host: %s:%d\n", inet_ntoa(cliaddr.sin_addr), (int) ntohs(cliaddr.sin_port));
        strcpy(srcIP,inet_ntoa(cliaddr.sin_addr)); //Populate source address (so you know where packet came from)
        *srcPort=(int) ntohs(cliaddr.sin_port); //Retrieve  sourceport and assign to srcPort
        //int k=close(handle);
        //printf("\33[2K\riptools:receiveUDP(). close():%d,",k);
        close(handle);
        return received_bytes;
    } else{
        close(handle);
        return -1;
    }  
}