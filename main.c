/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 * 
 
 * 
 * comma delimiting?
 * if fields empty, sometimes parse gets things in the wrong order
 * 
 * Note: Auto discovery mode will only work if the devices are on the 
 * same broadcast segment (because it relies upon broadcast udp messages).
 * 
 * Q: Will it work behind NAT?
 * 
 * The 'remote ip address' global _struct field can be set two ways
 * 1)By UDPControlMonitorThreadV2()
 *      -This function updates the global field based on the actual 'from'
 *       ip address in the UDP packet. This mechanism is always active, for
 *       all modes (provided that the camera is not already connected).
 * 
 *      Hence: If connectedFlag==1, any noew remote ip address is ignored
 *      UNLESS: allowRemoteIPAddrToChange is set to '1'. This prevents/allows
 *      hijack by another controller.
 * 
 * 2)By parseStatusControMessage() (*NOTE* NOT UDPStatusMonitorThread)
 *      -This function updates the global field based on the 'localIP' field
 *      hidden inside the received status message. This mechanism is only active
 *      for CAMERA_MODE and AUTO_DISCOVER operating modes, and only if
 *      connectedFlag is currently 0.
 * 
 * 
 * 
 * Improvements:-
 * 1)Auto/manual connection logic sort of works, but it's ugly and hard to understand
 * and fails when you have two cameras and two controllers:-
 *      First camera/controller is fine, but the second camera can hijack a controller
 *      even if it's not set to auto discover mode (i.e specifying a remote ip address 
 *      makes no difference - the controller is still 'grabbed' by the second camera).
 * 2)Prob more elegant to use an enum for 'operation_mode' 
 * 3)strok in parseStatusString and parseControl string is reportedly not thread safe.
 * 
 * 
 *      
 * 
 * Changelog
 * ---------
 * CameraRemote10
 * --------------
 * -Upgraded to iptools 2.3
 * -'u' supplied as argument 1 will print the list of startup options
 * 
 * CameraRemote12
 * --------------
 * -Removed connectedFlag now part of global _status struct. 
 * Modifed getConnectedFlag() and setConnectedFlag accordingly
 * 
 * -Added remoteIdentity field to status message struct.
 * Modified setRemoteIdentity() and getRemoteIdentity()
 * 
 * -Added timestamp field to cmd message struct
 * 
 * Added getStatusSnapshot() function (takes a local copy of the global _status struct
 * 
 * -redrawScreen()
 * Now takes a local copy of global _status struct
 * 
 * 
 * How it works:-
 * --------------
 * --------------
 * Functions that can send messages:-
 * ----------------------------------
 * broadcastIdentity()
 *      -Sends a snapshot of _status via UDP broadcast on _status.statusPort
 *      -Modifies the 'type' field to HELLO
 *      -periodically invoked by broadcastIdentityThread()
 * 
 * sendStatusRefresh()
 *      -Sends a snapshot of _status to _status.remoteIP on _status.statusPort
 *      -Modifies the 'type' field to REFRESH
 *      -periodically invoked by heartbeatThread()
 *      -Also called by parseStatusMessage (in response, like an ACK)
 * 
 * GPIOMonitorThreadV2()
 *      -Polls the GPI inputs. On detecting a change, sends a control message to 
 *      _status.remoteIP via the designated sending interface (determined by getControlInterface)
 *      on port _status.controlPort
 *      -Sets message type field to GPI
 * 
 * controlSurfaceThread()
 *      -Monitors the analogue interfaces by polling
 *      -On a change detected, sends a control message to 
 *       _status.remoteIP via the designated sending interface (determined by getControlInterface)
 *      on port _status.controlPort
 *      -Sets message type field to SERVO
 * 
 * 
 * 
 * Functions that receive messages:-
 * ---------------------------------
 * UDPControlMonitorThreadV2()
 *      -Listens for UDP messages on _status.statusPort
 *      -Passes the received string directly to parseStatusMessage
 * 
 * UDPControlMonitorThreadV2()
 *      -Listens for UDP messages on _status.controlPort
 *      -Examines the source ip to see if it's relevant
 *            -If message from itself, ignores it
 *            -If NOT currently connected, will adopt the source ip address as the new _status.remoteIP
 *              (i.e it will automatically latch onto the far end - note these are not broadcast messages,
 *              they are explicitly addressed, so this is a safe course of action)
 *            -However, if global allowRemoteIPAddrToChange=0, the receiving end WONT automatically latch 
 *              onto this new remote address
 *            -Assuming message is 'relevant', passes the received data string to parseControlMessage() for processing
 * 
 * monitorMemDevThread()
 *      -Continually creates then monitors the system device /dev/cameracontrol
 *      -Passes the reveived string to parseControlMessage() for processing 
 * 
 * 
 * Functions that can process messages:-
 * -------------------------------------
 * parseStatusMessage()
 *      -Acts on received 'status' type messages (i.e those received on _status.statusPort
 *      -Current messages acted on are "REFRESH" and "HELLO"
 *          -REFRESH Messages
 *              -These messages are (or should be) explicitly addressed to a receiving end
 *              -Attempt to compensate for lost/missed message packets by matching up a periodic send of 
 *              _status from the sender to _send of the receiver. If there is a discrepency,
 *               the receiver is matched to the parameters described in the received REFRESH message
 *          -The 'status.localIP' of the *received* message is examined to see if it's relevant
 *              -If it's from itself, it's ignored
 *              -If it's from an unexpected source, it's ignored.
 *              -If it's from a 'new' source, the receivers' _status.remoteIP might allow itself to be 
 *              -modified provided that allowRemoteIPAddrToChange==1
 *              -This should prevent a receiver from being hijacked
 * 
 *          -HELLO Messages
 *              -These are (or shouldbe ) broadcast UDP messages with no receipient specified
 *              -They can be used by an AUTO_DISCOVER conteoller to latch automatically find an
 *               latch onto a CAMERA_MODE device that is *NOT* currently connected
 *              -Since a status message contains the both the type (i.e camera, controller, auto controller etc)
 *              of a remote device, it's identity, its connected status and its address it can decide whether to
 *              auto connect to it. 
 *              -A CAMERA can connect to aa CONTROLLER but a CONTROLLER shouldn't be able to connect to another
 *              CONTROLLER, for instance
 * 
 * Getters()/Setters()
 * ----------------
 * -Mostly these provide thread-safe access/modification to the fields in the global _status struct and other global variables.
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 */

/* 
 * File:   main.c
 * Author: turnej04
 *
 * Adds memset() to initCmdStruct() and initStatusStruct() in camera_control.h
 * CameraRemote08
 */
#define _POSIX_C_SOURCE 200809L     //This seems to be necessary. Otherwise compiler 
//complains that it can't find the signal functions
//(like sigemptyset() for instance)
#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <mcp3004.h>
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>        //Remember to add -lpthread to linker options
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/signal.h>

#include <math.h>           //Remember to add -lm to linker options

#include "camera_control.h"

/*
 * 
 */
//for A2D conversion
#define BASE 100
#define SPI_CHAN 0

//For UDP packet sending/receiving
#define BASE_PORT 8000
#define CONTROL_PORT BASE_PORT
#define STATUS_PORT (BASE_PORT+1)

//Are we in controller, camera, test or automatic mode
#define CAMERA_MODE     0
#define CONTROL_MODE    1
#define TEST_MODE       2
#define AUTO_DISCOVER   3       //Controller mode. Waits and listens for a remote camera to say 'hello'   

//Setting DEBUG_MODE will generate printf's
#define DEBUG_MODE      0

//#define SRV_IP "127.0.0.1"

#define TIMEOUT 10       //No. of seconds before lost contact is assumed
//The 'refresh' interval is automatically set
//at TIMEOUT/2

//These constants define how many inputs (up to maximum of 7) and how
//many outputs you want (up to a maximum of 6)
#define NO_OF_GPIs  7               //Max is 7
#define NO_OF_GPOs  6               //Max is 6

//Global variables
int operation_mode; //Camera (0), Controller (1), test(2) Auto Discover (3) mode?
int debug = 0; //If debug=1, you'll get lots of printf's....

int writeToPipe = 0; //If writeToPipe =1, useful things sent to /dev/camerastatus

//Create array that maps cameraRemote pins to WiringPi pins (Inputs))
const int gpiPinArray[7] = {3, 4, 21, 22, 23, 24, 25}; //eg GPI0 equiv to WiringPi pin 3
const char *gpiPinName[7] = {"GPI0", "GPI1", "GPI2", "GPI3", "GPI4", "GPI5", "GPI6"};

const int gpoPinArray[6] = {5, 6, 26, 27, 28, 29}; //eg GPO0 equiv to WiringPi pin 5
const char *gpoPinName[7] = {"GPO0", "GPO1", "GPO2", "GPO3", "GPO4", "GPO5", "GPO6"};

int rxUDControlHandle; //Need to keep track of the UDP port bindings so
int rxUDPStatusHandle; //That we can close them gracefully

status _status; //global status struct
char controlInterface[MSG_FIELD_LENGTH] = {0}; //Holds the name of the network interface
//used for camera control messages
//int control_port;
//int status_port;
//int connectedFlag; //'connected' means watchdogTimer<TIMEOUT && _status.remoteIP set

int allowRemoteIPAddrToChange = 1; //This determines whether a camera will automatically latch 
//onto a new controller if it loses connection
//pthread_mutex_t connectedFlagMutex = PTHREAD_MUTEX_INITIALIZER; //Create a mutex lock for status


pthread_mutex_t controlMutex = PTHREAD_MUTEX_INITIALIZER; //Create a mutex lock for servodControl
pthread_mutex_t statusMutex = PTHREAD_MUTEX_INITIALIZER; //Create a mutex lock for status

int watchdogTimer = 0; //Counts elapsed time since last message received 
pthread_mutex_t watchdogMutex = PTHREAD_MUTEX_INITIALIZER; //Create a mutex lock for the wdt

pthread_mutex_t displayMutex = PTHREAD_MUTEX_INITIALIZER; //Create a mutex lock for the display routine

char lastMessageText[BUFFER] = {0}; //Will store the type or cmd fields of the
//most recent message, for display purposes only
time_t lastMessageTimeStamp = 0; //For most recent message, for display purposes only
pthread_mutex_t lastMessageTextMutex = PTHREAD_MUTEX_INITIALIZER; //Mutex for last message text


void getControlInterface(char output[], unsigned int outputLength) {
    /**
     * //Populates the supplied array with the 
     * contents of the global controlInterface[] array.
     *
     * @param output
     * @param outputLength
     */
    strlcpy(output, controlInterface, outputLength);
}

void setControlInterface(char interface[]) {
    /*
     * Sets the global interface variable (eg, "wlan0", "eth0" etc
     * ****This is the only function that can modify the global controlInterface[] array****
     */
    strlcpy(controlInterface, interface, MSG_FIELD_LENGTH);
}

void setRemoteIdentity(char inputText[]) {
    /**
     * Sets the global _status.remoteIdentity char array.
     * Reflects the name of the device we're currently connected to
     * @param inputText
     */
    pthread_mutex_lock(&statusMutex);
    memset(_status.remoteIdentity, 0, MSG_FIELD_LENGTH);
    strlcpy(_status.remoteIdentity, inputText, MSG_FIELD_LENGTH);
    pthread_mutex_unlock(&statusMutex);
    
}

void getRemoteIdentity(char output[], int outputSize) {
    /**
     * Retrieves the contents of the global char array _status.remoteIdentity
     * This var holds the name of the device we're connected to
     * @param output
     * @param outputSize
     */
    pthread_mutex_lock(&statusMutex);
    strlcpy(output, _status.remoteIdentity, outputSize);
    pthread_mutex_unlock(&statusMutex);
}

void setLastMessageText(char inputText[]) {
    /*
     *This function sets the 'lastMessageText' and 'lastMessageTimestamp'
     * global variables in a threadsafe way. The timestamp variable
     * is set for 
     */
    pthread_mutex_lock(&lastMessageTextMutex);
    strlcpy(lastMessageText, inputText, BUFFER); //Copy inputText to lastMessageText array
    time(&lastMessageTimeStamp); //Capture current time, assign it to lastMessagetimeStamp
    pthread_mutex_unlock(&lastMessageTextMutex);
}

time_t getLastMessageText(char* output, int size) {
    /*
     * This function retrieves the contents of the global variable 'lastMessageText' and
     * copies it into the supplied buffer 'output' of length 'size'. If the supplied buffer
     * is too small, it will immediately return 0
     * 
     * Additionally, it returns a data type time_t which is the timestamp associated with the message
     * text stored in 'lastMessageText'
     */
    pthread_mutex_lock(&lastMessageTextMutex);
    if (sizeof (lastMessageText) < size) { //Check supplied buffer is large enough
        printf("getLastMessageText(): Supplied buffer too small");
        return 0;
    } else strlcpy(output, lastMessageText, BUFFER); //Copy text into output buffer
    time_t t = lastMessageTimeStamp;
    pthread_mutex_unlock(&lastMessageTextMutex);
    return t; //Return the timestamp
}

void getStatusSnapshot(status *outputStatus){
    /**
     * 
     * @param outputStatus
     * 
     * Takes a pointer to a 'status' struct and copies into it the contents of the global _status.
     * It uses mutex locks so is thread safe.
     * Provides the calling function with a local snapshot of the _status struct.
     */
    initStatusStruct(outputStatus);     //Initialise the struct to clear out prev contents
    pthread_mutex_lock(&statusMutex);
    memcpy(outputStatus,&_status,sizeof(status));//Copy global struct
    pthread_mutex_unlock(&statusMutex);
}

int getConnectedFlag() {
    /*
     * This blocking function polls _status.connectedFlag to see if cameraRemote currently thinks
     * it has an active connection to the far end.
     * It is thread safe because it locks  statusMutex 
     */

    pthread_mutex_lock(&statusMutex);
    int status = _status.connectedFlag;
    pthread_mutex_unlock(&statusMutex);
    return status;
}

void setConnectedFlag(int value) {
    /*
     * This function safely sets the global variable _status.connectedFlag.
     * It is thread safe as it locks/unlocks the statusMutex
     * Any supplied value greater than 0 is considered a '1'
     */

    pthread_mutex_lock(&statusMutex);
    if (value > 0) _status.connectedFlag = 1;
    else _status.connectedFlag = 0;
    pthread_mutex_unlock(&statusMutex);
    
    
}

void setWatchdogTimer(int value) {
    /*
     * Sets the global watchdog timer to the supplied value.
     * 
     * Can be used to force the watchdog timer thread over the limit threshold
     * set by TIMEOUT
     */
    pthread_mutex_lock(&watchdogMutex);
    watchdogTimer = value;
    pthread_mutex_unlock(&watchdogMutex);
}

void resetWatchdogTimer() {
    /*
     * This function clears the global watchdogTimer variable in
     * a threadsafe way
     */
    pthread_mutex_lock(&watchdogMutex);
    watchdogTimer = 0;
    pthread_mutex_unlock(&watchdogMutex);
}

int getWatchDogTimer() {
    /*
     * This function can retrieve the current watchdog timer value in a threadsafe way
     */
    pthread_mutex_lock(&watchdogMutex);
    int value = watchdogTimer;
    pthread_mutex_unlock(&watchdogMutex);
    return value;
}

void incrementWatchdogTimer() {
    /*
     * This function increments the current watchdog timer value in a threadsafe way
     */
    pthread_mutex_lock(&watchdogMutex);
    watchdogTimer++;
    pthread_mutex_unlock(&watchdogMutex);
}

int getUDPControlPort() {
    /*
     *  Returns the UDP control message port no from the global _status struct
     */
    int retValue = 0;
    pthread_mutex_lock(&statusMutex); //Lock _status mutex
    retValue = _status.controlPort;
    pthread_mutex_unlock(&statusMutex); //Unlock
    return retValue;
}

int getUDPStatusPort() {
    /*
     *  Returns the UDP status message port no from the global _status struct
     */
    int retValue = 0;
    pthread_mutex_lock(&statusMutex); //Lock _status mutex
    retValue = _status.statusPort;
    pthread_mutex_unlock(&statusMutex); //Unlock
    return retValue;
}

int getLocalIP(char* buf, int length) {
    /*
     * This function fills a pointer to a char array with the  the ip address
     * of the local controller interface as held within _status.localIP
     * 
     * This value is required so often (particularly by the message-sending
     * functions that it makes sense have a function that specifically retrieves 
     * the IP address in a thread-safe fashion
     * 
     * If the supplied buffer is not large enough, the function will return 1,
     * otherwise it will return 1.
     * 
     * Allow a buffer size of 20 chars or so aaa.bbb.ccc.ddd\0
     * 
     */
    pthread_mutex_lock(&statusMutex); //Lock _status mutex
    strlcpy(buf, _status.localIP, length); //Copy ip addr string from global _struct into supplied buffer
    pthread_mutex_unlock(&statusMutex); //Unlock
    if (sizeof (_status.localIP) < length) return 1; //Confirm supplied buf large enough
    else return -1;
}

int getRemoteIP(char* buf, int length) {
    /*
     * This function fills a pointer to a char array with the  the ip address
     * of the remote end based on the current value held within _status.remoteIP
     * 
     * This value is required so often (particularly by the message-sending
     * functions that it makes sense have a function that specifically retrieves 
     * the IP address in a thread-safe fashion
     * 
     * If the supplied buffer is not large enough, the function will return 1,
     * otherwise it will return 1.
     * 
     */

    pthread_mutex_lock(&statusMutex); //Lock _status mutex
    strlcpy(buf, _status.remoteIP, length); //Copy ip addr string from global _struct into supplied buffer
    //printf("getRemoteIP(): _status.remoteIP: %s, buf:%s\n",_status.remoteIP,buf);
    pthread_mutex_unlock(&statusMutex); //Unlock
    if (sizeof (_status.remoteIP) < length) return 1; //Confirm supplied buf large enough
    else return -1;
}

int setRemoteIP(char remoteIPAddr[]) {
    /*
     * Sets the global _status.remoteIP field 
     * 
     * remoteIPAddr expressed as x.x.x.x
     * 
     * returns 1 on success, or -1 if the supplied address is invalid
     */
    //Check supplied address is valid
    struct sockaddr_in addrToTest;
    addrToTest.sin_family = AF_INET;
    if (inet_aton(remoteIPAddr, &addrToTest.sin_addr.s_addr) == 0) { //Test ip address is properly formed]
        printf("setRemoteIP(): Invalid address supplied: %s\n", remoteIPAddr);
        return -1;
    } else {
        pthread_mutex_lock(&statusMutex); //Lock _status mutex
        strlcpy(_status.remoteIP, remoteIPAddr, MSG_FIELD_LENGTH); //Update global
        pthread_mutex_unlock(&statusMutex); //Unlock
        return 1;
    }
}

int isRemoteIPSet() {
    /*
     * Determines wheter the _status.remoteIP field has been previously set 
     * (i.e changed from it's startup default of 0.0.0.0)
     * 
     * This is used by the message sending functions to check whether they should
     * bother sending a message or not.
     */
    char remoteIP[20] = {0};
    getRemoteIP(remoteIP, 20);

    if (compareIPAddress(remoteIP, "0.0.0.0") == 1) {
        //Have we a known remote ip to send to?
        if (debug)printf("\33[2K\risRemoteIPSet() remoteIP==0.0.0.0\n");
        return 0; //_status.remoteIP NOT set so, return 0
    } else {
        if (debug)printf("\33[2K\risRemoteIPSet() remoteIP==%s\n", remoteIP);
        return 1;
    } //_status.remoteIP is set so, return 1
}

int setControlPort(unsigned int port) {
    /**
     * Sets the udp listening control port in global _status struct
     * @param port
     * @return 
     */
    if (port > 0) {
        pthread_mutex_lock(&statusMutex); //Lock _status mutex
        _status.controlPort = port; //Update global
        pthread_mutex_unlock(&statusMutex); //Unlock
        return 1;
    } else {
        printf("setControlPort(): invalid port specified: %u\n", port);
        return -1;
    }
}

int setStatusPort(unsigned int port) {
    /**
     * Sets the udp listening status message port in global _status struct
     * @param port
     * @return 
     */
    if (port > 0) {
        pthread_mutex_lock(&statusMutex); //Lock _status mutex
        _status.statusPort = port; //Update global
        pthread_mutex_unlock(&statusMutex); //Unlock
        return 1;
    } else {
        printf("setStatusPort(): invalid port specified: %d\n", port);
        return -1;
    }
}

int establishControlInterface() {
    /**
     * This function examines the installed network cards to see what's installed
     * If wlan1 is detected (and has an ip address), it will pick it. Next it will 
     * choose wlan0 or, if no wireless interfaces at all, eth0.
     * 
     * It will then set the controlInterface[] global (by way of 
     * setControllerInterface() )
     * 
     * Returns 1 for a valid address/interface found, or -1 if none found
     * @return 
     */

#define FIRST_CHOICE_INTERFACE "wlan1"
#define SECOND_CHOICE_INTERFACE "wlan0"
#define THIRD_CHOICE_INTERFACE "eth0"

    char interfaceList[20][2][20];
    int noOfInterfaces = getLocalIPaddr(interfaceList, 20); //Display local ethernet interfaces
    printf("No of interfaces found:%d\n", noOfInterfaces);
    int l;
    for (int l = 0; l < noOfInterfaces; l++)
        printf("%d: %s %s\n", l, interfaceList[l][0], interfaceList[l][1]);

    //Iterate through all the interface names to see if we have one matching the first choice
    for (l = 0; l < noOfInterfaces; l++) {
        if (strcmp(FIRST_CHOICE_INTERFACE, interfaceList[l][0]) == 0) {
            //If strcmp returns '0', match found
            printf("establishControlInterface():Interface %s installed\n", FIRST_CHOICE_INTERFACE);
            setControlInterface(FIRST_CHOICE_INTERFACE);

            //Update _status.localIP field 
            pthread_mutex_lock(&statusMutex);
            //Take copy of interface ip address, control port no and status port no
            strlcpy(_status.localIP, interfaceList[l][1], MSG_FIELD_LENGTH);
            pthread_mutex_unlock(&statusMutex);
            return 1;
        }
    }

    //The preferred interface) not found, so search for second choice instead
    //Iterate through all the interface names to see if we have one called 'wlan0'
    for (l = 0; l < noOfInterfaces; l++) {
        if (strcmp(SECOND_CHOICE_INTERFACE, interfaceList[l][0]) == 0) {
            //If strcmp returns '0', match found
            printf("establishControlInterface():Interface %s installed\n", SECOND_CHOICE_INTERFACE);
            setControlInterface(SECOND_CHOICE_INTERFACE);

            //Update _status.localIP field 
            pthread_mutex_lock(&statusMutex);
            //Take copy of interface ip address
            strlcpy(_status.localIP, interfaceList[l][1], MSG_FIELD_LENGTH);
            pthread_mutex_unlock(&statusMutex);
            return 1;
        }
    }
    //Second choice not found either (or doesn't have an address), check for third choice instead
    for (l = 0; l < noOfInterfaces; l++) {
        if (strcmp(THIRD_CHOICE_INTERFACE, interfaceList[l][0]) == 0) {
            //If strcmp returns '0', match found
            printf("establishControlInterface():Interface %s installed\n", THIRD_CHOICE_INTERFACE);
            setControlInterface(THIRD_CHOICE_INTERFACE);

            //Update _status.localIP field 
            pthread_mutex_lock(&statusMutex);
            //Take copy of interface ip address
            strlcpy(_status.localIP, interfaceList[l][1], MSG_FIELD_LENGTH);
            pthread_mutex_unlock(&statusMutex);
            return 1;
        }
    }
    //If it gets this far, no interfaces have addresses
    return -1;
}

void *UDPControlMonitorThreadV2() {
    /*
     *This function creates a socket and continually monitors it using the 
     *blocking function recvfrom().
     *BY creating the socket itself, only a single fd (file descriptor)
     *is ever created. Therefore you don't seem to run out of 
     *availabke sockets (like you would if you were repeatedly calling
     *my utility function receieveUDP() (in iptools.c) every time.
     * 
     * In other words. You can't trust close(handle) to actually close
     * the fd and you quickly run out.
     * 
     * NOTE: This function is monitoring incoming UDP on port 'control_port'
     * and then passing those messages to parseControlMessage()
     */
    char rxBuffer[1024]; //Will be populated with the incoming message
    char srcIPAddr[20]; //Will be populated with the incoming src IP addr
    int srcPort; //Will be popluated with the incoming [port no.]
    char localIPAddr[20] = {0}; //Hold the 'currently stored' local ip address
    char remoteIPAddr[20] = {0}; //Hold the 'currently stored' remote ip address
    int control_port = getUDPControlPort(); //Retrieve control port from global _status struct
    char interface[MSG_FIELD_LENGTH] = {0}; //char array to hold interface we wish to bind to 
    getControlInterface(interface, MSG_FIELD_LENGTH); //Get current network interface name to bind to

    //Create socket for listening
    int handle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (handle < 0) {
        perror("UDPControlMonitorThread2():socket");
        close(handle);
        return;
    }
    /*
        //Optionally specify the interface to which to bind to
        if (setsockopt(handle, SOL_SOCKET, SO_BINDTODEVICE, interface, strlen(interface)) == -1) {
            perror("setsockopt - SOL_SOCKET, SO_BINDTODEVICE ");
            printf("UDPControlMonitorThread2(): Can't bind to device: %s\n", interface);
            close(handle);
            return;
        }
     */
    rxUDControlHandle = handle; //Store file descriptor in global
    struct sockaddr_in servaddr;
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(control_port);
    servaddr.sin_addr.s_addr = INADDR_ANY;

    if (bind(handle, (struct sockaddr*) &servaddr, sizeof (servaddr)) < 0) {
        perror("UDPControlMonitorThread2():bind");
        close(handle);
        return;
    }
    struct sockaddr_in cliaddr; //Address of remote end
    socklen_t len = sizeof (cliaddr);
    while (1) {
        //Sit in a loop waiting for data
        int received_bytes = recvfrom(handle, rxBuffer, 1024, 0, (struct sockaddr*) &cliaddr, &len);
        if (received_bytes > 0) {
            //Step 1) Determine where the message came from and whether the source IP addr is relevant
            strlcpy(srcIPAddr, inet_ntoa(cliaddr.sin_addr), 20); //Populate source address (so you know where packet came from)
            srcPort = (int) ntohs(cliaddr.sin_port); //Retrieve  sourceport and assign to srcPort

            getLocalIP(localIPAddr, 20); //Get 'local ip address'
            getRemoteIP(remoteIPAddr, 20); //Get last known remote ip address

            //Now compare src address of received message with the local address.
            //If they're different, latch onto the new address, or if unchained, do nothing
            if (compareIPAddress(srcIPAddr, localIPAddr) == 1) { //Are they the same   
                printf("*UDPControlMonitorThreadV2(): Control message received from self. Ignoring!\n");

            } else {
                //Or else, incoming control message is from a remote host
                if (compareIPAddress(srcIPAddr, remoteIPAddr) == 0) {
                    //Has the src address changed?
                    //This is the anti-hijack bit....
                    //Only allow the _status.remoteIP field to change if
                    //      a) We're not currently connected
                    //      b) The safety lock 'allowRemoteIPAddrToChange' has not been set

                    //If it has, modify the _status.remoteIP field accordingly with the new address
                    //BUT only if allowRemoteIPAddrToChange is set and we're not currently connected.

                    if ((getConnectedFlag() == 0) && (allowRemoteIPAddrToChange == 1)) {
                        printf("*UDPControlMonitorThreadV2(): Control message received from %s, changing _status.remoteIP from %s to %s\n\n",
                                srcIPAddr, remoteIPAddr, srcIPAddr);
                        setRemoteIP(srcIPAddr); //Update _status.remoteIP field based on last packet received 
                    } else {

                        printf("*UDPControlMonitorThreadV2(): Already connected AND/OR allowRemoteIPAddrToChange = 0, ignoring connection request.\n");
                    }
                } else {
                    //Control message is from the already known remote host
                    printf("*UDPControlMonitorThreadV2(): Control message received from %s. Current _status.remoteIP %s. No change.\n\n",
                            srcIPAddr, remoteIPAddr);
                    //Step 2) Now actually decode the command
                    parseControlMessage(rxBuffer);

                    //Now redraw the screen
                    redrawScreen();

                }
            }
            /*
            pthread_mutex_lock(&statusMutex);
            strlcpy(_status.remoteIP, srcIPAddr, MSG_FIELD_LENGTH);
            pthread_mutex_unlock(&statusMutex);
             */
            //Optionally echo incoming message to output pipe
            if (writeToPipe) {
                char pipeBuff[1024];
                snprintf(pipeBuff, 1024, "RX: %s\n", rxBuffer); //Create nice formatted string
                writeToOutputPipe(pipeBuff); //optionally write too pipe  
            }
        }
    }
    //Will never get here but need to think about a graceful close?
    close(handle);
}

void parseControlMessage(char *controlString) {
    //This function decodes and acts on control strings
    command rxCommand;
    int success = 0; //Stores the return value from control functions     
    parseCmdString(&rxCommand, controlString); //Parse control string (populate rxCommand)

    if (!strcmp(rxCommand.cmd, "SERVO")) {
        if (debug)printf("\33[2K\rcmd: %s, Arg1: %d, Arg2:%d\n", rxCommand.cmd, rxCommand.arg1, rxCommand.arg2);
        //Control messages might come from a variety of sources so make sure the 
        //controlMutex is unlocked before we write to it
        pthread_mutex_lock(&controlMutex);
        success = servodControl(rxCommand.arg1, rxCommand.arg2); //Pass args to servControl
        pthread_mutex_unlock(&controlMutex); //Now unlock

        //Now update global status
        pthread_mutex_lock(&statusMutex);
        //if (success)_status.servo0 = rxCommand.arg2;

        if (success)
            switch (rxCommand.arg1) {
                case 0:
                    _status.servo0 = rxCommand.arg2;
                    break;
                case 1:
                    _status.servo1 = rxCommand.arg2;
                    break;
                case 2:
                    _status.servo2 = rxCommand.arg2;
                    break;
                case 3:
                    _status.servo3 = rxCommand.arg2;
                    break;
            }

        pthread_mutex_unlock(&statusMutex);

        //Message received so reset watchdog timer
        resetWatchdogTimer();

        //Update lastMessageText display field
        setLastMessageText("SERVO");
        blinkGPO4(1); //Blink GPO5 LED to signify message received

    } else if (!strcmp(rxCommand.cmd, "HELLO")) {
        //This is a hello or 'ACK' type message from the camera

        //If an ack is received, we can reset the watchdog timer
        resetWatchdogTimer();
        if (debug)printf("Received HELLO.................\n");

        //Update lastMessageText display field
        setLastMessageText("HELLO");
        blinkGPO4(1); //Blink GPO5 LED to signify message received

    } else if (!strcmp(rxCommand.cmd, "GPI")) {
        if (debug) printf("\33[2K\rparseControlMessage():GPI message:%d,%d,%d,%s\n",
                rxCommand.arg1, rxCommand.arg2,
                rxCommand.arg3, rxCommand.text);
        int gpoPinChanged = rxCommand.arg1;
        if ((gpoPinChanged < 0) || (gpoPinChanged > 5)) {
            printf("parseControlMessage():Invalid GPO pin no specified: %d\n", gpoPinChanged);
            return;
        }
        int newGpoPinLevel = rxCommand.arg2;
        //Now set pin level accordingly
        //BUT MASK GPO5 because we want to use it as a status indicator
        if (gpoPinChanged < 5) { //gpoPinChanged 0-4 will get through, not 5      
            if (newGpoPinLevel > 0)digitalWrite(gpoPinArray[gpoPinChanged], HIGH);
            else digitalWrite(gpoPinArray[gpoPinChanged], LOW);
        }
        //Finally, update global status variable (in a threadsafe way))
        pthread_mutex_lock(&statusMutex);

        switch (gpoPinChanged) {
            case 0:
                _status.gpo0 = newGpoPinLevel;
                break;
            case 1:
                _status.gpo1 = newGpoPinLevel;
                break;
            case 2:
                _status.gpo2 = newGpoPinLevel;
                break;
            case 3:
                _status.gpo3 = newGpoPinLevel;
                break;
            case 4:
                _status.gpo4 = newGpoPinLevel;
                break;
            case 5:
                _status.gpo5 = newGpoPinLevel;
                break;
        }
        pthread_mutex_unlock(&statusMutex);

        //Message received so reset watchdog timer
        resetWatchdogTimer();

        //Update lastMessageText display field
        setLastMessageText("GPI");
        blinkGPO4(1); //Blink GPO5 LED to signify message received
    }

}

/*
void *UDPStatusMonitorThread() {
    //This thread waits for incoming messages on the nominated UDP status message port
    char rxBuffer[1024];
    char srcIPAddr[20]; //Will be populated with the incoming src IP addr
    int srcPort; //Will be popluated with the incoming [port no.]
    int status_port = getUDPStatusPort(); //Get local copy of status message UDP listening port
    while (1) {
        //Thread will sit in this loop indefinitely
        int n = receiveUDP(rxBuffer, 1024, status_port, srcIPAddr, 20,&srcPort); //Wait for incoming data
        if (n != -1) {
            //Optionally echo incoming message to output pipe
            if (writeToPipe) {
                char pipeBuff[1024];
                snprintf(pipeBuff, 1024,"RX: %s\n", rxBuffer); //Create nice formatted string
                writeToOutputPipe(pipeBuff); //optionally write too pipe  
            }
            if (debug)printf("UDPStatusMonitorThread(): Incoming status message, port:%d:%s\n", status_port, rxBuffer);
            parseStatusMessage(rxBuffer);
        }
    }
}
 */
void *UDPStatusMonitorThreadV2() {
    /*
     *This function creates a socket and continually monitors it using the 
     *blocking function recvfrom().
     *BY creating the socket itself, only a single fd (file descriptor)
     *is ever created. Therefore you don't seem to run out of 
     *availabke sockets (like you would if you were repeatedly calling
     *my utility function receieveUDP() (in iptools.c) every time.
     * 
     * In other words. You can't trust close(handle) to actually close
     * the fd and you quickly run out.
     * 
     * NOTE: This function is monitoring incoming UDP on port 'status_port'
     * and then passing those messages to parseStatusMessage()
     */
    char rxBuffer[1024]; //Will be populated with the incoming message
    char srcIPAddr[20]; //Will be populated with the incoming src IP addr
    int srcPort; //Will be popluated with the incoming [port no.]

    int status_port = getUDPStatusPort(); //Get local copy of status message UDP listening port

    char interface[MSG_FIELD_LENGTH] = {0}; //char array to hold interface we wish to bind to 
    getControlInterface(interface, MSG_FIELD_LENGTH); //Get current network interface name to bind to

    //Create socket for listening
    int handle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (handle < 0) {
        perror("UDPStatusMonitorThreadV2():socket");
        close(handle);
        return;
    }
    /*
        //Specify the interface to which to bind to
        if (setsockopt(handle, SOL_SOCKET, SO_BINDTODEVICE, interface, strlen(interface)) == -1) {
            perror("setsockopt - SOL_SOCKET, SO_BINDTODEVICE ");
            printf("UDPStatusMonitorThreadV2(): Can't bind to device: %s\n", interface);
            close(handle);
            return;
        }
     */
    rxUDPStatusHandle = handle; //Store file descriptor in global

    struct sockaddr_in servaddr;
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(status_port);
    servaddr.sin_addr.s_addr = INADDR_ANY;

    if (bind(handle, (struct sockaddr*) &servaddr, sizeof (servaddr)) < 0) {
        perror("UDPStatusMonitorThreadV2():bind");
        close(handle);
        return;
    }
    struct sockaddr_in cliaddr; //Address of remote end
    socklen_t len = sizeof (cliaddr);
    while (1) {
        //Sit in a loop waiting for data
        int received_bytes = recvfrom(handle, rxBuffer, 1024, 0, (struct sockaddr*) &cliaddr, &len);
        if (received_bytes > 0) {
            strlcpy(srcIPAddr, inet_ntoa(cliaddr.sin_addr), 20); //Populate source address (so you know where packet came from)
            srcPort = (int) ntohs(cliaddr.sin_port); //Retrieve  sourceport and assign to srcPort
            if (debug)printf("UDPStatusMonitorThreadV2(): Incoming status message %s from %s:%s:%d\n",
                    rxBuffer, interface, srcIPAddr, status_port);



            //Optionally echo incoming message to output pipe
            if (writeToPipe) {
                char pipeBuff[1024];
                snprintf(pipeBuff, 1024, "RX: %s\n", rxBuffer); //Create nice formatted string
                writeToOutputPipe(pipeBuff); //optionally write too pipe  
            }


            parseStatusMessage(rxBuffer);

            //Now redraw the screen
            redrawScreen();
        }
    }
    //Will never get here but need to think about a graceful close?
    close(handle);
}

void sendStatusRefresh() {
    /**
     * Sends a Status refresh message with the type REFRESH
     */

    status heartbeatStatusMessage; //Create carrier for heartbeat message 
    getStatusSnapshot(&heartbeatStatusMessage); //Tale local copy of global _status struct
    
    //Modify local status struct fields prior to sending
    strlcpy(heartbeatStatusMessage.type, "REFRESH", MSG_FIELD_LENGTH);
    heartbeatStatusMessage.timeStamp = time(NULL); //Assign current time;
    heartbeatStatusMessage.connectedFlag = getConnectedFlag();
    char txBuffer[BUFFER] = {0}; //create transmit buffer
    createStatusString(&heartbeatStatusMessage, txBuffer, BUFFER); //Create transmission string

    char interface[MSG_FIELD_LENGTH] = {0}; //Buffer to hold name of network i'face to send from
    getControlInterface(interface, MSG_FIELD_LENGTH); //Get interface to send message out on
    int status_port = getUDPStatusPort(); //Get local copy of status message UDP listening port

    char remoteIP[MSG_FIELD_LENGTH] = {0}; //Used to store local copy of _status.remoteIP field
    getRemoteIP(remoteIP, MSG_FIELD_LENGTH); //Get remote IP from _status global struct (in a threadsafe way)

    if(debug)printf("\33[2K\rSending REFRESH heartbeat: %s to: %s:%d via %s\n",
            txBuffer, remoteIP, status_port, interface);

    sendUDP(txBuffer, remoteIP, status_port, interface); //Transmit the string

    if (writeToPipe) {
        char pipeBuff[1024];
        snprintf(pipeBuff, 1024, "TX: %s\n", txBuffer); //Create nice formatted string
        writeToOutputPipe(pipeBuff); //optionally write too pipe  
    }

}

int compareIPAddress(char* addressA, char* addressB) {
    /**
     * Compares two ip addresses. Returns 1 if they're the same or 0 if different
     * @param addressA
     * @param address
     * @return 
     */

    unsigned int k = inet_network(addressA);
    unsigned int l = inet_network(addressB);
    if (k == l)return 1;
    else return 0;
}

int parseStatusMessage(char *statusString) {
    /*
     *  This function decodes and acts on status strings
     * 
     * A message of type HELLO can be used to set the ip address of the remote end
     * 
     */
    status rxStatus; //Will store the received status message
    initStatusStruct(&rxStatus);

    parseStatusString(&rxStatus, statusString); //Populate rxStatus struct with 
    //contents of statusString

    if (!strcmp(rxStatus.type, "REFRESH")) {
        //This function ensures that the _status structs remain synchronised
        //at both ends. Therefore if packets are lost in the mean time,
        //it shouldn't matter because the ommission will be rectified
        if (debug)printf("\33[2K\rparseStatusMessage: REFRESH received from %s\n", rxStatus.localIP);

        //Where did the message come from? Do we want to act on it?
        char remoteIP[20] = {0};
        getRemoteIP(remoteIP, 20); //Get _status.remoteIP

        char localIP[20] = {0};
        getLocalIP(localIP, 20); //Get _status.localIP

        //If the message is from itself, ignore it
        if (compareIPAddress(localIP, rxStatus.localIP) == 1) {
            printf("\33[2K\rparseStatusMessage: REFRESH received from self. Ignoring.\n");
            return 0;
        }

        //Now determine whether the REFRESH message is from who we were expecting. (Don't want a hijack).
        if (compareIPAddress(remoteIP, rxStatus.localIP) == 1) {
            //Message from a known source
            printf("\33[2K\rparseStatusMessage: REFRESH received from expected source %s. All good.\n", rxStatus.localIP);
        } else {
            //Message is from a new source. Have to decide whether to accept it or not
            printf("\33[2K\rparseStatusMessage: REFRESH received from new source %s..\n", rxStatus.localIP);
            //If we're currently connected, ignore the message
            if (getConnectedFlag() == 1) {
                printf("\33[2K\rparseStatusMessage: Already connected. Ignoring foreign REFRESH from %s\n", rxStatus.localIP);
                return 0;
            } else {
                //If we're not connected, accept the message, provided that 'allowRemoteIPAddrToChange' is set
                if (isRemoteIPSet() == 0) {
                    printf("\33[2K\rparseStatusMessage: _status.remoteIP not set. REFRESH message accepted\n");
                } else {
                    //Remote IP address has been set already. Now decide whether to accept the REFRESH message
                    if (allowRemoteIPAddrToChange == 0) {
                        printf("\33[2K\rparseStatusMessage: allowRemoteIPAddrToChange==0. Ignoring REFRESH message");
                        return 0;
                    } else {
                        printf("\33[2K\rparseStatusMessage: allowRemoteIPAddrToChange==1. REFRESH message accepted\n");
                    }
                }
            }

        }


        //Now refresh message has been received so act on it
        //First, check if anything has changed.
        if (operation_mode == CAMERA_MODE) {
            pthread_mutex_lock(&statusMutex);

            //Servo0
            if (_status.servo0 != rxStatus.servo0) {
                printf("\33[2K\rparseStatusMessage()REFRESH: Servo0 discrepancy\n");
                pthread_mutex_lock(&controlMutex);
                int success = servodControl(0, rxStatus.servo0); //Pass args to servControl
                if (success)_status.servo0 = rxStatus.servo0; //If valid, update global status
                pthread_mutex_unlock(&controlMutex); //Now unlock
            }
            if (_status.servo1 != rxStatus.servo1) {
                printf("\33[2K\rparseStatusMessage()REFRESH: Servo1 discrepancy\n");
                pthread_mutex_lock(&controlMutex);
                int success = servodControl(1, rxStatus.servo1); //Pass args to servControl
                if (success)_status.servo1 = rxStatus.servo1; //If valid, update global status
                pthread_mutex_unlock(&controlMutex); //Now unlock
            }
            if (_status.servo2 != rxStatus.servo2) {
                printf("\33[2K\rparseStatusMessage()REFRESH: Servo2 discrepancy\n");
                pthread_mutex_lock(&controlMutex);
                int success = servodControl(2, rxStatus.servo2); //Pass args to servControl
                if (success)_status.servo2 = rxStatus.servo2; //If valid, update global status
                pthread_mutex_unlock(&controlMutex); //Now unlock
            }
            if (_status.servo3 != rxStatus.servo3) {
                printf("\33[2K\rparseStatusMessage()REFRESH: Servo3 discrepancy\n");
                pthread_mutex_lock(&controlMutex);
                int success = servodControl(3, rxStatus.servo3); //Pass args to servControl
                if (success)_status.servo3 = rxStatus.servo3; //If valid, update global status
                pthread_mutex_unlock(&controlMutex); //Now unlock
            }
            pthread_mutex_unlock(&statusMutex);
        }
        if ((operation_mode == CAMERA_MODE) || (operation_mode == CONTROL_MODE) || (operation_mode == TEST_MODE) || (operation_mode == AUTO_DISCOVER)) {
            //Now update GPO outputs
            pthread_mutex_lock(&statusMutex);
            //Firstly, update global status - Map GPI field of sender to GPO field of receiver
            _status.gpo0 = rxStatus.gpi0;
            _status.gpo1 = rxStatus.gpi1;
            _status.gpo2 = rxStatus.gpi2;
            _status.gpo3 = rxStatus.gpi3;

            //Now update GPO's themselves
            digitalWrite(5, _status.gpo0); //Set GPO0 
            digitalWrite(6, _status.gpo1); //Set GPO1 
            pthread_mutex_unlock(&statusMutex);

            setRemoteIdentity(rxStatus.identity); //Update the global identity field

            //Next: We can assume that all REFRESH message are uniquely addressed (as opposed to
            //broadcast messages). Therefore we can trust the 'localip' field of the received
            //status message.

            //Next, check whether the remote ip address has changed. If it has, store the new
            //'remote ip address' and also send a refresh to the remote end (to act as an ACK)
            char remoteIP[MSG_FIELD_LENGTH] = {0};
            getRemoteIP(remoteIP, MSG_FIELD_LENGTH);
            printf("\33[2K\rREFRESH: remoteIP:%s, rxStatus.localIP:%s\n", remoteIP, rxStatus.localIP);
            if (compareIPAddress(remoteIP, rxStatus.localIP)==0) { //Are the addresses different?
                printf("\33[2K\rREFRESH: Address has changed from %s to %s\n", remoteIP, rxStatus.localIP);
                setRemoteIP(rxStatus.localIP); //Update the 'remoteIP' global field
                if (getConnectedFlag() == 0) {
                    printf("\33[2K\rREFRESH: Sending REFRESH as an ACK\n");
                    sendStatusRefresh(); //Send an ACK to the far end .
                }
            }

        }

        //Message received so reset watchdog timer
        resetWatchdogTimer();

        //Update lastMessageText display field
        setLastMessageText("REFRESH");

        blinkGPO4(1); //Blink GPO4 LED to signify refresh message received
    }
    if (!strcmp(rxStatus.type, "HELLO")) { //Hello message received

        if (debug)printf("\33[2K\rHELLO received from %s at %s:%d. \n",
                rxStatus.identity, rxStatus.localIP, rxStatus.statusPort);

        blinkGPO4(1); //Blink GPO4 LED to signify refresh message received


        //CAMERA_MODE and AUTO_DISCOVER mode rely on the far end 'telling them' 
        //what remote ip address to use.

        //However, we shouldn't allow a camera to connect to another camera, for instance.

        //Firstly see who the broadcast message is from. If its from yourself, ignore it.
        char localIP[MSG_FIELD_LENGTH] = {0}; //Temp char to hold local copy of the global _status localIP field
        getLocalIP(localIP, MSG_FIELD_LENGTH); //Get copy of local IP addr currently held in _status

        char remoteIP[MSG_FIELD_LENGTH] = {0}; //Temp char to hold local copy of the global _status remoteIP field
        getRemoteIP(remoteIP, MSG_FIELD_LENGTH); //Get copy of remote IP addr currently held in _status

        if (debug) printf("\33[2K\rHELLO: localIP:%s, rxStatus.localIP:%s\n", localIP, rxStatus.localIP);
        //if (strcmp(localIP, rxStatus.localIP)) { //If sender and and receiver different
        if (compareIPAddress(localIP, rxStatus.localIP) == 0) {
            //Message is from a remote end, so now determine whether we're currently connected or not
            if (debug)printf("\33[2K\rHELLO: They are clearly different addresses\n");

            //Determine what kind of decice is at the far end - First char of identity field identifies type of device
            char remoteDeviceDescription[MSG_FIELD_LENGTH] = {0};
            int remoteDeviceType = (int) rxStatus.identity[0] - '0'; //Subtract ASCII zero to reveal actual no.
            if (debug)printf("\33[2K\rremoteDeviceType: %d\n", remoteDeviceType);

            switch (remoteDeviceType) {
                case CAMERA_MODE: strlcpy(remoteDeviceDescription, "Camera", MSG_FIELD_LENGTH);
                    break;
                case CONTROL_MODE: strlcpy(remoteDeviceDescription, "Remote", MSG_FIELD_LENGTH);
                    break;
                case TEST_MODE: strlcpy(remoteDeviceDescription, "Test Controller", MSG_FIELD_LENGTH);
                    break;
                case AUTO_DISCOVER: strlcpy(remoteDeviceDescription, "Auto Controller", MSG_FIELD_LENGTH);
                    break;
                default: strlcpy(remoteDeviceDescription, "Unknown", MSG_FIELD_LENGTH);
                    break;
            }
            printf("\33[2K\rHELLO received from %s at %s:%d:%lu (%s)\n",
                    rxStatus.identity, rxStatus.localIP, rxStatus.statusPort,
                    rxStatus.timeStamp, remoteDeviceDescription);
            if (((operation_mode == AUTO_DISCOVER) || (operation_mode == CAMERA_MODE))
                    && (getConnectedFlag() == 0)) {
                //If NOT currently connected to a far end, take the senders ip address
                //Now set 'remote ip' field
                int temp = getConnectedFlag();
                if (debug)printf("\33[2K\rgetConnectedFlag: %d\n", temp);
                if (debug)printf("\33[2K\r(operation_mode == AUTO_DISCOVER"
                        "|| operation_mode==CAMERA_MODE)) && (getConnectedFlag()==0).Setting remote ip "
                        "from %s to %s\n", remoteIP, rxStatus.localIP);
                //Now decide acceptable connection behaviour based on local and remote device device type 

                switch (remoteDeviceType) {
                    case CAMERA_MODE: printf("\33[2K\rRemote device type: Camera\n");
                        //Is remote end already happily connected? If so, don't want to hijack control
                        if ((getOperationMode() == AUTO_DISCOVER) && (rxStatus.connectedFlag == 0)) {
                        //if ((getOperationMode() == AUTO_DISCOVER)) {
                            //An auto discover controller is allowed to connect to a camera
                            setRemoteIP(rxStatus.localIP);
                            setRemoteIdentity(rxStatus.identity);



                            //Message received so reset watchdog timer
                            resetWatchdogTimer();

                            //Now send REFRESH to the newly connected far end (to wake it up)
                            sendStatusRefresh();

                            //Update lastMessageText display field
                            setLastMessageText("HELLO");
                            redrawScreen();
                        } else {
                            printf("\33[2K\rIncompatible devices: Camera can't connect to a camera or remote camera already connected\n");
                        }
                        break;
                    case CONTROL_MODE: printf("\33[2K\rRemote device type: Controller\n");
                        if ((getOperationMode() == CAMERA_MODE) && (rxStatus.connectedFlag == 0)) {
                        //if ((getOperationMode() == CAMERA_MODE)) {
                            //A camera is allowed to connect to an auto discover controller
                            setRemoteIP(rxStatus.localIP);
                            setRemoteIdentity(rxStatus.identity);

                            //Now send REFRESH to the newly connected far end (to wake it up)
                            sendStatusRefresh();

                            //Message received so reset watchdog timer
                            resetWatchdogTimer();

                            //Update lastMessageText display field
                            setLastMessageText("HELLO");
                            redrawScreen();
                        } else {
                            printf("\33[2K\rIncompatible devices: Controller can't connect to an other Controller or remote controller already connected\n");
                        }
                        break;
                    case TEST_MODE: printf("\33[2K\rRemote device type: Test Controller\n");
                        break;
                    case AUTO_DISCOVER: printf("\33[2K\rRemote device type: Auto discovery controller\n");
                        break;
                    default: printf("\33[2K\rRemote device type: Unknown\n");
                        break;
                }

            } else {
                //We are already connected to a remote end, so just acknowledge receipt
                setLastMessageText("HELLO");
                redrawScreen();
            }
        } else {
            //Message is from ourself. Ignore it

            if (debug) printf("\33[2K\r Ignoring own status broadcast message\n");
        }


    }
    return 0;
}

int writeToOutputPipe(char *txBuffer) {
    /*This function creates an output FIFO in the /def folder.
     *It can be monitored from the command line using cat /dev/camerastatus
     * If the FIFO doesn't already exist, it will try to create it
     * It will return -1 on error or the no. of chars sent, if successful
     */

    //char txBuffer[1024];
    char deviceFile[] = "/dev/camerastatus";
    int fd = -1; //Will hold file descriptor table entry. Initialise to -1
    //Open buffer for writing
    //fd = open(deviceFile, O_RDWR | O_NONBLOCK);
    fd = open(deviceFile, O_RDWR);
    if (fd == -1) {
        printf("createOutputPipe(): Couln't open: %s, will try to create FIFO.\n", deviceFile);
        //create FIFO buffer (pipe)
        unlink(deviceFile);
        if (mkfifo(deviceFile, 0666) < 0) {
            printf("createOutputPipe(): Failed to create FIFO %s: n", deviceFile);
            return -1;
        }
        //Set ownership permisions
        if (chmod(deviceFile, 0666) < 0) {
            printf("writeToOutputPipe():createOutputPipe(): Failed to set permissions on FIFO %s:\n", deviceFile);
            return -1;
        }
        //Now try once more
        fd = open(deviceFile, O_RDWR); //Try to open the file
        if (fd == -1) {
            printf("createOutputPipe(): Still can't open pipe for writing\n");
            return -1;
        }
    } else {
        //printf("Get's here(2)................................\n");
        if (debug) printf("writeToOutputPipe():createOutputPipe():createOutputPipe(): Value of fd:%d\n", fd);
        //snprintf(txBuffer,1024,"Writing to pipe: %d\n",x);
        int m = write(fd, txBuffer, strlen(txBuffer));
        int n = close(fd);
        if (n != 0) {
            printf("writeToOutputPipe():createOutputPipe():Can't close %s\n", deviceFile);
            return -1;
        } else return m; //Return no. of chars written to pipe
    }
}

void *monitorMemDevThread() {
    //This thread attempts to create, then monitors a memory-mapped device
    // /dev/cameracontrol
    //It uses code cribbed from the servod sourcefile
    struct timeval tv;
    int fd;
    int n;
    char rxBuffer[1024];
    char deviceFile[] = "/dev/cameracontrol";

    //create FIFO buffer (pipe)
    unlink(deviceFile);
    if (mkfifo(deviceFile, 0666) < 0) {
        printf("monitorMemDevThread(): Failed to create FIFO %s: n", deviceFile);
        exit(0);
    }
    //Set ownership permisions
    if (chmod(deviceFile, 0666) < 0) {
        printf("monitorMemDevThread(): Failed to set permissions on FIFO %s:\n", deviceFile);
        exit(0);
    }

    fd = open(deviceFile, O_RDWR | O_NONBLOCK);
    if (fd == -1) {
        printf("monitorMemDevThread():Couln't create: %s\n", deviceFile);
        exit(0);
    } else {
        if (debug)printf("monitorMemDevThread():Value of fd:%d\n", fd);

        for (;;) { //Cludgy but works. Sit in endless for loop. Break out if select returns a '1'
            fd_set ifds;
            FD_ZERO(&ifds);
            FD_SET(fd, &ifds);
            tv.tv_sec = 0;
            tv.tv_usec = 100000; //100000us =0.1 seconds timeout
            if (n = select(fd + 1, &ifds, NULL, NULL, &tv) != 1) continue; // Sit in for loop until a '1', (actual data)


            read(fd, rxBuffer, 1024); //Read contents of pipe
            if (debug) printf("monitorMemDevThread():New data in pipe: %s: %s\n", deviceFile, rxBuffer);
            //Optionally echo incoming message to output pipe
            if (writeToPipe) {
                char pipeBuff[1024];
                snprintf(pipeBuff, 1024, "RX via %s: %s\n", deviceFile, rxBuffer); //Create nice formatted string

                if (writeToPipe)writeToOutputPipe(pipeBuff); //optionally write too pipe  
            }
            //Now see if the received command makes any sense
            //Now actually decode the command
            parseControlMessage(rxBuffer);

            //Now redraw the screen
            redrawScreen();

            memset(rxBuffer, 0, 1024); //erase buffer
        }
    }
}

void *heartbeatThread() {
    /*
     *This thread runs every TIMEOUT/2 period (i.e 3 seconds
     *It re-sends a snapshot of the current control surface 
     *parameters (even if they've not changed)
     *That way, the camera will always stay in sync with
     * the remote controller with only a moderate amount 
     * of additional network traffic.
     * 
     * This thread can only  run if connectedFlag has been set
     * (so it relies on the camera sending revertives back, 
     * otherwise the controller will think it's lost comms)
     * 
     * 
     */
    int refreshTimer = 0;
    int timeLimit = (int) (TIMEOUT / 2.0);
    if (debug) printf("refreshControlDataThread(): timeLimit:%d\n", timeLimit);




    while (1) {

        //if ((refreshTimer > timeLimit) && getConnectedFlag()) { //Has timeLimit been exceeded (and are we 'connected')
        char remoteIP[MSG_FIELD_LENGTH] = {0}; //Char array to hold remote ip address
        getRemoteIP(remoteIP, MSG_FIELD_LENGTH);

        if (refreshTimer > timeLimit) { //Has timeLimit been exceeded
            //if (strcmp(remoteIP, "0.0.0.0")) { //Have we a known remote ip to send to?
            if (isRemoteIPSet() == 1) { //Have we a known remote ip to send to?

                if ((operation_mode == CONTROL_MODE) || (operation_mode == CAMERA_MODE)
                        || (operation_mode == TEST_MODE) || (operation_mode == AUTO_DISCOVER)) {
                    if (debug) printf("\33[2K\rheartbeatThread(): Sending REFRESH to %s\n", remoteIP);
                    sendStatusRefresh(); //Send Status REFRESH message

                }
                refreshTimer = 0; //Clear refresh timer

                if (debug) printf("refreshControlDataThread() fired                 \n");
            } else {

                if (debug) printf("\33[2K\rheartbeatThread(): remoteIP not known yet. Can't send refresh\n");
            }
        }

        refreshTimer++; //Increment refresh timer
        sleep(1); //1 second delay
    }
}

void redrawScreen() {
    //This function shows a running display of state variables
    //It is a blocking function because it locks displayMutex
    status displayStatus;               //For local copy of global _status
    getStatusSnapshot(&displayStatus);  //Take local copy of status
    
    pthread_mutex_lock(&displayMutex); //Lock displayMutex
    
    if (operation_mode == CAMERA_MODE) {

        printf("\33[2K\r\x1B[7mMode: Camera, Control port:%d, Status port:%d\n",
                displayStatus.controlPort, displayStatus.statusPort);
    } else if (operation_mode == CONTROL_MODE) {
        printf("\33[2K\r\x1B[7mMode: Control, Control port:%d, Status port:%d\n",
                displayStatus.controlPort, displayStatus.statusPort);
    } else if (operation_mode == TEST_MODE) {
        printf("\33[2K\r\x1B[7mMode: Test, Control port:%d, Status port:%d\n",
                displayStatus.controlPort, displayStatus.statusPort);
    } else if (operation_mode == AUTO_DISCOVER) {
        printf("\33[2K\r\x1B[7mMode: Auto Discover, Control port:%d, Status port:%d\n",
                displayStatus.controlPort, displayStatus.statusPort);
    }

    //'\33[2K\r'   //Erase existing line, move cursor to start of line
    printf("\33[2K\r\x1B[7mLocal IP addr: %s, Remote IP addr: %s\x1B[0m\n"
            , displayStatus.localIP, displayStatus.remoteIP);
    if (operation_mode == CAMERA_MODE)
        printf("\33[2K\rServo 0:%d%%,\t1:%d%%,\t2:%d%%,\t3:%d%%\n",
            displayStatus.servo0, displayStatus.servo1, displayStatus.servo2, displayStatus.servo3);
    if (operation_mode == CONTROL_MODE || (operation_mode == TEST_MODE) || (operation_mode == AUTO_DISCOVER))
        printf("\33[2K\rAnalg 0:%d%%,\t1:%d%%,\t2:%d%%,\t3:%d%%\n",
            displayStatus.servo0, displayStatus.servo1, displayStatus.servo2, displayStatus.servo3);
    printf("\33[2K\rGPI   0:%d,\t1:%d,\t2:%d,\t3:%d,\t4:%d,\t5:%d,\t6:%d\n",
            displayStatus.gpi0, displayStatus.gpi1, displayStatus.gpi2, displayStatus.gpi3,
            displayStatus.gpi4, displayStatus.gpi5, displayStatus.gpi6);
    printf("\33[2K\rGPO   0:%d,\t1:%d,\t2:%d,\t3:%d,\t4:%d,\t5:%d\n",
            displayStatus.gpo0, displayStatus.gpo1, displayStatus.gpo2,
            displayStatus.gpo3, displayStatus.gpo4, displayStatus.gpo5);
    

    //Check to see whether we're connected or not
    char _remoteIdentity[BUFFER] = {0}; //Not to be confused with the global version!
    getRemoteIdentity(_remoteIdentity, BUFFER);
    if (getConnectedFlag())printf("\33[2K\rStatus: \x1B[32mConnected\x1B[0m to %s\n",
            _remoteIdentity); //Green text

    else printf("\33[2K\rStatus: \x1B[31mNot connected\x1B[0m\n"); //Red text

    //Print lastMessageText display field and timestamp
    char textBuffer[1024]; //Buffer to hold message
    char timeAsString[100]; //Buffer to hold human readable time
    time_t timeStamp = getLastMessageText(textBuffer, 1024); //Get latest message and timestamp values
    strftime(timeAsString, 100, "%H:%M:%S", localtime(&timeStamp)); //Format human readable time
    printf("\33[2K\r%d sec since last message received: %s, %s\n", getWatchDogTimer(), textBuffer, timeAsString);
    printf("\033[7A"); // Move cursor up 7 lines, ready for next time

    pthread_mutex_unlock(&displayMutex); //Unlock displayMutex
}

void *watchdogThread() {
    /*
     *This thread keeps track of the seconds since a message was last received
     * and determines whether or not the camera/controller is connected to a
     * the remote end (based on TIMEOUT value and also whether _status.remoteIP
     * has been populated
     * 
     * ********************
     *This should be THE ONLY THREAD that decides upon connectedFlag 
     * (and sets it via the threadsafe function setConnectedFlag()
     *However, other threads CAN and SHOULD modify watchdogTimer via
     * the resetWatchdogTimer(), getWatchdogTimer() and
     * incrementWatchDogTimer() functions
     * ********************
     * 
     * If the thread determines that a connection has been lost it will
     * turn off all outputs (servos and GPOs) and rewrite _status
     * with zero values in the output fields
     */
    sleep(2); //Allow time for servod to start before starting wdt

    setWatchdogTimer(TIMEOUT); //Initially exceed watchdog limit so that all indicators reflect 
    //a 'disconnected' state
    while (1) {


        incrementWatchdogTimer(); //Increment wdt
        //Test whether remote ip has been set and also how recently a message
        //was received

        //if ((getWatchDogTimer() < TIMEOUT)&&(strcmp(_status.remoteIP, "0.0.0.0"))) { //if remote IP set
        if ((getWatchDogTimer() < TIMEOUT)&&(isRemoteIPSet() == 1)) { //if remote IP set   
            setConnectedFlag(1); //Set connectedFlag to 'true'
/*
            pthread_mutex_lock(&statusMutex);
            _status.connectedFlag = getConnectedFlag(); //Update global _status flag
            pthread_mutex_unlock(&statusMutex); //Now unlock
*/
            digitalWrite(gpoPinArray[5], HIGH); //Illuminate LED on GPO5 to denote 'connected'
        } else { //else if not connected
            if (debug)printf("watchdogThread(): TIMEOUT exceeded or no remote IP. Setting all GPO and servos to 0\n");
            setConnectedFlag(0); //Set connectedFlag to false

            /*
            pthread_mutex_lock(&statusMutex);
            _status.connectedFlag = getConnectedFlag(); //Update global _status flag
            pthread_mutex_unlock(&statusMutex); //Now unlock
            */
            digitalWrite(gpoPinArray[5], LOW); //Turn off LED on GPO5 to denote 'not connected'
            setRemoteIdentity(" "); //Clear remote identity global 
            //Now, if in CAMERA_MODE, reset all status values and outputs to zero for safety
            if (operation_mode == CAMERA_MODE) {
                //If camera mode, modify _status with all zeros on servos

                pthread_mutex_lock(&statusMutex);
                _status.servo0 = 0;
                _status.servo1 = 0;
                _status.servo2 = 0;
                _status.servo3 = 0;
                pthread_mutex_unlock(&statusMutex); //Now unlock

                //Modify the servo outputs themselves
                pthread_mutex_lock(&controlMutex);
                servodControl(0, 0);
                servodControl(1, 0);
                servodControl(2, 0);
                servodControl(3, 0);
                pthread_mutex_unlock(&controlMutex);
            }
            if ((operation_mode == CAMERA_MODE) || (operation_mode == CONTROL_MODE)
                    || (operation_mode == TEST_MODE) || (operation_mode == AUTO_DISCOVER)) {
                //GPO's applicable to both CAMERA_MODE and REMOTE_MODE


                //Update _status values
                pthread_mutex_lock(&statusMutex);
                _status.gpo0 = 0;
                _status.gpo1 = 0;
                _status.gpo2 = 0;
                _status.gpo3 = 0;
                _status.gpo4 = 0;
                _status.gpo5 = 0;
                pthread_mutex_unlock(&statusMutex); //Now unlock
                //Now update GPO's themselves
                //digitalWrite(5, LOW); //Set GPO0 to 0 for safety;
                //digitalWrite(6, LOW); //Set GPO1 to 0 for safety;

                for (int x = 0; x < NO_OF_GPOs; x++) digitalWrite(gpoPinArray[x], LOW);
            }
        }

        redrawScreen(); //Now redraw the screen
        sleep(1); //Sleep for 1 second
    }
}

void *GPIOMonitorThreadV2() {
    /*This thread monitors the two GPI inputs that can be sent
     * It also sets up the output pins
     * 
     * Unlike th original GPIOMonitorThread, it uses polling rather
     * than interrupts
     * 
     * it will successively scan through the number of inputs specified
     * by NO_OF_GPIs looking for a change in state. If a change is detected,
     * the input is debounced (by taking the majority value of 10 samples), 
     * next, an identical (apart from an 'index' value) GPI message is 
     * sent 10 times just to make sure that the receiver has a good chance of 
     * receiving it.
     * 
     * A timestamp is inserted into the text field of the command struct
     * An incrementing index value is inserted into arg3 of the command struct.
     * (This might be useful to the receiver in determing packet loss of jitter
     * because it should received 10 UDP messages in order, with incrementing
     * index values)
     * 
     * 
     * At startup, if in camera mode, no remote IP address is set (i.e.
     * _status.remoteIP=0.0.0.0. This is interpreted as the loopback address
     * For the camera mode, this means that at startup, any GPI input messages
     * generated would be sent to 0.0.0.0 (i.e 127.0.0.1), i. the will be 
     * automatically looped back on themselves. This could be dangerous.
     * Therefore UDP sending is inhibited in this function if
     * _status.remoteIP=0.0.0.0
     * 
     * 
     * 
     * This pins are used by the servos:-
     * Servo mapping:
     0 on P1-7          BCM GPIO-4         servo0 (wPi pin 7)
     1 on P1-11         BCM GPIO-17        servo1 (wPi pin 0)
     2 on P1-12         BCM GPIO-18        servo2 (wPi pin 1)
     3 on P1-13         BCM GPIO-27        servo3 (wPi pin 2)
     * 
     * Nominate
     * GPI0: P1-15      BCM GPIO-22        wiringPi pin 3     
     * GPI1: P1-16      BCM GPIO-23        wiringPi pin 4
     * GPI2: P1-29      BCM GPIO-5         wiringPi pin 21          
     * GPI3: P1-31      BCM GPIO-6         wiringPi pin 22
     * GPI4: P1-33      BCM GPIO-13        wiringPi pin 23
     * GPI5: P1-35      BCM GPIO-19        wiringPi pin 24
     * GPI6: P1-37      BCM GPIO-26        wiringPi pin 25
     * 
     * GPO0: P1-18      BCM GPIO-24        wiringPi pin 5
     * GPO1: P1-22      BCM GPIO-25        wiringPi pin 6
     * GPO2: P1-32      BCM GPIO-12        wiringPi pin 26   
     * GPO3: P1-36      BCM GPIO-16        wiringPi pin 27
     * GPO4: P1-38      BCM GPIO-20        wiringPi pin 28
     * GPO5: P1-40      BCM GPIO-21        wiringPi pin 29
     */

    int samplePinValue; //Used for sampling the pin values for debouncing

    command txCmd; //Create tx command struct
    initCmdStruct(&txCmd); //Init command structure
    char txCmdBuf[1024] = {0}; //Create output buffer for txCmd string
    strlcpy(txCmd.cmd, "GPI", MSG_FIELD_LENGTH); //Set command message header
    char remoteIP[100] = {0}; //Used to store local copy of _status.remoteIP field
    //Array to hold old and new values of sampled GPI pins
    int gpiPinStatus[7][2] = {0}; //GPI0-GPI6 pin values, two columns: xOld and xNew

    //Create input and output GPIO pins
    for (int x = 0; x < NO_OF_GPIs; x++) { //Cycle through gpiPinArray
        if (debug) printf("\33[2K\rgpiPinArray[7]: %s:%d\n",
                gpiPinName[x], gpiPinArray[x]);
        pinMode(gpiPinArray[x], INPUT); //Create input pin
        pullUpDnControl(gpiPinArray[x], PUD_DOWN); //Enable pull-down
    }
    for (int x = 0; x < NO_OF_GPOs; x++) { //Cycle through gpoPinArray
        if (debug)printf("\33[2K\rgpoPinArray[6]: %s:%d\n",
                gpoPinName[x], gpoPinArray[x]);
        pinMode(gpoPinArray[x], OUTPUT); //Create output pins
    }

    //Sit in a loop, sampling the GPI inputs and watching for changes
    int gpiNo = 0;


    while (1) {
        samplePinValue = 0; //Used for sampling the pin values for debouncing

        //Monitor GPIx
        //gpiPinArray[gpiNo][2] = digitalRead(gpiPinArray[gpiNo][0]); //Copy digitalRead() value into xNew column
        gpiPinStatus[gpiNo][1] = digitalRead(gpiPinArray[gpiNo]); //Copy digitalRead() value into xNew column
        //if (gpiPinArray[gpiNo][2] != gpiPinArray[gpiNo][1]) { //Compare XNew and XOld value to detect change
        if (gpiPinStatus[gpiNo][1] != gpiPinStatus[gpiNo][0]) { //Compare XNew and XOld value to detect change 
            //Now debounce to see if a valid level change
            for (int n = 0; n < 10; n++) { //Take 10 samples
                //if (digitalRead(gpiPinArray[gpiNo][0])) samplePinValue++;
                if (digitalRead(gpiPinArray[gpiNo])) samplePinValue++; //Sample currently selected GPI pin
            }
            //if (samplePinValue > 4)gpiPinArray[gpiNo][2] = 1; //if 5 or more, Looks like a '1', set xNew column
            if (samplePinValue > 4) gpiPinStatus[gpiNo][1] = 1; //if 5 or more, Looks like a '1', set xNew column
                //else gpiPinArray[gpiNo][2] = 0; //else a '0'
            else gpiPinStatus[gpiNo][1] = 0; //else a '0', set xNew column accordingly
            //gpiPinArray[gpiNo][1] = gpiPinArray[gpiNo][2]; //Copy new pin value into xOld column
            gpiPinStatus[gpiNo][0] = gpiPinStatus[gpiNo][1]; //Copy new pin value into xOld column

            //Now we've established the level of GPIx, update global status and send the command
            pthread_mutex_lock(&statusMutex);
            //Update global _status depending upon which gpi pin we've sampled
            switch (gpiNo) {
                case 0:
                    //_status.gpi0 = gpiPinArray[gpiNo][2];
                    _status.gpi0 = gpiPinStatus[gpiNo][1];
                    break;
                case 1:
                    //_status.gpi1 = gpiPinArray[gpiNo][2];
                    _status.gpi1 = gpiPinStatus[gpiNo][1];
                    break;
                case 2:
                    //_status.gpi2 = gpiPinArray[gpiNo][2];
                    _status.gpi2 = gpiPinStatus[gpiNo][1];
                    break;
                case 3:
                    //_status.gpi3 = gpiPinArray[gpiNo][2];
                    _status.gpi3 = gpiPinStatus[gpiNo][1];
                    break;
                case 4:
                    //_status.gpi4 = gpiPinArray[gpiNo][2];
                    _status.gpi4 = gpiPinStatus[gpiNo][1];
                    break;
                case 5:
                    //_status.gpi5 = gpiPinArray[gpiNo][2];
                    _status.gpi5 = gpiPinStatus[gpiNo][1];
                    break;
                case 6:
                    //_status.gpi6 = gpiPinArray[gpiNo][2];
                    _status.gpi6 = gpiPinStatus[gpiNo][1];
                    break;
            }
            pthread_mutex_unlock(&statusMutex);

            //Now create GPI command to be sent
            txCmd.arg1 = gpiNo; //GPIx
            //txCmd.arg2 = gpiPinArray[gpiNo][2]; //Actual value of pin to be sent
            txCmd.arg2 = gpiPinStatus[gpiNo][1]; //Actual value of pin to be sent
            snprintf(txCmd.text, MSG_FIELD_LENGTH, "%d", (int) time(NULL)); //Send timestamp in text field. Possibly useful
            memset(remoteIP, 0, 100);
            getRemoteIP(remoteIP, 100); //Get remote IP from _status global struct (in a threadsafe way)

            //Send a stream of 10 identical commands. Hopefully at least one will get through
            //Only send udp message if we have a valid (i.e non 0.0.0.0 address to send to)
            //if (compareIPAddress(remoteIP, "0.0.0.0") != 1) {
            if (isRemoteIPSet() == 1) {
                //..But only if remote ip address is set 
                for (int n = 0; n < 10; n++) {
                    txCmd.arg3 = n; //Index value (might be useful to see if all packets make it through)
                    strlcpy(txCmd.text,"Test,ng!",MSG_FIELD_LENGTH);  //Test, so to see if delimiter char protection works
                                                                    //The ',' should be replaced with a '!'
                    createCmdString(&txCmd, txCmdBuf, 1024);
                    int control_port = getUDPControlPort(); //Retrieve control port from global _status struct
                    char interface[MSG_FIELD_LENGTH] = {0};
                    getControlInterface(interface, MSG_FIELD_LENGTH); //Get interface to send message out on
                    sendUDP(txCmdBuf, remoteIP, control_port, interface); //Transmit the string
                    //if (writeToPipe)writeToOutputPipe(txCmdBuf); //optionally write too pipe
                    if (writeToPipe) {
                        char pipeBuff[1024];
                        snprintf(pipeBuff, 1024, "TX: %s\n", txCmdBuf); //Create nice formatted string
                        writeToOutputPipe(pipeBuff); //optionally write too pipe  
                    }
                    delay(10); //10mS delay between bursts
                }

            } else {
                printf("GPIOMonitorThreadV2(): No ip address to send GPI message to\n");
            }
        }
        //Now redraw the screen
        redrawScreen();

        //Now increment gpiNo so we can scan the next button
        gpiNo++;

        if (gpiNo > (NO_OF_GPIs - 1)) gpiNo = 0; //Only scanning x buttons, after last button, go back to first

        delay(50); //50mS delay between polls

    }
}

void *controlSurfaceThread() {
    //This thread monitors the remote controller surfaces
    //Setup A2D converter chip
    mcp3004Setup(BASE, SPI_CHAN); // 3004 and 3008 are the same 4/8 channels
    if (!analogRead(BASE + 0)&&!analogRead(BASE + 1)&&!analogRead(BASE + 2)&&!analogRead(BASE + 3))
        //If result from all analogue inputs is zero, suggests that the MCP3008 might no be connected
        printf("\33[2K\r\x1B[31mcontrolSurfaceThread():Warning: check connection to MCP3008 A2D device. Analogue inputs may be unavailable.\x1B[0m\n");
    //int xNew, xOld;
    double convertedVal;
    int channel = 0; //Which of the four analogue inputs we're interested in
    //char txBuffer[100];
    command txCommand;
    //xNew = analogRead(BASE + channel); //Select channel 0
    int analogueValues[4][2]; //Stores the xOld and xNew values for analogue channels 0-3
    //so[0][0]= channel 0 xOld, [0][1]= channel 0 xNew

#define NEW 1           //Used for addresssing the array. Just makes code easier to read
#define OLD 0    
    //Capture initial values
    for (int x = 0; x < 4; x++)
        analogueValues[x][NEW] = analogRead(BASE + x);

    while (1) {
        analogueValues[channel][NEW] = analogRead(BASE + channel); //Capture value on channel x

        if (abs(analogueValues[channel][NEW] - analogueValues[channel][OLD]) > 1) {
            //if (abs(xNew - xOld) > 1)   //Should hopefully minimise unecessary
            //sends due to sampling jitter

            analogueValues[channel][OLD] = analogueValues[channel][NEW];
            // A2D runs 0 - 1024, servod runs 0 - 100 %
            //convertedVal = ((analogueValues[channel][NEW] / 1024.0)*100);       //Works for a linear pot
            if (analogueValues[channel][NEW] == 0)
                analogueValues[channel][NEW] = 1; //Can't allow zero, it ruins the log conversion
            convertedVal = 100.0 * (log(analogueValues[channel][NEW]) / log(1024.0)); //Works for a log pot

            convertedVal = (100.0 - convertedVal);

            if (debug)printf("\33[2K\rcontrolSurfaceThread(): Channel:%d, xNew:%d, converted value:%f\n",
                    channel, analogueValues[channel][NEW], convertedVal);
            initCmdStruct(&txCommand); //Init command structure
            strlcpy(txCommand.cmd, "SERVO", MSG_FIELD_LENGTH);
            txCommand.arg1 = channel; //Servo 0
            txCommand.arg2 = (int) convertedVal; //Value to send


            //Generate string to transmit
            char txBuffer[1024] = {0};
            //bzero(txBuffer, 1024);
            createCmdString(&txCommand, txBuffer, 1024);

            //Transmit the string
            char remoteIP[100]; //Used to store local copy of _status.remoteIP field
            //So that we don't tie up the statusMutex resource
            getRemoteIP(remoteIP, 100); //Get remote IP from _status global struct (in a threadsafe way)
            int control_port = getUDPControlPort(); //Retrieve control port from global _status struct

            char interface[MSG_FIELD_LENGTH] = {0};
            getControlInterface(interface, MSG_FIELD_LENGTH); //Get interface to send message out on

            //Only send udp message if we have a valid (i.e non 0.0.0.0 address to send to)
            //if (compareIPAddress(remoteIP, "0.0.0.0") != 1) {
            if (isRemoteIPSet() == 1) {
                printf("controlSurfaceThread(): Sending control message: %s to %s\n", txBuffer, remoteIP);
                sendUDP(txBuffer, remoteIP, control_port, interface);
            } else {
                printf("controlSurfaceThread(): Remote IP address not know yet. Nowhere to send control message to.\n");
            }
            //if (writeToPipe)writeToOutputPipe(txBuffer); //optionally write to pipe
            if (writeToPipe) {
                char pipeBuff[1024];
                snprintf(pipeBuff, 1024, "TX: %s\n", txBuffer); //Create nice formatted string
                writeToOutputPipe(pipeBuff); //optionally write too pipe  
            }
            //Now update global _status struct
            pthread_mutex_lock(&statusMutex);

            switch (channel) {
                case 0:
                    _status.servo0 = convertedVal;
                    break;
                case 1:
                    _status.servo1 = convertedVal;
                    break;
                case 2:
                    _status.servo2 = convertedVal;
                    break;
                case 3:
                    _status.servo3 = convertedVal;
                    break;
            }
            pthread_mutex_unlock(&statusMutex);

            //Now redraw the screen
            redrawScreen();
        }
        channel++; //Incremement A2D channel index for next time scanning around loop

        if (channel > 3) channel = 0; //After A2D ch 3 has been sampled, go back to ch 0
        delay(30); //30mS delay
    }
    /*    
        while (1) {

            //xNew = analogRead(BASE + channel); //Select channel 0
        


            if (abs(xNew - xOld) > 1) {
                //if (abs(xNew - xOld) > 1)   //Should hopefully minimise unecessary
                //sends due to sampling jitter

                xOld = xNew;
                // A2D runs 0 - 1024, servod runs 0 - 100 %
                //convertedVal = ((xNew / 1024.0)*100);       //Works for a linear pot
                if (xNew == 0)xNew = 1; //Can't allow zero, it ruins the log conversion
                convertedVal = 100.0 * (log(xNew) / log(1024.0)); //Works for a log pot

                if (debug)printf("\33[2K\rcontrolSurfaceThread(): xnew:%d, converted value:%f\n", xNew, convertedVal);
                initCmdStruct(&txCommand); //Init command structure
                strlcpy(txCommand.cmd, "SERVO",MSG_FIELD_LENGTH);
                txCommand.arg1 = 0; //Servo 0
                txCommand.arg2 = (int) convertedVal; //Value to send


                //Generate string to transmit
                char txBuffer[1024] = {0};
                //bzero(txBuffer, 1024);
                createCmdString(&txCommand, txBuffer, 1024);

                //Transmit the string
                pthread_mutex_lock(&statusMutex);
                sendUDP(txBuffer, _status.remoteIP, control_port);
                pthread_mutex_unlock(&statusMutex);
                if(writeToPipe)writeToOutputPipe(txBuffer); //optionally write too pipe
                //Now update global _status struct
                pthread_mutex_lock(&statusMutex);
                _status.servo0 = convertedVal;
                pthread_mutex_unlock(&statusMutex);

                //Now redraw the screen
                redrawScreen();

            }
            delay(100); //100mS delay

            //char input=getche();
            //if(input=='p')sendUDP("p",SRV_IP,SRV_PORT);
            //if(input=='l')sendUDP("l",SRV_IP,SRV_PORT);
        }
     */
}

int match(char text[], char input[]) {
    //This function tries to find the pattern input[] in text[]
    //If it finds it, it will return the position, otherwise
    //it will return -1
    int c, d, e, text_length, pattern_length, position = -1;
    char pattern[100];
    strlcpy(pattern, input, 100); //Make copy of input[] otherwise it'll be overwritten

    text_length = strlen(text);
    pattern_length = strlen(pattern);

    if (pattern_length > text_length) {
        return -1;
    }

    for (c = 0; c <= text_length - pattern_length; c++) {
        position = e = c;

        for (d = 0; d < pattern_length; d++) {
            if (pattern[d] == text[e]) {
                e++;
            } else {
                break;
            }
        }
        if (d == pattern_length) {

            return position;
        }
    }

    return -1;
}

void *blinkThread(void *arg) {
    //PThread invoked by blinkGPO4
    int x = *((int*) arg);
    free(arg); //Free up memory requested by malloc in blinkLED()
    //printf("\33[2K\rNo of blinks: %d\n",x);
    //int currentValue = digitalRead(gpoPinArray[4]); //Get current GPO5 value
    //delay(2); //5mS delay
    //printf("blinkThread(): currentValue:%d\n",currentValue);
    digitalWrite(gpoPinArray[4], LOW);
    delay(5); //Seemingly this 5mS delay n
    for (int n = 0; n < x; n++) {

        digitalWrite(gpoPinArray[4], HIGH);
        delay(5); //5mS delay
        digitalWrite(gpoPinArray[4], LOW);
        delay(5); //5mS delay
    }
    //digitalWrite(gpoPinArray[4], currentValue); //reinstate original value

}

void blinkGPO4(int noOfBlinks) {
    //This causes GPO4 to blink
    int *i = malloc(sizeof (*i)); //Create space for an integer pointer
    *i = noOfBlinks; //Assign supplied value to that pointer
    pthread_t _blinkThread;
    if (pthread_create(&_blinkThread, NULL, blinkThread, (void*) i)) {

        printf("main(): Error creating blinkThread\n");
    }
    pthread_detach(_blinkThread); //Don't care what happens to thread afterwards
}

void *pulseGPOThread(void *arg) {
    /*
     *Invoked by the pulseGPO() funtion.
     * Relies on the gpoPinArray[] to map Camera Control pins to wiringPi pins
     * delay() is part of wiringPi
     */
    //Cast the incoming threadInput variable as a pointer to a SharedDataStruct
    gpoPulse _gpoPulse = *((gpoPulse*) arg); //Tale local copy of arg
    free(arg); //Free up memory requested by malloc in pulseGPO()
    if (debug)printf("\33[2K\r*pulseGPOThread():Supplied arg: %d\n", _gpoPulse.noOfPulses);
    if (_gpoPulse.gpoPin < 6) { //Check that a valid pin has been specified
        for (int n = 0; n < _gpoPulse.noOfPulses; n++) {
            digitalWrite(gpoPinArray[_gpoPulse.gpoPin], HIGH); //Turn pin on
            delay(_gpoPulse.onDuration); //Wait
            digitalWrite(gpoPinArray[_gpoPulse.gpoPin], LOW); //Turn pin off
            delay(_gpoPulse.offDuration); //Wait
        }
    } else printf("mains.c:pulseGPOThread() Invalid pin specified: %d", _gpoPulse.gpoPin);


}

void pulseGPO(int pin, int noOfPulses, int onDuration, int offDuration) {
    /*
     *Pulses a specified GPO pin on/off x no of times, for a specified on/off duration
     * It is non blocking because it creates another thread to actually control the o/p pin
     */
    gpoPulse *_gpoPulse = malloc(sizeof (*_gpoPulse)); //Create/reserve mem for gpoPulse struct
    _gpoPulse->gpoPin = pin;
    _gpoPulse->noOfPulses = noOfPulses;
    _gpoPulse->onDuration = onDuration;
    _gpoPulse->offDuration = offDuration;

    //Create pulseGPO thread and pass gpoPulse struct to it
    pthread_t _pulseGPOThread;
    if (pthread_create(&_pulseGPOThread, NULL, pulseGPOThread, (void*) _gpoPulse)) {

        printf("main(): Error creating pulseGPOThread\n");
    }
    pthread_detach(_pulseGPOThread); //Don't care what happens to thread afterwards
}

void zeroAllOutputs() {
    /*
     *This function sets all GPO pins and servo outputs to zero
     */
    //Set  all GPO outputs to zero

    for (int x = 0; x < NO_OF_GPOs; x++) digitalWrite(gpoPinArray[x], LOW);

    //For safety, set all servo outputs to zero
    servodControl(0, 0);
    servodControl(1, 0);
    servodControl(2, 0);
    servodControl(3, 0);
}

void *shutDownThread(void *arg) {
    //This thread intercepts the SIGINT and SIGKILL Linux messages
    //Override the inherited signal 
    //Send signal from terminal using kill -s USR1 [pid]]
    sigset_t sigsToCatch;
    int caught; //Will store the returned 'signal number' that
    //corresponds with USR1 on this system
    sigemptyset(&sigsToCatch); //initialise and empty the signal set
    sigaddset(&sigsToCatch, SIGUSR1); //Add USR1 to our signal set
    sigaddset(&sigsToCatch, SIGUSR1); //Add USR1 to our signal set
    sigaddset(&sigsToCatch, SIGUSR2); //Add USR2 to our signal set
    sigaddset(&sigsToCatch, SIGKILL); //Add SIGKILL to our signal set
    sigaddset(&sigsToCatch, SIGTERM); //Add SIGTERM to our signal set
    sigaddset(&sigsToCatch, SIGINT); //Add SIGINT to our signal set

    while (1) {
        sigwait(&sigsToCatch, &caught); //Blocking call to wait for SIGUSR1
        switch (caught) { //Determine which of the messages has been received
            case SIGUSR1:
                printf("\33[2K\rSIGUSR1 not implemented\n");
                break;
            case SIGUSR2:
                printf("\33[2K\rSIGUSR2 not implemented\n");
                break;
            case SIGKILL:
                printf("\33[2K\rSIGKILL\n");
                if (rxUDControlHandle != 0) close(rxUDControlHandle); //Close UDP port binding
                if (rxUDPStatusHandle != 0) close(rxUDPStatusHandle); //Close UDP port binding
                zeroAllOutputs(); //Turn off all outputs
                printf("\033[7B"); // Move cursor down 7 lines for terminal neatness
                exit(1);
                break;
            case SIGTERM:
                printf("\33[2K\rSIGTERM\n");
                if (rxUDControlHandle != 0) close(rxUDControlHandle); //Close UDP port binding
                if (rxUDPStatusHandle != 0) close(rxUDPStatusHandle); //Close UDP port binding
                zeroAllOutputs(); //Turn off all outputs
                printf("\033[7B"); // Move cursor down 7 lines for terminal neatness
                exit(1);
                break;
            case SIGINT:
                if (debug)printf("\33[2K\rSIGINT. Closing fd's:Control:%d, Status:%d\n",
                        rxUDControlHandle, rxUDPStatusHandle);
                if (rxUDControlHandle != 0) close(rxUDControlHandle); //Close UDP port binding
                if (rxUDPStatusHandle != 0) close(rxUDPStatusHandle); //Close UDP port binding
                zeroAllOutputs(); //Turn off all outputs
                printf("\033[7B"); // Move cursor down 7 lines for terminal neatness
                exit(1);

                break;
        }
    }
}

void *testModeThread(void *arg) {
    /*
     * This thread generates random servo messages according the supplied servoTest struct
     * It will generate a new servo value at a period determined by test.rate
     * The total test will run for the time specified in test.totalDuration
     * If supplied totalDuration is zero, the test will run forever
     * If Supplied rate is zero, a new value will be calculated every 10th of a second
     * Using 'rate' allows the changes to be damped
     * min and max specify the min and max limits of the random value
     * duration specifies the length of time (in seconds) between every 
     * new calculation.
     * The calculated servo value is transmitted to the far end in the normal way
     * (by creating a commmand structure, forming it inta string and then sending it)
     * For added feedback, the thread populated the "lastMessage" field which is
     * displayed by the redrawScreen() function
     * 
     * 
     */

    servoTest test = *((servoTest*) arg); //Take a local copy of arg
    free(arg); //Free up memory used by arg
    srand(time(NULL)); //Seed random generator
    int runTime = 0; //Counts the time in seconds that test has been running
    int index = 0; //10 periods per second (i.e servo value recalculated every 10th of a second)
    float delta = 0.0; //Amount by which to change servo value every 0.1sec (equal to test.rate/10)
    float currentServoValue = 0;
    int currentTestTimer = 0;
    int newServoTarget = 0;
    int min = test.min;
    int max = test.max;
    int x = max - min;

    sleep(2); //Initial delay to allow things to settle down
    //Get initial random servo value
    newServoTarget = min + rand() % x; //returns a pseudo-random integer between 0 and %x, add it to min
    delta = ((float) test.rate / 10.0); //Max Amount by which to change currentServoValue per sec

    if (debug)printf("\33[2K\rtestModeThread() Delta:%.2f\n", delta);

    while (1) {

        //Sit inside this loop until the test has elapsed

        //////////Calculate new servo value based on newServoTarget
        if (test.rate == 0) currentServoValue = (float) newServoTarget; //If 'rate=0' servo changes immediately
            //else if ((delta<1)&&(index==9)) currentServoValue = newServoTarget;   //Fudge for if (delta<1))
        else if ((float) newServoTarget > currentServoValue) { //otherwise change at specified rate
            //target is higher, so increment servo value every second until we reach it
            currentServoValue = currentServoValue + delta; //Increase curentServoValue to meet newServoTarget
            if (currentServoValue > (float) newServoTarget) currentServoValue = (float) newServoTarget; //If overshot, back off   
        } else {
            //target is lower, so decrement servo value every second until we reach it
            currentServoValue = currentServoValue - delta; //Decrease curentServoValue to meet newServoTarget
            if (currentServoValue < (float) newServoTarget) currentServoValue = (float) newServoTarget; //If overshot, back off
        }
        //Has one second (or ten messges) elapsed 
        if (index > 9) {
            index = 0;
            currentTestTimer++; //Increment timer for this value of servoTarget after 1 sec has elapsed
            /////////If current test value period has elapsed, generate a new random newServoTarget
            if ((currentTestTimer > test.duration) || (test.rate == 0)) {
                //Current test is up, so generate a new random servo value
                //A test.rate of zero suggests we want a change every second (i.e we don't want any damping))
                newServoTarget = min + rand() % x; //returns a pseudo-random integer between 0 and %x, add it to min
                if (debug) printf("\33[2K\rNew Servo value: %d\n", newServoTarget);
                currentTestTimer = 0;
            }
            /////////If test is completely finished, close down gracefully
            runTime++; //Increment overall test timer after 1 second has elapsed
            if ((test.totalDuration != 0)&&(runTime > test.totalDuration)) {
                //If (testDuration==0), that suggests an infintite test
                //However, otherwise, monitor runTime and if test is over, exit the program
                printf("\033[7B"); // Move cursor down 7 lines for terminal neatness
                printf("\33[2K\rTest finshed (%d seconds elapsed)\n", runTime);
                printf("\33[2K\rClosing fd's:Control:%d, Status:%d\n",
                        rxUDControlHandle, rxUDPStatusHandle);
                if (rxUDControlHandle != 0) close(rxUDControlHandle); //Close UDP port binding
                if (rxUDPStatusHandle != 0) close(rxUDPStatusHandle); //Close UDP port binding
                zeroAllOutputs(); //Turn off all outputs
                printf("\033[7B"); // Move cursor down 7 lines for terminal neatness
                exit(1);
            }
        }
        ///////////Otherwise send the servo message to the camera 
        //Format a nice string that we can display in the "last message" field of redrawScreen()
        char buf[1024];
        snprintf(buf, 1024, "Total %d/%d. Current: %d/%d  Servo:%d:%.2f>>%d",
                runTime, test.totalDuration, currentTestTimer, test.duration, test.servoNo, currentServoValue, newServoTarget);
        if (debug) printf("\33[2K\rtestModeThread():Current Test:%d,\tElapsed time:%d,\tNew:%d,\tCurrent:%.2f,\tRate:%d\n",
                currentTestTimer, runTime, newServoTarget, currentServoValue, test.rate);
        setLastMessageText(buf);
        //Now actully send the servo command to the specified servo channel
        //char txBuffer[100];
        command txCommand;
        initCmdStruct(&txCommand); //Init command structure
        strlcpy(txCommand.cmd, "SERVO", MSG_FIELD_LENGTH);
        txCommand.arg1 = test.servoNo; //Servo 0
        txCommand.arg2 = (int) currentServoValue; //Value to send


        //Generate string to transmit
        char txBuffer[1024] = {0};
        //bzero(txBuffer, 1024);
        createCmdString(&txCommand, txBuffer, 1024);

        //Transmit the string
        char remoteIP[100]; //Used to store local copy of _status.remoteIP field
        //So that we don't tie up the statusMutex resource
        getRemoteIP(remoteIP, 100); //Get remote IP from _status global struct (in a threadsafe way)
        int control_port = getUDPControlPort(); //Retrieve control port from global _status struct

        char interface[MSG_FIELD_LENGTH] = {0};
        getControlInterface(interface, MSG_FIELD_LENGTH); //Get interface to send message out on
        sendUDP(txBuffer, remoteIP, control_port, interface);

        //if (writeToPipe)writeToOutputPipe(txBuffer); //optionally write too pipe
        if (writeToPipe) {
            char pipeBuff[1024];
            snprintf(pipeBuff, 1024, "TX: %s\n", txBuffer); //Create nice formatted string
            writeToOutputPipe(pipeBuff); //optionally write too pipe  
        }
        //Now update global _status struct
        pthread_mutex_lock(&statusMutex);

        switch (test.servoNo) {
            case 0:
                _status.servo0 = currentServoValue;
                break;
            case 1:
                _status.servo1 = currentServoValue;
                break;
            case 2:
                _status.servo2 = currentServoValue;
                break;
            case 3:
                _status.servo3 = currentServoValue;

                break;
        }
        pthread_mutex_unlock(&statusMutex);

        //Now redraw the screen
        redrawScreen();
        ///////////////////////

        index++; //Increment to next 100mS period
        delay(100); //100mS delay
    }
}

void setOperationMode(int opMode) {

    /*
     * Sets the global operation_mode variable 
     * 
     * 
     * This should be the only function that can modify operation_mode
     */
    operation_mode = opMode;
}

int getOperationMode() {

    /**
     * Returns the current device type (based on the global operation_mode 
     */
    return operation_mode;
}

void getIdentity(char output[], unsigned int outputLength) {

    /**
     * Creates a device name based on the type of device (camera, controller etc), 
     * Linux hostname and serial no of the unit
     * 
     * @param output
     * @param outputLength
     */
    char identity[MSG_FIELD_LENGTH] = {0}; //Temp buffer to hold hostname
    char hostname[MSG_FIELD_LENGTH] = {0};
    int opMode = getOperationMode(); //
    getHostName(hostname, MSG_FIELD_LENGTH); //Get hostname
    int serialNo = getSerialNumber(); //Get serial number
    snprintf(output, MSG_FIELD_LENGTH, "%d_%s_%X",
            opMode, hostname, serialNo); //Construct identity  from hostname+serialNo

}

void setIdentity() {

    /**
     *  Sets the 'identity' field in the global _status struct
     * by way of the identity returned by getIdentity()
     */
    char identity[MSG_FIELD_LENGTH] = {0};
    getIdentity(identity, MSG_FIELD_LENGTH); //Fill array with identity
    pthread_mutex_lock(&statusMutex);
    strlcpy(_status.identity, identity, MSG_FIELD_LENGTH); //Update global struct
    pthread_mutex_unlock(&statusMutex);
}

void broadcastIdentity() {
    /*
     * Sends a status message to the entire subnet, via udp broadcast
     * broadcastIdentity()
 *      -Sends a snapshot of _status via UDP broadcast on _status.statusPort
 *      -Modifies the 'type' field to HELLO
     * 
     */
    status broadcastStatus;
    getStatusSnapshot(&broadcastStatus);    //Take local copy of global _status struct
    
    //Now modify fields from the copied _status struct
    strlcpy(broadcastStatus.type, "HELLO", MSG_FIELD_LENGTH); //Modify 'type' 
    broadcastStatus.timeStamp = time(NULL); //Assign current time
    broadcastStatus.connectedFlag = getConnectedFlag(); //Transmit connecttion status    


    char txBuffer[BUFFER] = {0};
    createStatusString(&broadcastStatus, txBuffer, BUFFER);
    char interface[MSG_FIELD_LENGTH] = {0};
    getControlInterface(interface, MSG_FIELD_LENGTH); //Get interface to send message out on

    if(debug)printf("Sending broadcast: %s:%d:%s\n", interface, broadcastStatus.statusPort, txBuffer);
    sendBroadcastUDP(txBuffer, broadcastStatus.statusPort, interface);

}

void *broadcastIdentityThread(void *arg) {
    /**
     * Monitors getConnectedFlag. 
     * 
     * If connected, sends out a broadcast udp message every 5 seconds
     * If not connected, sends out a broadcast udp message every second
     * 
     * @param void
     * @return void
     */
#define BROADCAST_INTERVAL     5       
#define BROADCAST_INTERVAL_IF_NOT_CONNECTED 1       //Used for camera mode

    int timerPeriod = 0, elapsedTime = 0;
    while (1) {
        if (operation_mode == CAMERA_MODE) {
            if (getConnectedFlag() == 1) timerPeriod = BROADCAST_INTERVAL;
            else timerPeriod = BROADCAST_INTERVAL_IF_NOT_CONNECTED;

            if (elapsedTime >= timerPeriod) { //Fires every time timerPeriod reached

                broadcastIdentity(); //Broadcast status message to entire subnet
                elapsedTime = 0;
            }
        }
        if (operation_mode == CONTROL_MODE || operation_mode == TEST_MODE || operation_mode == AUTO_DISCOVER) {
            //i.e, anything but a camera
            timerPeriod = BROADCAST_INTERVAL;

            if (elapsedTime >= timerPeriod) { //Fires every time timerPeriod reached

                broadcastIdentity(); //Broadcast status message to entire subnet
                elapsedTime = 0;
            }
        }
        sleep(1);
        elapsedTime++;
    }
}

void main(int argc, char** argv) {

    //Initialise global status struct
    initStatusStruct(&_status);
    printf("Checking _status.remoteIP: %s\n", _status.remoteIP);
    /*STEP 1: Parse the incoming arguments
     * These can be:-
     * a) No arguments - Camera mode, listening ports set to BASE_PORT (control) and BASE_PORT+1 (status)
     * b) -c [port] -Camera mode, listening udp ports set to [port]
     * c) -r [ip address] [port] - Controller mode, connecting to [ip address] on ports [port] (control) 
     *          and [port+1] (status)
     * d) -t testmode - generates random servo messages
     * e) -d debug mode
     * f) -p output to pipe
     * g) -a //Automatic discover controller mode, no control port supplied. Default ports used
     * h) -a [port]     Auto discover controller mode, control ports supplied
     * 
     */
    servoTest _servoTest; //Create a struct to hold test parameters
    if (argc > 1) {
        //Get non camera/related checking out of the way first
        if (match(argv[1], "d") != -1) {
            printf("debug\n");
            debug = 1;
        }
        if (match(argv[1], "p") != -1) {
            printf("pipe\n");
            writeToPipe = 1;
        }

        char *ptr;
        if ((argc == 2) && match(argv[1], "c") != -1) {
            //-c camera mode specified but no listening ports supplied, so use defaults
            setOperationMode(CAMERA_MODE);
            setControlPort(CONTROL_PORT);
            setStatusPort(STATUS_PORT);


        } else if ((argc == 3) && match(argv[1], "c") != -1) {
            //-c port
            setOperationMode(CAMERA_MODE);

            //Set udp control message port
            if (setControlPort((int) strtol(argv[2], NULL, 10)) == -1) {
                printf("Invalid control port specified, using default control: %d and status: %d ports\n", CONTROL_PORT, STATUS_PORT);
                setControlPort(CONTROL_PORT);
                setStatusPort(STATUS_PORT);
            } else setStatusPort((int) strtol(argv[2], NULL, 10) + 1); //Specified control port valid, so set udp status message port (=control port +1)


            pthread_mutex_lock(&statusMutex); //Lock _status mutex
            printf("Camera Mode, UDP ports: [control:%d , status:%d]\n", _status.controlPort, _status.statusPort);
            pthread_mutex_unlock(&statusMutex); //Unlock

        } else if ((argc == 4) && match(argv[1], "r") != -1) {
            //-r ip-addr, port
            setOperationMode(CONTROL_MODE);
            //Now parse and verify supplied IP address is valid

            if (setRemoteIP(argv[2]) == 1) {
                //Address successfully validated. 

                //Set udp control message port
                if (setControlPort((int) strtol(argv[3], NULL, 10)) == -1) {
                    printf("Invalid control port specified, using default control: %d and status: %d ports\n", CONTROL_PORT, STATUS_PORT);
                    setControlPort(CONTROL_PORT);
                    setStatusPort(STATUS_PORT);
                } else setStatusPort((int) strtol(argv[3], NULL, 10) + 1); //Specified control port valid, so set udp status message port (=control port +1)



                pthread_mutex_lock(&statusMutex);
                printf("Controller Mode: Connecting to %s UDP ports: [control:%d , status:%d]\n",
                        _status.remoteIP, _status.controlPort, _status.statusPort);
                pthread_mutex_unlock(&statusMutex);
            } else {
                printf("Invalid remote IP supplied. Exiting...\n");
                exit(1);
            }

        } else if ((argc == 2) && match(argv[1], "a") != -1) {
            //Automatic discover controller mode, no control port supplied
            //-a 
            setOperationMode(AUTO_DISCOVER);
            setControlPort(CONTROL_PORT);
            setStatusPort(STATUS_PORT);
            //listens out on the CONTROL_PORT for a "HELLO" message
            //printf("Auto mode, no port specified\n");
            pthread_mutex_lock(&statusMutex); //Lock _status mutex
            printf("Auto Discover Mode, UDP ports: [control:%d , status:%d]\n", _status.controlPort, _status.statusPort);
            pthread_mutex_unlock(&statusMutex); //Unlock


        } else if ((argc == 3) && match(argv[1], "a") != -1) {
            //Automatic discover controller mode, control port supplied
            //-a [control port]
            //listens out on the supplied control port for a "HELLO"
            setOperationMode(AUTO_DISCOVER);
            if (setControlPort((int) strtol(argv[2], NULL, 10)) == -1) {
                printf("Invalid control port specified, using default control: %d and status: %d ports\n", CONTROL_PORT, STATUS_PORT);
                setControlPort(CONTROL_PORT);
                setStatusPort(STATUS_PORT);
            } else setStatusPort((int) strtol(argv[3], NULL, 10) + 1); //Specified control port valid, so set udp status message port (=control port +1)
            pthread_mutex_lock(&statusMutex); //Lock _status mutex
            printf("Auto Discover Mode, UDP ports: [control:%d , status:%d]\n", _status.controlPort, _status.statusPort);
            pthread_mutex_unlock(&statusMutex); //Unlock

        } else if ((argc == 10) && match(argv[1], "t") != -1) {
            //-t means test mode
            //A modifed controller mode that generates random messages
            /*Supply the following args
             * ip addr,
             * port
             * length of test
             * min
             * max
             * length between changes
             * rate of change (% per second)
             * which servo (0-3)
             * 
             * eg: 
             */
            //operation_mode = TEST_MODE; //Specify test mode
            setOperationMode(TEST_MODE);
            //Now parse and verify supplied IP address is valid

            if (setRemoteIP(argv[2]) == 1) { //Extract ip address (2nd supplied arg)
                //Address successfully validated.
                //Set udp control message port
                if (setControlPort((int) strtol(argv[3], NULL, 10)) == -1) { //Extract port (3rd supplied arg)
                    printf("Invalid control port specified, using default %d\n", CONTROL_PORT);
                    setControlPort(CONTROL_PORT);
                }

                //set udp status message port (=control port +1)
                if (setStatusPort((int) strtol(argv[3], NULL, 10) + 1) == -1) { //Currently, status port = control port +1
                    printf("Invalid status message port specified, using default %d\n", STATUS_PORT);
                    setStatusPort(STATUS_PORT);
                }
            } else {
                printf("Invalid remote IP supplied. Exiting...\n");
                exit(1);
            }

            pthread_mutex_lock(&statusMutex);
            printf("Test Mode: Connecting to %s UDP ports: [control:%d , status:%d]\n",
                    _status.remoteIP, _status.controlPort, _status.statusPort);
            pthread_mutex_unlock(&statusMutex);


            _servoTest.totalDuration = (int) strtol(argv[4], NULL, 10);
            _servoTest.min = (int) strtol(argv[5], NULL, 10);
            _servoTest.max = (int) strtol(argv[6], NULL, 10);
            _servoTest.duration = (int) strtol(argv[7], NULL, 10);
            _servoTest.rate = (int) strtol(argv[8], NULL, 10);
            _servoTest.servoNo = (int) strtol(argv[9], NULL, 10);

            printf("Test mode selected:\n");
            if (_servoTest.totalDuration == 0)
                printf("Total Duration: \t\tIndefinite\n");
            else
                printf("Total Duration: \t\t%d\n", _servoTest.totalDuration);
            printf("Min: \t\t\t\t%d\n", _servoTest.min);
            printf("Max: \t\t\t\t%d\n", _servoTest.max);
            printf("Duration between changes: \t%d\n", _servoTest.duration);
            if (_servoTest.rate == 0)
                printf("Rate of change (%% per second):\tMaximum\n");
            else
                printf("Rate of change (%% per second):\t%d\n", _servoTest.rate);
            printf("Servo channel: \t\t\t%d\n", _servoTest.servoNo);




        } else if (match(argv[1], "u") != -1) {
            //Print usage instructions then quit.
            printf("Possible options are:-\n");
            printf("\tNo args:\n");
            printf("\t\tcamera mode. Default control port: %d, Default status port: %d\n", CONTROL_PORT, STATUS_PORT);

            printf("\n\tc:\n");
            printf("\t\tcamera mode. Default control port: %d, Default status port: %d\n", CONTROL_PORT, STATUS_PORT);

            printf("\n\tc [port]\n");
            printf("\t\tCamera mode. UDP listening control port=[port]. Status port=[port]+1\n");

            printf("\n\td\n");
            printf("\t\tDebug mode - verbose logging\n");

            printf("\n\tp\n");
            printf("\t\tWrite output info to fifo at /dev/camerastatus\n");

            printf("\n\tr [ip address] [port]\n");
            printf("\t\tRemote Controller mode: Attempt to connect to device at [ip address] on udp port [port]\n");

            printf("\n\ta\n");
            printf("\t\tAuto discover remote controller mode. Using default udp control (%d) and status (%d) ports\n",
                    CONTROL_PORT, STATUS_PORT);

            printf("\n\ta [port]\n");
            printf("\t\tAuto discover remote controller mode. Using supplied udp control ([port]) and status ([port]+1) ports\n");

            printf("\n\tt [ip addr] [port] [test length] [min] [max] [time between changes] [rate] [servo no]\n");
            printf("\t\tServo test message mode: Generates random servo control messages\n");
            printf("\n\t\t[ip addr]\tIP address of camera to connect to\n");
            printf("\t\t[port]\t\tUDP control port of camera to connect to\n");
            printf("\t\t[length]\tLength of test (in seconds). '0' means run forever\n");
            printf("\t\t[min]\t\tLower limit for random servo value (as a %%), minimum=0\n");
            printf("\t\t[max]\t\tUpper limit for random servo value (as a %%), maximum=100\n");
            printf("\t\t[time between changes]\tLength of time (in seconds) to hold each target servo value, before generating a new value\n");
            printf("\t\t[rate]\t\tThe maximum rate of change (as a %% per second) that the  servo value to be sent can change (a value of 100\n");
            printf("\t\t\t\twould mean that the servo control message could swing from 0%% to 100%% in 1 second)\n");
            printf("\t\t[servo no]\tWhich of the four servo outputs you wish to control (0,1,2 or 3)\n");

            printf("\nSending control messages via /dev/cameracontrol\n");
            printf("---------------------------------------\n");
            printf("\tMessage format:  cmd,status,arg1,arg2,arg3,text,identity\n");
            printf("\n\n\tGeneral example: echo SERVO,[status ignored],[servo no],[%% value (0-100),[arg 3 ignored],[text ignored],[identity ignored] > /dev/cameracontrol\n");
            printf("\n\tSpecific example: 'echo SERVO,*,0,50 > /dev/cameracontrol' will set servo output 0 to 50%%. The second argument ('*')  is ignored by this" ""
                    " particular control message but you have to pad it/insert some kind of character otherwise the message"
                    " parsing will fail (it's expecting to delimit based on commas).\n");

            exit(1);
        }

    } else {
        printf("No or bad args supplied\n");
        setOperationMode(CAMERA_MODE);
        setControlPort(CONTROL_PORT);
        setStatusPort(STATUS_PORT);
        pthread_mutex_lock(&statusMutex); //Lock _status mutex
        printf("Camera Mode, UDP ports: [control:%d , status:%d]\n", _status.controlPort, _status.statusPort);
        pthread_mutex_unlock(&statusMutex); //Unlock
    }
    //Main program starts here
    /////////

    int retVar = establishControlInterface();
    char interface[MSG_FIELD_LENGTH] = {0};
    getControlInterface(interface, MSG_FIELD_LENGTH);
    printf("retVar: %d, %s\n", retVar, interface);


    //retVar = setRemoteIP("1.2.3.4");
    setIdentity();

    pthread_mutex_lock(&statusMutex);
    printf("_status.localIP: %s\n", _status.localIP);
    printf("_status.remoteIP: %s\n", _status.remoteIP);
    printf("_status.controlPort: %d\n", _status.controlPort);
    printf("_status.cstatusPort: %d\n", _status.statusPort);
    printf("_status.identity(set): %s\n", _status.identity);
    printf("_status.type(set): %s\n", _status.type);

    pthread_mutex_unlock(&statusMutex);



    //exit(1);
    /////////
    /*
    char interfaceList[20][2][20];
    int ret = getLocalIPaddr(interfaceList, 20); //Display local ethernet interfaces
    printf("No of interfaces found:%d\n", ret);
    for (int n = 0; n < ret; n++)
        printf("%d: %s %s\n", n, interfaceList[n][0], interfaceList[n][1]);
    //If getLocalIPAddress returns >1:-
    //interfaceList[0][1]=loopback interface addr
    //interfaceList[1][1]=first proper interface addr
    //If so, safely update global status struct (_status)
    if (ret > 1) {
        pthread_mutex_lock(&statusMutex);
        strlcpy(_status.localIP, interfaceList[1][1],MSG_FIELD_LENGTH);
        pthread_mutex_unlock(&statusMutex);
    }
     */

    if (wiringPiSetup() == -1) { //Check wiringPi is up and running
        printf("WiringPi not running. Exiting... ");
        exit(1);

    }
    //Initialise all GPO outputs to zero

    for (int x = 0; x < NO_OF_GPOs; x++) digitalWrite(gpoPinArray[x], LOW);

    //Now start threads, depending upon whether controller or camera mode
    if (operation_mode == CAMERA_MODE) {


        //Start servod with '--p1pins=7,11,12,13'
        startServodV2("--p1pins=7,11,12,13");
        /*
        pid_t servodPID = startServod("--p1pins=7,11,12,13", getpid());
        if (servodPID == -1) {
            fprintf(stderr, "failed to fork() servod process\n");
            exit(1);
        } else if (servodPID == 0) {
            printf("cameraremote.c: startServod() returned '0'. Exiting...\n");
            exit(1);
        }
        printf("PID of CameraRemote process is:%d\n", getpid());
        printf("PID of servod is: %d\n", (int) servodPID);
         */
        //For safety, set all servo outputs to zero
        servodControl(0, 0);
        servodControl(1, 0);
        servodControl(2, 0);
        servodControl(3, 0);

    }

    //It seems like you have to start servod BEFORE you set up the KILL/USR1
    //signal handling. Otherwise it seems to stop the system command
    //"killall servod" from working, and you end up with multiple 
    //instances of servod

    //Set main thread's signal mask to block all signals
    //All other threads will inherit the mask and have it blocked too
    //We manually set the flags in shutDownThread() to override these defaults

    sigset_t sigsToBlock;
    sigemptyset(&sigsToBlock); //initialise and empty the signal set
    sigaddset(&sigsToBlock, SIGTERM); //Add SIGTERM to our signal set
    sigaddset(&sigsToBlock, SIGINT); //Add SIGINT to our signal set
    sigaddset(&sigsToBlock, SIGUSR1); //Add SIGUSR1 to our signal set
    sigaddset(&sigsToBlock, SIGUSR2); //Add SIGUSR2 to our signal set
    sigaddset(&sigsToBlock, SIGKILL); //Add SIGKILL to our signal set

    pthread_sigmask(SIG_BLOCK, &sigsToBlock, NULL); //Apply the mask to this and daughter threads


    //Now start threads one by one
    if ((operation_mode == CAMERA_MODE) || (operation_mode == CONTROL_MODE)
            || (operation_mode == TEST_MODE) || operation_mode == AUTO_DISCOVER) {

        pthread_t _shutDownThread;
        if (pthread_create(&_shutDownThread, NULL, shutDownThread, NULL)) {
            printf("main(): Error creating shutDownThread\n");
        }
        //start controller-listening thread 
        pthread_t _UDPControlMonitorThreadV2;
        if (pthread_create(&_UDPControlMonitorThreadV2, NULL, UDPControlMonitorThreadV2, NULL)) {
            printf("main(): Error creating UDPControlMonitorThreadV2\n");
        }
        //start UDPStatusMonitorThread
        pthread_t _UDPStatusMonitorThreadV2;
        if (pthread_create(&_UDPStatusMonitorThreadV2, NULL, UDPStatusMonitorThreadV2, NULL)) {
            printf("main(): Error creating UDPStatusMonitorThreadV2\n");
        }
        //Start watchdog thread
        pthread_t _watchdogThread;
        if (pthread_create(&_watchdogThread, NULL, watchdogThread, NULL)) {
            printf("main(): Error creating watchdogThread\n");
        }

        //start heartbeatThread thread
        pthread_t _heartbeatThread;
        if (pthread_create(&_heartbeatThread, NULL, heartbeatThread, NULL)) {
            printf("main(): Error creating heartbeatThread\n");
        }

        //Start memory device monitoring thread
        //(so we can inject control messages locally to FIFO /dev/cameracontrol)
        pthread_t _monitorMemDevThread;
        if (pthread_create(&_monitorMemDevThread, NULL, monitorMemDevThread, NULL)) {
            printf("main(): Error creating memory device thread\n");
        }
        /*
                //Start GPIOMonitorThread (interrupt driven))
                pthread_t _GPIOMonitorThread;
                if (pthread_create(&_GPIOMonitorThread, NULL, GPIOMonitorThread, NULL)) {
                    printf("main(): Error creating GPIOMonitorThread\n");
                }
         */
        //Start GPIOMonitorThread (uses polling)
        pthread_t _GPIOMonitorThreadV2;
        if (pthread_create(&_GPIOMonitorThreadV2, NULL, GPIOMonitorThreadV2, NULL)) {
            printf("main(): Error creating GPIOMonitorThreadV2\n");
        }
        //Start *broadcastIdentityThread
        pthread_t _broadcastIdentityThread;

        if (pthread_create(&_broadcastIdentityThread, NULL, broadcastIdentityThread, NULL)) {
            printf("main(): Error creating broadcastIdentityThread\n");
        }

        //Now start threads specific to certain modes

        if (operation_mode == CONTROL_MODE || operation_mode == AUTO_DISCOVER) {

            //start handset/UI monitoring thread  
            pthread_t _controlSurfaceThread;
            if (pthread_create(&_controlSurfaceThread, NULL, controlSurfaceThread, NULL)) {
                printf("main(): Error creating controlSurfaceThread\n");
            }
        }

        if (operation_mode == TEST_MODE) {
            //servoTest struct holds the test parameters
            //_servoTest struct is populated by the arguments supplied at startup
            //_servoTest contents then copied to a local struct that can be passed to 
            //the testModeThread

            servoTest *_test = malloc(sizeof (*_test)); //Reserve mem for servoTest struct
            memcpy(_test, &_servoTest, sizeof (servoTest)); //Take copy of exising _servoTest struct
            //This is the version we will pass to the 
            //testModeThread())
            //Start test thread
            pthread_t _testModeThread;
            if (pthread_create(&_testModeThread, NULL, testModeThread, (void*) _test)) {
                printf("main(): Error creating testModeThread\n");
            }
            pthread_detach(_testModeThread); //Don't care what happens to thread afterwards

        }


        while (1); //Infinite loop
    }

    return;
}