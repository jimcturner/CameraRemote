/*
 *      Collection of utility functions.
 *      James Turner. Nov 2016
 *      V1.0
 * 
 * strlcpy() copy a string and always null terminate 
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define UTILITY_FIELD 1024

size_t strlcpy(char* dst, const char* src, size_t bufsize) {
    /*
     * Safer version of strncpy() which always termainates the copied string with
     * the null character (strncpy() doesn't terminate a string)
     */
    size_t srclen = strlen(src);
    size_t result = srclen; /* Result is always the length of the src string */
    if (bufsize > 0) {
        if (srclen >= bufsize)
            srclen = bufsize - 1;
        if (srclen > 0)
            memcpy(dst, src, srclen);
        dst[srclen] = '\0';
    }
    return result;
}

int getSerialNumber() {
    /*
     * Retrieves the serial number string from /cat/proc
     * Note: serial number on the Pi is expressed as a Hex number
     * Sample code:-
     *          int serialNo=getSerialNumber();
     *          printf("Serial number: %X\n",serialNo);
     */
    char output[UTILITY_FIELD];
    memset(output, 0, UTILITY_FIELD);
    //sysCmd2("grep -Po '^Serial\\s*:\\s*\\K[[:xdigit:]]{16}' /proc/cpuinfo", output, FIELD);
    sysCmd2("cat /proc/cpuinfo | grep Serial | cut -d ':' -f 2", output, UTILITY_FIELD);
    char *ptr;
    unsigned int serialNumber = (int) strtol(output, &ptr, 16); //Serial number is written in 
    //printf("getSerialNumber(): raw: %s, integer:%d, as Hex: %x\n", output, serialNumber, serialNumber);
    return serialNumber;
}

int getHostName(char output[], int outputLength) {
    memset(output, 0, outputLength);
    int ret = sysCmd2("hostname", output, UTILITY_FIELD); //Run 'hostname' in terminal, Copy response to output[]]
    output[ret - 1] = 0; //Removes a wierd carriage return character appended by 'hostname'

    return ret - 1; //Return length of returned name (not including null char)                      
}

int isWlan0Present() {
    /*
     *  This function determines whether wlan0 interface is installed.
     * Returns '1' if it is, or '0' if not detected
     */
    char commandResponse[UTILITY_FIELD] = {0};
    char interfaceList[20][2][20]; //Can store 20 addresses
    int noOfInterfaces = listAllInterfaces(interfaceList, 20); //Display all local ethernet interfaces
    int l;
    for (l = 0; l < noOfInterfaces; l++) {
        //Iterate through all the interface names to see if we have one called 'wlan1'
        if (strcmp("wlan0", interfaceList[l][0]) == 0) {
            //If strcmp returns '0', match found
            printf("isWlan0Present():Interface wlan0 installed\n");
            return 1;
            //l = noOfInterfaces; //Force for() loop to terminate
        }
    }
    printf("isWlan0Present(): wlan0 not present\n");
    return 0;


}




