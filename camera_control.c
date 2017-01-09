/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 * 
 * Overview of the functions:-
 * 
 * initStatusStruct(), initCmdStruct()
 * -----------------------------------
 * These functions populate a supplied 'empty' struct with default values.
 * They should be called before a the struct is actually used because they'll
 * ensure that the struct is not corrupted with any previous remnants from memory.
 * 
 * createStatusString(), createCmdString()
 * ---------------------------------------
 * These functions serialise all the fields present in either 
 * status or cmd structures to allow them to be sent as a string of chars
 * 
 * parseStatusString(), parseCmdString()
 * -------------------------------------
 * These functions take a string (delimited by the delimiter character specified
 * by DELIMITER), extract the contents and populate a supplied status or cmd struct
 * 
 * 
 * 
 * HOW TO ADD ADDITIONAL FIELDS TO status AND cmd STRUCTS......
 * You need to:-
 * 
 * Step 1)
 * Modify the status and cmd structs in camera_control.h
 * 
 * Step2)
 * Modify the following functions:-
 * 
 * a) initStatusStruct() or initCmdStruct()
 * b) createStatusString() or createCmdString()
 * c) parseStatusString() or parseStatusString()
 * 
 *  
 */

#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include "camera_control.h"
#include "iptools2.3.h"
//#include <signal.h>


#define DELIMITER ","
#define SERVOD_CONTROL_STRING_SIZE 256

int createStatusString(status *camStatus, char output[], int outputLength) {

    /*
     * This function takes a pointer to the self-defined structure 'camStatus'
     * and also a pointer to an output buffer.
     * 
     * It will then extract the values from the structure and create an
     * appropriately formatted/delimited output string.
     * 
     * Note: It depends upon #include "camera_control.h" for the 
     * command struct 
     * 
     *First, check to see if supplied output buffer is long enough
     *Rough guess. We have 12 Integers and 1 Long in our status struct
     *plus some comma delimiters.
     *
     * 
     *According to this list, the ranges are:-
     * int:  32 bits, -2,147,483,648 to 2,147,483,647 (11 digits with sign)
     * unsigned long: 64 bits, 0 to 18,446,744,073,709,551,615 (20 digits)
     * No of commas, 14)
     * 
     * This Function returns:
     * 0: supplied output buffer insufficient length
     * 1: No errors
     */

    //if (outputLength < (12 * 12) + 40) return -1;

    //Check Supplied output buffer is large enough
    int maxBufferLength = ((NO_STATUS_ARGS * MSG_FIELD_LENGTH)
            +(NO_STATUS_ARGS + 1)); //Allow for commas either side
    if (outputLength < maxBufferLength) {
        printf("createStatusString(): supplied output buffer (%d) too small. Should be >=%d\n", outputLength, maxBufferLength);
        return 0;
    }
    
    //Now check the text fields for instances of the delimiter. If present, replace them with a !
    //This is essential, otherwise the strok() comma delimiting will fail and the message won't correctly decode
    //Do this for all text fields
    const char delimiterChar[2] = DELIMITER;    //Copy delimiter character into an array
    for(int n=0;n<MSG_FIELD_LENGTH;n++){
        if(camStatus->localIP[n]==delimiterChar[0])       //Compare each char in turn with the delimiter
            camStatus->localIP[n]='!';
        if(camStatus->remoteIP[n]==delimiterChar[0])       
            camStatus->remoteIP[n]='!';
        if(camStatus->type[n]==delimiterChar[0])       
            camStatus->type[n]='!';
        if(camStatus->identity[n]==delimiterChar[0])       
            camStatus->identity[n]='!';
        if(camStatus->remoteIdentity[n]==delimiterChar[0])       
            camStatus->remoteIdentity[n]='!';
        }
    snprintf(output, outputLength, ",%s,%s" //localIP, remoteIP
            ",%d,%d,%d,%d" //servo0-3
            ",%d,%d,%d,%d" //gpi0-3
            ",%d,%d,%d,%d" //gpo0-3
            ",%ld" //timestamp
            ",%s" //type
            ",%d,%d,%d,%d,%d" //gpi4,gpi5,gpi6,gpo4,gpo5
            ",%s" //identity
            ",%d,%d,%d" //control port, Status port, http port
            ",%d" //connectedFlag
            ",%s,", //remoteIdentity
            camStatus->localIP,
            camStatus->remoteIP,
            camStatus->servo0,
            camStatus->servo1,
            camStatus->servo2,
            camStatus->servo3,
            camStatus->gpi0,
            camStatus->gpi1,
            camStatus->gpi2,
            camStatus->gpi3,
            camStatus->gpo0,
            camStatus->gpo1,
            camStatus->gpo2,
            camStatus->gpo3,
            camStatus->timeStamp,
            camStatus->type,
            camStatus->gpi4,
            camStatus->gpi5,
            camStatus->gpi6,
            camStatus->gpo4,
            camStatus->gpo5,
            camStatus->identity,
            camStatus->controlPort,
            camStatus->statusPort,
            camStatus->httpPort,
            camStatus->connectedFlag,
            camStatus->remoteIdentity);
    return 1;
}

int parseStatusString(status *camStatus, char input[]) {
    /*
     * This function takes a pointer to the self-defined structure 'camStatus'
     * and a delimited string,  It then populates the structure with the 
     * appropriate contents from the supplied input string array
     * 
     * This function returns: 
     * 
     * 0: One of the recovered fields was too long (LENGTH! inserted instead)
     * 1: Success, no errors
     */
    //Create array of chars to temporarily store results  

    char argArray[NO_STATUS_ARGS][MSG_FIELD_LENGTH] = {0}; //21 is the largest number representation
    const char s[2] = DELIMITER;
    char *token;
    int index = 0;
    int fieldTooLong = 1;

    /* isolate the first argument */
    token = strtok(input, s);
    while ((token != NULL) && (index < (NO_STATUS_ARGS))) {
        //token holds the argument isolated from the string.
        //Check the argument isn't longer than the array provided
        //If it is, write "LENGTH!" in it's place
        if (strlen(token) < MSG_FIELD_LENGTH) {
            sprintf(argArray[index], "%s", token);
        } else {
            sprintf(argArray[index], "%s", "LENGTH!");
            fieldTooLong = 0; //Catch error
        }
        //printf("parseStatusString() %d:%s\n", index, argArray[index]);
        /* walk through other arguments */
        token = strtok(NULL, s);
        index++;
    }

    //Firstly clear existing values from camStatus structure
    memset(camStatus, 0, sizeof (camStatus));

    //Now copy array contents to the command data type we have created
    char *ptr; //Stores any textual part of the recovered string. Not used

    strcpy(camStatus->localIP, argArray[0]);
    strcpy(camStatus->remoteIP, argArray[1]);
    camStatus->servo0 = (int) strtol(argArray[2], &ptr, 10);
    camStatus->servo1 = (int) strtol(argArray[3], &ptr, 10);
    camStatus->servo2 = (int) strtol(argArray[4], &ptr, 10);
    camStatus->servo3 = (int) strtol(argArray[5], &ptr, 10);
    camStatus->gpi0 = (int) strtol(argArray[6], &ptr, 10);
    camStatus->gpi1 = (int) strtol(argArray[7], &ptr, 10);
    camStatus->gpi2 = (int) strtol(argArray[8], &ptr, 10);
    camStatus->gpi3 = (int) strtol(argArray[9], &ptr, 10);
    camStatus->gpo0 = (int) strtol(argArray[10], &ptr, 10);
    camStatus->gpo1 = (int) strtol(argArray[11], &ptr, 10);
    camStatus->gpo2 = (int) strtol(argArray[12], &ptr, 10);
    camStatus->gpo3 = (int) strtol(argArray[13], &ptr, 10);
    camStatus->timeStamp = (time_t) strtol(argArray[14], &ptr, 10);
    strlcpy(camStatus->type, argArray[15], MSG_FIELD_LENGTH);
    camStatus->gpi4 = (int) strtol(argArray[16], &ptr, 10);
    camStatus->gpi5 = (int) strtol(argArray[17], &ptr, 10);
    camStatus->gpi6 = (int) strtol(argArray[18], &ptr, 10);
    camStatus->gpo4 = (int) strtol(argArray[19], &ptr, 10);
    camStatus->gpo5 = (int) strtol(argArray[20], &ptr, 10);
    //camStatus->identity = (int) strtol(argArray[21], &ptr, 10);
    strlcpy(camStatus->identity, argArray[21], MSG_FIELD_LENGTH);
    camStatus->controlPort = (int) strtol(argArray[22], &ptr, 10);
    camStatus->statusPort = (int) strtol(argArray[23], &ptr, 10);
    camStatus->httpPort = (int) strtol(argArray[24], &ptr, 10);
    camStatus->connectedFlag = (int) strtol(argArray[25], &ptr, 10);
    strlcpy(camStatus->remoteIdentity, argArray[26], MSG_FIELD_LENGTH);
    return fieldTooLong;
}

int initStatusStruct(status *_status) {
    /*
     * This function will fill a supplied status struct
     * with zeroes and "-" as a default string in the 
     * char fields.
     * It will also assign the current system time
     * to timestamp
     * 
     * It will return -1 if the supplied struct is NULL
     * Otherwise it will return 1;
     */
    if (_status == NULL) return -1;
    memset(_status, 0, sizeof (status));
    strcpy(_status->localIP, "0.0.0.0");
    strcpy(_status->remoteIP, "0.0.0.0");
    _status->servo0 = 0;
    _status->servo1 = 0;
    _status->servo2 = 0;
    _status->servo3 = 0;
    _status->gpi0 = 0;
    _status->gpi1 = 0;
    _status->gpi2 = 0;
    _status->gpi3 = 0;
    _status->gpo0 = 0;
    _status->gpo1 = 0;
    _status->gpo2 = 0;
    _status->gpo3 = 0;
    _status->timeStamp = time(NULL); //Assign current time
    strlcpy(_status->type, "-", MSG_FIELD_LENGTH); //type of message
    _status->gpi4 = 0;
    _status->gpi5 = 0;
    _status->gpi6 = 0;
    _status->gpo4 = 0;
    _status->gpo5 = 0;
    //_status->identity = 0;
    strlcpy(_status->identity, "-", MSG_FIELD_LENGTH);
    _status->controlPort = 0;
    _status->statusPort = 0;
    _status->httpPort = 0;
    _status->connectedFlag = 0;
    strlcpy(_status->remoteIdentity, "-", MSG_FIELD_LENGTH);

    return 1;
}

int initCmdStruct(command *cmd) {
    /*
     * This function will fill a supplied command struct
     * with zeroes and "-" as a default string in the 
     * char fields.
     * 
     * It will also assign the current system time
     * to timestamp
     * 
     * If supplied struct is NULL, returns -1,
     * Otherwise returns 1
     */
    if (cmd == NULL) return -1;
    memset(cmd, 0, sizeof (cmd));
    strlcpy(cmd->cmd, "-", MSG_FIELD_LENGTH);
    strlcpy(cmd->status, "-", MSG_FIELD_LENGTH);
    char *ptr; //Stores the textual part of the recovered string. Not used
    cmd->arg1 = 0;
    cmd->arg2 = 0;
    cmd->arg3 = 0;
    strlcpy(cmd->text, "-", MSG_FIELD_LENGTH);
    strlcpy(cmd->identity, "-", MSG_FIELD_LENGTH);
    cmd->timeStamp = time(NULL); //Assign current time
    return 1;

}

int createCmdString(command *cmd, char output[], int outputLength) {
    /*
     * This function takes a pointer to the self-defined structure 'commmand'
     * and also a pointer to a buffer.
     * 
     * It will then extract the values from the structure and create an
     * appropriately formatted/delimited control string.
     * 
     * Note: It depends upon #include "camera_control.h" for the 
     * command struct
     * 
     * This Function returns:
     * 0: supplied output buffer insufficient length
     * 1: No errors 
     * 
     */


    //if (outputLength < (NO_CTRL_ARGS * MSG_FIELD_LENGTH) + 20) return -1;
    //Check Supplied output buffer is large enough
    int maxBufferLength = ((NO_CTRL_ARGS * MSG_FIELD_LENGTH)
            +(NO_CTRL_ARGS + 1)); //Allow for commas either side
    if (outputLength < maxBufferLength) {
        printf("createStatusString(): supplied output buffer too small. Should be >=%d\n", maxBufferLength);
        return 0;
    }
    //Now check the text fields for instances of the delimiter. If present, replace them with a !
    //This is essential, otherwise the strok() comma delimiting will fail and the message won't correctly decode
    //Do this for all text fields
    const char delimiterChar[2] = DELIMITER;    //Copy delimiter character into an array
    for(int n=0;n<MSG_FIELD_LENGTH;n++){
        if(cmd->cmd[n]==delimiterChar[0])       //Compare each char in turn with the delimiter
            cmd->cmd[n]='!';
        if(cmd->status[n]==delimiterChar[0])
            cmd->status[n]='!';
        if(cmd->text[n]==delimiterChar[0])
            cmd->text[n]='!';
        if(cmd->identity[n]==delimiterChar[0])
            cmd->identity[n]='!';
        }
    sprintf(output, ",%s,%s,%d,%d,%d,%s,%s,%ld,",
            cmd->cmd, cmd->status, cmd->arg1, cmd->arg2, cmd->arg3, cmd->text,
            cmd->identity, cmd->timeStamp);
    return 1;
}

int parseCmdString(command *cmd, char input[]) {
    /*
     * This function takes a pointer to the self-defined structure 'commmand'
     * and a delimited string,  It then populates the structure with the 
     * appropriate contents from the supplied input string array
     * 
     * strtok() does the actual delimiting
     *  
     * Delimited values 1,2 and 6 are copied straight into the corresponding 
     * structure slots (cmd, status and text).
     * 
     * If a field is longer than MSG_FIELD_LENGTH, that field is populated with 
     * the error message 'LENGTH!', the function will continue but -1
     * will be returned  
     * 
     * Delimited values 3,4 and 5 are expected to be integers. 
     * 
     * strtol() takes these integers encapsulated within the string and 
     * attempts to convert them to a long.
     * 
     * A non-number present in the arg1, arg2
     * or arg3 fields (fields 3,4 and 5) should result in strtol() 
     * evaluating to zero. Floats seem to be rounded down
     *  * 
     * For an example of how to call this function see below
     * 
     * Fields in input[] beyond 'NO_CTRL_ARGS' are ignored
     * If the no of fields is less, only the first n fields will
     * be populated
     * 
     * Note: It depends upon #include "camera_control.h" for the 
     * command struct
     */

    /*
     * //Sample main routine to make use of parseCmdString()
     *void main(int argc, char** argv) {
        char cmdString[] = "string1,string2,1234,-5678,78.9,string3";

        command cmd;
        int output = parseCmdString(&cmd, cmdString);
        //strcpy(cmd.cmd,"test");
        printf("cmd.cmd: %s\n", cmd.cmd);
        printf("cmd.status: %s\n", cmd.status);
        printf("cmd.arg1: %d\n", cmd.arg1);
        printf("cmd.arg2: %d\n", cmd.arg2);
        printf("cmd.arg3: %d\n", cmd.arg3);
        printf("cmd.text: %s\n", cmd.text);
    }
     */
    //Create array of chars to temporarily store results
    char argArray[NO_CTRL_ARGS][MSG_FIELD_LENGTH + 1] = {0};
    //input[] = "Here,is,0,1,2,my test string";
    const char s[2] = DELIMITER;
    char *token;
    int index = 0;
    int fieldTooLong = 0;

    /* isolate the first argument */
    token = strtok(input, s);

    while ((token != NULL) && (index < (NO_CTRL_ARGS))) {
        //token holds the argument isolated from the string.
        //Check the argument isn't longer than the array provided
        //If it is, write "LENGTH!" in it's place
        if (strlen(token) < MSG_FIELD_LENGTH) {
            sprintf(argArray[index], "%s", token);
        } else {
            sprintf(argArray[index], "%s", "LENGTH!");
            fieldTooLong = 1; //Catch error
        }
        //printf("%d:%s\n", index, argArray[index]);            //Uncomment for debugging
        /* walk through other arguments */
        token = strtok(NULL, s);
        index++;
    }

    //Firstly clear existing values
    memset(cmd, 0, sizeof (cmd));

    //Now copy array contents to the command data type we have created
    strlcpy(cmd->cmd, argArray[0], MSG_FIELD_LENGTH);
    strlcpy(cmd->status, argArray[1], MSG_FIELD_LENGTH);
    char *ptr; //Stores the textual part of the recovered string. Not used
    cmd->arg1 = (int) strtol(argArray[2], &ptr, 10);
    cmd->arg2 = (int) strtol(argArray[3], &ptr, 10);
    cmd->arg3 = (int) strtol(argArray[4], &ptr, 10);
    strlcpy(cmd->text, argArray[5], MSG_FIELD_LENGTH);
    //cmd->identity = (int) strtol(argArray[6], &ptr, 10);
    strlcpy(cmd->identity, argArray[6], MSG_FIELD_LENGTH);
    cmd->timeStamp = (time_t) strtol(argArray[7], &ptr, 10);
    return fieldTooLong; //0 okay, 1 means a field was too long
}

int startServodV2(char setupString[]) {
    /*
     * This starts the servod process but in a much cruder way than
     * original startServod() which used fork() and, for some reason
     * always created two instances of servod
     */
    char commandString[1024];
    sprintf(commandString, "/home/tc/./servod %s", setupString); //Append setupstring to command string

    //Firstly, kill any existing instances of servod
    system("killall servod");

    sleep(2); //wait for killall to do its business

    //Now start servod process
    system(commandString);
    sleep(2); //Give it a chance to settle
    return 0;
}

pid_t startServod(char setupString[], int pidOfParent) {
    //Start servod with additional arguments supplied in setupString
    //Returns the PID if successful, or -1 if fork failed

    char buffer[SERVOD_CONTROL_STRING_SIZE];
    char commandString[] = "/home/tc/./servod";
    //Check incoming input array doesn't overflow buffer array
    if (((strlen(commandString) + strlen(setupString)) + 4) < SERVOD_CONTROL_STRING_SIZE) {
        sprintf(buffer, "%s %s", commandString, setupString); //concatenate commannd and setup strings
        //printf("concatenated result: %s\n",buffer);

        //Firstly, kill any existing instances of servod
        system("killall servod");

        pid_t pid = fork();
        if (pid == -1) {
            //fork failed
            perror("camera_control.c:startServod():fork");
            return -1;
        }
        /*
         *At this point, we now have two identical processes (child and parent)
         *Each with a seperate PID and each with a separate area in memory.
         *How do we tell them apart? And how do we split the two processes
         * such that they can now execute different code
         * 
         *We do this by testing the value of pid returned by fork() (obviously there
         *will now be 'two concurrent variables with the name 'pid')
         *
         *If fork() returns 0, this is is the child
         *process.  If fork() returns non-zero, we are the parent and the
         *return value is the PID of the child process.
         */
        if (pid == 0) {
            /* this is the child process.  now we can call one of the exec
             * family of functions to execute servod.  when you call execlp(),
             * it replaces the currently running process (in this
             * case, the child process) with whatever we pass to exec.  
             * So our child process will now be running servod.
             * exec() will never return except in an error
             * case, since it is now running the servod code and not my code.
             */
            //The newly executed program (servod) will assume the pid
            //of this process (by using execlp())
            execlp("/home/tc/servod", "servod", setupString, (char*) NULL); //Execute the Linux shell command

            //Normally execlp won't return (i.e if successful), however, if 
            //it's unable to execute the specified system command, execution
            //will continue. We don't want the parent process to carry on
            //regardless, so we need to kill it using it's own pid.

            //Note: This should be very straightforward using kill(),
            //However my C compiler appears to be partly broken and I can't
            //use any functions that rely on the signal.h standard library
            //The solution is to get the shell to execute the command instead
            printf("cameracontrol.c:startServod():Can't start servod. Quitting...\n");

            char buf[30]; //temp buffer
            sprintf(buf, "kill %d", pidOfParent);
            system(buf); //Kill parent

            exit(0);
        } else {
            //printf("cameracontrol.c:startServod(): PID of child process (servod):%d\n",pid);
            /* parent, return the child's PID back to main. */
            return pid;
        }


    } else {
        printf("camera_control.c:startServod() - Input buffer not large enough. Can't start servod. Exiting..");
        exit(1);
    }
    return 0;
}

int servodControlString(char *controlString) {
    //printf("servodControlString() input value:%s\n",controlString);
    //Open FILE descriptor to servod device
    FILE *fp; //Create a pointer to a file
    fp = fopen("/dev/servoblaster", "w");
    fprintf(fp, controlString);
    //fprintf(fp, "debug\n");    //Tells servod to dump current servo values
    fclose(fp);
    return 1;
}

int servodControl(int servo, int servoPosition) {
    //Check incoming values seem reasonable
    if ((servo > 3) || (servo < 0)) {
        printf("camera_control.c:servodControl()): Invalid servo specified: %d\n", servo);
        return 0;
    }
    if ((servoPosition > 100) || (servoPosition < 0)) {
        printf("camera_control.c:servodControl()): Invalid servoPosition specified: %d\n", servoPosition);
        return 0;
    }
    char buf[512]; //tempprary char buffer
    sprintf(buf, "%d=%d%\n", servo, servoPosition); //format servod control string
    //printf("servodControl() buf value:%s\n",buf);
    servodControlString(buf);
    return 1;
}
