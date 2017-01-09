/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   camera_control.h
 * Author: turnej04
 *
 * HOW TO ADD ADDITIONAL FIELDS TO status AND cmd STRUCTS......
 * You need to:-
 * 
 * -Be aware of the maximum field length in chars (set by MSG_FIELD_LENGTH)
 * 
 * Step 1)
 * -If you're adding a field to 'status', increment NO_STATUS_ARGS because this determines the length
 * of the tx buffer required to send the status message string
 * 
 * -If you're adding a field to 'cmd', increment NO_CTRL_ARGS
 * 
 * Step 2)
 * -Modify the functions in camera_control.c
 * 
 * 
 */

#ifndef CAMERA_CONTROL_H
#define CAMERA_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif




#ifdef __cplusplus
}
#endif

//ADD MY OWN STUFF AFTER HERE
//REMEMBER TO ADD: #include "camera_control.h" TO THE SOURCE FILE
#include <time.h>
#define MSG_FIELD_LENGTH  21      //Has to be a minimum of 8
#define NO_CTRL_ARGS  8       //No of control message arguments
#define NO_STATUS_ARGS  27    //No of status message arguments 


#define BUFFER   1024       //General purpose (small) tx buffer. Used elsewhere.   
typedef struct Command {
    char cmd[MSG_FIELD_LENGTH];
    char status[MSG_FIELD_LENGTH];
    int arg1;
    int arg2;
    int arg3;
    char text[MSG_FIELD_LENGTH];
    //int identity;
    char identity[MSG_FIELD_LENGTH];
    time_t timeStamp;                   //Time message sent
} command; //Define new 'commands' with 'command', not 'Command'

typedef struct Status {
    char localIP [MSG_FIELD_LENGTH];   //1: Stores local IP addr
    char remoteIP[MSG_FIELD_LENGTH];   //2: Stores remote ip addr
    int servo0;                        //3:
    int servo1;                        //4:
    int servo2;                        //5:
    int servo3;                         //6:
    int gpi0;                           //7:
    int gpi1;                           //8:
    int gpi2;                           //9:
    int gpi3;                           //10:
    int gpo0;                           //11:
    int gpo1;                           //12:
    int gpo2;                           //13:
    int gpo3;                           //14:
    time_t timeStamp;                   //15: Time message sent
    char type [MSG_FIELD_LENGTH];       //16:
    int gpi4;                           //17:
    int gpi5;                           //18:
    int gpi6;                           //19:
    int gpo4;                           //20:
    int gpo5;                           //21:
    char identity[MSG_FIELD_LENGTH];    //22:
    int controlPort;                    //23:
    int statusPort;                     //24:
    int httpPort;                       //25:
    int connectedFlag;                  //26:
    char remoteIdentity[MSG_FIELD_LENGTH];    //27:
} status; //Define new 'statuses' with 'status', not 'Status'

typedef struct GPOPulse{
    int gpoPin;
    int noOfPulses;
    int onDuration;
    int offDuration;
    
} gpoPulse;     //Declare new 'gpoPulse' structs with gpoPulse not GPOPulse

typedef struct ServoTest{
    int totalDuration;         //length in seconds
    int min;                    //minimum value that will be sent
    int max;                    //maximum value that will be sent
    int duration;               //Length of time between changes
    int rate;                   //Maximum rate of change in % per second
    int servoNo;                //Which servo channel under test
} servoTest;

//AND BEFORE HERE

#endif /* CAMERA_CONTROL_H */

