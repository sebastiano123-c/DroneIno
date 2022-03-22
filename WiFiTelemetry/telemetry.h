#pragma once

#include "app_httpd.h"
#include "HardwareSerial.h"


//           (ROLL)
extern float PID_P_GAIN_ROLL;                     //Gain setting for the roll P-controller (1.3)
extern float PID_I_GAIN_ROLL;                     //Gain setting for the roll I-controller  (0.0002)
extern float PID_D_GAIN_ROLL;                     //Gain setting for the roll D-controller (10.0)


//           (PITCH)                                             
extern float PID_P_GAIN_PITCH;                    //Gain setting for the pitch P-controller
extern float PID_I_GAIN_PITCH;                    //Gain setting for the pitch I-controller
extern float PID_D_GAIN_PITCH;                    //Gain setting for the pitch D-controller


//           (YAW)                                             
extern float PID_P_GAIN_YAW;                      //Gain setting for the pitch P-controller. (2.0)
extern float PID_I_GAIN_YAW;                      //Gain setting for the pitch I-controller. (0.04)
extern float PID_D_GAIN_YAW;                      //Gain setting for the pitch D-controller. (0.0)


//           (GYROSCOPE)
extern float GYROSCOPE_ROLL_FILTER;               // read your gyroscope data after the calibration, try different values and choose the best one
extern float GYROSCOPE_ROLL_CORR;                 // (0.) after set GYROSCOPE_ROLL_FILTER, put here the angle roll you read eneabling DEBUG
extern float GYROSCOPE_PITCH_CORR;                // (-1.65.) after set GYROSCOPE_PITCH_FILTER, put here the angle pitch you read eneabling DEBUG


//           (ALTITUDE)                                                                                          
extern float PID_P_GAIN_ALTITUDE;                 //Gain setting for the pitch P-controller. (2.0)
extern float PID_I_GAIN_ALTITUDE;                 //Gain setting for the pitch I-controller. (0.04)
extern float PID_D_GAIN_ALTITUDE;                 //Gain setting for the pitch D-controller. (0.0)


//           (TELEMETRY)                                      
extern float rollAngle;
extern float pitchAngle;
extern float flightMode;
extern float batteryPercentage;
extern float altitudeMeasure;
extern float rollTrim;
extern float pitchTrim;
extern float yawTrim;
extern float throttleTrim;
#define flightDataHeaderCSV "roll, pitch, flightMode, battery, altitude, rec. roll, rec. pitch, rec. yaw, rec. throttle\n"

//           (UART)                                      
const int dataTransferSize = 9;
const int dataControllerSize = 12;


//          (TELEMETRY FUNCTIONS)
extern void writeDataTransfer();
extern void readDataTransfer();
extern void beginUARTCOM();
