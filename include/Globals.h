/**
 * @file Globals.h
 * @author @sebastiano123-cs
 * @brief Global variables used in all the project.
 * 
 * Global variables are divided here into:
 *  @li PID: roll, yaw, pitch and altitude;
 *  @li GYROSCOPE: low/high filter percentage, pitch and roll correction;
 *  @li PWM channels: for the use of the ledWrite;
 *  @li BATTERY: battery constants and calculation of the battery level;
 *  @li ALTIMETER: constants;
 *  @li TELEMETRY: data transfer and data controller size and array;
 *  @li UNDEFINED: variables not yet defined;
 * 
 * @version 0.1
 * @date 2022-02-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */



/**
 * -----------------------------------------------------------------------------------------------------------
 * @brief PID parameters for roll, pitch, yaw and altitude.
 * 
 * @note Roll and pitch parameters have the same values.
 * 
 */
/**
 *    (ROLL PID)
 */
float PID_P_GAIN_ROLL            = 1.1;                      //Gain setting for the roll P-controller (1.3)
float PID_I_GAIN_ROLL            = 0.0015;                   //Gain setting for the roll I-controller  (0.0002)
float PID_D_GAIN_ROLL            = 14.0;                     //Gain setting for the roll D-controller (10.0)
int PID_MAX_ROLL                 = 400;                      //Maximum output of the PID-controller   (+/-)
                                                                        
/**
 *    (PITCH PID)
 */                                             
float PID_P_GAIN_PITCH           = PID_P_GAIN_ROLL;          //Gain setting for the pitch P-controller
float PID_I_GAIN_PITCH           = PID_I_GAIN_ROLL;          //Gain setting for the pitch I-controller
float PID_D_GAIN_PITCH           = PID_D_GAIN_ROLL;          //Gain setting for the pitch D-controller
int PID_MAX_PITCH                = PID_MAX_ROLL;             //Maximum output of the PID-controller   (+/-)
                                                                            
/**
 *    (YAW PID)
 */                                       
float PID_P_GAIN_YAW             = 3.0;                      //Gain setting for the pitch P-controller. (2.0)
float PID_I_GAIN_YAW             = 0.05;                     //Gain setting for the pitch I-controller. (0.04)
float PID_D_GAIN_YAW             = 0.0;                      //Gain setting for the pitch D-controller. (0.0)
int PID_MAX_YAW                  = 400;                      //Maximum output of the PID-controller     (+/-)
                                                                               
/**
 *    (ALTITUDE PID)
 */
float PID_P_GAIN_ALTITUDE        = 1.4;                      //Gain setting for the altitude P-controller (default = 1.4).
float PID_I_GAIN_ALTITUDE        = 0.3;                      //Gain setting for the altitude I-controller (default = 0.2).
float PID_D_GAIN_ALTITUDE        = 0.75;                     //Gain setting for the altitude D-controller (default = 0.75).
int PID_MAX_ALTITUDE             = 400;                      //Maximum output of the PID-controller (+/-).

/**
 *    (GPS PID) 
 */
float GPS_P_GAIN = 2.7;                    //Gain setting for the GPS P-controller (default = 2.7).
float GPS_D_GAIN = 6.5;                    //Gain setting for the GPS D-controller (default = 6.5).


//    (PID MISC.)
float pidIMemRoll, pidRollSetpoint, gyroRollInput, pidOutputRoll, pidLastRollDError;
float pidIMemPitch, pidPitchSetpoint, gyroPitchInput, pidOutputPitch, pidLastPitchDError;
float pidIMemYaw, pidYawSetpoint, gyroYawInput, pidOutputYaw, pidLastYawDError;
float pidErrorTemp;                                                     

//    (ALTITUDE PID MISC.)
float pidErrorGainAltitude;
float pidIMemAltitude, pidAltitudeSetpoint, pidAltitudeInput, pidOutputAltitude;
uint8_t parachuteRotatingMemLocation;
int32_t parachuteBuffer[35], parachuteThrottle;
float pressureParachutePrevious;
int32_t pressureRotatingMem[50], pressureTotalAvarage;
uint8_t pressureRotatingMemLocation;
uint8_t manualAltitudeChange;
int16_t manualThrottle;



/**
 * -----------------------------------------------------------------------------------------------------------
 * @brief GYROSCOPE
 * 
 *    (HIGH-LOW FILTER)                                                                    
 */
float GYROSCOPE_ROLL_FILTER      = 0.96;                       // read your gyroscope data after the calibration, try different values and choose the best one
float GYROSCOPE_PITCH_FILTER     = GYROSCOPE_PITCH_FILTER;     // read your gyroscope data after the calibration, try different values and choose the best one

//    (ANGLE CORRECTION)
float GYROSCOPE_ROLL_CORR        = 0.;                         // (0.) after set GYROSCOPE_ROLL_FILTER, put here the angle roll you read eneabling DEBUG
float GYROSCOPE_PITCH_CORR       = 0.;                         // (-1.65.) after set GYROSCOPE_PITCH_FILTER, put here the angle pitch you read eneabling DEBUG

//    (DATASHEET)
const int gyroFrequency          = 250;                         // (Hz)
const float gyroSensibility      = 65.5;                                
const int correctionPitchRoll    = 15;                          // correction for the pitch and roll
float convDegToRad               = 180.0 / PI;                                 
float travelCoeff                = 1/((float)gyroFrequency *
                                   gyroSensibility);     
float travelCoeffToRad           = travelCoeff / convDegToRad;
float anglePitchOffset           = 0;                           // NOT touch, for future 
float angleRollOffset            = 0;                           // NOT touch, for future 

//      (MISC.)
boolean gyroAnglesSet;
int16_t gyroTemp, accAxis[4], gyroAxis[4];   
double gyroAxisCalibration[4], accAxisCalibration[4];
float angleRollAcc, anglePitchAcc, anglePitch, angleRoll;
float rollLevelAdjust, pitchLevelAdjust;
long accTotalVector;



/**
 * -----------------------------------------------------------------------------------------------------------
 * @brief PWM
 * 
 * @note PWM is used for ESC pulses and LEDs
 * 
 *    (CHANNELS)
 */
const int pwmLedChannel          = 0;
const int pwmChannel1            = 1;
const int pwmChannel2            = 2;
const int pwmChannel3            = 3;
const int pwmChannel4            = 4;
const int pwmLedBatteryChannel   = 5;

//    (SPECIFICS)
const int freq                   = 500;                           // (Hz) for what I know, 500 is the best 
const int resolution             = 11;                            // (bits) 11 is the best guess
const int MAX_DUTY_CYCLE         = (int)(pow(2, resolution) - 1);
const int HALF_DUTY_CYCLE        = (int)(0.5*MAX_DUTY_CYCLE);



/**
 * -----------------------------------------------------------------------------------------------------------
 * @brief BATTERY
 * 
 * @note total resistance calculations, the important quantity is totalDrop
 * 
 *    (POWER SUPPLY CIRCUIT SPECIFICS)
 *    IMPORTANT: this is in my case, you have to calculate YOUR total drop
 */
float totalDrop                  = RESISTANCE_2 / (RESISTANCE_2 + RESISTANCE_1);// (V) total drop due to the voltage partitor

/**
 *     (DIGITAL PINS ACCURACY)
 *     Boards measure the pin inputs signals with a certain digit precision, which is adcBits (ADC stands for analog to digital converter).
 *     maximumWidth is the maximum width the board can measure.
 */
uint8_t adcBits                  = 12;                                    // (bits) of width when measuring the voltage
float maximumWidth               = pow(2., (float)adcBits)-1;             // maximum width that the pin can read

/**
 *     (CONVERSIONS)
 *     Microcontrollers calculates the input voltage giving a signal output in the range (0., maximumWidth).
 *     fromVtoWidth converts the width to voltage of the signal.
 *     This signal is by the way dropped by your voltage partitor. Thus, maxBatteryLevelDropped is the measure of maximum voltage after the partitor
 *     The correction factor is calculated because some boards accepts 5v input, others 3v3.
 *     Finally, minBatteryLevelThreshold is the board minimum voltage under which it is not safe to go.
 */ 
float pinVoltageMax             = MAX_BATTERY_VOLTAGE * totalDrop;
float pinVoltageMin             = MIN_BATTERY_VOLTAGE * totalDrop;
float fromWidthToV              = BOARD_LIMIT_VOLTAGE / maximumWidth;           // see https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32c3/api-reference/peripherals/adc.html
float correctionBattery         = BOARD_LIMIT_VOLTAGE / pinVoltageMax;

/**
 *     (BATTERY COMPENSATION*)
 *     WORK IN PROGRESS: use these parameters only, or vary by small steps
 *     In the first flight use the parameters here.
 *     STUDY THIS: After a complete flight experience, using the esp-cam telemetryAnalysis tool, you can analyse the data readings saved on your SD.
 *     The analysis program fits the battery data (the plot on the bottom right) and gives you the m rect pendency and the intercept b.
 *     Use put here the results.
 */
float  batteryCurvePendency      = 6e-2;                                 // compensate the rotors power loss when battery drops down

//    (MISC.)
float pinPulseWidth, pinPulseVoltage, batteryVoltage, batteryPercentage;
int16_t escCorr;



/**
 * -----------------------------------------------------------------------------------------------------------
 * @brief ALTIMETER
 * 
 *    (CONSTS.)
 */
const float PRESSURE_SEA_LEVEL   = 1013.25;                              // (hPa) sea level pressure reference

//    (BMP280)
uint16_t dig_P1, dig_T1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9, dig_H2, dig_H4, dig_H5;
int8_t dig_H1, dig_H3, dig_H6;
long adcP, adcT;
unsigned long int tempRaw, presRaw;
signed long int tFine;
signed long int tempCal;
unsigned long int pressCal;

//    (MISC.)
float pressure, altitudeMeasure;
float temperature;
uint8_t barometerCounter;
float actualPressure, actualPressureSlow, actualPressureFast, actualPressureDiff, pressureForPID;



/**
 * -----------------------------------------------------------------------------------------------------------
 * @brief TELEMETRY
 * 
 *    (WIFI AP)
 *    Defines the name of the access point (AP) and the PSW.
 *    IMPORTANT! PSW must be composed AT LEAST by 8 characters otherwise the network simply don't work 
 */
const char *ssid                 = "DroneInoTelemetry";
const char *password             = "DroneIno";                            
const int refreshRate            = 200;                                   // (ms) the refresh rate of the page
int refreshCounter               = 0;

//    (ESP32 BROWSER TAGS)
const char* P_ROLL_GET           = "rollP";
const char* I_ROLL_GET           = "rollI";
const char* D_ROLL_GET           = "rollD";
const char* P_YAW_GET            = "yawP";
const char* I_YAW_GET            = "yawI";
const char* D_YAW_GET            = "yawD";
const char* FILTER_P_R_GET       = "filterPitchRoll";
const char* ROLL_CORR_GET        = "correctionRoll";
const char* PITCH_CORR_GET       = "correctionPitch";
const char* P_ALTITUDE_GET       = "altitudeP";
const char* I_ALTITUDE_GET       = "altitudeI";
const char* D_ALTITUDE_GET       = "altitudeD";

/**
 *    (RX)
 *    Defines the number of elements and the that ESP32 waits to receive from telemetry system
 */
const int dataControllerSize     = 12;
float dataController[dataControllerSize];



/**
 * -----------------------------------------------------------------------------------------------------------
 * @brief RC-CONTROLLER
 * 
 *    (CONTROLLER STICKS)
 *    Structure defined for the controller's sticks, trims and switches
 */
struct trimPosition{
  byte reverse;                                            // if is inverted
  int16_t low;                                             // low value for the specific receiver input channel
  int16_t center;                                          // center value for the specific receiver input channel
  int16_t high;                                            // high value for the specific receiver input channel
  int16_t actual;                                          // instantaneous receiver value
} trimCh[5];

/**
 *    (ISR)
 */
volatile int receiverInputChannel1, receiverInputChannel2, receiverInputChannel3, receiverInputChannel4, receiverInputChannel5;
byte lastChannel1, lastChannel2, lastChannel3, lastChannel4, lastChannel5;
unsigned long timer1, timer2, timer3, timer4, timer5, currentTime, loopTimer;
int16_t esc1, esc2, esc3, esc4;
int16_t throttle;

/**
 *   (FLIGHT MODE)
 *    1 = only auto leveling (or nothing if AUTO_LEVELING = false)
 *    2 = altitude hold*
 *    3 = GPS**
 */
byte flightMode;                                           

//    (MISC.)
byte eepromData[EEPROM_SIZE], errWire, msg;
int16_t calInt, start;
unsigned long timer;
int error;
bool flag;



/**
 * -----------------------------------------------------------------------------------------------------------
 * @brief GPS 
 * 
 * GPS print using serial communication (in my case I use Serial 2).
 * The strings printed on serial follows the NMEA standard. They have to the utmost 80 chars and begins
 * always with a '$' and ends always with a '*' char.
 */
uint8_t NMEANewline              = false;

//    (MISC.)
static char GPSString[100];
char GPSIncomingString;
uint8_t GPSSatNumber, latNorth, lonEast, GPSFixType, newGPSDataCounter, newGPSDataAvailable, waypointGPS;
uint8_t GPSRotatingMemLocation;
uint16_t GPSStringCounter;
uint16_t GPSAddCounter;
int32_t longLatGPS, longLonGPS, latGPSPrevious, lonGPSPrevious, longLatWaypoint, longLonWaypoint;
int32_t latActualGPS, lonActualGPS;
int32_t GPSLatError, GPSLonError;
int32_t GPSLatErrorPrevious, GPSLonErrorPrevious;
int32_t GPSLatRotatingMem[40], GPSLonRotatingMem[40];
int32_t GPSLatAvarage, GPSLonAvarage;
uint32_t timerGPS;
float latLoop, lonLoop, latGPSAdd, lonGPSAdd;
float GPSPitchAdjustNorth, GPSPitchAdjust, GPSRollAdjustNorth, GPSRollAdjust;

float latGPSAdjust, lonGPSAdjust, GPSManAdjustHeading;
float latitudeGPS, longitudeGPS;

