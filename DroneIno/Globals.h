/**
 * @file Globals.h
 * @author @sebastiano123-cs
 * @brief Global variables used in all the project.
 * 
 * Global variables are divided here into:
 *  - PID: roll, yaw, pitch and altitude;
 *  - GYROSCOPE: low/high filter percentage, pitch and roll correction;
 *  - PWM channels: for the use of the ledWrite;
 *  - BATTERY: battery constants and calculation of the battery level;
 *  - ALTIMETER: constants;
 *  - TELEMETRY: data transfer and data controller size and array;
 *  - UNDEFINED: variables not yet defined;
 * 
 * @version 0.1
 * @date 2022-02-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */


/**
 * @brief PID parameters for roll, pitch, yaw and altitude.
 * 
 * @note Roll and pitch parameters have the same values.
 * 
 */

//              (ROLL)
float PID_P_GAIN_ROLL            = 1.0;                      //Gain setting for the roll P-controller (1.3)
float PID_I_GAIN_ROLL            = 0.000001;                 //Gain setting for the roll I-controller  (0.0002)
float PID_D_GAIN_ROLL            = 10.0;                     //Gain setting for the roll D-controller (10.0)
int PID_MAX_ROLL                 = 400;                      //Maximum output of the PID-controller   (+/-)
                                                                        
//              (PITCH)                                              
float PID_P_GAIN_PITCH           = PID_P_GAIN_ROLL;          //Gain setting for the pitch P-controller
float PID_I_GAIN_PITCH           = PID_I_GAIN_ROLL;          //Gain setting for the pitch I-controller
float PID_D_GAIN_PITCH           = PID_D_GAIN_ROLL;          //Gain setting for the pitch D-controller
int PID_MAX_PITCH                = PID_MAX_ROLL;             //Maximum output of the PID-controller   (+/-)
                                                                            
//              (YAW)                                          
float PID_P_GAIN_YAW             = 0.7;                      //Gain setting for the pitch P-controller. (2.0)
float PID_I_GAIN_YAW             = 0.05;                     //Gain setting for the pitch I-controller. (0.04)
float PID_D_GAIN_YAW             = 0.0;                      //Gain setting for the pitch D-controller. (0.0)
int PID_MAX_YAW                  = 400;                      //Maximum output of the PID-controller     (+/-)
                                                                               
//            (ALTITUDE)                                                     
float PID_P_GAIN_ALTITUDE        = 1.4;                      //Gain setting for the altitude P-controller (default = 1.4).
float PID_I_GAIN_ALTITUDE        = 0.3;                      //Gain setting for the altitude I-controller (default = 0.2).
float PID_D_GAIN_ALTITUDE        = 0.75;                     //Gain setting for the altitude D-controller (default = 0.75).
int PID_MAX_ALTITUDE             = 400;                      //Maximum output of the PID-controller (+/-).
                                                                 

/**
 * @brief GYROSCOPE
 * 
 */

//             (FILTER)                                                                    
float GYROSCOPE_ROLL_FILTER      = .9996;                      // read your gyroscope data after the calibration, try different values and choose the best one
float GYROSCOPE_PITCH_FILTER     = GYROSCOPE_PITCH_FILTER;    // read your gyroscope data after the calibration, try different values and choose the best one
float GYROSCOPE_ROLL_CORR        = -.30;                      // (0.) after set GYROSCOPE_ROLL_FILTER, put here the angle roll you read eneabling DEBUG
float GYROSCOPE_PITCH_CORR       = -1.65;                     // (-1.65.) after set GYROSCOPE_PITCH_FILTER, put here the angle pitch you read eneabling DEBUG

//            (SPECIFICS)
const int gyroFrequency          = 250;                               // (Hz)
const float gyroSensibility      = 65.5;                                 
const int correctionPitchRoll    = 15;                                // correction for the pitch and roll
float convDegToRad               = 180.0 / PI;                                 
float travelCoeff                = 1/((float)gyroFrequency * gyroSensibility);     
float travelCoeffToRad           = travelCoeff / convDegToRad;
float anglePitchOffset           = 0;                                 // NOT touch, for future 
float angleRollOffset            = 0;                                 // NOT touch, for future 


/**
 * @brief PWM
 * 
 * @note PWM is used for ESC pulses and LEDs
 * 
 */

//              (CHANNELS)
const int pwmLedChannel          = 0;
const int pwmChannel1            = 1;
const int pwmChannel2            = 2;
const int pwmChannel3            = 3;
const int pwmChannel4            = 4;
const int pwmLedFlyChannel       = 5;

//              (SPECIFICS)
const int freq                   = 500;                                   // (Hz) for what I know, 500 is the best 
const int resolution             = 11;                                    // (bits) 11 is the best guess
const int MAX_DUTY_CYCLE         = (int)(pow(2, resolution) - 1);
const int HALF_DUTY_CYCLE        = (int)(0.5*MAX_DUTY_CYCLE);


/**
 * @brief BATTERY
 * 
 * @note total resistance calculations, the important quantity is totalDrop
 * 
 */

//             (CIRCUIT SPECIFICS)
const int DIODE_DROP             = 700;                                   //generally it is -0.7V
float res3                       = 2.5;                                   // resistance between Vin and vout
float res2                       = 1.;                                    // load resistance
float totalDrop                  = res2 / (res2 + res3);                  // IMPORTANT: this is in my case, you have to calculate YOUR total drop

//             (DIGITAL PINS ACCURACY)
uint8_t adcBits                  = 12;                                    // (bits) of width when measuring the voltage
float maximumWidth               = pow(2., (float)adcBits)-1;             // maximum width that the pin can read

//             (CALCULATIONS)
double fromVtoWidth              = maximumWidth / (double)BOARD_LIMIT_VOLTAGE;
double maxBatteryLevelDropped    = (double)(MAX_BATTERY_VOLTAGE-DIODE_DROP) * totalDrop;
double correctionBattery         = (double)BOARD_LIMIT_VOLTAGE/maxBatteryLevelDropped;
double minBatteryLevelThreshold  = ((double)MIN_BATTERY_VOLTAGE-(double)DIODE_DROP) * totalDrop * correctionBattery;


/**
 * @brief ALTIMETER
 * 
 */
const float PRESSURE_SEA_LEVEL   = 1013.25;                       // hPa
uint8_t osrs_t                   = 1;                             // Temperature oversampling x 1
uint8_t osrs_p                   = 1;                             // Pressure oversampling x 1
uint8_t barometerMode            = 3;                             // Normal barometerMode
uint8_t t_sb                     = 5;                             // standby 1000ms
uint8_t filter                   = 0;                             // Filter off 
uint8_t spi3w_en                 = 0;                             // 3-wire SPI Disable

/**
 * @brief TELEMETRY
 * 
 */

//          (WIFI AP)
const char *ssid                 = "DroneInoTelemetry";
const char *password             = "DroneIno";
const int refreshRate            = 200;                                   // (ms) the refresh rate of the page
int refreshCounter               = 0;

//          (ESP32 BROWSER TAGS)
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

//           (TX)
const int dataTransferSize       = 5;
float dataTransfer[dataTransferSize];

//            (RX)
const int dataControllerSize     = 12;
float dataController[dataControllerSize];



// // esp-now telemetry
// uint8_t broadcastAddress[] = {0x58, 0xBF, 0x25, 0x82, 0x4D, 0x08}; // RECEIVER MAC Address
// // Create an Event Source on /events
// AsyncEventSource events("/events");

/**
 * @brief NOT DECLARED VARIABLES
 * 
 */

// trim position structure
struct trimPosition{
  byte reverse;                                            // if is inverted
  int16_t low;                                             // low value for the specific receiver input channel
  int16_t center;                                          // center value for the specific receiver input channel
  int16_t high;                                            // high value for the specific receiver input channel
  int16_t actual;                                          // instantaneous receiver value
} trimCh[6];

// flight mode
byte flightMode;                                           // 1 = only auto leveling (or nothing if AUTO_LEVELING = false), 2 = altitude hold

// very global
byte eepromData[36], errWire;
int16_t calInt, start;
int error;

// ISR
volatile int receiverInputChannel1, receiverInputChannel2, receiverInputChannel3, receiverInputChannel4, receiverInputChannel5;
byte lastChannel1, lastChannel2, lastChannel3, lastChannel4, lastChannel5;
unsigned long timer1, timer2, timer3, timer4, timer5, currentTime, loopTimer;
int16_t esc1, esc2, esc3, esc4;
int16_t throttle;

// PID
float pidIMemRoll, pidRollSetpoint, gyroRollInput, pidOutputRoll, pidLastRollDError;
float pidIMemPitch, pidPitchSetpoint, gyroPitchInput, pidOutputPitch, pidLastPitchDError;
float pidIMemYaw, pidYawSetpoint, gyroYawInput, pidOutputYaw, pidLastYawDError;
float pidErrorTemp;

// Altitude PID variables
float pidErrorGainAltitude;
float pidIMemAltitude, pidAltitudeSetpoint, pidAltitudeInput, pidOutputAltitude;
uint8_t parachuteRotatingMemLocation;
int32_t parachuteBuffer[35], parachuteThrottle;
float pressureParachutePrevious;
int32_t pressureRotatingMem[50], pressureTotalAvarage;
uint8_t pressureRotatingMemLocation;
uint8_t manualAltitudeChange;
int16_t manualThrottle;

// gyroscope
boolean gyroAnglesSet;
int16_t gyroTemp, accAxis[4], gyroAxis[4];   
double gyroAxisCalibration[4], accAxisCalibration[4];
float angleRollAcc, anglePitchAcc, anglePitch, angleRoll;
float rollLevelAdjust, pitchLevelAdjust;
long accTotalVector;

// battery
float batteryVoltage, batteryPercent;

// altitude sensor
float pressure, altitudeMeasure;
float temperature;
uint8_t barometerCounter;
float actualPressure, actualPressureSlow, actualPressureFast, actualPressureDiff;

// BMP280 Wire.read() values
uint16_t dig_P1, dig_T1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9, dig_H2, dig_H4, dig_H5;
int8_t dig_H1, dig_H3, dig_H6;
long adcP, adcT;
unsigned long int tempRaw, presRaw;
signed long int tFine;
signed long int tempCal;
unsigned long int pressCal;
