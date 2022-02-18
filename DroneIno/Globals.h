/**
 * @brief Global variables
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

// globals
byte eepromData[36], errWire;
int16_t calInt, start;
int error;

// ISR
volatile int receiverInputChannel1, receiverInputChannel2, receiverInputChannel3, receiverInputChannel4, receiverInputChannel5;
byte lastChannel1, lastChannel2, lastChannel3, lastChannel4, lastChannel5;
unsigned long timer1, timer2, timer3, timer4, timer5, currentTime, loopTimer;
int16_t esc1, esc2, esc3, esc4;
int16_t throttle;

//PID
float pidIMemRoll, pidRollSetpoint, gyroRollInput, pidOutputRoll, pidLastRollDError;
float pidIMemPitch, pidPitchSetpoint, gyroPitchInput, pidOutputPitch, pidLastPitchDError;
float pidIMemYaw, pidYawSetpoint, gyroYawInput, pidOutputYaw, pidLastYawDError;
float pidErrorTemp;
//Altitude PID variables
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
float pressure;
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


// telemetry tx
const int dataTransferSize = 5;
float dataTransfer[dataTransferSize];

// telemetry rx
const int dataControllerSize = 12;
float dataController[dataControllerSize];

// serial
HardwareSerial SUART(1); 

// server
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");
