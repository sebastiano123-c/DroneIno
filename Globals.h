byte lastChannel1, lastChannel2, lastChannel3, lastChannel4;
byte eepromData[36];
byte highByte, lowByte;
boolean gyroAnglesSet;
volatile int receiverInputChannel1, receiverInputChannel2, receiverInputChannel3, receiverInputChannel4;
int16_t counterChannel1, counterChannel2, counterChannel3, counterChannel4, loopCounter;
int16_t esc1, esc2, esc3, esc4;
int16_t throttle;
int16_t calInt, start, gyroAddress;
int16_t receiverInput[5];
int16_t temperature;
int16_t vibrationCounter;
int16_t accAxis[4], gyroAxis[4];   
float angleRollAcc, anglePitchAcc, anglePitch, angleRoll;
float rollLevelAdjust, pitchLevelAdjust;
float batteryVoltage;

long accX, accY, accZ, accTotalVector;
unsigned long timerChannel1, timerChannel2, timerChannel3, timerChannel4, escTimer, escLoopTimer;
unsigned long timer1, timer2, timer3, timer4, currentTime, loopTimer;
double gyroAxisCalibration[4];   

// Battery voltage constants:
// 65 is the voltage compensation for the diode.
// The voltage measured is assigned to a value between 0 and 4095, 
// in which 0 V corresponds to 0, and 3.3 V corresponds to 4095.
// Any voltage between 0 V and 3.3 V will be given the corresponding value in between.
float maximumBatteryVoltage = 11.1; // (V)
float fromVoltToTick = 3.3/4096;

//pid
float pidIMemRoll, pidRollSetpoint, gyroRollInput, pidOutputRoll, pidLastRollDError;
float pidIMemPitch, pidPitchSetpoint, gyroPitchInput, pidOutputPitch, pidLastPitchDError;
float pidIMemYaw, pidYawSetpoint, gyroYawInput, pidOutputYaw, pidLastYawDError;
float pidErrorTemp;

//gyro contant for communication
const int gyroFrequency = 250; // (Hz)
const float gyroSensibility = 65.5; //
float travelCoeff = 1/((float)gyroFrequency * gyroSensibility);
float convDegToRad = 180.0 / PI;
float travelCoeffToRad = travelCoeff / convDegToRad;

byte rawAX[2], rawAY[2], rawAZ[2];
byte rawGX[2], rawGY[2], rawGZ[2];

//PWM constants
const int freq = 500;               //30000;
const int pwmChannel1 = 1;
const int pwmChannel2 = 2;
const int pwmChannel3 = 3;
const int pwmChannel4 = 4;
const int resolution = 11;          //8;