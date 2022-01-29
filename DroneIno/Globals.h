byte lastChannel1, lastChannel2, lastChannel3, lastChannel4;
byte eepromData[36];
boolean gyroAnglesSet;
volatile int receiverInputChannel1, receiverInputChannel2, receiverInputChannel3, receiverInputChannel4;
int16_t counterChannel1, counterChannel2, counterChannel3, counterChannel4, loopCounter;
int16_t esc1, esc2, esc3, esc4;
int16_t throttle;
int16_t calInt, start, gyroAddress;
int16_t receiverInput[5];
int16_t temperature;
int16_t accAxis[4], gyroAxis[4];   
float angleRollAcc, anglePitchAcc, anglePitch, angleRoll;
float rollLevelAdjust, pitchLevelAdjust;
float batteryVoltage;

long accX, accY, accZ, accTotalVector;
unsigned long timerChannel1, timerChannel2, timerChannel3, timerChannel4, escTimer, escLoopTimer;
unsigned long timer1, timer2, timer3, timer4, currentTime, loopTimer;
double gyroAxisCalibration[4];   

//pid
float pidIMemRoll, pidRollSetpoint, gyroRollInput, pidOutputRoll, pidLastRollDError;
float pidIMemPitch, pidPitchSetpoint, gyroPitchInput, pidOutputPitch, pidLastPitchDError;
float pidIMemYaw, pidYawSetpoint, gyroYawInput, pidOutputYaw, pidLastYawDError;
float pidErrorTemp;

//gyro contant for communication
const int gyroFrequency = 250;      // (Hz)
const float gyroSensibility = 65.5; //
float travelCoeff = 1/((float)gyroFrequency * gyroSensibility);
float convDegToRad = 180.0 / PI;
float travelCoeffToRad = travelCoeff / convDegToRad;

byte rawAX[2], rawAY[2], rawAZ[2];
byte rawGX[2], rawGY[2], rawGZ[2];

//PWM constants
const int freq = 500;               //30000; (Hz)
const int pwmLedChannel = 0;
const int pwmLedFlyChannel = 5;
const int pwmChannel1 = 1;
const int pwmChannel2 = 2;
const int pwmChannel3 = 3;
const int pwmChannel4 = 4;
const int resolution = 11;          //8;    (bits)
const int MAX_DUTY_CYCLE = (int)(pow(2, resolution) - 1);
const int HALF_DUTY_CYCLE = (int)(0.5*MAX_DUTY_CYCLE);

//BATTERY
float Vin = (11100-700)/2.555;           // voltage drop from 11.1V battery voltage to 4.7V with res. R=R2/(R3+R2)=1/2.55 and diode (-700mV)
float Vout = Vin * 1.5 / (1 + 1.5);
uint8_t adcBits = 12;                    // gives 2^10=1024 bits of width when measuring the voltage
float fromVtoBit = pow(2, adcBits)/Vout;