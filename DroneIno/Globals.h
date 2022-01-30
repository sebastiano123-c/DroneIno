byte lastChannel1, lastChannel2, lastChannel3, lastChannel4;
byte eepromData[36];
boolean gyroAnglesSet;
volatile int receiverInputChannel1, receiverInputChannel2, receiverInputChannel3, receiverInputChannel4, receiverInputChannel5;
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

//PID
float pidIMemRoll, pidRollSetpoint, gyroRollInput, pidOutputRoll, pidLastRollDError;
float pidIMemPitch, pidPitchSetpoint, gyroPitchInput, pidOutputPitch, pidLastPitchDError;
float pidIMemYaw, pidYawSetpoint, gyroYawInput, pidOutputYaw, pidLastYawDError;
float pidErrorTemp;

//gyro readings
byte rawAX[2], rawAY[2], rawAZ[2];
byte rawGX[2], rawGY[2], rawGZ[2];