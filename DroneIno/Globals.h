// trim position structure
struct trimPosition{
  byte reverse;                           // if is inverted
  int16_t low;                            // low value for the specific receiver input channel
  int16_t center;                         // center value for the specific receiver input channel
  int16_t high;                           // high value for the specific receiver input channel
  int16_t actual;                         // instantaneous receiver value
} trimCh[5];

// globals
byte eepromData[36];
int16_t calInt, start;

// ISR
int16_t receiverInput[5];
volatile int receiverInputChannel1, receiverInputChannel2, receiverInputChannel3, receiverInputChannel4, receiverInputChannel5;
byte lastChannel1, lastChannel2, lastChannel3, lastChannel4, lastChannel5;
unsigned long timer1, timer2, timer3, timer4, timer5, currentTime, loopTimer;
int16_t esc1, esc2, esc3, esc4;
int16_t throttle;
double gyroAxisCalibration[4];   

//PID
float pidIMemRoll, pidRollSetpoint, gyroRollInput, pidOutputRoll, pidLastRollDError;
float pidIMemPitch, pidPitchSetpoint, gyroPitchInput, pidOutputPitch, pidLastPitchDError;
float pidIMemYaw, pidYawSetpoint, gyroYawInput, pidOutputYaw, pidLastYawDError;
float pidErrorTemp;

//gyro readings
boolean gyroAnglesSet;
int16_t gyroAddress;
int16_t accAxis[4], gyroAxis[4];   
float angleRollAcc, anglePitchAcc, anglePitch, angleRoll;
float rollLevelAdjust, pitchLevelAdjust;
long accX, accY, accZ, accTotalVector;
byte rawAX[2], rawAY[2], rawAZ[2];
byte rawGX[2], rawGY[2], rawGZ[2];

// battery
float batteryVoltage;

// instantiate classes
#if ALTITUDE_SENSOR == BMP280
    Adafruit_BMP280 bmp; // I2C 
#elif ALTITUDE_SENSOR == BME280
    Adafruit_BME280 bmp; // I2C 
#endif