// gyro constants
const int gyroFrequency = 250;                                      // (Hz)
const float gyroSensibility = 65.5;                                 
float travelCoeff = 1/((float)gyroFrequency * gyroSensibility);
float convDegToRad = 180.0 / PI;
float travelCoeffToRad = travelCoeff / convDegToRad;
// correction for the pitch and roll
const int correctionPitchRoll = 15;

// PWM constants
const int freq = 500;                                               // (Hz) for what I know, 500 is the best 
const int pwmLedChannel = 0;
const int pwmLedFlyChannel = 5;
const int pwmChannel1 = 1;
const int pwmChannel2 = 2;
const int pwmChannel3 = 3;
const int pwmChannel4 = 4;
const int resolution = 11;                                          // (bits) 11 is the best guess
const int MAX_DUTY_CYCLE = (int)(pow(2, resolution) - 1);
const int HALF_DUTY_CYCLE = (int)(0.5*MAX_DUTY_CYCLE);

// battery
const int maxBatteryLevel = 11100;                              // the battery nominal voltage
const int minBatteryLevel = 3100;                                   // min battery level above which it is better not to reach

// battery calculations
float res3 = 1.5;
float res2 = 1;
float res4 = 1.;
int diodeDrop = 700;                                                // -0.7V generally
int esp32LimitVoltage = 3300;                                       // voltage limit of the pins
float dropTo5V = (res2 + res4) / (res3 + res4 + res2);              // first voltage partitor drop
float dropTo3V3 = res4 / (res4 + res2);                             // second voltage partitor drop
float totalDrop = dropTo5V*dropTo3V3;                  // IMPORTANT: this is in my case, you have to calculate YOUR total drop
uint8_t adcBits = 12;                                               // (bits) of width when measuring the voltage
float maximumWidth = pow(2., (float)adcBits);
float fromVtoWidth = maximumWidth / esp32LimitVoltage;
float maxBatteryLevelDropped = (float)(maxBatteryLevel-diodeDrop) * totalDrop;
float correctionBattery = esp32LimitVoltage/maxBatteryLevelDropped;
int minBatteryLevelThreshold = (float)(minBatteryLevel-diodeDrop) * totalDrop / correctionBattery;

// altitude sensor
const int flightMode = 1;                                           // 1 = only auto leveling, 2 = altitude hold
float pressure;