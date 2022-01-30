// correction for the pitch and roll
const int correctionPitchRoll = 15;

// gyro constants
const int gyroFrequency = 250;                                      // (Hz)
const float gyroSensibility = 65.5;                                 
float travelCoeff = 1/((float)gyroFrequency * gyroSensibility);
float convDegToRad = 180.0 / PI;
float travelCoeffToRad = travelCoeff / convDegToRad;

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
const int inputBatteryVoltage = 11100;                              // the battery nominal voltage
const int minBatteryLevel = 3000;                                   // min battery level above which it is better not to reach

// battery calculations
float dropTo5V = 1 / (1 + 1.556);                                   // first voltage partitor drop
float dropTo3V3 = 1.5 / (1 + 1.5);                                  // second voltage partitor drop
float Vout = (inputBatteryVoltage-700)*dropTo5V* dropTo3V3;         // voltage drop from 11.1V battery voltage to 4.7V with res. R=R2/(R3+R2)=1/2.55 and diode (-700mV)
float voltageRatio = inputBatteryVoltage / Vout;
uint8_t adcBits = 12;                                               // gives 2^10=1024 bits of width when measuring the voltage
float fromVtoBit = pow(2, adcBits)/ Vout;
const int minBatteryVoltageInBits = minBatteryLevel / voltageRatio * fromVtoBit; // mV

// altitude sensor
const int flightMode = 1;                                           // 1 = auto leveling, 2 = altitude hold
float pressure = 0.;
