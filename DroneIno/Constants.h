/* Constants
* @author @sebastiano123-c
*/

// gyro constants
const int gyroFrequency       = 250;                               // (Hz)
const float gyroSensibility   = 65.5;                                 
const int correctionPitchRoll = 15;                                // correction for the pitch and roll
float convDegToRad            = 180.0 / PI;                                 
float travelCoeff             = 1/((float)gyroFrequency * gyroSensibility);     
float travelCoeffToRad        = travelCoeff / convDegToRad;

// PWM channels
const int pwmLedChannel       = 0;
const int pwmChannel1         = 1;
const int pwmChannel2         = 2;
const int pwmChannel3         = 3;
const int pwmChannel4         = 4;
const int pwmLedFlyChannel    = 5;

// PWM constants
const int freq = 500;                                               // (Hz) for what I know, 500 is the best 
const int resolution = 11;                                          // (bits) 11 is the best guess
const int MAX_DUTY_CYCLE = (int)(pow(2, resolution) - 1);
const int HALF_DUTY_CYCLE = (int)(0.5*MAX_DUTY_CYCLE);

// battery
// total resistance calculations, the important quantity is totalDrop
const int DIODE_DROP = 700;                                         //generally it is -0.7V
float res3 = 2.5;                                                   // resistance between Vin and vout
float res2 = 1.;                                                    // load resistance
float totalDrop = res2 / (res2 + res3);                             // IMPORTANT: this is in my case, you have to calculate YOUR total drop

// digital read bits accuracy
uint8_t adcBits = 12;                                               // (bits) of width when measuring the voltage
float maximumWidth = pow(2., (float)adcBits)-1;                     // maximum width that the pin can read

// battery calculations
float fromVtoWidth = maximumWidth / BOARD_LIMIT_VOLTAGE;
float maxBatteryLevelDropped = (float)(MAX_BATTERY_VOLTAGE-DIODE_DROP) * totalDrop;
float correctionBattery = (float)BOARD_LIMIT_VOLTAGE/maxBatteryLevelDropped;
double minBatteryLevelThreshold = ((double)MIN_BATTERY_VOLTAGE-(double)DIODE_DROP) * totalDrop * correctionBattery;

// altimeter
uint8_t osrs_t = 1;                                                 //Temperature oversampling x 1
uint8_t osrs_p = 1;                                                 //Pressure oversampling x 1
uint8_t barometerMode = 3;                                          //Normal barometerMode
uint8_t t_sb = 5;                                                   //Tstandby 1000ms
uint8_t filter = 0;                                                 //Filter off 
uint8_t spi3w_en = 0;                                               //3-wire SPI Disable
