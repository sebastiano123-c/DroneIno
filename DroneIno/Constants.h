/*
* Constants
* @author @sebastiano123-c
*/


// PID:
//              (ROLL)
float PID_P_GAIN_ROLL            = 1.3;                      //Gain setting for the roll P-controller (1.3)
float PID_I_GAIN_ROLL            = 0.001;                    //Gain setting for the roll I-controller  (0.0002)
float PID_D_GAIN_ROLL            = 8.0;                      //Gain setting for the roll D-controller (10.0)
int PID_MAX_ROLL                 = 400;                      //Maximum output of the PID-controller   (+/-)

//              (PITCH)
float PID_P_GAIN_PITCH           = PID_P_GAIN_ROLL;          //Gain setting for the pitch P-controller
float PID_I_GAIN_PITCH           = PID_I_GAIN_ROLL;          //Gain setting for the pitch I-controller
float PID_D_GAIN_PITCH           = PID_D_GAIN_ROLL;          //Gain setting for the pitch D-controller
int PID_MAX_PITCH                = PID_MAX_ROLL;             //Maximum output of the PID-controller   (+/-)

//              (YAW)
float PID_P_GAIN_YAW             = 2.2;                      //Gain setting for the pitch P-controller. (2.0)
float PID_I_GAIN_YAW             = 0.01;                     //Gain setting for the pitch I-controller. (0.04)
float PID_D_GAIN_YAW             = 0.0;                      //Gain setting for the pitch D-controller. (0.0)
int PID_MAX_YAW                  = 400;                      //Maximum output of the PID-controller     (+/-)

//            (ALTITUDE)
float PID_P_GAIN_ALTITUDE        = 1.4;                      //Gain setting for the altitude P-controller (default = 1.4).
float PID_I_GAIN_ALTITUDE        = 0.3;                      //Gain setting for the altitude I-controller (default = 0.2).
float PID_D_GAIN_ALTITUDE        = 0.75;                     //Gain setting for the altitude D-controller (default = 0.75).
int PID_MAX_ALTITUDE             = 400;                      //Maximum output of the PID-controller (+/-).


// gyro constants
const int gyroFrequency          = 250;                               // (Hz)
const float gyroSensibility      = 65.5;                                 
const int correctionPitchRoll    = 15;                                // correction for the pitch and roll
float convDegToRad               = 180.0 / PI;                                 
float travelCoeff                = 1/((float)gyroFrequency * gyroSensibility);     
float travelCoeffToRad           = travelCoeff / convDegToRad;
float anglePitchOffset           = 0;                                 // NOT touch, for future 
float angleRollOffset            = 0;                                 // NOT touch, for future 
  
// PWM channels
const int pwmLedChannel          = 0;
const int pwmChannel1            = 1;
const int pwmChannel2            = 2;
const int pwmChannel3            = 3;
const int pwmChannel4            = 4;
const int pwmLedFlyChannel       = 5;

// PWM constants
const int freq                   = 500;                                   // (Hz) for what I know, 500 is the best 
const int resolution             = 11;                                    // (bits) 11 is the best guess
const int MAX_DUTY_CYCLE         = (int)(pow(2, resolution) - 1);
const int HALF_DUTY_CYCLE        = (int)(0.5*MAX_DUTY_CYCLE);

// battery
// total resistance calculations, the important quantity is totalDrop
const int DIODE_DROP             = 700;                                   //generally it is -0.7V
float res3                       = 2.5;                                   // resistance between Vin and vout
float res2                       = 1.;                                    // load resistance
float totalDrop                  = res2 / (res2 + res3);                  // IMPORTANT: this is in my case, you have to calculate YOUR total drop

// digital read bits accuracy
uint8_t adcBits                  = 12;                                    // (bits) of width when measuring the voltage
float maximumWidth               = pow(2., (float)adcBits)-1;             // maximum width that the pin can read

// battery calculations
double fromVtoWidth              = maximumWidth / (double)BOARD_LIMIT_VOLTAGE;
double maxBatteryLevelDropped    = (double)(MAX_BATTERY_VOLTAGE-DIODE_DROP) * totalDrop;
double correctionBattery         = (double)BOARD_LIMIT_VOLTAGE/maxBatteryLevelDropped;
double minBatteryLevelThreshold  = ((double)MIN_BATTERY_VOLTAGE-(double)DIODE_DROP) * totalDrop * correctionBattery;

// altimeter
uint8_t osrs_t = 1;                                                 //Temperature oversampling x 1
uint8_t osrs_p = 1;                                                 //Pressure oversampling x 1
uint8_t barometerMode = 3;                                          //Normal barometerMode
uint8_t t_sb = 5;                                                   //Tstandby 1000ms
uint8_t filter = 0;                                                 //Filter off 
uint8_t spi3w_en = 0;                                               //3-wire SPI Disable

// wifi telemetry
const char *ssid        = "DroneInoTelemetry";
const char *password    = "DroneIno";
const int refreshRate   = 200;                                      // (ms) the refresch rate of the page
int refreshCounter      = 0;
// roll
const char* P_ROLL_GET  = "rollP";
const char* I_ROLL_GET  = "rollI";
const char* D_ROLL_GET  = "rollD";
// pitch
const char* P_PITCH_GET = "pitchP";
const char* I_PITCH_GET = "pitchI";
const char* D_PITCH_GET = "pitchD";
// yaw
const char* P_YAW_GET   = "yawP";
const char* I_YAW_GET   = "yawI";
const char* D_YAW_GET   = "yawD";

// PID adjustment using wifi
AsyncWebServer server(80);

// // esp-now telemetry
// uint8_t broadcastAddress[] = {0x58, 0xBF, 0x25, 0x82, 0x4D, 0x08}; // RECEIVER MAC Address
// // Create an Event Source on /events
// AsyncEventSource events("/events");
