// Config.h
// @author: Sebastiano Cocchi
// 
// legend:
//      * = not available at the moment

// MICROCONTROLLER 
#define PINMAP                       ESP32                  // (OFF*, ATMEGA32*, ESP32) OFF if the board is Arduino Uno (directly write pin)
                                                            
// SKETCH CONSTANTS
#define BAUD_RATE                   115200                  // (9600, 57600, 115200)
#define WIRE_CLOCK                  400000L                 // (100000L, 400000L) 400000L is suggested
#define EEPROM_SIZE                 36                      // eeprom allocatable memory

// DEBUG MODE
#define DEBUG                       false                   // (true, false) true enable serial prints for debugging

// AUTOLEVEING
#define AUTO_LEVELING               true                    // (true, false)

// GYROSCOPE
#define GYROSCOPE                   MPU6050                 // unique for now

// ALTITUDE SENSOR
#define ALTITUDE_SENSOR             OFF                     // (OFF, BMP280, BME280*)

// ULTRASONIC SENSOR
#define PROXIMITY_SENSOR            OFF                     // (OFF, HCSR04*)
#define DANGER_DISTANCE             40                      // (cm) Proximity sensor distance above which is considered a danger zone

// PID:
//              (ROLL)
#define PID_P_GAIN_ROLL             1.3                     //Gain setting for the roll P-controller (1.3)
#define PID_I_GAIN_ROLL             0.0001                  //Gain setting for the roll I-controller (0.04)
#define PID_D_GAIN_ROLL             15.0                    //Gain setting for the roll D-controller (18.0)
#define PID_MAX_ROLL                400                     //Maximum output of the PID-controller (+/-)

//              (PITCH)
#define PID_P_GAIN_PITCH            PID_P_GAIN_ROLL          //Gain setting for the pitch P-controller
#define PID_I_GAIN_PITCH            PID_I_GAIN_ROLL          //Gain setting for the pitch I-controller
#define PID_D_GAIN_PITCH            PID_D_GAIN_ROLL          //Gain setting for the pitch D-controller
#define PID_MAX_PITCH               PID_MAX_ROLL             //Maximum output of the PID-controller (+/-)

//              (YAW)
#define PID_P_GAIN_YAW              2.0                      //Gain setting for the pitch P-controller. //4.0
#define PID_I_GAIN_YAW              0.02                     //Gain setting for the pitch I-controller. //0.02
#define PID_D_GAIN_YAW              0.0                      //Gain setting for the pitch D-controller.
#define PID_MAX_YAW                 400                      //Maximum output of the PID-controller (+/-)
