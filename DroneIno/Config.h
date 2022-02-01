// Config.h
// @author: Sebastiano Cocchi
// 
// legend:
//      * = not available at the moment

// MICROCONTROLLER BOARD
#define PINMAP                      ESP32                   // (OFF*, ATMEGA32*, ESP32) OFF if the board is Arduino Uno (directly write pin)

// DEBUG MODE
#define DEBUG                       false                   // (true, false) true enable serial prints for debugging
                                                            
// SKETCH CONSTANTS
#define BAUD_RATE                   115200                  // (9600, 57600, 115200)
#define WIRE_CLOCK                  400000L                 // (100000L, 400000L) 400000L is suggested
#define EEPROM_SIZE                 36                      // eeprom allocatable memory

// AUTO LEVElING
#define AUTO_LEVELING               true                    // (true, false)

// BATTERY
#define MAX_BATTERY_VOLTAGE         11100                   // (mV) battery nominal maximum voltage (use ONLY 11.1V batteries)
#define MIN_BATTERY_VOLTAGE         5000                    // (mV) battery level above which it is dangerous to use
#define DIODE_DROP                  700                     // generally it is -0.7V

// SENSORS
#define GYROSCOPE                   MPU6050                 // unique for now
#define ALTITUDE_SENSOR             OFF                     // (OFF, BMP280, BME280*)
#define PROXIMITY_SENSOR            OFF                     // (OFF, HCSR04*)
#define DANGER_DISTANCE             40                      // (cm) Proximity sensor distance above which is considered a danger zone

// PID:
//              (ROLL)
#define PID_P_GAIN_ROLL             1.3                     //Gain setting for the roll P-controller (1.3)
#define PID_I_GAIN_ROLL             0.00035                  //Gain setting for the roll I-controller (0.04)
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

//            (ALTITUDE)
#define PID_P_GAIN_ALTITUDE         1.4                      //Gain setting for the altitude P-controller (default = 1.4).
#define PID_I_GAIN_ALTITUDE         0.2                      //Gain setting for the altitude I-controller (default = 0.2).
#define PID_D_GAIN_ALTITUDE         0.75                     //Gain setting for the altitude D-controller (default = 0.75).
#define PID_MAX_ALTITUDE            400                      //Maximum output of the PID-controller (+/-).
