/**
 * @file Config.h
 * @author @sebastiano123-c
 * @brief Configuration file.
 * 
 * Config.h is intended to fit as your drone developing.
 * 
 * Accordingly to the documentation, after you built your drone:
 *  @li define UPLOADED_SKETCH as SETUP and upload the sketch
 *  @li define UPLOADED_SKETCH as CALIBRATION and upload the sketch
 *  @li define UPLOADED_SKETCH as FLIGHT_CONTROLLER and upload the sketch
 * 
 * ! ALWAYS WITHOUT PROPELLERS FIRST !
 * 
 * @note * = developing
 * @note ** = not available at the moment
 * @version 0.1
 * @date 2022-02-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */



/**
 * @brief SKETCH CONFIG
 * 
 */
//      (MICROCONTROLLER BOARD)
#define PINMAP                      ESP32                    // (OFF**, ATMEGA32**, ESP32) OFF if the board is Arduino Uno (directly write pin)

//      (SKETCH TO BE UPLOADED)
//      Defines the compiler which sketch is to be uploaded.
//      Following the list:
//          1) SETUP               tells the compiler to use the setup sketch,
//          2) CALIBRATION         ... calibration sketch,
//          3) FLIGHT_CONTROLLER   ... flight controller
//      and uploading the sketch each time, you first begin with SETUP.
//      This way you will calibrate RC-controller and more, and this will save information into EEPROM.
//      Then, upload with CALIBRATION and calibrate ESC and check that each components behaves in the correct way.
//      Finally upload with FLIGHT_CONTROLLER and start to fly.
#define UPLOADED_SKETCH             FLIGHT_CONTROLLER        // (SETUP, CALIBRATION, FLIGHT_CONTROLLER) 

//      (DEBUG MODE)
//      If true, some serial prints are enabled. Otherwise, it is not auspicable to set DEBUG true when flying
#define DEBUG                       false                    // (true, false) true enable serial prints for debugging

//      (SKETCH CONSTANTS)
#define BAUD_RATE                   115200                   // (9600, 57600, 115200)
#define WIRE_CLOCK                  400000L                  // (100000L, 400000L) 400000L is suggested
#define EEPROM_SIZE                 36                       // EEPROM size for the allocatable memory



/**
 * @brief AUTO LEVELING
 * 
 * If true, the drone will adjust its flying accordingly to the gyroscope readings.
 * If false, disables every autoleveling compensation.
 * Till now, MPU6050 is the only gyroscope admitted on DroneIno.
 */
#define AUTO_LEVELING               true                     // (true, false) 
#define GYROSCOPE                   MPU6050                  // (MPU6050) unique for now



/**
 * @brief ALTITUDE
 * 
 */
#define ALTITUDE_SENSOR             BMP280                   // (OFF, BMP280*, BME280**)



/**
 * @brief GPS
 * 
 */
//      
#define GPS                         BN_880                   // (OFF, BN_880*)
#define GPS_BAUD                    9600                     //

/**
 * @brief BATTERY CONFIG
 * 
 */
//      (BATTERY LIMITS)
#define MAX_BATTERY_VOLTAGE         11.40                    // (V) battery nominal maximum voltage (use ONLY 11.1V batteries)
#define MIN_BATTERY_VOLTAGE         6.00                     // (V) battery level under which it is dangerous to use

//      (COMPONSATE)
//      When flying, the motors rotating cause a voltage diminuition.
//      If true, BATTERY_COMPENSATION will compensate this behavior.
#define BATTERY_COMPENSATION        true                     // (true, false)

//      (VOLTAGE PARTITOR)
//      The voltage partitor is: 
//            11.1V --- res 1 ---+--- res2 --- GND
//                              Vpin              
#define RESISTANCE_1                5.10                     // (kOhm) the resistance before V_pin
#define RESISTANCE_2                2.22                     // (kOhm) the resistance after V_pin

//      (WIFI TELEMETRY)
//      Using WiFi a telemtry system is done, making everything simple and easy reach.
//      Use it to fine-tune your PID or fix gyroscope set point and altitude hold PID parameters.
//      Using this you can adjust on the fly these parameters and much more:
//          *) NATIVE uses the ESP32 wifi AP,
//          *) ESP_CAM uses the ESP32CAM wifi.
//      See https://github.com/sebastiano123-c/Esp32-cam-telemetry for more details
#define WIFI_TELEMETRY              ESP_CAM                  // (NATIVE, ESP_CAM) 



//      (SENSORS)
// #define PROXIMITYSENSOR             OFF                      // (OFF, HCSR04*)
// #define DANGER_DISTANCE             40                       // (cm) Proximity sensor distance under which is considered a danger zone
