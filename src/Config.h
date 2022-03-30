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
 * ----------------------------------------------------------------------------------------------------------------------------
 *  SKETCH CONFIGURATION
 * 
 *      (MICROCONTROLLER BOARD)
 */
#define PINMAP                      ESP32                    // (OFF**, ATMEGA32**, ESP32) OFF if the board is Arduino Uno (directly write pin)

/**
 *      (SKETCH TO BE UPLOADED)
 *      Defines the compiler which sketch is to be uploaded.
 *      Following the list:
 *          1) SETUP               tells the compiler to use the setup sketch,
 *          2) CALIBRATION         ... calibration sketch,
 *          3) FLIGHT_CONTROLLER   ... flight controller
 *      and uploading the sketch each time, you first begin with SETUP.
 *      This way you will calibrate RC-controller and more, and this will save information into EEPROM.
 *      Then, upload with CALIBRATION and calibrate ESC and check that each components behaves in the correct way.
 *      Finally upload with FLIGHT_CONTROLLER and start to fly.
 */
#define UPLOADED_SKETCH             FLIGHT_CONTROLLER        // (SETUP, CALIBRATION, FLIGHT_CONTROLLER) 

/**
 *      (DEBUG MODE)
 *      If true, some serial prints are enabled.
 *      Otherwise, it is not auspicable to set DEBUG true when flying.
 */
#define DEBUG                       false                    // (true, false) true enable serial prints for debugging

//      (SKETCH CONSTANTS)
#define BAUD_RATE                   115200                   // (9600, 57600, 115200)
#define WIRE_CLOCK                  400000L                  // (100000L, 400000L) 400000L is suggested
#define EEPROM_SIZE                 36                       // EEPROM size for the allocatable memory



/**
 * ----------------------------------------------------------------------------------------------------------------------------
 *  AUTO LEVELING
 * 
 *      If true, the drone will adjust its flying accordingly to the gyroscope readings.
 *      If false, disables every autoleveling compensation.
 *      Till now, MPU6050 is the only gyroscope admitted on DroneIno.
 */
#define AUTO_LEVELING               true                     // (true, false) 
#define GYROSCOPE                   MPU6050                  // (MPU6050) unique for now



/**
 * ----------------------------------------------------------------------------------------------------------------------------
 *  ALTITUDE
 * 
 *      Altitude readings are relevant for the 'altitude hold' feature.
 *      Leave OFF if you don't have any pressure sensor installed on DroneIno.
 *      BMP280 is the only sensor I have tested and, probably, not the most sensitive one.
 *      In the future, I will test other sensors.
 */
#define ALTITUDE_SENSOR             BMP280                   // (OFF, BMP280*, BME280**)



/**
 * ----------------------------------------------------------------------------------------------------------------------------
 *  GPS
 * 
 *      GPS sensor is useful to recover the latitude and longitude position of DroneIno.
 *      GPSs talk with DroneIno using a serial communication (TX/RX).
 *      String printed on serial are then read by the board and converted into information in the file GPS.h.
 *      Leave OFF if you don't have a GPS installed.
 *      Till now I have tested the Beitian BN 880 (which also incorporates the compass).
 *      For what I know, BN 880 should be very similar to the Ubox M8N. 
 * 
 */
#define GPS                         BN_880                   // (OFF, BN_880*)
#define GPS_BAUD                    9600                     // (9600, 57600, 115200) 9600 should be ok
#define UTC_TIME_ZONE               2                        // (0-23) Put your time zone here, for example 2 stands for UTC+2 



/**
 * ----------------------------------------------------------------------------------------------------------------------------
 *  BATTERY
 * 
 *      (BATTERY SPECS)
 *      I use a 3s LiPo Battery with 11.1V, 2200mAh capacity and 50C discharge rate
 *      For 3s batteries, the danger voltage is 3*3 V = 9 V, but it is preferable 
 *      to stop flying before reaching 9V.
 *      The warning battery voltage defines the value under which a led starts blinking.
 */     
#define NUMBER_OF_CELLS             3                        // (V) battery nominal maximum voltage (use ONLY 11.1V batteries)
#define WARNING_BATTERY_VOLTAGE     10.00                    // (V) when drone reaches this value, it will not take off 

/**
 *      (COMPENSATION*)
 *      When flying, the motors rotating cause a voltage diminuition.
 *      If true, BATTERY_COMPENSATION will compensate this behavior.
 *       
 *      WARNING: THIS FEATURE IS DISABLED DUE TO INCORRECTNESS, PLEASE LEAVE false.
 * 
 */
#define BATTERY_COMPENSATION        false                     // (true, false) IMPORTANT: leave false for now

/**
 *      (VOLTAGE DIVIDER)
 *      The voltage divider is: 
 *            Vin --- res 1 ---+--- res2 --- GND
 *                             |
 *                           V_pin     
 * 
 *      @note choose res1 and res2 so that the maximum V_pin is less than BOARD_MAXIMUM_VOLTAGE         
 */
#define RESISTANCE_1                5.11                     // (kOhm) the resistance before V_pin
#define RESISTANCE_2                1.55                     // (kOhm) the resistance after V_pin
#define TOTAL_DROP                  RESISTANCE_2 / (RESISTANCE_1 + RESISTANCE_2)


/**
 * ----------------------------------------------------------------------------------------------------------------------------
 *  TELEMETRY
 * 
 *      (WIFI)
 *      Using WiFi a telemtry system is done, making everything simple and easy reach.
 *      Use it to fine-tune your PID or fix gyroscope set point and altitude hold PID parameters.
 *      Using this you can adjust on the fly these parameters and much more:
 *          *) NATIVE uses the ESP32 wifi AP,
 *          *) ESP_CAM uses the ESP32CAM wifi.
 *      See https://github.com/sebastiano123-c/Esp32-cam-telemetry for more details.
 */
#define WIFI_TELEMETRY              ESP_CAM                     // (NATIVE, ESP_CAM) set NATIVE if you don't have an ESP32-CAM



//      (SENSORS)
// #define PROXIMITYSENSOR             OFF                      // (OFF, HCSR04*)
// #define DANGER_DISTANCE             40                       // (cm) Proximity sensor distance under which is considered a danger zone
