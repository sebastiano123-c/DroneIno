/**
 * @file Constants.h
 * @author @sebastiano123-c
 * @brief DroneIno constants for macros.
 * @version 0.1
 * @date 2022-02-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */


//      (Bool)
#define OFF                         0
#define ON                          1


//     (Boards)
#ifdef ESP32
    #undef ESP32
    #define ESP32                   2
#endif
#define ATMEGA32                    3


//      (Sketch type)
#define SETUP                       4
#define CALIBRATION                 5
#define FLIGHT_CONTROLLER           6


//      (Sensors)
#define MPU6050                     10
#define BMP280                      11
#define BME280                      12
#define HCSR04                      13


//      (WiFi)
#define NATIVE                      15
#define ESP_CAM                     16
