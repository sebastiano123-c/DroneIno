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
//      ESP32 is yet defined by compilers
#define ATMEGA32                    3
#define ESP32_D1_R32                4
#define ESP32_DEVKIT                5

//      (Serial type)
#define COMMON_SERIAL               20
#define WEB_SERIAL                  21

//      (Sketch type)
#define SETUP                       6
#define TEST_COMPONENTS             7
#define PROPELLERS_CALIBRATION      8
#define FLIGHT_CONTROLLER           9


//      (Sensors)
#define MPU6050                     10
#define BMP280                      11
#define BME280                      12
#define HCSR04                      13
#define BN_880                      14

//      (WiFi)
#define NATIVE                      15
#define ESP_CAM                     16
