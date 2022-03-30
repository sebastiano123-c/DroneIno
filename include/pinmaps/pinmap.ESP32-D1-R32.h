/**
 * @file pinmap.ESP32-D1-R32.h
 * @author @sebastiano123-c
 * @brief ESP32 D1 R32 pinmap.
 * @version 0.1
 * @date 2022-03-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */


// board limit voltage of the pins, for ESP32 is 3.3 V
#define BOARD_LIMIT_VOLTAGE         3.300


//      (ESC)
#define PIN_ESC_1                   17
#define PIN_ESC_2                   16
#define PIN_ESC_3                   27
#define PIN_ESC_4                   14

//      (RECEIVER)
#define PIN_RECEIVER_1              12
#define PIN_RECEIVER_2              13
#define PIN_RECEIVER_3              5
#define PIN_RECEIVER_4              23
#define PIN_RECEIVER_5              4

//      (LEDs)
#define PIN_BATTERY_LED             18 
#define PIN_SECOND_LED              19                         // if you don't have it, leave here an unused pin

//      (BATTERY LEVEL)
#define PIN_BATTERY_LEVEL           39                         // input pin to read the battery level
#define PIN_BATTERY_CHANNEL         ADC1_CHANNEL_0             // ADC channel related to GPIO of PIN_BATTERY_LEVEL  

// PROXIMITY_SENSOR
// #define PIN_PROXIMITY_SENSOR_ECHO   26
// #define PIN_PROXIMITY_SENSOR_TRIG   25

//      (I2C PINS)
#define PIN_SDA                     21
#define PIN_SCL                     22

//      (UART0 - Serial)
#define PIN_RX0                     1
#define PIN_TX0                     3

//      (UART1 - Serial1)
#define PIN_RX1                     26                           // by default is 16, but it is occupied by PIN_ESCs
#define PIN_TX1                     25                           // by default is 17, but it is occupied by PIN_ESCs

//      (UART2 - Serial2)
#define PIN_RX2                     36
#define PIN_TX2                     34

