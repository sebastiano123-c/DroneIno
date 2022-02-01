// ESP32 pinmap
// @author: Sebastiano Cocchi

// ATmega32 pinmap
// @author: Sebastiano Cocchi

// ESC
#define PIN_ESC_1 17
#define PIN_ESC_2 16
#define PIN_ESC_3 27
#define PIN_ESC_4 14

// RECEIVER
#define PIN_RECEIVER_1 12
#define PIN_RECEIVER_2 13
#define PIN_RECEIVER_3 5
#define PIN_RECEIVER_4 23
#define PIN_RECEIVER_5 4

// LED
#define PIN_BATTERY_LED             19 
#define PIN_SECOND_LED              18 

// BATTERY LEVEL
#define PIN_BATTERY_LEVEL           2

// PROXIMITY_SENSOR
#define PIN_PROXIMITY_SENSOR_ECHO   26
#define PIN_PROXIMITY_SENSOR_TRIG   25

// I2C PINS
#define PIN_SDA                     21
#define PIN_SCL                     22

// board limit voltage of the pins, for ESP32 is 3.mV
#define BOARD_LIMIT_VOLTAGE         3300
