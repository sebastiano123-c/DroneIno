// Config.h
// @author: Sebastiano Cocchi

// micro controller 
#define PINMAP OFF                  // OFF if the board is Arduino Uno (directly write pin)
                                    // ATmega32, ESP32 if there is digitalWrite()

#define BAUD_RATE 57600              // 9600, 57600

// autoleveling
#define AUTO_LEVELING ON            // ON, OFF

// altitude sensor
#define ALTITUDE_SENSOR BMP280      // OFF, BMP280, BME280

// ultrasonic sensor
#define PROXIMITY_SENSOR HCSR04     // OFF, HCSR04
#define DANGER_DISTANCE 40          //(cm) Proximity sensor distance above which is considered a danger zone


// USER DEFINED PINS  - - - - - - - PROVVISORIO
// BATTERY LEVEL 
#define PIN_BATTERY_LED 12 
#define PIN_DIGITAL_13 13 
#define PIN_BATTERY_LEVEL 0

// PROXIMITY_SENSOR
#define PIN_PROXIMITY_SENSOR_ECHO 2
#define PIN_PROXIMITY_SENSOR_TRIG 3

// ALTITUDE_SENSOR
#define PIN_ALTITUDE_SENSOR_SDA 4
#define PIN_ALTITUDE_SENSOR_SCL 5