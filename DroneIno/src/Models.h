// PINMAP
// @author: Sebastiano Cocchi

// PINMAP
#if PINMAP == OFF                                       // AVR_ATmega324U predefined
  #define PINMAP_STR "AVR_ATmega324U"
  // #define __AVR_ATmega32U4__ 
  // #include <avr/io.h>
  // #include <avr/iom32u4.h>
  #include "pinmaps/pinmap.OFF.h"
#elif PINMAP == ATmega32                                // ATmega32 
  #define PINMAP_STR "ATmega32"
  #include "pinmaps/pinmap.ATmega32.h"
#elif PINMAP == ESP32                                   // ESP32 
  #define PINMAP_STR "ESP32"
  #include "pinmaps/pinmap.ESP32.h"
#endif

// GYROSCOPE SENSOR
#if GYROSCOPE == MPU6050
  #include "sensors/sensor.MPU6050.h"
#endif

// ALTITUDE SENSOR
#if ALTITUDE_SENSOR == BMP280
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BMP280.h>
#endif

// PROXIMITY SENSOR
#if PROXIMITY_SENSOR == BMP280
  #include <HCSR04.h>
#endif