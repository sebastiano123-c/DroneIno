// PINMAP
// @author: Sebastiano Cocchi

// PINMAP
#if PINMAP == OFF                                       // AVR_ATmega324U predefined
  #define PINMAP_STR "AVR_ATmega324U"
  // #include <avr/io.h>
  #include <avr/iom32u4.h>
#endif
#if PINMAP == ATmega32                                  // ATmega32 
  #define PINMAP_STR "ATmega32"
  #include "pinmaps/pinmap.ATmega32.h"
#endif

// // ALTITUDE SENSOR
// #if ALTITUDE_SENSOR == BMP280                            // BMP280
//   #include "SPI.h" //Why? Because library supports SPI and I2C connection
//   #include <Adafruit_Sensor.h>
//   #include <Adafruit_BMP280.h>
// #endif

// // ALTITUDE SENSOR
// #if PROXIMITY_SENSOR == HCSR04                            // BMP280
//   #include <HCSR04.h>
// #endif