// PINMAP
// @author: Sebastiano Cocchi
#include "Constants.h"
#include "Config.h"

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
#elif PINMAP == ESP32_D1_R32                            // ESP32 D1 R32
  #define PINMAP_STR "ESP32D1R32"
  #include "pinmaps/pinmap.ESP32-D1-R32.h"
#elif PINMAP == ESP32_DEVKIT                            // ESP32 Dev Kit
  #define PINMAP_STR "ESP32DevKit"
  #include "pinmaps/pinmap.ESP32.h"
#endif

// GYROSCOPE SENSOR
#if GYROSCOPE == MPU6050
  #include "sensors/gyroscope.MPU6050.h"
#endif

// ALTITUDE SENSOR
#if ALTITUDE_SENSOR == BMP280
  #include "sensors\altitude_sensor.BMP280.h"
#elif ALTITUDE_SENSOR == BME280
  #include "sensors\altitude_sensor.BME280.h"
#endif

// PROXIMITY SENSOR
#if PROXIMITY_SENSOR == HCSR04
  #include <HCSR04.h>
#endif

// SERIAL
#if UPLOADED_SKETCH == CALIBRATION && SERIAL_TYPE == WEB_SERIAL
  #include "wifi/serial.web_serial.h"
#endif

// WIFI TELEMETRY
#if UPLOADED_SKETCH == FLIGHT_CONTROLLER && WIFI_TELEMETRY == ESP_CAM
  #include "wifi/wifi_telemetry.esp_cam.h"
#elif UPLOADED_SKETCH == FLIGHT_CONTROLLER && WIFI_TELEMETRY == NATIVE
  #include "wifi/wifi_telemetry.native.h"
#endif


/**
 *  GPS 
 */
#if GPS == BN_880
  #include "sensors/gps.BN_880.h"
#endif