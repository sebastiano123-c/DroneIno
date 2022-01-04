#ifndef INITIALIZE
#define INITIALIZE

    #include "Config.h"
    #include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
    #include <EEPROM.h>                        //Include the EEPROM.h library so we can store information onto the EEPROM
    #include <Arduino.h>
    #include "src/Autoleveling.h"
    // #include "src/Models.h"
    #include "SPI.h" //Why? Because library supports SPI and I2C connection
    #include <Adafruit_Sensor.h>
    #include <Adafruit_BMP280.h>

    void initialize(AutoLeveling myAutoLev, Adafruit_BMP280 bmp);

#endif 