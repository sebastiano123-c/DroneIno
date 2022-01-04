#ifndef BMP280_ino
    #define BMP280_ino
    #include "SPI.h" //Why? Because library supports SPI and I2C connection
    #include <Adafruit_Sensor.h>
    #include <Adafruit_BMP280.h>
    #include <Arduino.h>
    
    void readPresTempAlt(Adafruit_BMP280 bmp);
#endif