/**
 * @file Prototypes.h
 * @author @sebastiano123-c
 * @brief Definition of the function prototypes.
 * @version 0.1
 * @date 2022-03-20
 * 
 * Definition of all the functions used by DroneIno.
 * 
 * @copyright Copyright (c) 2022
 * 
 */


void myISR();//void*dummy);                                                 // see ISR.h


void setupPins();                                             // see Initialize.h


void initEEPROM();                                            // see Initialize.h


void printEEPROM();                                           // see Initialize.h


void intro();                                                 // see Initialize.h


void setupWireI2C();                                          // see Initialize.h           


void setGyroscopeRegisters();                                 // see Gyroscope.h            


void readGyroscopeStatus();                                   // see Gyroscope.h  


void calculateAnglePRY();                                     // see Gyroscope.h  


void calibrateGyroscope();                                    // see Gyroscope.h


void printGyroscopeStatus();                                  // see Gyroscope.h  


int convertReceiverChannel(byte ch);                          // see.ESC.h


void configureReceiverTrims();                                // see Initialize.h        


#if UPLOADED_SKETCH == SETUP                                  // all these functions are in Setup.h

    void welcomeMsg();

    void putTrimsInTheMiddle();

    void checkTrimInversion(byte ch);

    void findSticksLimits();

    void configureGyroscopeAxes(byte ch);

    void checkLed();

    void checkGyroscopeResult();

    void writeEEPROM();

#elif UPLOADED_SKETCH == CALIBRATION                          // functions without comments are in Calibration.h

    void setupSerial();

    void calibrationMsg();

    void printEEPROM();

    void printSignals();

    void getSerialMsg();

    void rFunction();

    void escFunction();

    void blinkLed();

    void checkAltitudeSensor();                               // see Altitude.h

    void calculateAltitudeHold();                             // see Altitude.h

    void printBarometer();                                    // see Altitude.h

    void initBattery();                                       // see Battery.h

    void readBatteryVoltage();                                // see Battery.h

    void printBatteryVoltage();                               // see Battery.h

    void setupGPS();                                          // see GPS.h

    void readGPS();                                           // see GPS.h
    
    void printGPS();                                          // see GPS.h

    void printGPSSerialLine();                                // see GPS.h

#elif UPLOADED_SKETCH == FLIGHT_CONTROLLER


    void setupWiFiTelemetry();                                // see WiFiTelemtry.h  


    void checkAltitudeSensor();                               // see Altitude.h
    

    void initBattery();                                       // see Battery.h


    void droneStart();


    void calculateAltitudeHold();                             // see Altitude.h


    void calculatePID();                                      // see PID.h

    void printPIDGainParameters();

    void readBatteryVoltage();                                // see Battery.h


    void printBatteryVoltage();                               // see Battery.h

    void setupGPS();                                          // see GPS.h

    void readGPS();                                           // see GPS.h


    #if WIFI_TELEMETRY == NATIVE

        void notFound(AsyncWebServerRequest *request);

        const char* index_html();

        String processor(const String& var);

    #elif WIFI_TELEMETRY == ESP_CAM

        void writeDataTransfer();

    #endif

    void sendWiFiTelemetry();                                 // see WiFi.h

#endif


void convertAllSignals();                                 // see ESC.h


void setEscPulses();                                      // see ESC.h


void waitController();                                    // see Controller.h               
