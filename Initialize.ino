// Initialize 
// @author: Sebastiano Cocchi
#include "Initialize.h"

void initialize(AutoLeveling myAutoLev, Adafruit_BMP280 bmp){
    Serial.begin(BAUD_RATE);

    Wire.begin();                                                             //Start the I2C as master.
    Wire.setClock(400000L);
        
    myAutoLev.begin();                                                       // inizialize auto-leveling
        
    // PROXIMITY SENSOR
    #if PROXIMITY_SENSOR == HCSR04
        pinMode(PIN_PROXIMITY_SENSOR_TRIG, OUTPUT);     // Sets the TRIG as an OUTPUT
        pinMode(PIN_PROXIMITY_SENSOR_ECHO, INPUT);      // Sets the ECHO as an INPUT
    #endif

    // ALTITUDE SENSOR
    #if ALTITUDE_SENSOR == BMP280
        bmp.begin();
    #endif

    //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.
    DDRD |= B11110000;                                                        //Configure digital port 4, 5, 6 and 7 as output.
    DDRB |= B00110000;                                                        //Configure digital port 12 and 13 as output.
    // pinMode(PIN_ESC_1, OUTPUT);
    // pinMode(PIN_ESC_2, OUTPUT);
    // pinMode(PIN_ESC_3, OUTPUT);
    // pinMode(PIN_ESC_4, OUTPUT);
    // pinMode(PIN_BATTERY_LED, OUTPUT);
    // pinMode(PIN_DIGITAL_13, OUTPUT);

    //Use the led on the Arduino for startup indication.
    digitalWrite(PIN_BATTERY_LED, HIGH);                                                    //Turn on the warning led.

    //Check the EEPROM signature to make sure that the setup program is executed.
    while(myAutoLev.eepromData[33] != 'J' || myAutoLev.eepromData[34] != 'M' || myAutoLev.eepromData[35] != 'B') delay(10);

    //The flight controller needs the MPU-6050 with gyro and accelerometer
    if(myAutoLev.eepromData[31] == 2 || myAutoLev.eepromData[31] == 3) delay(10);//If setup is completed without MPU-6050 stop the flight controller program  

    myAutoLev.setGyroscopeRegisters();                                                     //Set the specific gyro registers.
    myAutoLev.calibrateGyroscope();  

    PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
    PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT3);                                                  //Set PCINT3 (digital input 11)to trigger an interrupt on state change.

    //Load the battery voltage to the batteryVoltage variable.
    myAutoLev.getBatteryVoltage();

    //Set the timer for the next loop.
    myAutoLev.getLoopTimer();                                                

    //When everything is done, turn off the led.
    digitalWrite(PIN_BATTERY_LED, LOW);                                                     //Turn off the warning led.
}
