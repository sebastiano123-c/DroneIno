// DroneIno
// @author: Sebastiano Cocchi
#include "Config.h"
// #include "src/Models.h"

#include <Arduino.h>
#include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
#include <EEPROM.h>                        //Include the EEPROM.h library so we can store information onto the EEPROM
#include "SPI.h" //Why? Because library supports SPI and I2C connection
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#include "src/Autoleveling.h"

// sensors functions
#include "Initialize.h"
#include "HCSR04.h"
#include "BMP280.h"
#include "AutoLevel.h"
// #include "ISR.h"

#include "Globals.h"

// start the autoleveling
AutoLeveling myAutoLev(autoLeveling);                                       //Start the gyro

#if ALTITUDE_SENSOR == BMP280
    //Setup connection of the sensor
    Adafruit_BMP280 bmp; // I2C
#endif

void setup(){
  initialize(myAutoLev, bmp);
}

// Loop
void loop(){
  readPresTempAlt(bmp);
  dangerAlert();
  autoLevelLoop(myAutoLev);
}

//This routine is called every time input 8, 9, 10 or 11 changed state. This is used to read the receiver signals. 
ISR(PCINT0_vect){
  currentTime = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                                     //Is input 8 high?
    if(lastChannel1 == 0){                                                //Input 8 changed from 0 to 1.
      lastChannel1 = 1;                                                   //Remember current input state.
      timer1 = currentTime;                                               //Set timer1 to currentTime.
    }
  }
  else if(lastChannel1 == 1){                                             //Input 8 is not high and changed from 1 to 0.
    lastChannel1 = 0;                                                     //Remember current input state.
    myAutoLev.receiverInput[1] = currentTime - timer1;                             //Channel 1 is currentTime - timer1.
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                                    //Is input 9 high?
    if(lastChannel2 == 0){                                                //Input 9 changed from 0 to 1.
      lastChannel2 = 1;                                                   //Remember current input state.
      timer2 = currentTime;                                               //Set timer2 to currentTime.
    }
  }
  else if(lastChannel2 == 1){                                             //Input 9 is not high and changed from 1 to 0.
    lastChannel2 = 0;                                                     //Remember current input state.
    myAutoLev.receiverInput[2] = currentTime - timer2;                             //Channel 2 is currentTime - timer2.
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                                    //Is input 10 high?
    if(lastChannel3 == 0){                                                //Input 10 changed from 0 to 1.
      lastChannel3 = 1;                                                   //Remember current input state.
      timer3 = currentTime;                                               //Set timer3 to currentTime.
    }
  }
  else if(lastChannel3 == 1){                                             //Input 10 is not high and changed from 1 to 0.
    lastChannel3 = 0;                                                     //Remember current input state.
    myAutoLev.receiverInput[3] = currentTime - timer3;                             //Channel 3 is currentTime - timer3.

  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                                    //Is input 11 high?
    if(lastChannel4 == 0){                                                //Input 11 changed from 0 to 1.
      lastChannel4 = 1;                                                   //Remember current input state.
      timer4 = currentTime;                                               //Set timer4 to currentTime.
    }
  }
  else if(lastChannel4 == 1){                                             //Input 11 is not high and changed from 1 to 0.
    lastChannel4 = 0;                                                     //Remember current input state.
    myAutoLev.receiverInput[4] = currentTime - timer4;                             //Channel 4 is currentTime - timer4.
  }
}
