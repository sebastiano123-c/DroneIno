/**
 * @file DroneIno.ino
 * @author @sebastiano123-c
 * @brief Pilot your ESP32 based DIY drone.
 * 
 * @version 0.2
 * @date 2022-02-18
 * 
 * 
 * TERMS OF USE
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * 
 * @note Always remove the propellers and stay away from the motors unless you are 100% certain of what you are doing.
 * 
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

#include "Constants.h"
#include <Config.h>
#include "src/Models.h"
#include "Globals.h"

void setup(){
  
  
  if(DEBUG) Serial.begin(BAUD_RATE);                   // use serial only if debugging mode


  // setup wifi AP
  setupWiFiTelemetry();                                // see WiFiTelemtry.ino  

  
 // if(DEBUG) intro();


  initEEPROM();                                        // see Initialize.ino


  configureReceiverTrims();                            // see Initialize.ino


  //Set start back to zero.
  start = 0; 
       

  // pinmode
  setupPins();                                         // see Initialize.ino
   

  // signaling the start point
  ledcWrite(pwmLedChannel, MAX_DUTY_CYCLE);                                         
  ledcWrite(pwmLedFlyChannel, MAX_DUTY_CYCLE); 


  //Start the I2C as master.
  setupWireI2C();                                      // see Initialize.ino           


  //Set the specific gyro registers.  
  setGyroscopeRegisters();                             // see Gyroscope.ino                      

  // few seconds for calibrating the gyroscope
  calibrateGyroscope();                                // see Gyroscope.ino


  // check Pressure
  checkAltitudeSensor();                               // see Altitude.ino


  // wait until the rx is connected
  waitController();                                                           
  
  
  start = 0;                                           // Set start back to 0.
  flightMode = 1;                                      // start without any mode (except for autoleveling if true)                                 


  initBattery();                                       // see Battery.ino


  //When everything is done, turn off the led.
  ledcWrite(pwmLedChannel, 0);                         // Turn off the warning led.      
  ledcWrite(pwmLedFlyChannel, 0);                      // Turn off the warning led.      


  //Set the timer for the next loop.
  loopTimer = micros();  

  
  if(DEBUG) Serial.println("setup finished");

}



void loop(){                                           // loop runs at 250Hz => each loop lasts 4000us

  // select mode via SWC of the controller:
  if      (trimCh[5].actual < 1050) flightMode = 1;    // SWC UP: no mode on, (only auto leveling if enabled)

  else if (trimCh[5].actual < 1550 &&
           trimCh[5].actual > 1450) flightMode = 2;    // SWC CENTER: altitude hold 

  else if (trimCh[5].actual < 2050 &&
           trimCh[5].actual > 1950) flightMode = 3;    // SWC DOWN: GPS*


  // calculate the gyroscope values for pitch, roll and yaw
  calculateAnglePRY();                                 // see Gyro.ino


  // convert the signal of the rx
  convertAllSignals();                                 // see ESC.ino


  // starting sequence of the quadcopter:
  if(receiverInputChannel3 < 1050 &&                   // a) to start the motors: throttle low and yaw left (step 1).
     receiverInputChannel4 < 1050)    start = 1;          

  if(start                 == 1   &&                   // b) when yaw stick is back in the center position start the motors (step 2).
     receiverInputChannel3 < 1050 && 
     receiverInputChannel4 > 1450)    droneStart();       

  if(start                 == 2   &&                   // c) stopping the motors: throttle low and yaw right.
     receiverInputChannel3 < 1050 &&  
     receiverInputChannel4 > 1950)    start = 0;      


  // calculate the altitude hold pressure parameters
  calculateAltitudeHold();                             // see Altitude.ino


  // calculate PID values
  calculatePID();                                      // see PID.ino


  // battery voltage can affect the efficiency 
  batteryVoltageCompensation();                        // see Battery.ino


  // create ESC pulses
  setEscPulses();                                      // see ESC.ino


  // send telemetry via wifi
  sendWiFiTelemetry();


  // finish the loop
  if(micros() - loopTimer > 4050)
         ledcWrite(pwmLedFlyChannel, MAX_DUTY_CYCLE);  // turn on the LED if the loop time exceeds 4050us
  else ledcWrite(pwmLedFlyChannel, 0);


  // wait until 4000us are passed
  while(micros() - loopTimer < 4000);                  // the refresh rate is 250Hz, thus esc's pulse update is every 4ms.


  loopTimer = micros();                                // set the timer for the next loop

}
