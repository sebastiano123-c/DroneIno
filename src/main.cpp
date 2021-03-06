/**
 *
 *                       **********************************     
 *                       *           DroneIno !           *
 *                       **********************************
 *                      
 *                     Pilot your ESP32 based DIY quadcopter.
 * 
 *                
 * 
 *                                 DISCLAIMER: 
 * 
 * This version is compatible with the PlatformIO IDE.
 * This project can be casted to Arduino IDE with the following steps: 
 * copy the content of the /include dir into /src dir, and rename main.cpp 
 * and /src folder with the same name (as well as for all the Arduino proj.).
 * 
 * 
 * 
 *                                TERMS OF USE:
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * 
 * 
 * 
 *                               DOCUMENTATION:
 * 
 * This is the main file governing DroneIno and contains the architecture of
 * the software, including the setup, the calibration and the flight controller.
 * The constants, the global variables and the functions are all defined in the 
 * '../include' directory.
 * Take a look at the Config.h file and modify it accordingly to your drone hardware.
 * See '../docs/README.md' for the full documentation.
 * 
 * 
 *                            
 *                       ! TRY FIRST WITHOUT PROPELLERS !
 *  
 * Always remove the propellers and stay away from the motors unless you are
 * 100% certain of what you are doing.
 * 
 * 
 * 
 * @file main.cpp
 * @author Sebastiano Cocchi (@sebastiano123-c)
 * @brief Main DroneIno file, defining the architecture of the software.
 * @version 1.1
 * @date 2022-03-30 
 * @copyright Copyright (c) 2022
 */


//    (INCLUDES)
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Preferences.h>
Preferences preferences;

//    (DRONEINO FILES)
#include <Constants.h>
#include <Config.h>
#include <Models.h>
#include <Globals.h>
#include <Prototypes.h>

/**
 * ------------------------------------------------------------------------------------------------------
 * SKETCHES:
 * 
 *    (SETUP)
 *    This is the first sketch you have to upload, by changing the UPLOADED_SETUP in the Config.h file.
 *    Follow the instructions you read on serial monitor.
 *  
 */
#if   UPLOADED_SKETCH == SETUP

   void setup() {

      Serial.begin(BAUD_RATE);                             // begin serial 
      EEPROM.begin(EEPROM_SIZE);                           // and EEPROM
      
      welcomeMsg();

      setupWireI2C();                                      // instantiate Wire.h


      setupPins();                                         // see Initialize.h
      
      
      setGyroscopeRegisters();                             // see Gyroscope.h


      waitController();                                    // see Controller.h

   }


   void loop() {

      // controller setup
      putTrimsInTheMiddle();                                // find controller sticks' center positions

      if(error == 0) checkTrimInversion(1);                 // check if the sticks are inverted
      if(error == 0) checkTrimInversion(2);
      if(error == 0) checkTrimInversion(3);
      if(error == 0) checkTrimInversion(4);

      if(error == 0) findSticksLimits();                    // finds the boundaries of the controller sticks


      // gyroscope
      if(error == 0)
         calibrateGyroscope();                              // calibrate gyroscope to proper measures
      
      if(error == 0){                                       // detect the left wing up movement
         Serial.println("\n\nLIFT the LEFT side of the "
         "quadcopter to a 45 degree angle within 10 seconds"); 
         configureGyroscopeAxes(1);
      }

      if(error == 0){                                        // detect the nose up movement
         Serial.println("\n\nLIFT the NOSE of the "
            "quadcopter to a 45 degree angle within 10 seconds");
         configureGyroscopeAxes(2);
      }

      if(error == 0){                                       // detect the nose right movement
         Serial.println("\n\nROTATE the NOSE of the "
            "quadcopter to the right within 10 seconds");  
         configureGyroscopeAxes(3);
      }

      if(error == 0) 
         checkGyroscopeResult();                             // verify everything is OK


      // led test
      if(error == 0)
         checkLed();


      // update EEPROM
      if(error == 0)
         writeEEPROM();


      // finish
      if(error == 0){
         Serial.println(F("\nSetup is finished SUCCESSFULLY."));
         Serial.println(F("\nYou can now calibrate the esc's. Write in the Config.h file 'UPLOADED_SKETCH CALIBRATION' and upload."));
      }
      else{
         Serial.println(F("ERROR! The setup is aborted."));
      }
      while(1);
   }

   #include <Setup.h>

/**
 *    (CALIBRATION)
 *    If the setup sketch exited with success, change the UPLOADED_FILE token in the Config.h file to CALIBRATION.
 *    In this part you have to connect for the first time the battery to the motors.
 *    Since also USB cable will be connected (to reed the serial) please,
 *    
 *             DO NOT CONNECT THE BATTERY TO THE BOARD IN THIS STEP FOR ANY REASONS, otherwise the board will be demaged.
 * 
 *    See the ../include/Calibration.h file for more details.
 */
#elif UPLOADED_SKETCH == CALIBRATION

   #include <AutoPID.h>

   void setup(){

      Serial.begin(BAUD_RATE);                             // begin
      EEPROM.begin(EEPROM_SIZE);

      
      initEEPROM();                                        // see Initialize.h


      configureReceiverTrims();                            // see Initialize.h
         

      // pinmode
      setupPins();                                         // see Initialize.h


      //Start the I2C as master.
      setupWireI2C();                                      // see Initialize.h           


      //Set the specific gyro registers.  
      setGyroscopeRegisters();                             // see Gyroscope.h                      


      // wait until the rx is connected
      waitController();                                    // wait until the receiver is active.


      // set pin precision
      initBattery();             


      calibrationMsg();                                    // introduction


      pidOutputPitch = 0;                                  // make setEscPulses to work
      pidOutputRoll = 0;
      pidOutputYaw = 0;
      batteryVoltage = 0;

      loopTimer = micros();                                //Set the zeroTimer for the first loop.
      while(Serial.available()) msg = Serial.read();       //Empty the serial buffer.
      msg = 0;                                             //Set the data variable back to zero.

   }

   void loop() {

      while(loopTimer + 4000 > micros());                  // Start the pulse after 4000 micro seconds.
      loopTimer = micros();                                // Reset the zero timer.

      if(Serial.available() > 0) getSerialMsg();

      receiverInputChannel3 = convertReceiverChannel(3);   // Convert the actual receiver signals for throttle to the standard 1000 - 2000us.

      if(receiverInputChannel3 < 1025)
         flag = false;                                     // If the throttle is in the lowest position set the request flag to false.

      // run the ESC calibration program to start with.
      if(msg == 0 && flag == false){                       // Only start the calibration mode at first start. 
         receiverInputChannel3 = convertReceiverChannel(3);// Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
         esc1 = receiverInputChannel3;                     // Set the pulse for motor 1 equal to the throttle channel.
         esc2 = receiverInputChannel3;                     // Set the pulse for motor 2 equal to the throttle channel.
         esc3 = receiverInputChannel3;                     // Set the pulse for motor 3 equal to the throttle channel.
         esc4 = receiverInputChannel3;                     // Set the pulse for motor 4 equal to the throttle channel.
         setEscPulses();                                   // Send the ESC control pulses.
      }

      if(msg == 'r')
         rFunction();

      if(msg == '1' || msg == '2' || msg == '3' || msg == '4' || msg == '5')//If motor 1, 2, 3 or 4 is selected by the user.
         escFunction();

      if(msg == 'g'){
         calculateAnglePRY();                                 // see Gyroscope.h
         printGyroscopeStatus();
      }
      
      if(msg == 'e'){
         printEEPROM();
         msg = 0;
      }

      if(msg == 'l') 
         blinkLed();

      if(msg == 'a'){
         calculateAltitudeHold();
         printBarometer();
      }

      if(msg == 'b'){
         readBatteryVoltage();
         printBatteryVoltage();
      }

      if(msg == 's'){
         readGPS();
         printGPS();
      }

      if(msg == 'p'){
         // calculatePID();                                      // calculate the PIDs
         calibrateAutoPID(structure);                                  // calibrate autoPID parameters
      }
   } 

   #include <Calibration.h>
   #include <Battery.h>
   #include <Altitude.h>
   #include <PID.h>
   #include <GPS.h>

/**
 *    (FLIGHT_CONTROLLER)
 *    After the props calibration (see ../docs/README.md), if all the previous steps have successfully being done,
 *    change the UPLOADED_SKETCH to FLIGHT_CONTROLLER and upload the sketch.
 *    If USB cable is detached, you can connect the battery safely also to the board.
 *    Now you can adjust the PID parameters.
 */
#elif UPLOADED_SKETCH == FLIGHT_CONTROLLER

   #include <AutoPID.h>


   void setup() {

      if(DEBUG) Serial.begin(BAUD_RATE);                   // use serial only if debugging mode


      // setup wifi AP
      setupWiFiTelemetry();                                // see WiFiTelemtry.h  

      
      #if DEBUG == true 
       intro();
      #endif

      initEEPROM();                                        // see Initialize.h


      configureReceiverTrims();                            // see Initialize.h


      //Set start back to zero.
      start = 0; 
            

      // pinmode
      setupPins();                                         // see Initialize.h
         

      // signaling the start point
      ledcWrite(pwmLedChannel, MAX_DUTY_CYCLE);                                         
      ledcWrite(pwmLedBatteryChannel, MAX_DUTY_CYCLE); 


      //Start the I2C as master.
      setupWireI2C();                                      // see Initialize.h           


      // GPS
      #if GPS != OFF
         setupGPS();
      #endif


      //Set the specific gyro registers.  
      setGyroscopeRegisters();                             // see Gyroscope.h                      

      // few seconds for calibrating the gyroscope
      calibrateGyroscope();                                // see Gyroscope.h


      // check Pressure
      #if ALTITUDE_SENSOR != OFF
         checkAltitudeSensor();                               // see Altitude.h
      #endif


      // wait until the rx is connected
      waitController();                                                           

      
      start = 0;                                           // Set start back to 0.
      flightMode = 1;                                      // start without any mode (except for auto-leveling if true)                                 


      initBattery();                                       // see Battery.h


      //When everything is done, turn off the led.
      ledcWrite(pwmLedChannel, 0);                         // Turn off the warning led.      
      ledcWrite(pwmLedBatteryChannel, 0);                  // Turn off the warning led.      


      //Set the timer for the next loop.
      loopTimer = micros();  


      // initialize the auto pid objects
      initAutoPID(structure, zLRoll, aLRoll, biasRoll, deltaBiasRoll, weightsRoll, deltaWeightsRoll, 200, {"roll-bias","roll-weights"});// pitch has the same values
      initAutoPID(structure, zLYaw, aLYaw, biasYaw, deltaBiasYaw, weightsYaw, deltaWeightsYaw, 200, {"yaw-bias","yaw-weights"});

 
      #if DEBUG
         Serial.println("Setup finished");
      #endif

   }



   void loop() {                                           // loop runs at 250Hz => each loop lasts 4000us

      // select mode via SWC of the controller:
      if      (trimCh[0].actual < 1050) flightMode = 1;    // SWC UP: only auto leveling if enabled

      else if (trimCh[0].actual < 1550 &&
               trimCh[0].actual > 1450) flightMode = 2;    // SWC CENTER: altitude hold 

           if (trimCh[0].actual < 2050 &&
               trimCh[0].actual > 1950) flightMode = 3;    // SWC DOWN: GPS*

      // GPS
      #if GPS != OFF
         readGPS();
      #endif
      

      // calculate the gyroscope values for pitch, roll and yaw
      calculateAnglePRY();                                 // see Gyroscope.h


      // convert the signal of the rx
      convertAllSignals();                                 // see ESC.h


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
      #if ALTITUDE_SENSOR != OFF
         calculateAltitudeHold();                          // see Altitude.h
      #endif


      // calculate PID values
      calculatePID();                                      // see PID.h


      // battery voltage can affect the efficiency 
      readBatteryVoltage();                                // see Battery.h


      // create ESC pulses
      setEscPulses();                                      // see ESC.h


      // send telemetry via wifi
      sendWiFiTelemetry();


      // refine PIDs
      autotunePID();


      // finish the loop
      if(micros() - loopTimer > 4050)
               ledcWrite(pwmLedChannel, MAX_DUTY_CYCLE);  // turn on the LED if the loop time exceeds 4050us
      else ledcWrite(pwmLedChannel, 0);


      // wait until 4000us are passed
      while(micros() - loopTimer < 4000);                  // the refresh rate is 250Hz, thus esc's pulse update is every 4ms.


      loopTimer = micros();                                // set the timer for the next loop

   }

   #include <Battery.h>
   #include <WiFiTelemetry.h>
   #include <PID.h>
   #include <Altitude.h>
   #include <GPS.h>
   // #include <Compass.h>

#else 
   #error "NO SKETCH UPLOADED"
#endif


//    (PROJECT FILES)
#include <Initialize.h>
#include <Gyroscope.h>
#include <ISR.h>
#include <ESC.h>
#include <Controller.h>