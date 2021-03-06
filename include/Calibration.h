/**
 * @file Calibration.h
 * @author @sebastiano123-c
 * @brief Calibration sketch for the motors, sensors and receiver inputs.
 * @version 0.1
 * @date 2022-02-28
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
 * Dial these on the serial:
 *  @li h: help.
 *  @li r: print receiver signals.
 *  @li g: print quadcopter angles.
 *  @li l: blinking leds.
 *  @li a: altitude sensor.
 *  @li e: print EEPROM.
 *  @li b: battery check.
 *  @li s: GPS check.
 *  @li 1: check rotation / vibrations for motor 1 (right front CCW).
 *  @li 2: check rotation / vibrations for motor 2 (right rear CW).
 *  @li 3: check rotation / vibrations for motor 3 (left rear CCW).
 *  @li 4: check rotation / vibrations for motor 4 (left front CW).
 *  @li 5: check vibrations for all motors together.
 * 
 * @copyright Copyright (c) 2022
 * 
 */


int loopCounter, vibrationCounter;
boolean firstAngle;
long accAvVector,  accVectorNorm[20], vibrationTotalResult;


  /**
   * @brief Setup the serial communication.
   * 
   */
  void setupSerial(){

    Serial.begin(BAUD_RATE);                             // begin

  }


  /**
   * @brief Dials the instructions.
   * 
   */
  void dialInstructions(){

    Serial.println(" Dial these on the serial:");
    Serial.println("   h: help.");
    Serial.println("   r: print receiver signals.");
    Serial.println("   g: print quadcopter angles.");
    Serial.println("   e: print EEPROM.");
    Serial.println("   l: blink leds.");
    Serial.println("   a: print altitude readings.");
    Serial.println("   b: print battery readings.");
    Serial.println("   s: print GPS readings.");
    Serial.println("   p: calibrate autoPID.");
    Serial.println("   1: check rotation / vibrations for motor 1 (right front CCW).");
    Serial.println("   2: check rotation / vibrations for motor 2 (right rear CW).");
    Serial.println("   3: check rotation / vibrations for motor 3 (left rear CCW).");
    Serial.println("   4: check rotation / vibrations for motor 4 (left front CW).");
    Serial.println("   5: check vibrations for all motors together.");
    Serial.println();

  }


  /**
   * @brief Print the commands
   * 
   */
  void calibrationMsg(){

    intro();

    delay(1000);

    Serial.println("\nDroneIno Calibration\n");
    
    delay(2000);

    dialInstructions();

  }


  /**
   * @brief Get the Serial Msg object
   * 
   */
  void getSerialMsg(){
    if(Serial.available() > 0){
      msg = Serial.read();                                                               //Read the incoming byte.
      while(Serial.available() > 0) loopCounter = Serial.read();                         //Empty the Serial buffer.
      flag = true;                                                                       //Set the new request flag.
      loopCounter = 0;                                                                   //Reset the loopCounter variable.
      calInt = 0;                                                                        //Reset the calInt variable to undo the calibration.
      start = 0;                                                                         //Set start to 0.
      firstAngle = false;                                                                //Set firstAngle to false.
      //Confirm the choice on the serial monitor.
      if(msg == 'h') dialInstructions();
      if(msg == 'r') Serial.println("Reading receiver signals.");
      if(msg == 'g'){
        Serial.println("Print the quadcopter angles.");
        calibrateGyroscope();                                                           //  few seconds for calibrating the gyroscope
      }
      if(msg == 'a'){
        Serial.println("Print altitude readings.");
        checkAltitudeSensor();                                                          // few seconds for calibrating the barometer       
      }
      if(msg == 'b') Serial.println("Print battery readings.");
      if(msg == 's') {
        setupGPS();
        Serial.println("Print GPS readings");
      }
      if(msg == '1') Serial.println("Test motor 1 (right front CCW.)\n Connect battery to motors BUT NOT TO THE BOARD !! ");
      if(msg == '2') Serial.println("Test motor 2 (right rear CW.)\n Connect battery to motors BUT NOT TO THE BOARD !! ");
      if(msg == '3') Serial.println("Test motor 3 (left rear CCW.)\n Connect battery to motors BUT NOT TO THE BOARD !! ");
      if(msg == '4') Serial.println("Test motor 4 (left front CW.)\n Connect battery to motors BUT NOT TO THE BOARD !! ");
      if(msg == '5') Serial.println("Test all motors together\n  Connect battery to motors BUT NOT TO THE BOARD !!");

      //Let's create a small delay so the message stays visible for 2.5 seconds.
      //We don't want the ESC's to beep and have to send a 1000us pulse to the ESC's.
      for(vibrationCounter = 0; vibrationCounter < 200; vibrationCounter++){           //Do this loop 625 times
        vTaskDelay(3/portTICK_PERIOD_MS);                                                                         //Wait 3000us.
        esc1 = 1000;                                                                     //Set the pulse for ESC 1 to 1000us.
        esc2 = 1000;                                                                     //Set the pulse for ESC 2 to 1000us.
        esc3 = 1000;                                                                     //Set the pulse for ESC 3 to 1000us.
        esc4 = 1000;                                                                     //Set the pulse for ESC 4 to 1000us.
        setEscPulses();                                                                  //Send the ESC control pulses.
      }
      vibrationCounter = 0;                                                              //Reset the vibrationCounter variable.
    }
    
    vTaskDelay(100/portTICK_PERIOD_MS);                                                //Wait for any other bytes to come in

  }


  /**
   * @brief Blink leds
   * 
   */
  void blinkLed(){

      Serial.println("\nBlinking leds...\n");

      while (Serial.available() == 0){

        ledcWrite(pwmLedChannel, HALF_DUTY_CYCLE);
        ledcWrite(pwmLedBatteryChannel, HALF_DUTY_CYCLE);     

        vTaskDelay(100/portTICK_PERIOD_MS);

        ledcWrite(pwmLedChannel, MAX_DUTY_CYCLE);
        ledcWrite(pwmLedBatteryChannel, MAX_DUTY_CYCLE);

        vTaskDelay(100/portTICK_PERIOD_MS);

        ledcWrite(pwmLedChannel, HALF_DUTY_CYCLE);
        ledcWrite(pwmLedBatteryChannel, HALF_DUTY_CYCLE);     

        vTaskDelay(100/portTICK_PERIOD_MS);

        ledcWrite(pwmLedChannel, 0);
        ledcWrite(pwmLedBatteryChannel, 0);  
        vTaskDelay(100/portTICK_PERIOD_MS);

      }

      dialInstructions();

      msg = Serial.read();
      
  }


  /**
   * @brief Prints the signals.
   * 
   */
  void printSignals(){

    convertAllSignals();
    
    Serial.print("Start:");
    Serial.print(start);
    
    Serial.print("  Roll:");
    if(receiverInputChannel1 - 1480 < 0)Serial.print("<<<");
    else if(receiverInputChannel1 - 1520 > 0)Serial.print(">>>");
    else Serial.print("-+-");
    Serial.print(receiverInputChannel1);

    Serial.print("  Pitch:");
    if(receiverInputChannel2 - 1480 < 0)Serial.print("^^^");
    else if(receiverInputChannel2 - 1520 > 0)Serial.print("vvv");
    else Serial.print("-+-");
    Serial.print(receiverInputChannel2);

    Serial.print("  Throttle:");
    if(receiverInputChannel3 - 1480 < 0)Serial.print("vvv");
    else if(receiverInputChannel3 - 1520 > 0)Serial.print("^^^");
    else Serial.print("-+-");
    Serial.print(receiverInputChannel3);

    Serial.print("  Yaw:");
    if(receiverInputChannel4 - 1480 < 0)Serial.print("<<<");
    else if(receiverInputChannel4 - 1520 > 0)Serial.print(">>>");
    else Serial.print("-+-");
    Serial.print(receiverInputChannel4);

    Serial.print("  Flight mode:");
    if(trimCh[0].actual - 1480 < 0)Serial.print("^^^");
    else if(trimCh[0].actual - 1520 > 0)Serial.print("vvv");
    else Serial.print("-+-");
    Serial.println(trimCh[0].actual);
  }


  /**
   * @brief Main routine used in the sketch.
   * 
   */
  void rFunction(){
      loopCounter ++;                                                              //Increase the loopCounter variable.
      trimCh[1].actual = convertReceiverChannel(1);                           //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
      trimCh[2].actual = convertReceiverChannel(2);                           //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
      trimCh[3].actual = convertReceiverChannel(3);                           //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
      trimCh[4].actual = convertReceiverChannel(4);                           //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.

      if(loopCounter == 125){                                                      //Print the receiver values when the loopCounter variable equals 250.
        printSignals();                                                            //Print the receiver values on the serial monitor.
        loopCounter = 0;                                                           //Reset the loopCounter variable.
      }

      //For starting the motors: throttle low and yaw left (step 1).
      if(trimCh[3].actual < 1050 && trimCh[4].actual < 1050)start = 1;
      //When yaw stick is back in the center position start the motors (step 2).
      if(start == 1 && trimCh[3].actual < 1050 && trimCh[4].actual > 1450)start = 2;
      //Stopping the motors: throttle low and yaw right.
      if(start == 2 && trimCh[3].actual < 1050 && trimCh[4].actual > 1950)start = 0;

      esc1 = 1000;                                                                  //Set the pulse for ESC 1 to 1000us.
      esc2 = 1000;                                                                  //Set the pulse for ESC 1 to 1000us.
      esc3 = 1000;                                                                  //Set the pulse for ESC 1 to 1000us.
      esc4 = 1000;                                                                  //Set the pulse for ESC 1 to 1000us.
      setEscPulses();                                                               //Send the ESC control pulses.
  }


  /**
   * @brief Calculate the ESC pulse for the outputs.
   * 
   */
  void escFunction(){

      loopCounter ++;                                                                //Add 1 to the loopCounter variable.
      if(flag == true && loopCounter == 250){                                        //Wait for the throttle to be set to 0.
        Serial.print("Set throttle to 1000 (low). It's now set to: ");               //Print message on the serial monitor.
        Serial.println(trimCh[3].actual);                                       //Print the actual throttle position.
        loopCounter = 0;                                                             //Reset the loopCounter variable.
      }
      if(flag == false){                                                             //When the throttle was in the lowest position do this.
        trimCh[3].actual = convertReceiverChannel(3);                           //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.

        if(msg == '1' || msg == '5')  esc1 = trimCh[3].actual;                  //If motor 1 is requested set the pulse for motor 1 equal to the throttle channel.
        else esc1 = 1000;                                                            //If motor 1 is not requested set the pulse for the ESC to 1000us (off).
        if(msg == '2' || msg == '5')  esc2 = trimCh[3].actual;                  //If motor 2 is requested set the pulse for motor 1 equal to the throttle channel.
        else esc2 = 1000;                                                            //If motor 2 is not requested set the pulse for the ESC to 1000us (off).
        if(msg == '3' || msg == '5')  esc3 = trimCh[3].actual;                  //If motor 3 is requested set the pulse for motor 1 equal to the throttle channel.
        else esc3 = 1000;                                                            //If motor 3 is not requested set the pulse for the ESC to 1000us (off).
        if(msg == '4' || msg == '5')  esc4 = trimCh[3].actual;                  //If motor 4 is requested set the pulse for motor 1 equal to the throttle channel.
        else esc4 = 1000;                                                            //If motor 4 is not requested set the pulse for the ESC to 1000us (off).

        // write to ESCs
        #if defined(MOTOR_PULSE_BY_MCPWM)

          mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, (uint32_t)esc1);//((float)esc1/2000.*(float)MAX_DUTY_CYCLE));
          mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, (uint32_t)esc2);//((float)esc2/2000.*(float)MAX_DUTY_CYCLE));
          mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, (uint32_t)esc3);//((float)esc3/2000.*(float)MAX_DUTY_CYCLE));
          mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, (uint32_t)esc4);//((float)esc4/2000.*(float)MAX_DUTY_CYCLE));
          
        #elif defined(MOTOR_PULSE_BY_LEDC)

          ledcWrite(pwmChannel1, (uint32_t)((float)esc1/2000.*(float)MAX_DUTY_CYCLE));
          ledcWrite(pwmChannel2, (uint32_t)((float)esc2/2000.*(float)MAX_DUTY_CYCLE));
          ledcWrite(pwmChannel3, (uint32_t)((float)esc3/2000.*(float)MAX_DUTY_CYCLE));
          ledcWrite(pwmChannel4, (uint32_t)((float)esc4/2000.*(float)MAX_DUTY_CYCLE));

        #endif


        //For balancing the propellors it's possible to use the accelerometer to measure the vibrations.
        if(eepromData[31] == 1){                                                     //The MPU-6050 is installed
          Wire.beginTransmission(GYRO_ADDRESS);                                       //Start communication with the gyro.
          Wire.write(0x3B);                                                          //Start reading @ register 43h and auto increment with every read.
          Wire.endTransmission();                                                    //End the transmission.
          Wire.requestFrom(GYRO_ADDRESS,6);                                           //Request 6 bytes from the gyro.
          while(Wire.available() < 6);                                               //Wait until the 6 bytes are received.
          accAxis[2] = Wire.read()<<8 | Wire.read();                                 //Add the low and high byte to the accAxis[2] variable.
          accAxis[1] = Wire.read()<<8 | Wire.read();                                 //Add the low and high byte to the accAxis[1] variable.
          accAxis[3] = Wire.read()<<8 | Wire.read();                                 //Add the low and high byte to the accAxis[3] variable.

          accVectorNorm[0] = sqrt((accAxis[2]*accAxis[2])+(accAxis[1]*accAxis[1])+(accAxis[3]*accAxis[3]));//Calculate the total accelerometer vector.

          accAvVector = accVectorNorm[0];                                            //Copy the total vector to the accelerometer average vector variable.

          for(start = 16; start > 0; start--){                                       //Do this loop 16 times to create an array of accelrometer vectors.
            accVectorNorm[start] = accVectorNorm[start - 1];                         //Shift every variable one position up in the array.
            accAvVector += accVectorNorm[start];                                     //Add the array value to the accAvVector variable.
          }

          accAvVector /= 17;                                                          //Divide the accAvVector by 17 to get the avarage total accelerometer vector.

          if(vibrationCounter < 20){                                                  //If the vibrationCounter is less than 20 do this.
            vibrationCounter ++;                                                      //Increment the vibrationCounter variable.
            vibrationTotalResult += abs(accVectorNorm[0] - accAvVector);              //Add the absolute difference between the avarage vector and current vector to the vibrationTotalResult variable.
          }
          else{
            vibrationCounter = 0;                                                      //If the vibrationCounter is equal or larger than 20 do this.
            Serial.println(vibrationTotalResult/50);                                   //Print the total accelerometer vector divided by 50 on the serial monitor.
            vibrationTotalResult = 0;                                                  //Reset the vibrationTotalResult variable.
          }
        }
      }
  }


  void calibrateAutoPID(std::vector<int> structure){

    Serial.println("Train the neural network to calibrate the PID parameters.");
    delay(2000);

    // initialize the auto pid objects
      initAutoPID(structure, zLRoll, aLRoll, biasRoll, deltaBiasRoll, weightsRoll, deltaWeightsRoll, 200, {"roll-bias","roll-weights"});// pitch has the same values
      initAutoPID(structure, zLYaw, aLYaw, biasYaw, deltaBiasYaw, weightsYaw, deltaWeightsYaw, 200, {"yaw-bias","yaw-weights"});


    calibrateGyroscope();                                 // calibrate gyroscope
    
    
    while (Serial.available() == 0){                      // if no char is sent to the serial, autotune PID
      convertAllSignals();
      calculateAnglePRY();
      calculatePID();
      autotunePID();
    }

    // save the weights on the EEPROM
    Serial.printf("\n Saving weights on EEPROM...\n");

    int memoryAddress = 0;              // the starting index of memory
    int kk, ii, jj;

    preferences.begin("roll-bias", false);
    preferences.clear();// Remove all preferences under the opened namespace

    char numChar[20 + sizeof(char)];

    // roll biases
    for(jj = 1; jj < structure.size(); jj++){
      Serial.printf("Roll bias layer%i: \n", jj);
        for (ii = 0; ii < structure[jj]; ii++) {
            Serial.printf("%f ",biasRoll[jj-1][ii]);
            sprintf(numChar, "%i", memoryAddress);
            preferences.putFloat(numChar, biasRoll[jj-1][ii]);
            memoryAddress++;
        }
         Serial.printf("\n");
      }

    Serial.printf("\n\n");
    preferences.end();
    preferences.begin("roll-weights", false);
    preferences.clear();// Remove all preferences under the opened namespace
    memoryAddress = 0;

    // roll weights
    for(jj = 1; jj < structure.size(); jj++){
      Serial.printf("Roll weights layer%i: \n", jj);
      for (kk = 0; kk < structure[jj-1]; kk++){
          for (ii = 0; ii < structure[jj]; ii++) {
              Serial.printf("%f ",weightsRoll[jj-1][kk][ii]);
              sprintf(numChar, "%i", memoryAddress);
              preferences.putFloat(numChar, weightsRoll[jj-1][kk][ii]);
              memoryAddress++;
          }
          Serial.printf("\n");
        }
        Serial.printf("\n\n");
    }

    preferences.end();
    preferences.begin("yaw-bias", false);
    preferences.clear();// Remove all preferences under the opened namespace
    memoryAddress = 0;

    // yaw biases
    for(jj = 1; jj < structure.size(); jj++){
      Serial.printf("Yaw bias layer%i: \n", jj);
        for (ii = 0; ii < structure[jj]; ii++) {
            Serial.printf("%f ",biasYaw[jj-1][ii]);
            sprintf(numChar, "%i", memoryAddress);
            preferences.putFloat(numChar, biasYaw[jj-1][ii]);
            memoryAddress++;
        }
         Serial.printf("\n");
      }

    Serial.printf("\n\n");
    preferences.end();
    preferences.begin("yaw-weights", false);
    preferences.clear();// Remove all preferences under the opened namespace
    memoryAddress = 0;

    // yaw weights
  for(jj = 1; jj < structure.size(); jj++){
      Serial.printf("Yaw weights layer%i: \n", jj);
     for (kk = 0; kk < structure[jj-1]; kk++){
        for (ii = 0; ii < structure[jj]; ii++) {
            Serial.printf("%f ",weightsYaw[jj-1][kk][ii]);
            sprintf(numChar, "%i", memoryAddress);
            preferences.putFloat(numChar, weightsYaw[jj-1][kk][ii]);
            memoryAddress++;
        }
        Serial.printf("\n");
      }
      Serial.printf("\n\n");
  }
  preferences.end();
  

  // read preferences
  preferences.begin("roll-weights", true);
  memoryAddress=0;
    
  for(jj = 1; jj < structure.size(); jj++){
  Serial.printf("Print roll weights layer%i: \n", jj);
    for (kk = 0; kk < structure[jj-1]; kk++){
      for (ii = 0; ii < structure[jj]; ii++) {
        sprintf(numChar, "%i", memoryAddress);
        float f = preferences.getFloat(numChar, 0.0f);
        Serial.printf("%f ",f);
        memoryAddress++;
      }
      Serial.printf("\n");
    }
    Serial.printf("\n\n");
  }
  preferences.end();

  preferences.begin("yaw-weights", true);
  memoryAddress=0;
  
  for(jj = 1; jj < structure.size(); jj++){
    Serial.printf("Print Yaw weights layer%i: \n", jj);
     for (kk = 0; kk < structure[jj-1]; kk++){
        for (ii = 0; ii < structure[jj]; ii++) {
          sprintf(numChar, "%i", memoryAddress);
          float f = preferences.getFloat(numChar, 0.0f);
          Serial.printf("%f ",f);
          memoryAddress++;
        }
        Serial.printf("\n");
      }
      Serial.printf("\n\n");
    }
    preferences.end();


    Serial.printf("\n NN saved on flash memory \n \n");

    dialInstructions();

    msg = Serial.read();
  }









  // void calibrateAutoPID(std::vector<int> structure){

  //   Serial.println("Train the neural network to calibrate the PID parameters.");

  //   // initialize the auto pid objects
  //   initAutoPID(structure, zLRoll, aLRoll, biasRoll, deltaBiasRoll, weightsRoll, deltaWeightsRoll, 500, {"roll-bias", "roll-weights"});  // pitch has the same values
  //   initAutoPID(structure, zLYaw, aLYaw, biasYaw, deltaBiasYaw, weightsYaw, deltaWeightsYaw, 500, {"yaw-bias", "yaw-weights"});


  //   calibrateGyroscope();                                 // calibrate gyroscope
    
    
  //   while (Serial.available() == 0){                      // if no char is sent to the serial, autotune PID
  //     convertAllSignals();
  //     calculateAnglePRY();
  //     calculatePID();
  //     autotunePID();
  //   }

  //   // save the weights on the EEPROM
  //   Serial.printf("\n Saving weights on EEPROM...\n");

  //   int memoryAddress = 0;              // the starting index of memory
  //   int kk, ii, jj;

  //   preferences.begin("roll-weights", false);
  //   preferences.clear();// Remove all preferences under the opened namespace

  //   char numChar[20 + sizeof(char)];

  // // roll over the structure vector
  // std::vector<int>::iterator itInt = structure.begin();
  // ++itInt;
  // jj = 1;

  // while(itInt != structure.end())
  // {
  //   // 1) save bias vector
  //   for (ii = 0; ii < *itInt; ii++)
  //   {
  //     sprintf(numChar, "%i", memoryAddress);
  //     preferences.putFloat(numChar, biasRoll[jj-1][ii]);
  //     memoryAddress++;
  //   }

  //   // 2) save weight matrix
  //   for (kk = 0; kk < *(itInt - 1); kk++)
  //   {
  //     for (ii = 0; ii < *itInt; ii++)
  //     {
  //       sprintf(numChar, "%i", memoryAddress);
  //       preferences.putFloat(numChar, weightsRoll[jj-1][kk][ii]);
  //       memoryAddress++;
  //     }

  //   }
  //   jj++;
  //   ++itInt;
  // }
  // preferences.end();


  //   preferences.end();
  //   preferences.begin("yaw-weights", false);
  //   preferences.clear();// Remove all preferences under the opened namespace
  //   memoryAddress = 0;

  //   itInt = structure.begin();
  //   ++itInt;
  //   jj = 1;

  //   while(itInt != structure.end())
  //   {
  //     // 1) save bias vector
  //     for (ii = 0; ii < *itInt; ii++)
  //     {
  //       sprintf(numChar, "%i", memoryAddress);
  //       preferences.putFloat(numChar, biasYaw[jj-1][ii]);
  //       memoryAddress++;
  //     }

  //     // 2) save weight matrix
  //     for (kk = 0; kk < *(itInt - 1); kk++)
  //     {
  //       for (ii = 0; ii < *itInt; ii++)
  //       {
  //         sprintf(numChar, "%i", memoryAddress);
  //         preferences.putFloat(numChar, weightsYaw[jj-1][kk][ii]);
  //         memoryAddress++;
  //       }

  //     }
  //     jj++;
  //     ++itInt;
  //   }
  //   preferences.end();


  //   // read preferences
  //   preferences.begin("roll-weights", true);
  //   memoryAddress=0;
    
  //   for(jj = 1; jj < structure.size(); jj++){

  //     // 1) read bias
  //     Serial.printf("\nRoll bias for layer %i: \n", jj);
  //     for (ii = 0; ii < structure[jj]; ii++)
  //     {
  //       sprintf(numChar, "%i", memoryAddress);
  //       Serial.printf("%f ", preferences.getFloat(numChar, 0.0f));
  //       memoryAddress++;
  //     }

  //     // 2) read weight matrix
  //     Serial.printf("\nRoll weights for layer %i: \n", jj);
  //     for (kk = 0; kk < structure[jj-1]; kk++)
  //     {
  //       for (ii = 0; ii < structure[jj]; ii++)
  //       {
  //         sprintf(numChar, "%i", memoryAddress);
  //         Serial.printf("%f ", preferences.getFloat(numChar, 0.0f));
  //         memoryAddress++;
  //       }
  //       Serial.printf("\n");
  //     }
  //     Serial.printf("\n\n");
  //   }
  //   preferences.end();

  //   // read yaw bias and weights
  //   preferences.begin("yaw-weights", true);
  //   memoryAddress=0;
    
  //   for(jj = 1; jj < structure.size(); jj++){

  //     // 1) read bias
  //     Serial.printf("\nYaw bias for layer %i: \n", jj);
  //     for (ii = 0; ii < structure[jj]; ii++)
  //     {
  //       sprintf(numChar, "%i", memoryAddress);
  //       Serial.printf("%f ", preferences.getFloat(numChar, 0.0f));
  //       memoryAddress++;
  //     }

  //     // 2) read weight matrix
  //     Serial.printf("\nYaw weights for layer %i: \n", jj);
  //     for (kk = 0; kk < structure[jj-1]; kk++)
  //     {
  //       for (ii = 0; ii < structure[jj]; ii++)
  //       {
  //         sprintf(numChar, "%i", memoryAddress);
  //         Serial.printf("%f ", preferences.getFloat(numChar, 0.0f));
  //         memoryAddress++;
  //       }
  //       Serial.printf("\n");
  //     }
  //     Serial.printf("\n\n");
  //   }
  //   preferences.end();


  //   Serial.printf("\n NN saved on flash memory \n \n");

  //   dialInstructions();

  //   msg = Serial.read();
  // }