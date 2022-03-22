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
 *  @li r: print receiver signals.
 *  @li a: print quadcopter angles.
 *  @li e: print EEPROM.
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
 * @brief Print the commands
 * 
 */
void calibrationMsg(){

  intro();

  delay(1000);

  Serial.println("\nDroneIno Calibration\n");
  
  delay(2000);

   Serial.println(" Dial these on the serial:");
   Serial.println(  " r: print receiver signals.");
   Serial.println(  " a: print quadcopter angles.");
   Serial.println(  " e: print EEPROM.");
   Serial.println(  " 1: check rotation / vibrations for motor 1 (right front CCW).");
   Serial.println(  " 2: check rotation / vibrations for motor 2 (right rear CW).");
   Serial.println(  " 3: check rotation / vibrations for motor 3 (left rear CCW).");
   Serial.println(  " 4: check rotation / vibrations for motor 4 (left front CW).");
   Serial.println(  " 5: check vibrations for all motors together.");
   Serial.println();
}


/**
 * @brief Get the Serial Msg object
 * 
 */
void getSerialMsg(){
    msg = Serial.read();                                                               //Read the incoming byte.
    vTaskDelay(100/portTICK_PERIOD_MS);                                                                         //Wait for any other bytes to come in
    while(Serial.available() > 0) loopCounter = Serial.read();                          //Empty the Serial buffer.
    flag = true;                                                        //Set the new request flag.
    loopCounter = 0;                                                                   //Reset the loopCounter variable.
    calInt = 0;                                                                        //Reset the calInt variable to undo the calibration.
    start = 0;                                                                          //Set start to 0.
    firstAngle = false;                                                                //Set firstAngle to false.
    //Confirm the choice on the serial monitor.
    if(msg == 'r') Serial.println("Reading receiver signals.");
    if(msg == 'a') Serial.println("Print the quadcopter angles.");
    if(msg == 'a') Serial.println("Gyro calibration starts in 2 seconds (don't move the quadcopter).");
    if(msg == '1') Serial.println("Test motor 1 (right front CCW.)");
    if(msg == '2') Serial.println("Test motor 2 (right rear CW.)");
    if(msg == '3') Serial.println("Test motor 3 (left rear CCW.)");
    if(msg == '4') Serial.println("Test motor 4 (left front CW.)");
    if(msg == '5') Serial.println("Test all motors together");

    //Let's create a small delay so the message stays visible for 2.5 seconds.
    //We don't want the ESC's to beep and have to send a 1000us pulse to the ESC's.
    for(vibrationCounter = 0; vibrationCounter < 625; vibrationCounter++){           //Do this loop 625 times
      vTaskDelay(3/portTICK_PERIOD_MS);                                                                         //Wait 3000us.
      esc1 = 1000;                                                                     //Set the pulse for ESC 1 to 1000us.
      esc2 = 1000;                                                                     //Set the pulse for ESC 2 to 1000us.
      esc3 = 1000;                                                                     //Set the pulse for ESC 3 to 1000us.
      esc4 = 1000;                                                                     //Set the pulse for ESC 4 to 1000us.
      setEscPulses();                                                                  //Send the ESC control pulses.
    }
    vibrationCounter = 0;                                                              //Reset the vibrationCounter variable.
}

/**
 * @brief Prints the signals.
 * 
 */
void printSignals(){
  Serial.print("Start:");
  Serial.print(start);

  Serial.print("  Throttle:");
  if(receiverInputChannel3 - 1480 < 0)Serial.print("vvv");
  else if(receiverInputChannel3 - 1520 > 0)Serial.print("^^^");
  else Serial.print("-+-");
  Serial.print(receiverInputChannel3);

  Serial.print("  Yaw:");
  if(receiverInputChannel4 - 1480 < 0)Serial.print("<<<");
  else if(receiverInputChannel4 - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.println(receiverInputChannel4);
  
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
}

/**
 * @brief Main routine used in the sketch.
 * 
 */
void rFunction(){
    loopCounter ++;                                                              //Increase the loopCounter variable.
    receiverInputChannel1 = convertReceiverChannel(1);                           //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
    receiverInputChannel2 = convertReceiverChannel(2);                           //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
    receiverInputChannel3 = convertReceiverChannel(3);                           //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
    receiverInputChannel4 = convertReceiverChannel(4);                           //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.

    if(loopCounter == 125){                                                      //Print the receiver values when the loopCounter variable equals 250.
      printSignals();                                                            //Print the receiver values on the serial monitor.
      loopCounter = 0;                                                           //Reset the loopCounter variable.
    }

    //For starting the motors: throttle low and yaw left (step 1).
    if(receiverInputChannel3 < 1050 && receiverInputChannel4 < 1050)start = 1;
    //When yaw stick is back in the center position start the motors (step 2).
    if(start == 1 && receiverInputChannel3 < 1050 && receiverInputChannel4 > 1450)start = 2;
    //Stopping the motors: throttle low and yaw right.
    if(start == 2 && receiverInputChannel3 < 1050 && receiverInputChannel4 > 1950)start = 0;

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
      Serial.println(receiverInputChannel3);                                       //Print the actual throttle position.
      loopCounter = 0;                                                             //Reset the loopCounter variable.
    }
    if(flag == false){                                                             //When the throttle was in the lowest position do this.
      receiverInputChannel3 = convertReceiverChannel(3);                           //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.

      if(msg == '1' || msg == '5')  esc1 = receiverInputChannel3;                  //If motor 1 is requested set the pulse for motor 1 equal to the throttle channel.
      else esc1 = 1000;                                                            //If motor 1 is not requested set the pulse for the ESC to 1000us (off).
      if(msg == '2' || msg == '5')  esc2 = receiverInputChannel3;                  //If motor 2 is requested set the pulse for motor 1 equal to the throttle channel.
      else esc2 = 1000;                                                            //If motor 2 is not requested set the pulse for the ESC to 1000us (off).
      if(msg == '3' || msg == '5')  esc3 = receiverInputChannel3;                  //If motor 3 is requested set the pulse for motor 1 equal to the throttle channel.
      else esc3 = 1000;                                                            //If motor 3 is not requested set the pulse for the ESC to 1000us (off).
      if(msg == '4' || msg == '5')  esc4 = receiverInputChannel3;                  //If motor 4 is requested set the pulse for motor 1 equal to the throttle channel.
      else esc4 = 1000;                                                            //If motor 4 is not requested set the pulse for the ESC to 1000us (off).

      // write to ESCs
      ledcWrite(pwmChannel1, (float)esc1/2000.*(float)MAX_DUTY_CYCLE);
      ledcWrite(pwmChannel2, (float)esc2/2000.*(float)MAX_DUTY_CYCLE);
      ledcWrite(pwmChannel3, (float)esc3/2000.*(float)MAX_DUTY_CYCLE);
      ledcWrite(pwmChannel4, (float)esc4/2000.*(float)MAX_DUTY_CYCLE);

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