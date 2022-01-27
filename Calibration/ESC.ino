void escFunction(){
    loopCounter ++;                                                                    //Add 1 to the loopCounter variable.
    if(new_function_request == true && loopCounter == 250){                            //Wait for the throttle to be set to 0.
      Serial.print("Set throttle to 1000 (low). It's now set to: ");                    //Print message on the serial monitor.
      Serial.println(receiverInputChannel3);                                         //Print the actual throttle position.
      loopCounter = 0;                                                                 //Reset the loopCounter variable.
    }
    if(new_function_request == false){                                                  //When the throttle was in the lowest position do this.
      receiverInputChannel3 = convert_receiverChannel(3);                           //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
      if(data == '1' || data == '5')esc_1 = receiverInputChannel3;                   //If motor 1 is requested set the pulse for motor 1 equal to the throttle channel.
      else esc_1 = 1000;                                                                //If motor 1 is not requested set the pulse for the ESC to 1000us (off).
      if(data == '2' || data == '5')esc_2 = receiverInputChannel3;                   //If motor 2 is requested set the pulse for motor 1 equal to the throttle channel.
      else esc_2 = 1000;                                                                //If motor 2 is not requested set the pulse for the ESC to 1000us (off).
      if(data == '3' || data == '5')esc_3 = receiverInputChannel3;                   //If motor 3 is requested set the pulse for motor 1 equal to the throttle channel.
      else esc_3 = 1000;                                                                //If motor 3 is not requested set the pulse for the ESC to 1000us (off).
      if(data == '4' || data == '5')esc_4 = receiverInputChannel3;                   //If motor 4 is requested set the pulse for motor 1 equal to the throttle channel.
      else esc_4 = 1000;                                                                //If motor 4 is not requested set the pulse for the ESC to 1000us (off).

      esc_pulse_output();                                                               //Send the ESC control pulses.

      //For balancing the propellors it's possible to use the accelerometer to measure the vibrations.
      if(eepromData[31] == 1){                                                         //The MPU-6050 is installed
        Wire.beginTransmission(gyroAddress);                                           //Start communication with the gyro.
        Wire.write(0x3B);                                                               //Start reading @ register 43h and auto increment with every read.
        Wire.endTransmission();                                                         //End the transmission.
        Wire.requestFrom(gyroAddress,6);                                               //Request 6 bytes from the gyro.
        while(Wire.available() < 6);                                                    //Wait until the 6 bytes are received.
        accAxis[2] = Wire.read()<<8 | Wire.read();                                             //Add the low and high byte to the accAxis[2] variable.
        accAxis[1] = Wire.read()<<8 | Wire.read();                                             //Add the low and high byte to the accAxis[1] variable.
        accAxis[3] = Wire.read()<<8 | Wire.read();                                             //Add the low and high byte to the accAxis[3] variable.

        accTotalVector[0] = sqrt((accAxis[2]*accAxis[2])+(accAxis[1]*accAxis[1])+(accAxis[3]*accAxis[3]));          //Calculate the total accelerometer vector.

        accAvVector = accTotalVector[0];                                            //Copy the total vector to the accelerometer average vector variable.

        for(start = 16; start > 0; start--){                                            //Do this loop 16 times to create an array of accelrometer vectors.
          accTotalVector[start] = accTotalVector[start - 1];                        //Shift every variable one position up in the array.
          accAvVector += accTotalVector[start];                                     //Add the array value to the accAvVector variable.
        }

        accAvVector /= 17;                                                            //Divide the accAvVector by 17 to get the avarage total accelerometer vector.

        if(vibrationCounter < 20){                                                     //If the vibrationCounter is less than 20 do this.
          vibrationCounter ++;                                                         //Increment the vibrationCounter variable.
          vibrationTotalResult += abs(accTotalVector[0] - accAvVector);           //Add the absolute difference between the avarage vector and current vector to the vibrationTotalResult variable.
        }
        else{
          vibrationCounter = 0;                                                        //If the vibrationCounter is equal or larger than 20 do this.
          Serial.println(vibrationTotalResult/50);                                    //Print the total accelerometer vector divided by 50 on the serial monitor.
          vibrationTotalResult = 0;                                                   //Reset the vibrationTotalResult variable.
        }
      }
    }
   }

void esc_pulse_output(){
  zeroTimer = micros();
  //Set digital port 4, 5, 6 and 7 high.
  digitalWrite(PIN_ESC_1, HIGH);
  digitalWrite(PIN_ESC_2, HIGH);
  digitalWrite(PIN_ESC_3, HIGH);
  digitalWrite(PIN_ESC_4, HIGH);
 
  timerChannel1 = esc_1 + zeroTimer;                          //Calculate the time when digital port 4 is set low.
  timerChannel2 = esc_2 + zeroTimer;                          //Calculate the time when digital port 5 is set low.
  timerChannel3 = esc_3 + zeroTimer;                          //Calculate the time when digital port 6 is set low.
  timerChannel4 = esc_4 + zeroTimer;                          //Calculate the time when digital port 7 is set low.

  bool i1 = false; 
  bool i2 = false; 
  bool i3 = false; 
  bool i4 = false;

  while( i1 == false | i2 == false | i3 == false | i4 == false ){//Execute the loop until digital port 4 to 7 is low.
    escLoopTimer = micros();                                    //Check the current time.
    if(timerChannel1 <= escLoopTimer){                          //When the delay time is expired, digital port 4 is set low.
      digitalWrite(PIN_ESC_1, LOW);
      i1 = true;
    }
    if(timerChannel2 <= escLoopTimer){                          //When the delay time is expired, digital port 5 is set low.
      digitalWrite(PIN_ESC_2, LOW);
      i2 = true;
    }
    if(timerChannel3 <= escLoopTimer){                          //When the delay time is expired, digital port 6 is set low.
      digitalWrite(PIN_ESC_3, LOW);
      i3 = true;
    }
    if(timerChannel4 <= escLoopTimer){                          //When the delay time is expired, digital port 7 is set low.
      digitalWrite(PIN_ESC_4, LOW);
      i4 = true;
    }
  }
}
