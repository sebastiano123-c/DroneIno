/**
 * @file Setup.h
 * @author Sebastiano Cocchi
 * @brief Setup sketch routines
 * 
 * Setup:
 *  @li check gyroscope;
 *  @li check altitude sensor;
 *  @li controller output;
 * 
 * @version 0.1
 * @date 2022-03-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */


byte channel1Assign, channel2Assign, channel3Assign, channel4Assign;
byte receiverCheckByte, gyroCheckByte;
byte gyroCalibratedAxis[4];
byte type = 1;

/**
 * @brief Initial message
 * 
 */
void welcomeMsg(){

    intro();

    Serial.println(F("==================================================="));
    vTaskDelay(1500 / portTICK_PERIOD_MS);
    Serial.println(F(""));
    Serial.println(F("      Setup sketch"));
    vTaskDelay(300 / portTICK_PERIOD_MS);
    Serial.println(F(""));
    Serial.println(F(""));

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // 
    trimCh[1].actual = 0;
    trimCh[2].actual = 0;
    trimCh[3].actual = 0;
    trimCh[4].actual = 0;
}


/**
 * @brief Move the trims in the center position to store the middle value
 * 
 */
void putTrimsInTheMiddle(){

    
    Serial.println(F("Place all sticks and subtrims in the center position within 7 seconds."));
    for(int i = 6; i > 0;i--){
      delay(1000);
      Serial.print(i);
      Serial.print(" ");
    }
    Serial.println(" ");

    //Store the central stick positions
    trimCh[1].center = trimCh[1].actual;
    trimCh[2].center = trimCh[2].actual;
    trimCh[3].center = trimCh[3].actual;
    trimCh[4].center = trimCh[4].actual;

    Serial.println(F(""));
    Serial.println(F("Center positions stored."));
    Serial.print(F("Digital input 08 = "));
    Serial.println(trimCh[1].center);
    Serial.print(F("Digital input 09 = "));
    Serial.println(trimCh[2].center);
    Serial.print(F("Digital input 10 = "));
    Serial.println(trimCh[3].center);
    Serial.print(F("Digital input 11 = "));
    Serial.println(trimCh[4].center);
    Serial.println(F(""));
    Serial.println(F(""));

}


/**
 * @brief Check if a receiver input value is changing within 30 seconds
 * 
 * @param movement 
 */
void checkReceiverInputs(byte movement){
  byte trigger = 0;
  int pulseLength;
  unsigned long timer = millis() + 30000;
  while(timer > millis() && trigger == 0){
    vTaskDelay(250 / portTICK_PERIOD_MS);
    if(trimCh[1].actual > 1750 || trimCh[1].actual < 1250){
      trigger = 1;
      receiverCheckByte |= 0b00000001;
      pulseLength = trimCh[1].actual;
    }
    if(trimCh[2].actual > 1750 || trimCh[2].actual < 1250){
      trigger = 2;
      receiverCheckByte |= 0b00000010;
      pulseLength = trimCh[2].actual;
    }
    if(trimCh[3].actual > 1750 || trimCh[3].actual < 1250){
      trigger = 3;
      receiverCheckByte |= 0b00000100;
      pulseLength = trimCh[3].actual;
    }
    if(trimCh[4].actual > 1750 || trimCh[4].actual < 1250){
      trigger = 4;
      receiverCheckByte |= 0b00001000;
      pulseLength = trimCh[4].actual;
    } 
  }
  if(trigger == 0){
    error = 1;
    Serial.println(F("No stick movement detected in the last 30 seconds!!! (ERROR 2)"));
  }
  //Assign the stick to the function.
  else{
    if(movement == 1){
      channel3Assign = trigger;
      if(pulseLength < 1250)channel3Assign += 0b10000000;
    }
    if(movement == 2){
      channel1Assign = trigger;
      if(pulseLength < 1250)channel1Assign += 0b10000000;
    }
    if(movement == 3){
      channel2Assign = trigger;
      if(pulseLength < 1250)channel2Assign += 0b10000000;
    }
    if(movement == 4){
      channel4Assign = trigger;
      if(pulseLength < 1250)channel4Assign += 0b10000000;
    }
  }
}


/**
 * @brief Check if the transmitter sticks are in the neutral position
 * 
 */
void waitSticksZero(){
  byte zero = 0;
  while(zero < 15){
    if(trimCh[1].actual < trimCh[1].center + 20 && trimCh[1].actual > trimCh[1].center - 20) zero |= 0b00000001;
    if(trimCh[2].actual < trimCh[2].center + 20 && trimCh[2].actual > trimCh[2].center - 20) zero |= 0b00000010;
    if(trimCh[3].actual < trimCh[3].center + 20 && trimCh[3].actual > trimCh[3].center - 20) zero |= 0b00000100;
    if(trimCh[4].actual < trimCh[4].center + 20 && trimCh[4].actual > trimCh[4].center - 20) zero |= 0b00001000;
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}


/**
 * @brief Check the inversion of the channel sticks
 * 
 * @param ch 
 */
void checkTrimInversion(byte ch){

    switch (ch) {
      case 1:
          Serial.println(F("Move the throttle stick to full throttle and back to center"));
          //Check for throttle movement
          checkReceiverInputs(1);
          Serial.print(F("Throttle is connected to digital input "));
          Serial.println((channel3Assign & 0b00000111) + 7);
          if(channel3Assign & 0b10000000)Serial.println(F("Channel inverted = yes"));
          else Serial.println(F("Channel inverted = no"));
          waitSticksZero();
          
          Serial.println(F(""));
          Serial.println(F(""));
          break;

      case 2:
          Serial.println(F("Move the roll stick to simulate left wing up and back to center"));
          //Check for throttle movement
          checkReceiverInputs(2);
          Serial.print(F("Roll is connected to digital input "));
          Serial.println((channel1Assign & 0b00000111) + 7);
          if(channel1Assign & 0b10000000)Serial.println(F("Channel inverted = yes"));
          else Serial.println(F("Channel inverted = no"));
          waitSticksZero();

          Serial.println(F(""));
          Serial.println(F(""));
          break;

      case 3:
          Serial.println(F("Move the pitch stick to simulate nose up and back to center"));
          //Check for throttle movement
          checkReceiverInputs(3);
          Serial.print(F("Pitch is connected to digital input "));
          Serial.println((channel2Assign & 0b00000111) + 7);
          if(channel2Assign & 0b10000000)Serial.println(F("Channel inverted = yes"));
          else Serial.println(F("Channel inverted = no"));
          waitSticksZero();
          Serial.println(F(""));
          Serial.println(F(""));
          break;
      
      case 4:
          Serial.println(F("Move the yaw stick to simulate nose right and back to center"));
          //Check for throttle movement
          checkReceiverInputs(4);
          Serial.print(F("Yaw is connected to digital input "));
          Serial.println((channel4Assign & 0b00000111) + 7);
          if(channel4Assign & 0b10000000)Serial.println(F("Channel inverted = yes"));
          else Serial.println(F("Channel inverted = no"));
          waitSticksZero();
          Serial.println(F(""));
          Serial.println(F(""));
          break;

      default:
          Serial.printf("ERROR. Channel not in range 1, 2, 3 and 4.");
          break;
    }

}


/**
 * @brief Register the min and max receiver values and exit when the sticks are back in the neutral position
 * 
 */
void register_min_max(){
  byte zero = 0;
  trimCh[1].low = trimCh[1].actual;
  trimCh[2].low = trimCh[2].actual;
  trimCh[3].low = trimCh[3].actual;
  trimCh[4].low = trimCh[4].actual;
  while(trimCh[1].actual < trimCh[1].center + 20 && trimCh[1].actual > trimCh[1].center - 20)delay(250);
  Serial.println(F("Measuring endpoints...."));
  while(zero < 15){
    if(trimCh[1].actual < trimCh[1].center + 20 && trimCh[1].actual > trimCh[1].center - 20) zero |= 0b00000001;
    if(trimCh[2].actual < trimCh[2].center + 20 && trimCh[2].actual > trimCh[2].center - 20) zero |= 0b00000010;
    if(trimCh[3].actual < trimCh[3].center + 20 && trimCh[3].actual > trimCh[3].center - 20) zero |= 0b00000100;
    if(trimCh[4].actual < trimCh[4].center + 20 && trimCh[4].actual > trimCh[4].center - 20) zero |= 0b00001000;
    if(trimCh[1].actual < trimCh[1].low) trimCh[1].low = trimCh[1].actual;
    if(trimCh[2].actual < trimCh[2].low) trimCh[2].low = trimCh[2].actual;
    if(trimCh[3].actual < trimCh[3].low) trimCh[3].low = trimCh[3].actual;
    if(trimCh[4].actual < trimCh[4].low) trimCh[4].low = trimCh[4].actual;
    if(trimCh[1].actual > trimCh[1].high) trimCh[1].high = trimCh[1].actual;
    if(trimCh[2].actual > trimCh[2].high) trimCh[2].high = trimCh[2].actual;
    if(trimCh[3].actual > trimCh[3].high) trimCh[3].high = trimCh[3].actual;
    if(trimCh[4].actual > trimCh[4].high) trimCh[4].high = trimCh[4].actual;
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

/**
 * @brief Wait to continue
 * 
 */
void checkToContinue(){
  byte continue_byte = 0;
  while(continue_byte == 0){
    if(channel2Assign == 0b00000001 && trimCh[1].actual > trimCh[1].center + 150) continue_byte = 1;
    if(channel2Assign == 0b10000001 && trimCh[1].actual < trimCh[1].center - 150) continue_byte = 1;
    if(channel2Assign == 0b00000010 && trimCh[2].actual > trimCh[2].center + 150) continue_byte = 1;
    if(channel2Assign == 0b10000010 && trimCh[2].actual < trimCh[2].center - 150) continue_byte = 1;
    if(channel2Assign == 0b00000011 && trimCh[3].actual > trimCh[3].center + 150) continue_byte = 1;
    if(channel2Assign == 0b10000011 && trimCh[3].actual < trimCh[3].center - 150) continue_byte = 1;
    if(channel2Assign == 0b00000100 && trimCh[4].actual > trimCh[4].center + 150) continue_byte = 1;
    if(channel2Assign == 0b10000100 && trimCh[4].actual < trimCh[4].center - 150) continue_byte = 1;
    vTaskDelay(10 / portTICK_PERIOD_MS);

  }
  waitSticksZero();
}


/**
 * @brief Finds the boundaries of the controller sticks
 * 
 */
void findSticksLimits(){

    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Gently move all the sticks simultaneously to their extends"));
    Serial.println(F("When ready put the sticks back in their center positions"));

    //Register the min and max values of the receiver channels
    register_min_max();
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("High, low and center values found during setup"));
    Serial.print(F("Digital input 08 values:"));
    Serial.print(trimCh[1].low);
    Serial.print(F(" - "));
    Serial.print(trimCh[1].center);
    Serial.print(F(" - "));
    Serial.println(trimCh[1].high);
    Serial.print(F("Digital input 09 values:"));
    Serial.print(trimCh[2].low);
    Serial.print(F(" - "));
    Serial.print(trimCh[2].center);
    Serial.print(F(" - "));
    Serial.println(trimCh[2].high);
    Serial.print(F("Digital input 10 values:"));
    Serial.print(trimCh[3].low);
    Serial.print(F(" - "));
    Serial.print(trimCh[3].center);
    Serial.print(F(" - "));
    Serial.println(trimCh[3].high);
    Serial.print(F("Digital input 11 values:"));
    Serial.print(trimCh[4].low);
    Serial.print(F(" - "));
    Serial.print(trimCh[4].center);
    Serial.print(F(" - "));
    Serial.println(trimCh[4].high);
    Serial.println(F("Move stick 'nose up' and back to center to continue"));
    checkToContinue();

}


/**
 * @brief Check if the angular position of a gyro axis is changing within 10 seconds
 * 
 * @param movement 
 */
void checkGyroAxes(byte movement){
  byte triggerAxis = 0;
  float gyroAngleRoll, gyroAnglePitch, gyroAngleYaw;
  //Reset all axes
  gyroAngleRoll = 0;
  gyroAnglePitch = 0;
  gyroAngleYaw = 0;

  readGyroscopeStatus();

  timer = millis();// + 10000; // milliseconds 
  //Serial.print(int());
  int delayMS = 4;
  float incrementCoefficient = 2*1 / ( 250 *65.5 );  // 0.0000611 = 1 / 65.5 (LSB deg/s) / 250(Hz)
  int angleLimit = 30;

  while(millis() - timer <= 1e4 && gyroAngleRoll > -angleLimit && gyroAngleRoll < angleLimit && gyroAnglePitch > -angleLimit && gyroAnglePitch < angleLimit && gyroAngleYaw > -angleLimit && gyroAngleYaw < angleLimit){
   
    readGyroscopeStatus();

    gyroAngleRoll += gyroAxis[1] * incrementCoefficient;          
    gyroAnglePitch += gyroAxis[2] * incrementCoefficient;
    gyroAngleYaw += gyroAxis[3] * incrementCoefficient;
  
  //   // --- debug
  //  Serial.print(millis()-timer);
  //  Serial.print(" ms, ");
  //  Serial.print(" R: ");
  //  Serial.print(gyroAngleRoll);
  //  Serial.print(" P: ");
  //  Serial.print(gyroAnglePitch);
  //  Serial.print(" Y: ");
  //  Serial.println(gyroAngleYaw);
    vTaskDelay(delayMS / portTICK_PERIOD_MS); //Loop is running @ 250(Hz). +/-300us is used for communication with the gyro

    //delayMicroseconds(3700);
  }

  //Assign the moved axis to the corresponding function (pitch, roll, yaw)
  if((gyroAngleRoll < -angleLimit || gyroAngleRoll > angleLimit) && gyroAnglePitch > -angleLimit && gyroAnglePitch < angleLimit && gyroAngleYaw > -angleLimit && gyroAngleYaw < angleLimit){
    gyroCheckByte |= 0b00000001;
    if(gyroAngleRoll < 0)triggerAxis = 0b10000001;
    else triggerAxis = 0b00000001;
  }
  if((gyroAnglePitch < -angleLimit || gyroAnglePitch > angleLimit) && gyroAngleRoll > -angleLimit && gyroAngleRoll < angleLimit && gyroAngleYaw > -angleLimit && gyroAngleYaw < angleLimit){
    gyroCheckByte |= 0b00000010;
    if(gyroAnglePitch < 0)triggerAxis = 0b10000010;
    else triggerAxis = 0b00000010;
  }
  if((gyroAngleYaw < -angleLimit || gyroAngleYaw > angleLimit) && gyroAngleRoll > -angleLimit && gyroAngleRoll < angleLimit && gyroAnglePitch > -angleLimit && gyroAnglePitch < angleLimit){
    gyroCheckByte |= 0b00000100;
    if(gyroAngleYaw < 0)triggerAxis = 0b10000011;
    else triggerAxis = 0b00000011;
  }
  
  if(triggerAxis == 0){
    error = 1;
    Serial.println(F("No angular motion is detected in the last 10 seconds!!! (ERROR 4)"));
  }
  else
  if(movement == 1) gyroCalibratedAxis[1] = triggerAxis;
  if(movement == 2) gyroCalibratedAxis[2] = triggerAxis;
  if(movement == 3) gyroCalibratedAxis[3] = triggerAxis;
}


/**
 * @brief Characterizes the gyroscope axes
 * 
 * @param ch 
 */
void configureGyroscopeAxes(byte ch){
    //Check axis movement
    checkGyroAxes(ch);
    
    if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(gyroCalibratedAxis[ch] & 0b00000011);
      if(gyroCalibratedAxis[ch] & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));
      Serial.println(F("Put the quadcopter back in its original position"));
      Serial.println(F("Move stick 'nose up' and back to center to continue"));
      checkToContinue();
    }
}


/**
 * @brief Verify everything is ok
 * 
 */
void checkGyroscopeResult(){
    
    if(receiverCheckByte == 0b00001111){
      Serial.println(F("Receiver channels ok"));
    }
    else{
      Serial.println(F("Receiver channel verification failed!!! (ERROR 6)"));
      error = 1;
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if(gyroCheckByte == 0b00000111){
      Serial.println(F("Gyro axes ok"));
    }
    else{
      Serial.println(F("Gyro exes verification failed!!! (ERROR 7)"));
      Serial.println(gyroCheckByte);
      error = 1;
    }  
}


/**
 * @brief Light up leds
 * 
 */
void checkLed(){

    ledcWrite(pwmLedFlyChannel, MAX_DUTY_CYCLE);  // turn on the LED if the loop time exceeds 4050us
    ledcWrite(pwmLedChannel, MAX_DUTY_CYCLE);     // turn on the LED if the loop time exceeds 4050us

    Serial.println(F("\n\nThe LED should now be lit"));
    Serial.println(F("Move stick 'nose up' and back to center to continue"));
    checkToContinue();

    ledcWrite(pwmLedFlyChannel, 0);  // turn on the LED if the loop time exceeds 4050us
    ledcWrite(pwmLedChannel, 0);     // turn on the LED if the loop time exceeds 4050us

}


/**
 * @brief Write to EEPROM
 * 
 */
void writeEEPROM(){
     //If all is good, store the information in the EEPROM
    Serial.println(F("\n\n"));
    Serial.println(F("Storing EEPROM information"));
    Serial.println(F("Writing EEPROM"));
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.println(F("Done!"));
    EEPROM.write(0, trimCh[1].center & 0b11111111);
    EEPROM.write(1, trimCh[1].center >> 8);
    EEPROM.write(2, trimCh[2].center & 0b11111111);
    EEPROM.write(3, trimCh[2].center >> 8);
    EEPROM.write(4, trimCh[3].center & 0b11111111);
    EEPROM.write(5, trimCh[3].center >> 8);
    EEPROM.write(6, trimCh[4].center & 0b11111111);
    EEPROM.write(7, trimCh[4].center >> 8);
    EEPROM.write(8, trimCh[1].high & 0b11111111);
    EEPROM.write(9, trimCh[1].high >> 8);
    EEPROM.write(10, trimCh[2].high & 0b11111111);
    EEPROM.write(11, trimCh[2].high >> 8);
    EEPROM.write(12, trimCh[3].high & 0b11111111);
    EEPROM.write(13, trimCh[3].high >> 8);
    EEPROM.write(14, trimCh[4].high & 0b11111111);
    EEPROM.write(15, trimCh[4].high >> 8);
    EEPROM.write(16, trimCh[1].low & 0b11111111);
    EEPROM.write(17, trimCh[1].low >> 8);
    EEPROM.write(18, trimCh[2].low & 0b11111111);
    EEPROM.write(19, trimCh[2].low >> 8);
    EEPROM.write(20, trimCh[3].low & 0b11111111);
    EEPROM.write(21, trimCh[3].low >> 8);
    EEPROM.write(22, trimCh[4].low & 0b11111111);
    EEPROM.write(23, trimCh[4].low >> 8);
    EEPROM.write(24, channel1Assign);
    EEPROM.write(25, channel2Assign);
    EEPROM.write(26, channel3Assign);
    EEPROM.write(27, channel4Assign);
    EEPROM.write(28, gyroCalibratedAxis[1]);
    EEPROM.write(29, gyroCalibratedAxis[2]);
    EEPROM.write(30, gyroCalibratedAxis[3]);
    EEPROM.write(31, type);
    EEPROM.write(32, GYRO_ADDRESS);
    //Write the EEPROM signature
    EEPROM.write(33, 'J'); 
    EEPROM.write(34, 'M');
    EEPROM.write(35, 'B');
    
    //Commit changes
    EEPROM.commit();

    //To make sure evrything is ok, verify the EEPROM data.
    Serial.println(F("Verify EEPROM data"));
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    
    if(trimCh[1].center != ((EEPROM.read(1) << 8) | EEPROM.read(0))){error = 1;Serial.println(" EEPROM error writing --> trimCh[1].center");}
    if(trimCh[2].center != ((EEPROM.read(3) << 8) | EEPROM.read(2))){error = 1;Serial.println(" EEPROM error writing --> trimCh[2].center");}
    if(trimCh[3].center != ((EEPROM.read(5) << 8) | EEPROM.read(4))){error = 1;Serial.println(" EEPROM error writing --> trimCh[3].center");}
    if(trimCh[4].center != ((EEPROM.read(7) << 8) | EEPROM.read(6))){error = 1;Serial.println(" EEPROM error writing --> trimCh[4].center");}

    if(trimCh[1].high != ((EEPROM.read(9) << 8) | EEPROM.read(8))){error = 1;Serial.println(" EEPROM error writing --> trimCh[1].high");}
    if(trimCh[2].high != ((EEPROM.read(11) << 8) | EEPROM.read(10))){error = 1;Serial.println(" EEPROM error writing --> trimCh[2].high");}
    if(trimCh[3].high != ((EEPROM.read(13) << 8) | EEPROM.read(12))){error = 1;Serial.println(" EEPROM error writing --> trimCh[3].high");}
    if(trimCh[4].high != ((EEPROM.read(15) << 8) | EEPROM.read(14))){error = 1;Serial.println(" EEPROM error writing --> trimCh[4].high");}
    
    if(trimCh[1].low != ((EEPROM.read(17) << 8) | EEPROM.read(16))){error = 1;Serial.println(" EEPROM error writing --> trimCh[1].low");}
    if(trimCh[2].low != ((EEPROM.read(19) << 8) | EEPROM.read(18))){error = 1;Serial.println(" EEPROM error writing --> trimCh[2].low");}
    if(trimCh[3].low != ((EEPROM.read(21) << 8) | EEPROM.read(20))){error = 1;Serial.println(" EEPROM error writing --> trimCh[3].low");}
    if(trimCh[4].low != ((EEPROM.read(23) << 8) | EEPROM.read(22))){error = 1;Serial.println(" EEPROM error writing --> trimCh[4].low");}
    
    if(channel1Assign != EEPROM.read(24)){error = 1;Serial.println(" EEPROM error writing --> channel1Assign");}
    if(channel2Assign != EEPROM.read(25)){error = 1;Serial.println(" EEPROM error writing --> channel2Assign");}
    if(channel3Assign != EEPROM.read(26)){error = 1;Serial.println(" EEPROM error writing --> channel3Assign");}
    if(channel4Assign != EEPROM.read(27)){error = 1;Serial.println(" EEPROM error writing --> channel4Assign");}
    
    if(gyroCalibratedAxis[1] != EEPROM.read(28)){error = 1;Serial.println(" EEPROM error writing --> gyroCalibratedAxis[1]");}
    if(gyroCalibratedAxis[2] != EEPROM.read(29)){error = 1;Serial.println(" EEPROM error writing --> gyroCalibratedAxis[2]");}
    if(gyroCalibratedAxis[3] != EEPROM.read(30)){error = 1;Serial.println(" EEPROM error writing --> gyroCalibratedAxis[3]");}
    if(type != EEPROM.read(31)){error = 1;Serial.println(" EEPROM error writing --> type");}
    if(GYRO_ADDRESS != EEPROM.read(32)){error = 1;Serial.println(" EEPROM error writing --> gyroAddress");}
    
    if('J' != EEPROM.read(33)){error = 1;Serial.println(" EEPROM error writing --> j");}
    if('M' != EEPROM.read(34)){error = 1;Serial.println(" EEPROM error writing --> m");}
    if('B' != EEPROM.read(35)){error = 1;Serial.println(" EEPROM error writing --> b");}
  
    if(error == 1)Serial.println(F("EEPROM verification failed!!! (ERROR 5)"));
    else{
      Serial.println(F("Verification done"));

      Serial.println("");
      Serial.println("========================================");    
      Serial.println("EEPROM DATA");   
      Serial.println(); 
      for(int i = 0; i < EEPROM_SIZE; i++){
        Serial.println(EEPROM.read(i));
      }
    }
}
