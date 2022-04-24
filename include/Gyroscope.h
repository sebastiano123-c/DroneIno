/**
 * @file Gyroscope.h
 * @author @sebastiano123-c
 * @brief Gyroscopes routines.
 * @version 0.1
 * @date 2022-02-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// void setupMPU(){
//   Wire.setClock(WIRE_CLOCK);
//   Wire.begin();
//   Wire.beginTransmission(gyroAddress);                        //Start communication with the MPU-6050.
//   int error = Wire.endTransmission();                              //End the transmission and register the exit status.
//   while (error != 0) {                                          //Stay in this loop because the MPU-6050 did not respond.
//     error = 1;                                                  //Set the error status to 1.
//     digitalWrite(PIN_BATTERY_LED, !digitalRead(PIN_BATTERY_LED));
//     delay(40);                                                   //Simulate a 250Hz refresc rate as like the main loop.
//     Serial.println("MPU6050 ERROR.");
//   }
// }

/**
 * @brief Try a first communication with the gyroscope
 */
void setGyroscopeRegisters(){

  //Start communication with the gyroscope.                           
  Wire.beginTransmission(GYRO_ADDRESS);

  error = Wire.endTransmission();                              //End the transmission and register the exit status.
  while (error != 0) {                                             //Stay in this loop because the MPU-6050 did not respond.
    error = 1;                                                     //Set the error status to 1.
    ledcWrite(pwmLedChannel, abs(MAX_DUTY_CYCLE - ledcRead(pwmLedChannel)));
    Serial.print("GYROSCOPE ERROR at address: ");
    Serial.println(GYRO_ADDRESS);
    vTaskDelay(80/portTICK_PERIOD_MS);                             //Simulate a 250Hz refresc rate as like the main loop.
  }
  ledcWrite(pwmLedChannel, 0);

  //Setup the MPU-6050
  #if GYRO_ADDRESS == MPU_6050_ADDRESS
    Wire.beginTransmission(GYRO_ADDRESS);                        //Start communication with the address found during search.
    Wire.write(PWR_MGMT_1);                                     //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                           //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                     //End the transmission with the gyro.

    Wire.beginTransmission(GYRO_ADDRESS);                        //Start communication with the address found during search.
    Wire.write(GYRO_CONFIG);                                    //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(GYRO_REGISTERS_BITS);                            //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                     //End the transmission with the gyro

    Wire.beginTransmission(GYRO_ADDRESS);                        //Start communication with the address found during search.
    Wire.write(ACCEL_CONFIG);                                   //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(ACC_REGISTERS_BITS);                             //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();                                     //End the transmission with the gyro

    //Let's perform a random register check to see if the values are written correct
    Wire.beginTransmission(GYRO_ADDRESS);                        //Start communication with the address found during search
    Wire.write(GYRO_CONFIG);                                    //Start reading @ register GYRO_CONFIG
    Wire.endTransmission();                                     //End the transmission
    Wire.requestFrom(GYRO_ADDRESS, 1);                           //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                //Wait until the 6 bytes are received
    if(Wire.read() != GYRO_REGISTERS_BITS){                     //Check if the value is 0x08
      ledcWrite(pwmLedChannel, MAX_DUTY_CYCLE);                      //Turn on the warning led
      while(1)vTaskDelay(10/portTICK_PERIOD_MS);                //Stay in this loop for ever
    }

    Wire.beginTransmission(GYRO_ADDRESS);                        //Start communication with the address found during search
    Wire.write(0x1A);                                           //We want to write to the CONFIG register (1A hex)
    Wire.write(DIGITAL_LOW_PASS_FILTER);                        //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();                                     //End the transmission with the gyro    
  #endif
}

/** 
 * @brief Read the gyroscope data
 */
void readGyroscopeStatus(){
  
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x3B);                                             // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADDRESS, 14);                           // request a total of 14 registers

  accAxis[1]  = Wire.read()<<8|Wire.read();                     // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  accAxis[2]  = Wire.read()<<8|Wire.read();                     // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accAxis[3]  = Wire.read()<<8|Wire.read();                     // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  gyroTemp    = Wire.read()<<8|Wire.read();                     // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyroAxis[1] = Wire.read()<<8|Wire.read();                     // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyroAxis[2] = Wire.read()<<8|Wire.read();                     // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyroAxis[3] = Wire.read()<<8|Wire.read();                     // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  #if UPLOADED_SKETCH == CALIBRATION || UPLOADED_SKETCH == FLIGHT_CONTROLLER

    if(eepromData[28] & 0b10000000)gyroAxis[1] *= -1;               //Invert gyroAxis[1] if the MSB of EEPROM bit 28 is set.
    if(eepromData[29] & 0b10000000)gyroAxis[2] *= -1;               //Invert gyroAxis[2] if the MSB of EEPROM bit 29 is set.
    if(eepromData[30] & 0b10000000)gyroAxis[3] *= -1;               //Invert gyroAxis[3] if the MSB of EEPROM bit 30 is set.
    if(eepromData[28] & 0b10000000)accAxis[1] *= -1;                //Invert accAxis[1] if the MSB of EEPROM bit 28 is set.
    if(eepromData[29] & 0b10000000)accAxis[2] *= -1;                //Invert accAxis[2] if the MSB of EEPROM bit 29 is set.
    if(eepromData[30] & 0b10000000)accAxis[3] *= -1;                //Invert accAxis[3] if the MSB of EEPROM bit 30 is set.
  
  #endif

  if(calInt >= 2000){
    gyroAxis[1] -= gyroAxisCalibration[1];                       //Only compensate after the calibration.
    gyroAxis[2] -= gyroAxisCalibration[2];                       //Only compensate after the calibration.
    gyroAxis[3] -= gyroAxisCalibration[3];                       //Only compensate after the calibration.
    // accAxis[1]  -= accAxisCalibration[1];
    // accAxis[2]  -= accAxisCalibration[2];
    // accAxis[3]  -= accAxisCalibration[3];
  }

}

/**
 * @brief Uses the gyroscope measurement to calculate the accelerometer and the gyroscope calibration
 */
void calibrateGyroscope(){
 
  #if DEBUG || UPLOADED_SKETCH != FLIGHT_CONTROLLER 
    Serial.printf("\nCalibrating gyroscope... Please wait 8s\n");
  #endif

  accAxisCalibration[1] = 0;
  accAxisCalibration[2] = 0;
  accAxisCalibration[3] = 0;
  gyroAxisCalibration[1] = 0;                                       //Ad roll value to gyro_roll_cal.
  gyroAxisCalibration[2] = 0;                                       //Ad pitch value to gyro_pitch_cal.
  gyroAxisCalibration[3] = 0;                                       //Ad yaw value to gyro_yaw_cal.
  
  vTaskDelay(1000/portTICK_PERIOD_MS);                              //Wait before continuing.
  
  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  for (calInt = 0; calInt < 2000; calInt ++){                       //Take 2000 readings for calibration.
    
    if(calInt % 25 == 0){                                           //Change the led status to indicate calibration.
      ledcWrite(pwmLedChannel, abs(MAX_DUTY_CYCLE - ledcRead(pwmLedChannel)));
      #if DEBUG
        Serial.print(".");
      #endif
    } else
      ledcWrite(pwmLedBatteryChannel, abs(MAX_DUTY_CYCLE - ledcRead(pwmLedChannel)));

    
    readGyroscopeStatus();                                           //Read the gyro output.
    
    gyroAxisCalibration[1] += gyroAxis[1];                           //Ad roll value to gyro_roll_cal.
    gyroAxisCalibration[2] += gyroAxis[2];                           //Ad pitch value to gyro_pitch_cal.
    gyroAxisCalibration[3] += gyroAxis[3];                           //Ad yaw value to gyro_yaw_cal.

    accAxisCalibration[1] += accAxis[1];
    accAxisCalibration[2] += accAxis[2];
    accAxisCalibration[3] += accAxis[3];

    vTaskDelay(4/portTICK_PERIOD_MS);

  }

  // divide by 2000 to get the average gyro offset.
  gyroAxisCalibration[1] /= 2000;                                     //Divide the roll total by 2000.
  gyroAxisCalibration[2] /= 2000;                                     //Divide the pitch total by 2000.
  gyroAxisCalibration[3] /= 2000;                                     //Divide the yaw total by 2000.

  accAxisCalibration[1] /= 2000;
  accAxisCalibration[2] /= 2000;
  accAxisCalibration[3] /= 2000;

}

/** 
 * @brief Prints gyroscope readings
 */
void printGyroscopeStatus(){

  Serial.print("Pitch: ");
  Serial.print(anglePitch, 3);
  Serial.print(" Roll: ");
  Serial.print(angleRoll, 3);
  Serial.print(" Yaw: ");
  Serial.print(gyroAxis[3]/65.5 , 3);

  // Serial.print(" Fileter: ");
  // Serial.print(GYROSCOPE_ROLL_FILTER, 6);
  // Serial.print(" GYROSCOPE_ROLL_CORR: ");
  // Serial.print(GYROSCOPE_ROLL_CORR, 2);
  // Serial.print(" GYROSCOPE_PITCH_CORR: ");
  // Serial.print(GYROSCOPE_PITCH_CORR ,0);

  Serial.println();

}

/**
 * @brief Calculate the pitch, roll and yaw angles from the previous gyroscope readings
 */
void calculateAnglePRY(){
 
  // first of all, read the gyro
  readGyroscopeStatus();
  
  #if AUTO_LEVELING
    // gyroSensibility = [deg/sec] (check the datasheet of the MPU-6050 for more information).
    gyroRollInput = (gyroRollInput * 0.7) +
                    ((gyroAxis[1] / gyroSensibility) * 0.3);                 //Gyro pid input is deg/sec.
    gyroPitchInput = (gyroPitchInput * 0.7) + 
                    ((gyroAxis[2] / gyroSensibility) * 0.3);                 //Gyro pid input is deg/sec.
    gyroYawInput = (gyroYawInput * 0.7) +
                    ((gyroAxis[3] / gyroSensibility) * 0.3);                 //Gyro pid input is deg/sec.
  
    //Gyro angle calculations
    anglePitch += gyroAxis[2] * travelCoeff;                                  //Calculate the traveled pitch angle and add it to the anglePitch variable.
    angleRoll += gyroAxis[1] * travelCoeff;                                   //Calculate the traveled roll angle and add it to the angleRoll variable. 
    
    //The Arduino sin function is in radians
    anglePitch -= angleRoll * sin(gyroAxis[3] * travelCoeffToRad);            //If the IMU has yawed transfer the roll angle to the pitch angel.
    angleRoll += anglePitch * sin(gyroAxis[3] * travelCoeffToRad);            //If the IMU has yawed transfer the pitch angle to the roll angel.

    //Accelerometer angle calculations
    accTotalVector = sqrt((accAxis[2]*accAxis[2])+
                          (accAxis[1]*accAxis[1])+
                          (accAxis[3]*accAxis[3]));                           //Calculate the total accelerometer vector.

    //The Arduino asin function is in radians
    if(abs(accAxis[1]) < accTotalVector){                                     //Prevent the asin function to produce a NaN
      anglePitchAcc = asin((float)accAxis[1]/accTotalVector)* convDegToRad;   //Calculate the pitch angle.
    }
    if(abs(accAxis[2]) < accTotalVector){                                     //Prevent the asin function to produce a NaN
      angleRollAcc = asin((float)accAxis[2]/accTotalVector)* (-convDegToRad); //Calculate the roll angle.
    }
    
    //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
    anglePitchAcc -= GYROSCOPE_PITCH_CORR;                                    //Accelerometer calibration value for pitch.
    angleRollAcc -= GYROSCOPE_ROLL_CORR;                                      //Accelerometer calibration value for roll. 

    anglePitch = anglePitchAcc + 
                  GYROSCOPE_PITCH_FILTER * (anglePitch - anglePitchAcc);      //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
    angleRoll = angleRollAcc + 
                  GYROSCOPE_ROLL_FILTER * (angleRoll - angleRollAcc);         //Correct the drift of the gyro roll angle with the accelerometer roll angle.


    pitchLevelAdjust = anglePitch * correctionPitchRoll;                       //Calculate the pitch angle correction
    rollLevelAdjust = angleRoll * correctionPitchRoll;                         //Calculate the roll angle correction
  #else
    if(AUTO_LEVELING == false){                                                        //If the quadcopter is not in auto-level mode
      pitchLevelAdjust = 0;                                                    //Set the pitch angle correction to zero.
      rollLevelAdjust = 0;                                                     //Set the roll angle correcion to zero.
    }
  #endif

  #if DEBUG == true
    // printGyroscopeStatus();                                                  //print gyro status
  #endif
  
}
