/**
 * @file Gyroscope.h
 * @author @sebastiano123-c
 * @brief Gyroscopes routines.
 * @version 0.1
 * @date 2022-02-28
 * 
 * @copyright Copyright (c) 2022
 * 
 * There is a large variation in the values as the sensor starts up, especially in the yaw data.
 * The gyroscope sensor drift stops after around 13 seconds, probably due to the completion of an integrated auto-calibration process.
 * In some occasions take up to 40 seconds to complete its calibration. Therefore we should take this delay into account in our program.
 * If we want to use the yaw data, the robot should wait for around 40 seconds before beginning to use the sensor and starting the main program.
 * Since it is very unlikely to wait this amount of time
 * 
 */

#define CALINT_MAX        2000        // number of acquisition for the calibration
#define CALINT_DELAY_MS   5          // (ms) time delay for each acquisition

/**
 * @brief Try a first communication with the gyroscope
 * 
 */
void setGyroscopeRegisters(){

  //Start communication with the gyroscope.                           
  Wire.beginTransmission(GYRO_ADDRESS);

  error = Wire.endTransmission();                              //End the transmission and register the exit status.
  while (error != 0) {                                             //Stay in this loop because the MPU-6050 did not respond.
    error = 1;                                                     //Set the error status to 1.
    ledcWrite(pwmLedChannel, abs(MAX_DUTY_CYCLE - (int)ledcRead(pwmLedChannel)));
    Serial.print("GYROSCOPE ERROR at address: ");
    Serial.println(GYRO_ADDRESS);
    vTaskDelay(80/portTICK_PERIOD_MS);                             //Simulate a 250Hz refresc rate as like the main loop.
  }
  ledcWrite(pwmLedChannel, 0);

  //Setup the MPU-6050
  #if GYRO_ADDRESS == MPU_6050_ADDRESS
  //   Wire.beginTransmission(GYRO_ADDRESS);                        //Start communication with the address found during search.
  //   Wire.write(PWR_MGMT_1);                                     //We want to write to the PWR_MGMT_1 register (6B hex)
  //   Wire.write(0x00);                                           //Set the register bits as 00000000 to activate the gyro
  //   Wire.endTransmission();                                     //End the transmission with the gyro.

  //   Wire.beginTransmission(GYRO_ADDRESS);                        //Start communication with the address found during search.
  //   Wire.write(GYRO_CONFIG);                                    //We want to write to the GYRO_CONFIG register (1B hex)
  //   Wire.write(GYRO_REGISTERS_BITS);                            //Set the register bits as 00001000 (500dps full scale)
  //   Wire.endTransmission();                                     //End the transmission with the gyro

  //   Wire.beginTransmission(GYRO_ADDRESS);                        //Start communication with the address found during search.
  //   Wire.write(ACCEL_CONFIG);                                   //We want to write to the ACCEL_CONFIG register (1A hex)
  //   Wire.write(ACC_REGISTERS_BITS);                             //Set the register bits as 00010000 (+/- 8g full scale range)
  //   Wire.endTransmission();                                     //End the transmission with the gyro

  //   //Let's perform a random register check to see if the values are written correct
  //   Wire.beginTransmission(GYRO_ADDRESS);                        //Start communication with the address found during search
  //   Wire.write(GYRO_CONFIG);                                    //Start reading @ register GYRO_CONFIG
  //   Wire.endTransmission();                                     //End the transmission
  //   Wire.requestFrom(GYRO_ADDRESS, 1);                           //Request 1 bytes from the gyro
  //   while(Wire.available() < 1);                                //Wait until the 6 bytes are received
  //   if(Wire.read() != GYRO_REGISTERS_BITS){                     //Check if the value is 0x08
  //     ledcWrite(pwmLedChannel, MAX_DUTY_CYCLE);                      //Turn on the warning led
  //     while(1)vTaskDelay(10/portTICK_PERIOD_MS);                //Stay in this loop for ever
  //   }

  //   Wire.beginTransmission(GYRO_ADDRESS);                        //Start communication with the address found during search
  //   Wire.write(0x1A);                                           //We want to write to the CONFIG register (1A hex)
  //   Wire.write(DIGITAL_LOW_PASS_FILTER);                        //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  //   Wire.endTransmission();                                     //End the transmission with the gyro


  Wire.beginTransmission(GYRO_ADDRESS);                        //Start communication with the MPU-6050.
  Wire.write(0x6B);                                            //We want to write to the PWR_MGMT_1 register (6B hex).
  Wire.write(0x00);                                            //Set the register bits as 00000000 to activate the gyro.
  Wire.endTransmission();                                      //End the transmission with the gyro.

  Wire.beginTransmission(GYRO_ADDRESS);                        //Start communication with the MPU-6050.
  Wire.write(0x1B);                                            //We want to write to the GYRO_CONFIG register (1B hex).
  Wire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale).
  Wire.endTransmission();                                      //End the transmission with the gyro.

  Wire.beginTransmission(GYRO_ADDRESS);                        //Start communication with the MPU-6050.
  Wire.write(0x1C);                                            //We want to write to the ACCEL_CONFIG register (1A hex).
  Wire.write(0x10);                                            //Set the register bits as 00010000 (+/- 8g full scale range).
  Wire.endTransmission();                                      //End the transmission with the gyro.

  Wire.beginTransmission(GYRO_ADDRESS);                        //Start communication with the MPU-6050.
  Wire.write(0x1A);                                            //We want to write to the CONFIG register (1A hex).
  Wire.write(0x03);                                            //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz).
  Wire.endTransmission();                                      //End the transmission with the gyro.


  #endif
}

/** 
 * @brief Read the gyroscope data
 * 
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

  if(calInt == CALINT_MAX){
    gyroAxis[1] -= gyroAxisCalibration[1];                       //Only compensate after the calibration.
    gyroAxis[2] -= gyroAxisCalibration[2];                       //Only compensate after the calibration.
    gyroAxis[3] -= gyroAxisCalibration[3];                       //Only compensate after the calibration.

    // no need of calibrating the accelerometer. Otherwise gyro has no reference on the zero axis
    // accAxis[1]  -= accAxisCalibration[1];
    // accAxis[2]  -= accAxisCalibration[2];
    // accAxis[3]  -= accAxisCalibration[3];
  }

}

/**
 * @brief Uses the gyroscope measurement to calculate the accelerometer and the gyroscope calibration
 * 
 */
void calibrateGyroscope(){
 
  #if DEBUG || UPLOADED_SKETCH != FLIGHT_CONTROLLER 
    Serial.printf("\nCalibrating gyroscope... Please wait %is\n", CALINT_MAX*CALINT_DELAY_MS/1000 );
  #endif

  accAxisCalibration[1] = 0.;
  accAxisCalibration[2] = 0.;
  accAxisCalibration[3] = 0.;
  gyroAxisCalibration[1] = 0.;                                       //Ad roll value to gyro_roll_cal.
  gyroAxisCalibration[2] = 0.;                                       //Ad pitch value to gyro_pitch_cal.
  gyroAxisCalibration[3] = 0.;                                       //Ad yaw value to gyro_yaw_cal.
  
  vTaskDelay(1000/portTICK_PERIOD_MS);                              //Wait before continuing.
  
  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  for (calInt = 0; calInt < CALINT_MAX; calInt ++){                       //Take 2000 readings for calibration.
    
    if(calInt % 25 == 0){                                           //Change the led status to indicate calibration.
      ledcWrite(pwmLedChannel, abs(MAX_DUTY_CYCLE - (int)ledcRead(pwmLedChannel)));
      #if DEBUG || UPLOADED_SKETCH == CALIBRATION
        Serial.printf("%.1fs - ", (float)(CALINT_MAX - calInt)*CALINT_DELAY_MS/1000.);
      #endif
    } else
      ledcWrite(pwmLedBatteryChannel, abs(MAX_DUTY_CYCLE - (int)ledcRead(pwmLedChannel)));

    
    readGyroscopeStatus();                                           //Read the gyro output.
    
    // although inefficient, the division is done inside the loop due to overflow issues 
    // (which is not a problem since we are in the setup)
    gyroAxisCalibration[1] += (double)gyroAxis[1]/(double)CALINT_MAX;                     // gyro roll calibration to set the zero
    gyroAxisCalibration[2] += (double)gyroAxis[2]/(double)CALINT_MAX;                     // gyro pitch calibration to set the zero
    gyroAxisCalibration[3] += (double)gyroAxis[3]/(double)CALINT_MAX;                     // gyro yaw calibration to set the zero

    accAxisCalibration[1] += (double)accAxis[1]/(double)CALINT_MAX;                       // roll acc calibration preventing drifts when hovering
    accAxisCalibration[2] += (double)accAxis[2]/(double)CALINT_MAX;                       // pitch acc calibration preventing drifts when hovering
    accAxisCalibration[3] += (double)accAxis[2]/(double)CALINT_MAX;                       // pitch acc calibration preventing drifts when hovering

    vTaskDelay(CALINT_DELAY_MS/portTICK_PERIOD_MS);

  }

}


/** 
 * @brief Prints gyroscope readings
 */
void printGyroscopeStatus(){

  Serial.printf("Roll: %.4f, Pitch: %.4f, Yaw: %.4f,  \t aX: %i, aY: %i, aZ: %i\n ", angleRoll, anglePitch, gyroAxis[3]/65.5f, accAxis[1], accAxis[2], accAxis[3]);

}


/**
 * @brief Calculate the pitch, roll and yaw angles from the previous gyroscope readings
 * 
 */
void calculateAnglePRY(){
 
  // first of all, read the gyro
  readGyroscopeStatus();

  
  // gyroSensibility = [deg/sec] (check the datasheet of the MPU-6050 for more information).
  gyroRollInput = (gyroRollInput * filterHigh) +
                  (((float)gyroAxis[1] / gyroSensibility) * filterLow);                 //Gyro pid input is deg/sec.
  gyroPitchInput = (gyroPitchInput * filterHigh) + 
                  (((float)gyroAxis[2] / gyroSensibility) * filterLow);                 //Gyro pid input is deg/sec.
  gyroYawInput = (gyroYawInput * 0.85f) +
                  (((float)gyroAxis[3] / gyroSensibility) * 0.15f);                 //Gyro pid input is deg/sec.


  //Gyro angle calculations
  anglePitch += (float)gyroAxis[2] * travelCoeff;                                  //Calculate the traveled pitch angle and add it to the anglePitch variable.
  angleRoll += (float)gyroAxis[1] * travelCoeff;                                   //Calculate the traveled roll angle and add it to the angleRoll variable. 
  
  //The Arduino sin function is in radians
  anglePitch -= angleRoll * sin((float)gyroAxis[3] * travelCoeffToRad);            //If the IMU has yawed transfer the roll angle to the pitch angel.
  angleRoll += anglePitch * sin((float)gyroAxis[3] * travelCoeffToRad);            //If the IMU has yawed transfer the pitch angle to the roll angel.


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


  #if AUTO_LEVELING
    
    pitchLevelAdjust = anglePitch * correctionPitchRoll;                       //Calculate the pitch angle correction
    rollLevelAdjust = angleRoll * correctionPitchRoll;                         //Calculate the roll angle correction

  #else

      pitchLevelAdjust = 0;                                                    //Set the pitch angle correction to zero.
      rollLevelAdjust = 0;                                                     //Set the roll angle correcion to zero.

  #endif


  #if DEBUG == true && defined(DEBUG_GYRO)
    printGyroscopeStatus();                                                  //print gyro status
  #endif
  
}
