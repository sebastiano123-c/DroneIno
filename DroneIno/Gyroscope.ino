void setupGyroscope(){
  // set gyro address
  gyroAddress = GYRO_ADDRESS; 
  
  // setup wire
  Wire.setClock(WIRE_CLOCK);
  Wire.begin(PIN_SDA, PIN_SCL);
  vTaskDelay(40/portTICK_PERIOD_MS);

  Wire.beginTransmission(gyroAddress);

  //Start communication with the MPU-6050.                           
  int error = Wire.endTransmission();                              //End the transmission and register the exit status.
  while (error != 0) {                                             //Stay in this loop because the MPU-6050 did not respond.
    error = 1;                                                     //Set the error status to 1.
    ledcWrite(pwmLedChannel, MAX_DUTY_CYCLE);
    vTaskDelay(40/portTICK_PERIOD_MS);                             //Simulate a 250Hz refresc rate as like the main loop.
    ledcWrite(pwmLedChannel, 0);
    vTaskDelay(80/portTICK_PERIOD_MS);                             //Simulate a 250Hz refresc rate as like the main loop.
    Serial.print("MPU6050 ERROR at address: ");
    Serial.println(gyroAddress);
  }
  ledcWrite(pwmLedChannel, 0);
  if(DEBUG) Serial.println("Gyroscope setup: OK");
}

void setGyroscopeRegisters(){
  //Setup the MPU-6050
  if(eepromData[31] == 1){
    Wire.beginTransmission(gyroAddress);                        //Start communication with the address found during search.
    Wire.write(PWR_MGMT_1);                                     //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                           //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                     //End the transmission with the gyro.

    Wire.beginTransmission(gyroAddress);                        //Start communication with the address found during search.
    Wire.write(GYRO_CONFIG);                                    //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(GYRO_REGISTERS_BITS);                            //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                     //End the transmission with the gyro

    Wire.beginTransmission(gyroAddress);                        //Start communication with the address found during search.
    Wire.write(ACCEL_CONFIG);                                   //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(ACC_REGISTERS_BITS);                             //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();                                     //End the transmission with the gyro

    //Let's perform a random register check to see if the values are written correct
    Wire.beginTransmission(gyroAddress);                        //Start communication with the address found during search
    Wire.write(GYRO_CONFIG);                                    //Start reading @ register GYRO_CONFIG
    Wire.endTransmission();                                     //End the transmission
    Wire.requestFrom(gyroAddress, 1);                           //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                //Wait until the 6 bytes are received
    if(Wire.read() != GYRO_REGISTERS_BITS){                     //Check if the value is 0x08
      ledcWrite(pwmLedChannel, MAX_DUTY_CYCLE);                      //Turn on the warning led
      while(1)vTaskDelay(10/portTICK_PERIOD_MS);                //Stay in this loop for ever
    }

    Wire.beginTransmission(gyroAddress);                        //Start communication with the address found during search
    Wire.write(0x1A);                                           //We want to write to the CONFIG register (1A hex)
    Wire.write(DIGITAL_LOW_PASS_FILTER);                        //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();                                     //End the transmission with the gyro    
  } 
}

void calibrateGyroscope(){
for (calInt = 0; calInt < 1250 ; calInt ++){                           //Wait 5 seconds before continuing.
    ledcWrite(pwmChannel1, MAX_DUTY_CYCLE);
    ledcWrite(pwmChannel2, MAX_DUTY_CYCLE);
    ledcWrite(pwmChannel3, MAX_DUTY_CYCLE);
    ledcWrite(pwmChannel4, MAX_DUTY_CYCLE);
    
    vTaskDelay(1/portTICK_PERIOD_MS);                                                //Wait 1000us.
    ledcWrite(pwmChannel1, HALF_DUTY_CYCLE);
    ledcWrite(pwmChannel2, HALF_DUTY_CYCLE);
    ledcWrite(pwmChannel3, HALF_DUTY_CYCLE);
    ledcWrite(pwmChannel4, HALF_DUTY_CYCLE);                                                     //Set digital port 4, 5, 6 and 7 low.
    vTaskDelay(3/portTICK_PERIOD_MS);                                                //Wait 3000us.
  }

  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  for (calInt = 0; calInt < 2000 ; calInt ++){                           //Take 2000 readings for calibration.
    if(calInt % 15 == 0) ledcWrite(pwmLedChannel, MAX_DUTY_CYCLE);                //Change the led status to indicate calibration.
    else ledcWrite(pwmLedChannel, 0);
    readGyroscopeStatus();                                                        //Read the gyro output.
    gyroAxisCalibration[1] += gyroAxis[1];                                       //Ad roll value to gyro_roll_cal.
    gyroAxisCalibration[2] += gyroAxis[2];                                       //Ad pitch value to gyro_pitch_cal.
    gyroAxisCalibration[3] += gyroAxis[3];                                       //Ad yaw value to gyro_yaw_cal.

    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
    ledcWrite(pwmChannel1, MAX_DUTY_CYCLE);
    ledcWrite(pwmChannel2, MAX_DUTY_CYCLE);
    ledcWrite(pwmChannel3, MAX_DUTY_CYCLE);
    ledcWrite(pwmChannel4, MAX_DUTY_CYCLE);
    
    vTaskDelay(1/portTICK_PERIOD_MS);                                                //Wait 1000us.
    
    ledcWrite(pwmChannel1, HALF_DUTY_CYCLE);
    ledcWrite(pwmChannel2, HALF_DUTY_CYCLE);
    ledcWrite(pwmChannel3, HALF_DUTY_CYCLE);
    ledcWrite(pwmChannel4, HALF_DUTY_CYCLE);                                         //Set digital port 4, 5, 6 and 7 low.
    vTaskDelay(3/portTICK_PERIOD_MS);                                                //Wait 3000us.
  }
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  gyroAxisCalibration[1] /= 2000;                                                 //Divide the roll total by 2000.
  gyroAxisCalibration[2] /= 2000;                                                 //Divide the pitch total by 2000.
  gyroAxisCalibration[3] /= 2000;                                                 //Divide the yaw total by 2000.
  if(DEBUG) Serial.println("calibrateGyroscope: OK \n calibrating...");
}

void calculateAnglePRY(){
  // first of all, read the gyro!
  readGyroscopeStatus();
  
  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyroRollInput = (gyroRollInput * 0.7) + ((gyroAxis[1] / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyroPitchInput = (gyroPitchInput * 0.7) + ((gyroAxis[2] / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyroYawInput = (gyroYawInput * 0.7) + ((gyroAxis[3] / 65.5) * 0.3);      //Gyro pid input is deg/sec.

  //Gyro angle calculations
  anglePitch += gyroAxis[2] * travelCoeff;                 //Calculate the traveled pitch angle and add this to the anglePitch variable.
  angleRoll += gyroAxis[1] * travelCoeff;                  //Calculate the traveled roll angle and add this to the angleRoll variable. 
  
  //The Arduino sin function is in radians
  anglePitch -= angleRoll * sin(gyroAxis[3] * travelCoeffToRad); //If the IMU has yawed transfer the roll angle to the pitch angel.
  angleRoll += anglePitch * sin(gyroAxis[3] * travelCoeffToRad); //If the IMU has yawed transfer the pitch angle to the roll angel.

  //Accelerometer angle calculations
  accTotalVector = sqrt((accAxis[2]*accAxis[2])+(accAxis[1]*accAxis[1])+(accAxis[3]*accAxis[3]));  //Calculate the total accelerometer vector.

  //The Arduino asin function is in radians
  if(abs(accAxis[1]) < accTotalVector){                                        //Prevent the asin function to produce a NaN
    anglePitchAcc = asin((float)accAxis[1]/accTotalVector)* convDegToRad; //Calculate the pitch angle.
  }
  if(abs(accAxis[2]) < accTotalVector){                                        //Prevent the asin function to produce a NaN
    angleRollAcc = asin((float)accAxis[2]/accTotalVector)* (-convDegToRad); //Calculate the roll angle.
  }

  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
  anglePitchAcc -= 0.0;                                                   //Accelerometer calibration value for pitch.
  angleRollAcc -= 0.0;                                                    //Accelerometer calibration value for roll.

  anglePitch = anglePitch * 0.9996 + anglePitchAcc * 0.0004;   //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angleRoll = angleRoll * 0.9996 + angleRollAcc * 0.0004;      //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitchLevelAdjust = anglePitch * correctionPitchRoll;                    //Calculate the pitch angle correction
  rollLevelAdjust = angleRoll * correctionPitchRoll;                      //Calculate the roll angle correction

  if(!AUTO_LEVELING){                                                    //If the quadcopter is not in auto-level mode
    pitchLevelAdjust = 0;                                                 //Set the pitch angle correction to zero.
    rollLevelAdjust = 0;                                                  //Set the roll angle correcion to zero.
  }

  //if(DEBUG) printGyroscopeStatus();                //print gyro status
}

void printGyroscopeStatus(){
  Serial.print("Pitch: ");
  Serial.print(anglePitch ,0);
  Serial.print(" Roll: ");
  Serial.print(angleRoll ,0);
  Serial.print(" Yaw: ");
  Serial.println(gyroAxis[3] / 65.5 ,0);
}

void readGyroscopeStatus(){
  
  Wire.beginTransmission(GYRO_ADDRESS) ;
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADDRESS, 14);  // request a total of 14 registers
  accAxis[1]  = Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  accAxis[2]  = Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accAxis[3]  = Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  gyroTemp    = Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyroAxis[1] = Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyroAxis[2] = Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyroAxis[3] = Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  if(calInt >= 2000){
    gyroAxis[1] -= gyroAxisCalibration[1];                            //Only compensate after the calibration.
    gyroAxis[2] -= gyroAxisCalibration[2];                            //Only compensate after the calibration.
    gyroAxis[3] -= gyroAxisCalibration[3];                            //Only compensate after the calibration.
  }

  if(eepromData[28] & 0b10000000)gyroAxis[1] *= -1;               //Invert gyroAxis[1] if the MSB of EEPROM bit 28 is set.
  if(eepromData[29] & 0b10000000)gyroAxis[2] *= -1;              //Invert gyroAxis[2] if the MSB of EEPROM bit 29 is set.
  if(eepromData[30] & 0b10000000)gyroAxis[3] *= -1;                //Invert gyroAxis[3] if the MSB of EEPROM bit 30 is set.
  if(eepromData[28] & 0b10000000)accAxis[1] *= -1;                   //Invert accAxis[1] if the MSB of EEPROM bit 28 is set.
  if(eepromData[29] & 0b10000000)accAxis[2] *= -1;                   //Invert accAxis[2] if the MSB of EEPROM bit 29 is set.
  if(eepromData[30] & 0b10000000)accAxis[3] *= -1;                   //Invert accAxis[3] if the MSB of EEPROM bit 30 is set.
}
