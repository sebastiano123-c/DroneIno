#define GYRO_CONFIG             0x1B
#define ACCEL_CONFIG            0x1C

#define ACCEL_XOUT_H            0x3B
#define ACCEL_YOUT_H            0x3D
#define ACCEL_ZOUT_H            0x3F
#define GYRO_XOUT_H             0x43
#define GYRO_YOUT_H             0x45
#define GYRO_ZOUT_H             0x47

#define PWR_MGMT_1              0x6B
#define PWR_MGMT_2              0x6C

void setupMPU(){
  Wire.setClock(WIRE_CLOCK);
  Wire.begin();
  Wire.beginTransmission(gyroAddress);                        //Start communication with the MPU-6050.
  int error = Wire.endTransmission();                              //End the transmission and register the exit status.
  while (error != 0) {                                          //Stay in this loop because the MPU-6050 did not respond.
    error = 1;                                                  //Set the error status to 1.
    digitalWrite(PIN_BATTERY_LED, !digitalRead(PIN_BATTERY_LED));
    delay(40);                                                   //Simulate a 250Hz refresc rate as like the main loop.
    Serial.print("MPU6050 ERROR at address: ");
    Serial.println(gyroAddress);
  }
  Serial.println("Gyroscope setup: OK");
}

void setGyroscopeRegisters(){
    //Setup the MPU-6050
  if(eepromData[31] == 1){
    Wire.beginTransmission(gyroAddress);                        //Start communication with the address found during search.
    Wire.write(0x6B);                                            //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                            //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                      //End the transmission with the gyro.

    Wire.beginTransmission(gyroAddress);                        //Start communication with the address found during search.
    Wire.write(0x1B);                                            //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                      //End the transmission with the gyro

    Wire.beginTransmission(gyroAddress);                        //Start communication with the address found during search.
    Wire.write(0x1C);                                            //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(0x10);                                            //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();                                      //End the transmission with the gyro

    //Let's perform a random register check to see if the values are written correct
    Wire.beginTransmission(gyroAddress);                        //Start communication with the address found during search
    Wire.write(0x1B);                                            //Start reading @ register 0x1B
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(gyroAddress, 1);                           //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                 //Wait until the 6 bytes are received
    if(Wire.read() != 0x08){                                     //Check if the value is 0x08
      digitalWrite(PIN_BATTERY_LED, HIGH);                                     //Turn on the warning led
      while(1)vTaskDelay(10/portTICK_PERIOD_MS);                                         //Stay in this loop for ever
    }

    Wire.beginTransmission(gyroAddress);                        //Start communication with the address found during search
    Wire.write(0x1A);                                            //We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);                                            //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();                                      //End the transmission with the gyro    

  }  
}

void calibrateGyroscope(){
  Serial.print("Calibrating the gyro");
  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  for (calInt = 0; calInt < 2000 ; calInt ++){                                   //Take 2000 readings for calibration.
    if(calInt % 125 == 0){
      digitalWrite(PIN_BATTERY_LED, !digitalRead(PIN_BATTERY_LED));   //Change the led status to indicate calibration.
      Serial.print(".");
    }
    
    readGyroscopeStatus();                                                                //Read the gyro output.
    
    gyroAxisCalibration[1] += gyroAxis[1];                                               //Ad roll value to gyroAxis[1]Cal.
    gyroAxisCalibration[2] += gyroAxis[2];                                               //Ad pitch value to gyroAxis[2]Cal.
    gyroAxisCalibration[3] += gyroAxis[3];                                               //Ad yaw value to gyroAxis[3]Cal.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
    //Set digital port 4, 5, 6 and 7 high.
    digitalWrite(PIN_ESC_1, HIGH);
    digitalWrite(PIN_ESC_2, HIGH);
    digitalWrite(PIN_ESC_3, HIGH);
    digitalWrite(PIN_ESC_4, HIGH);
    
    vTaskDelay(1/portTICK_PERIOD_MS);                                                        //Wait 1000us.
    
    //Set digital port 4, 5, 6 and 7 low.
    digitalWrite(PIN_ESC_1, LOW);
    digitalWrite(PIN_ESC_2, LOW);
    digitalWrite(PIN_ESC_3, LOW);
    digitalWrite(PIN_ESC_4, LOW);
    vTaskDelay(3/portTICK_PERIOD_MS);                                                                       //Wait 3 milliseconds before the next loop.
  }
  Serial.println(".");
  //Now that we have 2000 measures, we need to divide by 2000 to get the average gyro offset.
  gyroAxisCalibration[1] /= 2000;                                                         //Divide the roll total by 2000.
  gyroAxisCalibration[2] /= 2000;                                                         //Divide the pitch total by 2000.
  gyroAxisCalibration[3] /= 2000;                                                         //Divide the yaw total by 2000.
  Serial.println("");
  Serial.println(gyroAxisCalibration[1]);
  Serial.println(gyroAxisCalibration[2]);
  Serial.println(gyroAxisCalibration[3]);
  vTaskDelay(1000/portTICK_PERIOD_MS); 
}

void calculateAnglePRY(){
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

  pitchLevelAdjust = anglePitch * 15;                                    //Calculate the pitch angle correction
  rollLevelAdjust = angleRoll * 15;                                      //Calculate the roll angle correction

  if(!AUTO_LEVELING){                                                          //If the quadcopter is not in auto-level mode
    pitchLevelAdjust = 0;                                                 //Set the pitch angle correction to zero.
    rollLevelAdjust = 0;                                                  //Set the roll angle correcion to zero.
  }
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
  
  getAcc();
  getGyro();

  receiverInputChannel1 = convertReceiverChannel(1);           //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
  receiverInputChannel2 = convertReceiverChannel(2);           //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
  receiverInputChannel3 = convertReceiverChannel(3);           //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
  receiverInputChannel4 = convertReceiverChannel(4);           //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.

  accAxis[1] = rawAX[1] | rawAX[0]<<8;
  accAxis[2] = rawAY[1] | rawAY[0]<<8;
  accAxis[3] = rawAZ[1] | rawAZ[0]<<8;
  gyroAxis[1] = rawGX[1] | rawGX[0]<<8;
  gyroAxis[2] = rawGY[1] | rawGY[0]<<8;
  gyroAxis[3] = rawGZ[1] | rawGZ[0]<<8;

  // gyroAxis[1] = gyroAxis[1]/*/gyroCorrection*/;           //Set gyroAxis[1] to the correct axis that was stored in the EEPROM.
  // gyroAxis[2] = gyroAxis[2]/*/gyroCorrection*/;          //Set gyroAxis[2] to the correct axis that was stored in the EEPROM.
  // gyroAxis[3] = gyroAxis[3]/*/gyroCorrection*/;            //Set gyroAxis[3] to the correct axis that was stored in the EEPROM.
  // accAxis[2] = accAxis[2]/*/accCorrection*/ ;                //Set accAxis[2] to the correct axis that was stored in the EEPROM.
  // accAxis[1] = accAxis[1]/*/accCorrection*/ ;                //Set accAxis[1] to the correct axis that was stored in the EEPROM.
  // accAxis[3] = accAxis[3]/*/accCorrection*/ ;                //Set accAxis[3] to the correct axis that was stored in the EEPROM.

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

void getAcc(){
  Wire.beginTransmission(gyroAddress);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission();
  
  Wire.requestFrom(gyroAddress,2,1);  
  for(int i = 0; i<2 ; i++)
  {
    rawAX[i] = Wire.read();
  }

  //==================================================
  Wire.beginTransmission(gyroAddress);
  Wire.write(ACCEL_YOUT_H);
  Wire.endTransmission();
  
  Wire.requestFrom(gyroAddress,2,1);
  for(int i = 0; i<2 ; i++)
  {
    rawAY[i] = Wire.read();
  }

  //==================================================
  Wire.beginTransmission(gyroAddress);
  Wire.write(ACCEL_ZOUT_H);
  Wire.endTransmission();
  
  Wire.requestFrom(gyroAddress,2,1);
  for(int i = 0; i<2 ; i++)
  {
    rawAZ[i] = Wire.read();
  }
}

void getGyro(){
  Wire.beginTransmission(gyroAddress);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission();
  
  Wire.requestFrom(gyroAddress,2,1);
  for(int i = 0; i<2 ; i++)
  {
    rawGX[i] = Wire.read();
  }
  
  //==================================================
  Wire.beginTransmission(gyroAddress);
  Wire.write(GYRO_YOUT_H);
  Wire.endTransmission();
  
  Wire.requestFrom(gyroAddress,2,1);
  for(int i = 0; i<2 ; i++)
  {
    rawGY[i] = Wire.read();
  }

  //==================================================
  Wire.beginTransmission(gyroAddress);
  Wire.write(GYRO_ZOUT_H);
  Wire.endTransmission();
  
  Wire.requestFrom(gyroAddress,2,1);
  for(int i = 0; i<2 ; i++)
  {
    rawGZ[i] = Wire.read();
  }
}