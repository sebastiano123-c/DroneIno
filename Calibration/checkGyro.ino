/**
 * @file checkGyro.ino
 * @author @sebastiano123-c
 * @brief Gyroscope routines.
 * @version 0.1
 * @date 2022-03-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */

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

#define BOUND_POSITION          4096//4096

byte rawAX[2], rawAY[2], rawAZ[2];
byte rawGX[2], rawGY[2], rawGZ[2];

/**
 * @brief Calculate the pitch and roll angle.
 * 
 * @param PRY 
 * @param XYZ 
 */
void ymfcFunction(bool PRY = true, bool XYZ = false){
  //Gyro angle calculations
  anglePitch += gyroAxis[2] * travelCoeff;                                            //Calculate the traveled pitch angle and add this to the anglePitch variable.
  angleRoll += gyroAxis[1] * travelCoeff;                                              //Calculate the traveled roll angle and add this to the angleRoll variable.

  //The Arduino sin function is in radians
  anglePitch -= angleRoll * sin(gyroAxis[3] * travelCoeffToRad);                        //If the IMU has yawed transfer the roll angle to the pitch angel.
  angleRoll += anglePitch * sin(gyroAxis[3] * travelCoeffToRad);                        //If the IMU has yawed transfer the pitch angle to the roll angel.

  //Accelerometer angle calculations
  accTotalVector[0] = sqrt((accAxis[2]*accAxis[2])+(accAxis[1]*accAxis[1])+(accAxis[3]*accAxis[3]));                    //Calculate the total accelerometer vector.

  //The Arduino asin function is in radians
  float pitchArg = (float)accAxis[1]/accTotalVector[0];
  float rollArg = (float)accAxis[2]/accTotalVector[0];
    anglePitchAcc = asin(pitchArg)* convDegToRad;                //Calculate the pitch angle.
    angleRollAcc = asin(rollArg)*(-convDegToRad);               //Calculate the roll angle.
  
  if(!firstAngle){
    anglePitch = anglePitchAcc;                                                     //Set the pitch angle to the accelerometer angle.
    angleRoll = angleRollAcc;                                                       //Set the roll angle to the accelerometer angle.
    firstAngle = true;
  }
  else{
    anglePitch = anglePitch * 0.9996 + anglePitchAcc * 0.0004;                      //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
    angleRoll = angleRoll * 0.9996 + angleRollAcc * 0.0004;                         //Correct the drift of the gyro roll angle with the accelerometer roll angle.
  }

  //We can't print all the data at once. This takes to long and the angular readings will be off.
  if(PRY == true){
    Serial.print("Pitch: ");
    Serial.print(anglePitch ,0);
    Serial.print(" Roll: ");
    Serial.print(angleRoll ,0);
    Serial.print(" Yaw: ");
    Serial.println(gyroAxis[3] / 65.5 ,0);
  }

  if(XYZ == true){
    Serial.print("X: ");
    Serial.print(accAxis[2] ,0);
    Serial.print(" Y: ");
    Serial.print(accAxis[1] ,0);
    Serial.print(" Z: ");
    Serial.println(accAxis[3] ,0);
  }
  
  loopCounter ++;
  if(loopCounter == 60)loopCounter = 0;  
}

/**
 * @brief Main routine.
 * 
 */
void checkGyro(){
  if(calInt != 2000){
      calibrateGyro();
    }
    else{
      ///We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
      //Set digital port 4, 5, 6 and 7 high.
      digitalWrite(PIN_ESC_1, HIGH);
      digitalWrite(PIN_ESC_2, HIGH);
      digitalWrite(PIN_ESC_3, HIGH);
      digitalWrite(PIN_ESC_4, HIGH);
      
      vTaskDelay(1 / portTICK_PERIOD_MS);                                                        //Wait 1000us.
      
      //Set digital port 4, 5, 6 and 7 low.
      digitalWrite(PIN_ESC_1, LOW);
      digitalWrite(PIN_ESC_2, LOW);
      digitalWrite(PIN_ESC_3, LOW);
      digitalWrite(PIN_ESC_4, LOW);
      vTaskDelay(3/portTICK_PERIOD_MS);                                                                       //Wait 3 milliseconds before the next loop.

      //Let's get the current gyro data.
      getGyroSignal();

      ymfcFunction();
  }
}

void calibrateGyro(){
  Serial.print("Calibrating the gyro");
  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  for (calInt = 0; calInt < 2000 ; calInt ++){                                   //Take 2000 readings for calibration.
    if(calInt % 125 == 0){
      digitalWrite(PIN_BATTERY_LED, !digitalRead(PIN_BATTERY_LED));   //Change the led status to indicate calibration.
      Serial.print(".");
    }
    
    getGyroSignal();                                                                //Read the gyro output.
    
    gyroAxisCal[1] += gyroAxis[1];                                               //Ad roll value to gyroAxis[1]Cal.
    gyroAxisCal[2] += gyroAxis[2];                                               //Ad pitch value to gyroAxis[2]Cal.
    gyroAxisCal[3] += gyroAxis[3];                                               //Ad yaw value to gyroAxis[3]Cal.
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
  gyroAxisCal[1] /= 2000;                                                         //Divide the roll total by 2000.
  gyroAxisCal[2] /= 2000;                                                         //Divide the pitch total by 2000.
  gyroAxisCal[3] /= 2000;                                                         //Divide the yaw total by 2000.
  Serial.println("");
  Serial.println(gyroAxisCal[1]);
  Serial.println(gyroAxisCal[2]);
  Serial.println(gyroAxisCal[3]);
  vTaskDelay(1000/portTICK_PERIOD_MS); 
}

void getGyroSignal(){
  getAcc();
  getGyro();

  gyroAxis[1] = gyroAxis[1]/gyroCorrection;           //Set gyroAxis[1] to the correct axis that was stored in the EEPROM.
  gyroAxis[2] = gyroAxis[2]/gyroCorrection;          //Set gyroAxis[2] to the correct axis that was stored in the EEPROM.
  gyroAxis[3] = gyroAxis[3]/gyroCorrection;            //Set gyroAxis[3] to the correct axis that was stored in the EEPROM.
  accAxis[2] = accAxis[2]/accCorrection ;                //Set accAxis[2] to the correct axis that was stored in the EEPROM.
  accAxis[1] = accAxis[1]/accCorrection ;                //Set accAxis[1] to the correct axis that was stored in the EEPROM.
  accAxis[3] = accAxis[3]/accCorrection ;                //Set accAxis[3] to the correct axis that was stored in the EEPROM.

  if(calInt >= 2000){
    gyroAxis[1] -= gyroAxisCal[1];                            //Only compensate after the calibration.
    gyroAxis[2] -= gyroAxisCal[2];                            //Only compensate after the calibration.
    gyroAxis[3] -= gyroAxisCal[3];                            //Only compensate after the calibration.
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
  accAxis[1] = rawAX[1] | rawAX[0]<<8;

  //==================================================
  Wire.beginTransmission(gyroAddress);
  Wire.write(ACCEL_YOUT_H);
  Wire.endTransmission();
  
  Wire.requestFrom(gyroAddress,2,1);
  for(int i = 0; i<2 ; i++)
  {
    rawAY[i] = Wire.read();
  }
  accAxis[2] = rawAY[1] | rawAY[0]<<8;
  

  //==================================================
  Wire.beginTransmission(gyroAddress);
  Wire.write(ACCEL_ZOUT_H);
  Wire.endTransmission();
  
  Wire.requestFrom(gyroAddress,2,1);
  for(int i = 0; i<2 ; i++)
  {
    rawAZ[i] = Wire.read();
  }
  accAxis[3] = rawAZ[1] | rawAZ[0]<<8;
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
  gyroAxis[1] = rawGX[1] | rawGX[0]<<8;

  //==================================================
  Wire.beginTransmission(gyroAddress);
  Wire.write(GYRO_YOUT_H);
  Wire.endTransmission();
  
  Wire.requestFrom(gyroAddress,2,1);
  for(int i = 0; i<2 ; i++)
  {
    rawGY[i] = Wire.read();
  }
  gyroAxis[2] = rawGY[1] | rawGY[0]<<8;


  //==================================================
  Wire.beginTransmission(gyroAddress);
  Wire.write(GYRO_ZOUT_H);
  Wire.endTransmission();
  
  Wire.requestFrom(gyroAddress,2,1);
  for(int i = 0; i<2 ; i++)
  {
    rawGZ[i] = Wire.read();
  }
  gyroAxis[3] = rawGZ[1] | rawGZ[0]<<8;
}


void setupMPU(){
  Wire.setClock(WIRE_CLOCK);
  Wire.begin();
  Wire.beginTransmission(gyroAddress);                        //Start communication with the MPU-6050.
  int error = Wire.endTransmission();                              //End the transmission and register the exit status.
  while (error != 0) {                                          //Stay in this loop because the MPU-6050 did not respond.
    error = 1;                                                  //Set the error status to 1.
    digitalWrite(PIN_BATTERY_LED, !digitalRead(PIN_BATTERY_LED));
    delay(40);                                                   //Simulate a 250Hz refresc rate as like the main loop.
    Serial.println("MPU6050 ERROR.");
  }
}

void setGyroRegisters(){
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
