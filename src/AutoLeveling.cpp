// Autoleveling
// @author: Sebastiano Cocchi
// #include "Header.h"
// // #include "src/Models.h"

// #include <Arduino.h>
// #include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
// #include <EEPROM.h>                        //Include the EEPROM.h library so we can store information onto the EEPROM
#include "Autoleveling.h"

void AutoLeveling::begin(){
//Copy the EEPROM data for fast access data.
  for(start = 0; start <= 35; start++)eepromData[start] = EEPROM.read(start);
  start = 0;                                                            //Set start back to zero.
  gyroAddress = eepromData[32];                                           //Store the gyro address in the variable.
}

void AutoLeveling::readGyroscopeStatus(){
  //Read the MPU-6050
  if(eepromData[31] == 1){
    Wire.beginTransmission(gyroAddress);                                   //Start communication with the gyro.
    Wire.write(0x3B);                                                       //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                                 //End the transmission.
    Wire.requestFrom(gyroAddress,14);                                      //Request 14 bytes from the gyro.
    
    receiverInputChannel1 = convertReceiverChannel(1);                 //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
    receiverInputChannel2 = convertReceiverChannel(2);                 //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
    receiverInputChannel3 = convertReceiverChannel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
    receiverInputChannel4 = convertReceiverChannel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.
    
    while(Wire.available() < 14);                                           //Wait until the 14 bytes are received.
    accAxis[1] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the accX variable.
    accAxis[2] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the accY variable.
    accAxis[3] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the accZ variable.
    temperature = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the temperature variable.
    gyroAxis[1] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyroAxis[2] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyroAxis[3] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
  }

  if(calInt == 2000){
    gyroAxis[1] -= gyroAxisCalibration[1];                                       //Only compensate after the calibration.
    gyroAxis[2] -= gyroAxisCalibration[2];                                       //Only compensate after the calibration.
    gyroAxis[3] -= gyroAxisCalibration[3];                                       //Only compensate after the calibration.
  }
  gyroRoll = gyroAxis[eepromData[28] & 0b00000011];                      //Set gyroRoll to the correct axis that was stored in the EEPROM.
  if(eepromData[28] & 0b10000000)gyroRoll *= -1;                          //Invert gyroRoll if the MSB of EEPROM bit 28 is set.
  gyroPitch = gyroAxis[eepromData[29] & 0b00000011];                     //Set gyroPitch to the correct axis that was stored in the EEPROM.
  if(eepromData[29] & 0b10000000)gyroPitch *= -1;                         //Invert gyroPitch if the MSB of EEPROM bit 29 is set.
  gyroYaw = gyroAxis[eepromData[30] & 0b00000011];                       //Set gyroYaw to the correct axis that was stored in the EEPROM.
  if(eepromData[30] & 0b10000000)gyroYaw *= -1;                           //Invert gyroYaw if the MSB of EEPROM bit 30 is set.

  accX = accAxis[eepromData[29] & 0b00000011];                           //Set accX to the correct axis that was stored in the EEPROM.
  if(eepromData[29] & 0b10000000)accX *= -1;                              //Invert accX if the MSB of EEPROM bit 29 is set.
  accY = accAxis[eepromData[28] & 0b00000011];                           //Set accY to the correct axis that was stored in the EEPROM.
  if(eepromData[28] & 0b10000000)accY *= -1;                              //Invert accY if the MSB of EEPROM bit 28 is set.
  accZ = accAxis[eepromData[30] & 0b00000011];                           //Set accZ to the correct axis that was stored in the EEPROM.
  if(eepromData[30] & 0b10000000)accZ *= -1;                              //Invert accZ if the MSB of EEPROM bit 30 is set.
}

void AutoLeveling::getBatteryVoltage(){
    //65 is the voltage compensation for the diode.
    //12.6V equals ~5V @ Analog 0.
    //12.6V equals 1023 analogRead(0).
    //1260 / 1023 = 1.2317.
    //The variable batteryVoltage holds 1050 if the battery voltage is 10.5V.
  batteryVoltage = (analogRead(0) + 65) * 1.2317;
}

void AutoLeveling::getLoopTimer(){
  loopTimer = (analogRead(0) + 65) * 1.2317;
}

int AutoLeveling::convertReceiverChannel(byte function){
  byte channel, reverse;                                                       //First we declare some local variables
  int low, center, high, actual;
  int difference;

  channel = eepromData[function + 23] & 0b00000111;                           //What channel corresponds with the specific function
  if(eepromData[function + 23] & 0b10000000)reverse = 1;                      //Reverse channel when most significant bit is set
  else reverse = 0;                                                            //If the most significant is not set there is no reverse

  actual = receiverInput[channel];                                            //Read the actual receiver value for the corresponding function
  low = (eepromData[channel * 2 + 15] << 8) | eepromData[channel * 2 + 14];  //Store the low value for the specific receiver input channel
  center = (eepromData[channel * 2 - 1] << 8) | eepromData[channel * 2 - 2]; //Store the center value for the specific receiver input channel
  high = (eepromData[channel * 2 + 7] << 8) | eepromData[channel * 2 + 6];   //Store the high value for the specific receiver input channel

  if(actual < center){                                                         //The actual receiver value is lower than the center value
    if(actual < low)actual = low;                                              //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center - actual) * (long)500) / (center - low);       //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 + difference;                                  //If the channel is reversed
    else return 1500 - difference;                                             //If the channel is not reversed
  }
  else if(actual > center){                                                                        //The actual receiver value is higher than the center value
    if(actual > high)actual = high;                                            //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center) * (long)500) / (high - center);      //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 - difference;                                  //If the channel is reversed
    else return 1500 + difference;                                             //If the channel is not reversed
  }
  else return 1500;
}

void AutoLeveling::setGyroscopeRegisters(){
  //Setup the MPU-6050
  if(eepromData[31] == 1){
    Wire.beginTransmission(gyroAddress);                                      //Start communication with the address found during search.
    Wire.write(0x6B);                                                          //We want to write to the PWR_MGMT1 register (6B hex)
    Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                                    //End the transmission with the gyro.

    Wire.beginTransmission(gyroAddress);                                      //Start communication with the address found during search.
    Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    Wire.beginTransmission(gyroAddress);                                      //Start communication with the address found during search.
    Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    //Let's perform a random register check to see if the values are written correct
    Wire.beginTransmission(gyroAddress);                                      //Start communication with the address found during search
    Wire.write(0x1B);                                                          //Start reading @ register 0x1B
    Wire.endTransmission();                                                    //End the transmission
    Wire.requestFrom(gyroAddress, 1);                                         //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                               //Wait until the 6 bytes are received
    if(Wire.read() != 0x08){                                                   //Check if the value is 0x08
      digitalWrite(PIN_BATTERY_LED, HIGH);                                       //Turn on the warning led
      while(1)delay(10);                                                       //Stay in this loop for ever
    }

    Wire.beginTransmission(gyroAddress);                                      //Start communication with the address found during search
    Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();                                                    //End the transmission with the gyro    

  }  
}

void AutoLeveling::calibrateGyroscope(){
  for (calInt = 0; calInt < 1250 ; calInt ++){                           //Wait 5 seconds before continuing.
    PORTD |= B11110000;                                                     //Set digital port 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital port 4, 5, 6 and 7 low.
    delayMicroseconds(3000);                                                //Wait 3000us.
  }

  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  for (calInt = 0; calInt < 2000 ; calInt ++){                           //Take 2000 readings for calibration.
    if(calInt % 15 == 0)digitalWrite(PIN_BATTERY_LED, !digitalRead(12));                //Change the led status to indicate calibration.
    this->readGyroscopeStatus();                                                        //Read the gyro output.
    gyroAxisCalibration[1] += gyroAxis[1];                                       //Ad roll value to gyroRollCal.
    gyroAxisCalibration[2] += gyroAxis[2];                                       //Ad pitch value to gyroPitchCal.
    gyroAxisCalibration[3] += gyroAxis[3];                                       //Ad yaw value to gyroYawCal.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
    PORTD |= B11110000;                                                     //Set digital port 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital port 4, 5, 6 and 7 low.
    delay(3);                                                               //Wait 3 milliseconds before the next loop.
  }
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  gyroAxisCalibration[1] /= 2000;                                                 //Divide the roll total by 2000.
  gyroAxisCalibration[2] /= 2000;                                                 //Divide the pitch total by 2000.
  gyroAxisCalibration[3] /= 2000;                                                 //Divide the yaw total by 2000.
}

void AutoLeveling::waitController(){
      //Wait until the receiver is active and the throtle is set to the lower position.
  while(receiverInputChannel3 < 990 || receiverInputChannel3 > 1020 || receiverInputChannel4 < 1400){
    receiverInputChannel3 = convertReceiverChannel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
    receiverInputChannel4 = convertReceiverChannel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
    start ++;                                                               //While waiting increment start whith every loop.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
    PORTD |= B11110000;                                                     //Set digital port 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital port 4, 5, 6 and 7 low.
    delay(3);                                                               //Wait 3 milliseconds before the next loop.
    if(start == 125){                                                       //Every 125 loops (500ms).
      digitalWrite(PIN_BATTERY_LED, !digitalRead(12));                                   //Change the led status.
      start = 0;                                                            //Start again at 0.
    }
  }
  start = 0;                                                                //Set start back to 0.
}

void AutoLeveling::setAutoLevelParameters(){
  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyroRollInput = (gyroRollInput * 0.7) + ((gyroRoll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyroPitchInput = (gyroPitchInput * 0.7) + ((gyroPitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyroYawInput = (gyroYawInput * 0.7) + ((gyroYaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //This is the added IMU code from the videos:
  //https://youtu.be/4BoIE8YQwM8
  //https://youtu.be/j-kE0AMEWy4
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  anglePitch += gyroPitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the anglePitch variable.
  angleRoll += gyroRoll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angleRoll variable.

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  anglePitch -= angleRoll * sin(gyroYaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angleRoll += anglePitch * sin(gyroYaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

  //Accelerometer angle calculations
  accTotalVector = sqrt((accX*accX)+(accY*accY)+(accZ*accZ));       //Calculate the total accelerometer vector.

  if(abs(accY) < accTotalVector){                                        //Prevent the asin function to produce a NaN
    anglePitchAcc = asin((float)accY/accTotalVector)* 57.296;          //Calculate the pitch angle.
  }
  if(abs(accX) < accTotalVector){                                        //Prevent the asin function to produce a NaN
    angleRollAcc = asin((float)accX/accTotalVector)* -57.296;          //Calculate the roll angle.
  }

  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
  anglePitchAcc -= 0.0;                                                   //Accelerometer calibration value for pitch.
  angleRollAcc -= 0.0;                                                    //Accelerometer calibration value for roll.

  anglePitch = anglePitch * 0.9996 + anglePitchAcc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angleRoll = angleRoll * 0.9996 + angleRollAcc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitchLevelAdjust = anglePitch * 15;                                    //Calculate the pitch angle correction
  rollLevelAdjust = angleRoll * 15;                                      //Calculate the roll angle correction

  if(!autoLevel){                                                          //If the quadcopter is not in auto-level mode
    pitchLevelAdjust = 0;                                                 //Set the pitch angle correction to zero.
    rollLevelAdjust = 0;                                                  //Set the roll angle correcion to zero.
  }
}

void AutoLeveling::startAutoLeveling(){
  start = 2;

  anglePitch = anglePitchAcc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
  angleRoll = angleRollAcc;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
  gyroAnglesSet = true;                                                 //Set the IMU started flag.

  //Reset the PID controllers for a bumpless start.
  pidIMemRoll = 0;
  pidLastRollDError = 0;
  pidIMemPitch = 0;
  pidLastPitchDError = 0;
  pidIMemYaw = 0;
  pidLastYawDError = 0;
}

void AutoLeveling::setPID(){
  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pidRollSetpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiverInputChannel1 > 1508)pidRollSetpoint = receiverInputChannel1 - 1508;
  else if(receiverInputChannel1 < 1492)pidRollSetpoint = receiverInputChannel1 - 1492;

  pidRollSetpoint -= rollLevelAdjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
  pidRollSetpoint /= 3.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pidPitchSetpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiverInputChannel2 > 1508)pidPitchSetpoint = receiverInputChannel2 - 1508;
  else if(receiverInputChannel2 < 1492)pidPitchSetpoint = receiverInputChannel2 - 1492;

  pidPitchSetpoint -= pitchLevelAdjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
  pidPitchSetpoint /= 3.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pidYawSetpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiverInputChannel3 > 1050){ //Do not yaw when turning off the motors.
    if(receiverInputChannel4 > 1508)pidYawSetpoint = (receiverInputChannel4 - 1508)/3.0;
    else if(receiverInputChannel4 < 1492)pidYawSetpoint = (receiverInputChannel4 - 1492)/3.0;
  }
}

void AutoLeveling::calculatePID(){
  //Roll calculations
  pidErrorTemp = gyroRollInput - pidRollSetpoint;
  pidIMemRoll += PID_I_GAIN_ROLL * pidErrorTemp;
  if(pidIMemRoll > PID_MAX_ROLL)pidIMemRoll = PID_MAX_ROLL;
  else if(pidIMemRoll < PID_MAX_ROLL * -1)pidIMemRoll = PID_MAX_ROLL * -1;

  pidOutputRoll = PID_P_GAIN_ROLL * pidErrorTemp + pidIMemRoll + PID_D_GAIN_ROLL * (pidErrorTemp - pidLastRollDError);
  if(pidOutputRoll > PID_MAX_ROLL)pidOutputRoll = PID_MAX_ROLL;
  else if(pidOutputRoll < PID_MAX_ROLL * -1)pidOutputRoll = PID_MAX_ROLL * -1;

  pidLastRollDError = pidErrorTemp;

  //Pitch calculations
  pidErrorTemp = gyroPitchInput - pidPitchSetpoint;
  pidIMemPitch += PID_I_GAIN_PITCH * pidErrorTemp;
  if(pidIMemPitch > PID_MAX_PITCH)pidIMemPitch = PID_MAX_PITCH;
  else if(pidIMemPitch < PID_MAX_PITCH * -1)pidIMemPitch = PID_MAX_PITCH * -1;

  pidOutputPitch = PID_P_GAIN_PITCH * pidErrorTemp + pidIMemPitch + PID_D_GAIN_PITCH * (pidErrorTemp - pidLastPitchDError);
  if(pidOutputPitch > PID_MAX_PITCH)pidOutputPitch = PID_MAX_PITCH;
  else if(pidOutputPitch < PID_MAX_PITCH * -1)pidOutputPitch = PID_MAX_PITCH * -1;

  pidLastPitchDError = pidErrorTemp;

  //Yaw calculations
  pidErrorTemp = gyroYawInput - pidYawSetpoint;
  pidIMemYaw += PID_I_GAIN_YAW * pidErrorTemp;
  if(pidIMemYaw > PID_MAX_YAW)pidIMemYaw = PID_MAX_YAW;
  else if(pidIMemYaw < PID_MAX_YAW * -1)pidIMemYaw = PID_MAX_YAW * -1;

  pidOutputYaw = PID_P_GAIN_YAW * pidErrorTemp + pidIMemYaw + PID_D_GAIN_YAW * (pidErrorTemp - pidLastYawDError);
  if(pidOutputYaw > PID_MAX_YAW)pidOutputYaw = PID_MAX_YAW;
  else if(pidOutputYaw < PID_MAX_YAW * -1)pidOutputYaw = PID_MAX_YAW * -1;

  pidLastYawDError = pidErrorTemp;
}

void AutoLeveling::batteryVoltageCompensation(){
  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //0.09853 = 0.08 * 1.2317.
  batteryVoltage = batteryVoltage * 0.92 + (analogRead(0) + 65) * 0.09853;

  //Turn on the led if battery voltage is to low.
  if(batteryVoltage < 1000 && batteryVoltage > 600) digitalWrite(PIN_BATTERY_LED, HIGH);


  int throttle = receiverInputChannel3;                                      //We need the throttle signal as a base signal.

  if (start == 2){                                                          //The motors are started.
    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    esc1 = throttle - pidOutputPitch + pidOutputRoll - pidOutputYaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc2 = throttle + pidOutputPitch + pidOutputRoll + pidOutputYaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc3 = throttle + pidOutputPitch - pidOutputRoll - pidOutputYaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc4 = throttle - pidOutputPitch - pidOutputRoll + pidOutputYaw; //Calculate the pulse for esc 4 (front-left - CW)

    if (batteryVoltage < 1240 && batteryVoltage > 800){                   //Is the battery connected?
      esc1 += esc1 * ((1240 - batteryVoltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
      esc2 += esc2 * ((1240 - batteryVoltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
      esc3 += esc3 * ((1240 - batteryVoltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
      esc4 += esc4 * ((1240 - batteryVoltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
    } 

    if (esc1 < 1100) esc1 = 1100;                                         //Keep the motors running.
    if (esc2 < 1100) esc2 = 1100;                                         //Keep the motors running.
    if (esc3 < 1100) esc3 = 1100;                                         //Keep the motors running.
    if (esc4 < 1100) esc4 = 1100;                                         //Keep the motors running.

    if(esc1 > 2000)esc1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
    if(esc2 > 2000)esc2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
    if(esc3 > 2000)esc3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
    if(esc4 > 2000)esc4 = 2000;                                           //Limit the esc-4 pulse to 2000us.  
  }

  else{
    esc1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }
}

void AutoLeveling::setEscPulses(){
  if(micros() - loopTimer > 4050) digitalWrite(PIN_BATTERY_LED, HIGH);                   //Turn on the LED if the loop time exceeds 4050us.
  
  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while(micros() - loopTimer < 4000);                                      //We wait until 4000us are passed.
  loopTimer = micros();                                                    //Set the timer for the next loop.

  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timerChannel1 = esc1 + loopTimer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timerChannel2 = esc2 + loopTimer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timerChannel3 = esc3 + loopTimer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timerChannel4 = esc4 + loopTimer;                                     //Calculate the time of the faling edge of the esc-4 pulse.
  
  //There is always 1000us of spare time. So let's do something usefull that is very time consuming.
  //Get the current gyro and receiver data and scale it to degrees per second for the pid calculations.
  readGyroscopeStatus();

  while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    escLoopTimer = micros();                                              //Read the current time.
    if(timerChannel1 <= escLoopTimer)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(timerChannel2 <= escLoopTimer)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(timerChannel3 <= escLoopTimer)PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(timerChannel4 <= escLoopTimer)PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
  }
}