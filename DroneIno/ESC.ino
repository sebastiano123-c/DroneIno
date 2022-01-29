void droneStart(){
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

int convertReceiverChannel(byte function){
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

void setEscPulses(){
  throttle = receiverInputChannel3; //We need the throttle signal as a base signal.

  if (start == 2){                                                          //The motors are started.
    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    esc1 = throttle - pidOutputPitch + pidOutputRoll - pidOutputYaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc2 = throttle + pidOutputPitch + pidOutputRoll + pidOutputYaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc3 = throttle + pidOutputPitch - pidOutputRoll - pidOutputYaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc4 = throttle - pidOutputPitch - pidOutputRoll + pidOutputYaw; //Calculate the pulse for esc 4 (front-left - CW)

    if (batteryVoltage < 1240 && batteryVoltage > 800){                   //Is the battery connected?
      esc1 += esc1 * ((1240 - batteryVoltage)/(float)3500);//Compensate the esc-1 pulse for voltage drop.
      esc2 += esc2 * ((1240 - batteryVoltage)/(float)3500);//Compensate the esc-2 pulse for voltage drop.
      esc3 += esc3 * ((1240 - batteryVoltage)/(float)3500);//Compensate the esc-3 pulse for voltage drop.
      esc4 += esc4 * ((1240 - batteryVoltage)/(float)3500);//Compensate the esc-4 pulse for voltage drop.
    } 

    if (esc1 < 1100) esc1 = 1100;                          //Keep the motors running.
    if (esc2 < 1100) esc2 = 1100;                          //Keep the motors running.
    if (esc3 < 1100) esc3 = 1100;                          //Keep the motors running.
    if (esc4 < 1100) esc4 = 1100;                          //Keep the motors running.

    if(esc1 > 2000) esc1 = 2000;                 //Limit the esc-1 pulse to 2000us.
    if(esc2 > 2000) esc2 = 2000;                 //Limit the esc-2 pulse to 2000us.
    if(esc3 > 2000) esc3 = 2000;                 //Limit the esc-3 pulse to 2000us.
    if(esc4 > 2000) esc4 = 2000;                 //Limit the esc-4 pulse to 2000us.  
  }
  else{
    esc1 = 1000;                   //If start is not 2 keep a 1000us pulse for ess-1.
    esc2 = 1000;                   //If start is not 2 keep a 1000us pulse for ess-2.
    esc3 = 1000;                   //If start is not 2 keep a 1000us pulse for ess-3.
    esc4 = 1000;                   //If start is not 2 keep a 1000us pulse for ess-4.
  }

  // write to escs
  ledcWrite(pwmChannel1, esc1); ///2000*MAX_DUTY_CYCLE
  ledcWrite(pwmChannel2, esc2); ///2000*MAX_DUTY_CYCLE
  ledcWrite(pwmChannel3, esc3); ///2000*MAX_DUTY_CYCLE
  ledcWrite(pwmChannel4, esc4); ///2000*MAX_DUTY_CYCLE

  if(micros() - loopTimer > 4050) ledcWrite(pwmLedChannel, MAX_DUTY_CYCLE);                   //Turn on the LED if the loop time exceeds 4050us.
  
  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while(micros() - loopTimer < 4000) ledcWrite(pwmLedFlyChannel, MAX_DUTY_CYCLE);  //We wait until 4000us are passed.
  ledcWrite(pwmLedFlyChannel, 0);
  loopTimer = micros();                                                    //Set the timer for the next loop.
  
  timerChannel1 = esc1 + loopTimer;                                     //Calculate the time of the falling edge of the esc-1 pulse.
  timerChannel2 = esc2 + loopTimer;                                     //Calculate the time of the falling edge of the esc-2 pulse.
  timerChannel3 = esc3 + loopTimer;                                     //Calculate the time of the falling edge of the esc-3 pulse.
  timerChannel4 = esc4 + loopTimer;                                     //Calculate the time of the falling edge of the esc-4 pulse.
}
