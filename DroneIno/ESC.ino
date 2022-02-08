/* 
* ESC 
* @author @sebastiano123-c
*/

void droneStart(){
  /* 
  * @brief first settings of the PID and angles
  */

  start = 2;

  anglePitch = anglePitchAcc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
  angleRoll = angleRollAcc;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
  gyroAnglesSet = true;                                                //Set the IMU started flag.

  //Reset the PID controllers for a bumpless start.
  pidIMemRoll = 0;
  pidLastRollDError = 0;
  pidIMemPitch = 0;
  pidLastPitchDError = 0;
  pidIMemYaw = 0;
  pidLastYawDError = 0;
}

void setEscPulses(){
  /* 
  * @brief set ESC pulses
  */

  throttle = receiverInputChannel3;                                   //We need the throttle signal as a base signal.

  switch(start){
    case 2:
      if (throttle > 1800) throttle = 1800;                            //We need some room to keep full control at full throttle.
      esc1 = throttle - pidOutputPitch + pidOutputRoll - pidOutputYaw; //Calculate the pulse for esc 1 (front-right - CCW)
      esc2 = throttle + pidOutputPitch + pidOutputRoll + pidOutputYaw; //Calculate the pulse for esc 2 (rear-right - CW)
      esc3 = throttle + pidOutputPitch - pidOutputRoll - pidOutputYaw; //Calculate the pulse for esc 3 (rear-left - CCW)
      esc4 = throttle - pidOutputPitch - pidOutputRoll + pidOutputYaw; //Calculate the pulse for esc 4 (front-left - CW)

      if (batteryVoltage < 1240 && batteryVoltage > 800){              //Is the battery connected?
        esc1 += esc1 * ((1240 - batteryVoltage)/(float)3500);          //Compensate the esc-1 pulse for voltage drop.
        esc2 += esc2 * ((1240 - batteryVoltage)/(float)3500);          //Compensate the esc-2 pulse for voltage drop.
        esc3 += esc3 * ((1240 - batteryVoltage)/(float)3500);          //Compensate the esc-3 pulse for voltage drop.
        esc4 += esc4 * ((1240 - batteryVoltage)/(float)3500);          //Compensate the esc-4 pulse for voltage drop.
      }

      if (esc1 < 1100) esc1 = 1100;                                    //Keep the motors running.
      if (esc2 < 1100) esc2 = 1100;                                    //Keep the motors running.
      if (esc3 < 1100) esc3 = 1100;                                    //Keep the motors running.
      if (esc4 < 1100) esc4 = 1100;                                    //Keep the motors running.

      if(esc1 > 2000) esc1 = 2000;                                     //Limit the esc-1 pulse to 2000us.
      if(esc2 > 2000) esc2 = 2000;                                     //Limit the esc-2 pulse to 2000us.
      if(esc3 > 2000) esc3 = 2000;                                     //Limit the esc-3 pulse to 2000us.
      if(esc4 > 2000) esc4 = 2000;                                     //Limit the esc-4 pulse to 2000us.  
      break;
    
    default:
      esc1 = 1000;                                                      //If start is not 2 keep a 1000us pulse for ess-1.
      esc2 = 1000;                                                      //If start is not 2 keep a 1000us pulse for ess-2.
      esc3 = 1000;                                                      //If start is not 2 keep a 1000us pulse for ess-3.
      esc4 = 1000;                                                      //If start is not 2 keep a 1000us pulse for ess-4.
  }

  // write to ESCs
  ledcWrite(pwmChannel1, esc1); ///2000*MAX_DUTY_CYCLE
  ledcWrite(pwmChannel2, esc2); ///2000*MAX_DUTY_CYCLE
  ledcWrite(pwmChannel3, esc3); ///2000*MAX_DUTY_CYCLE
  ledcWrite(pwmChannel4, esc4); ///2000*MAX_DUTY_CYCLE
}

void convertAllSignals(){
  receiverInputChannel1 = convertReceiverChannel(1);           //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
  receiverInputChannel2 = convertReceiverChannel(2);           //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
  receiverInputChannel3 = convertReceiverChannel(3);           //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
  receiverInputChannel4 = convertReceiverChannel(4);           //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.
}
