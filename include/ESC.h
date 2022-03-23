/**
 * @file ESC.h
 * @author @sebastiano123-c
 * @brief ESC routines to govern the motors.
 * @version 0.1
 * @date 2022-02-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/**
 * @brief First settings of the PID and angles.
 * 
 */
void droneStart(){

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

/** 
* @brief Set ESC pulses.
*/
void setEscPulses(){

  throttle = receiverInputChannel3;                                 //We need the throttle signal as a base signal.

  switch(start){
    case 2:
      
      // set throttle for altitude hold
      if (flightMode >= 2) {                                         //If altitude mode is active.
        throttle = 1500 + pidOutputAltitude;                           // add the altitude hold
      }

      if (throttle > 1800) throttle = 1800;                            //We need some room to keep full control at full throttle.

      esc1 = throttle - pidOutputPitch + pidOutputRoll - pidOutputYaw; //Calculate the pulse for esc 1 (front-right - CCW)
      esc2 = throttle + pidOutputPitch + pidOutputRoll + pidOutputYaw; //Calculate the pulse for esc 2 (rear-right - CW)
      esc3 = throttle + pidOutputPitch - pidOutputRoll - pidOutputYaw; //Calculate the pulse for esc 3 (rear-left - CCW)
      esc4 = throttle - pidOutputPitch - pidOutputRoll + pidOutputYaw; //Calculate the pulse for esc 4 (front-left - CW)

      if (batteryVoltageV > 2. && batteryVoltageV < 10.7){              // Is the battery connected?
        int16_t escCorrection = (int16_t)((10.7 - batteryVoltageV)/batteryCurvePendency);// Compensate the esc-1 pulse for voltage drop.
        esc1 += escCorrection;                                         // Compensate the esc-1 pulse for voltage drop.
        esc2 += escCorrection;                                         // Compensate the esc-2 pulse for voltage drop.
        esc3 += escCorrection;                                         // Compensate the esc-3 pulse for voltage drop.
        esc4 += escCorrection;                                         // Compensate the esc-4 pulse for voltage drop.
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
  ledcWrite(pwmChannel1, (float)esc1/2000.*(float)MAX_DUTY_CYCLE);
  ledcWrite(pwmChannel2, (float)esc2/2000.*(float)MAX_DUTY_CYCLE);
  ledcWrite(pwmChannel3, (float)esc3/2000.*(float)MAX_DUTY_CYCLE);
  ledcWrite(pwmChannel4, (float)esc4/2000.*(float)MAX_DUTY_CYCLE);

  // Serial.printf("%i   %i   %i   %i \n", 
  //   (uint32_t)(((float)esc1/2000.)*MAX_DUTY_CYCLE), 
  //   (uint32_t)(((float)esc2/2000.)*MAX_DUTY_CYCLE), 
  //   (uint32_t)(((float)esc3/2000.)*MAX_DUTY_CYCLE), 
  //   (uint32_t)(((float)esc4/2000.)*MAX_DUTY_CYCLE));
}

/**
 * @brief Converts PWM signals input into to the standard 1000 - 2000us.
 * 
 */
void convertAllSignals(){

    receiverInputChannel1 = convertReceiverChannel(1);           //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
    receiverInputChannel2 = convertReceiverChannel(2);           //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
    receiverInputChannel3 = convertReceiverChannel(3);           //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
    receiverInputChannel4 = convertReceiverChannel(4);           //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.

}
