/**
 * @file PID.h
 * @author @sebastiano123-c
 * @brief PID routines.
 * @version 0.1
 * @date 2022-02-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/**
 * @brief Set the PID adjustments for output.
 * @note PID set point in degrees per second is determined by the roll receiver input.
 */
void setPID(){

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
  pidPitchSetpoint /= 3.0;                                               //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pidYawSetpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiverInputChannel3 > 1050){ //Do not yaw when turning off the motors.
    if(receiverInputChannel4 > 1508)pidYawSetpoint = (receiverInputChannel4 - 1508)/3.0;
    else if(receiverInputChannel4 < 1492)pidYawSetpoint = (receiverInputChannel4 - 1492)/3.0;
  }
}

/** 
 * @brief Calculates PID adjustments for outputs.
 * Calculates PID in degrees per seconds.
 * 
 */
void calculatePID(){


  //set PID parameters
  setPID();


  //Roll calculations
  pidErrorTemp = gyroRollInput - pidRollSetpoint;
  pidIMemRoll += pidErrorTemp;
  if(pidIMemRoll > PID_MAX_ROLL*IGainRoll)pidIMemRoll = PID_MAX_ROLL;
  else if(pidIMemRoll < PID_MAX_ROLL * (-IGainRoll))pidIMemRoll = PID_MAX_ROLL * -1;

  pidOutputRoll = (PGainRoll * pidErrorTemp) + (IGainRoll * pidIMemRoll) + (DGainRoll * (pidErrorTemp - pidLastRollDError));
  if(pidOutputRoll > PID_MAX_ROLL)pidOutputRoll = PID_MAX_ROLL;
  else if(pidOutputRoll < PID_MAX_ROLL * -1)pidOutputRoll = PID_MAX_ROLL * -1;

  pidLastRollDError = pidErrorTemp;


  //Pitch calculations
  pidErrorTemp = gyroPitchInput - pidPitchSetpoint;
  pidIMemPitch += pidErrorTemp;
  if(pidIMemPitch > PID_MAX_PITCH*IGainPitch)pidIMemPitch = PID_MAX_PITCH;
  else if(pidIMemPitch < PID_MAX_PITCH * (-IGainPitch))pidIMemPitch = PID_MAX_PITCH * -1;

  pidOutputPitch = (PGainPitch * pidErrorTemp) + (IGainPitch * pidIMemPitch) + (DGainPitch * (pidErrorTemp - pidLastPitchDError));
  if(pidOutputPitch > PID_MAX_PITCH)pidOutputPitch = PID_MAX_PITCH;
  else if(pidOutputPitch < PID_MAX_PITCH * -1)pidOutputPitch = PID_MAX_PITCH * -1;

  pidLastPitchDError = pidErrorTemp;


  //Yaw calculations
  pidErrorTemp = gyroYawInput - pidYawSetpoint;
  pidIMemYaw += pidErrorTemp;
  if(pidIMemYaw > PID_MAX_YAW * IGainYaw)pidIMemYaw = PID_MAX_YAW;
  else if(pidIMemYaw < PID_MAX_YAW * (-IGainYaw))pidIMemYaw = PID_MAX_YAW * -1;

  pidOutputYaw = (PGainYaw * pidErrorTemp) + (IGainYaw * pidIMemYaw) + (DGainYaw * (pidErrorTemp - pidLastYawDError));
  if(pidOutputYaw > PID_MAX_YAW)pidOutputYaw = PID_MAX_YAW;
  else if(pidOutputYaw < PID_MAX_YAW * -1)pidOutputYaw = PID_MAX_YAW * -1;

  pidLastYawDError = pidErrorTemp;

  #if DEBUG && defined(DEBUG_PID)
    printPIDGainParameters();
  #endif

  #if DEBUG && defined(DEBUG_PID_SIGNALS)
    printPIDGainParameters();
  #endif
}


/**
 * @brief Prints the PID signal input values.
 * 
 */
void printInputSignalsPID(){
  Serial.printf("Ry:%f, Rr:%f Re:%f, ",gyroRollInput, pidRollSetpoint,gyroRollInput - pidRollSetpoint);
  Serial.printf("Py:%f, Pr:%f Pe:%f, ",gyroPitchInput, pidPitchSetpoint,gyroPitchInput - pidPitchSetpoint);
  Serial.printf("Yy:%f, Yr:%f Ye:%f\n",gyroYawInput, pidYawSetpoint,gyroYawInput - pidYawSetpoint);
}


/**
 * @brief Prints the PID gain values.
 * 
 */
void printPIDGainParameters(){
      Serial.printf(" Pr: %.2f, Ir: %.5f, Dr: %.2f, ", PGainRoll, IGainRoll, DGainRoll);
      Serial.printf(" Pp: %.2f, Ip: %.5f, Dp: %.2f, ", PGainPitch, IGainPitch, DGainPitch);
      Serial.printf(" Py: %.3f, Iy: %.5f, Dy: %.2f, ", PGainYaw, IGainYaw, DGainYaw);
      Serial.printf(" Fi: %.5f, Pcor: %.2f, Rcor: %.2f \n", GYROSCOPE_ROLL_FILTER, GYROSCOPE_PITCH_CORR, GYROSCOPE_ROLL_CORR);
}