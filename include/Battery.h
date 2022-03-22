/**
 * @file Battery.h
 * @author @sebastiano123-c
 * @brief Battery calculations routines.
 * @version 0.1
 * @date 2022-02-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */


/**
 * @brief Calculates the voltage partitor given the total drop defined in the @ref Constants.ino "Constants.ino".
 *
 * @param volt 
 * @return float 
 */
float voltagePartitor(int volt){
  return (float)(volt-DIODE_DROP) * totalDrop;
}

/**
 * @brief Calculates the the coversion between the pin readings and the voltage.
 * 
 * @param width 
 * @param corr 
 * @return float 
 */
float fromWidthToVPin(int width, float corr = 1.){
  return (float)width / (fromVtoWidth * corr);
}

/**
 * @brief Calculates the conversion between the analog pin readings and the battery initial voltage.
 * 
 * @param width 
 * @param corr 
 * @return float 
 */
float fromWidthToVBattery(int width, float corr = 1.){
  return (float)width / (totalDrop * fromVtoWidth * corr) + DIODE_DROP;
}


/** 
 * @brief Get battery voltage.
 */
void getBatteryVoltage(){

  batteryVoltage = (float)analogRead(PIN_BATTERY_LEVEL) / fromVtoWidth;

}


/** 
 * @brief Get the initial value of the battery voltage.
 */
void initBattery(){
  
  analogSetWidth(adcBits);                                      // set 2^10=1024 width (analogSetWidth can go from 9-12 (default 12))
  
  
  getBatteryVoltage();                                          // get battery voltage

  // if(DEBUG) {Serial.print("initBattery: OK; voltage: "); Serial.println(batteryVoltage);}
  
}


/** 
 * @brief Compensate the ESCs pulses with battery voltage.
 */
void batteryVoltageCompensation(){
 
  // The battery voltage is needed for compensation.
  batteryVoltage = batteryVoltage * 0.92 + analogRead(PIN_BATTERY_LEVEL) / fromVtoWidth * 0.08;
 // batteryPercent = batteryVoltage / correctionBattery / maxBatteryLevelDropped;
  batteryPercent = fromWidthToVBattery(batteryVoltage)/1e3;///correctionBattery/maxBatteryLevelDropped;

//  if(DEBUG) {
//    Serial.print("batteryVoltage: "); Serial.print(batteryVoltage);
//    Serial.print(", analog read width: "); Serial.print(analogRead(2));
//    Serial.print(", analog read volt: "); Serial.print(analogRead(PIN_BATTERY_LEVEL) / fromVtoWidth/1000);
//    Serial.print(", minBatteryLevelThreshold: "); Serial.println(minBatteryLevelThreshold);
//  }

  //Turn on the led if battery voltage is too low.
  if(batteryVoltage < minBatteryLevelThreshold) ledcWrite(pwmLedChannel, MAX_DUTY_CYCLE);
  else ledcWrite(pwmLedChannel, 0);
}
