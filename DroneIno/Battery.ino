/**
 * @brief Battery calculations routines
 * 
 * @param volt 
 * @return float 
 */

float voltagePartitor(int volt){
  /** 
    * @brief voltage partitor 
    * @param (int)volt battery voltage in mV
    */

  return (float)(volt-DIODE_DROP) * totalDrop;
}

float fromWidthToVPin(int width, float corr = 1.){
  return (float)width / (fromVtoWidth * corr);
}

float fromWidthToVBattery(int width, float corr = 1.){
  return (float)width / (totalDrop * fromVtoWidth * corr) + DIODE_DROP;
}

void initBattery(){
  /** 
    * @brief get the initial value of the battery voltage
    */

  
  analogSetWidth(adcBits);                                      // set 2^10=1024 width (analogSetWidth can go from 9-12 (default 12))
  
  
  getBatteryVoltage();                                          // get battery voltage

  // if(DEBUG) {Serial.print("initBattery: OK; voltage: "); Serial.println(batteryVoltage);}
  
}

int getBatteryVoltage(){
  /** 
    * @brief get battery voltage
    */

  batteryVoltage = (float)analogRead(PIN_BATTERY_LEVEL) / fromVtoWidth;

}

void batteryVoltageCompensation(){
  /** 
    * @brief compensate the ESCs pulses with battery voltage
    */
 
  //The battery voltage is needed for compensation.
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
