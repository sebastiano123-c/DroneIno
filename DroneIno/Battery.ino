// Battery
// @author: Sebastiano Cocchi

void initBattery(){
  // set 2^10=1024 width (analogSetWidth can go from 9-12 (default 12))
  analogSetWidth(adcBits);  
  
  // get battery voltage
  getBatteryVoltage();

  if(DEBUG) {Serial.print("initBattery: OK; voltage: "); Serial.println(batteryVoltage);}
}

int getBatteryVoltage(){
  // get battery voltage
  batteryVoltage = analogRead(PIN_BATTERY_LEVEL) / fromVtoWidth;
}

void batteryVoltageCompensation(){
  //The battery voltage is needed for compensation.
  batteryVoltage = batteryVoltage * 0.92 + analogRead(PIN_BATTERY_LEVEL) / fromVtoWidth * 0.08;
  
  if(DEBUG) {
    Serial.print("batteryVoltage: "); Serial.print(batteryVoltage);
    Serial.print(",analog read width: "); Serial.print(analogRead(PIN_BATTERY_LEVEL));
    Serial.print(",analog read volt: "); Serial.print(analogRead(PIN_BATTERY_LEVEL) / fromVtoWidth);
    Serial.print(",minBatteryLevelThreshold: "); Serial.println(minBatteryLevelThreshold);
  }

  //Turn on the led if battery voltage is too low.
  if(batteryVoltage < (float)minBatteryLevelThreshold) ledcWrite(pwmLedChannel, MAX_DUTY_CYCLE);
  else ledcWrite(pwmLedChannel, 0);
}
