// Battery
// @author: Sebastiano Cocchi

void initBattery(){
  // set 2^10=1024 width (analogSetWidth can go from 9-12 (default 12))
  analogSetWidth(adcBits);  
  
  // get battery voltage
  getBatteryVoltage();
}

int getBatteryVoltage(){
  // get battery voltage
  batteryVoltage = analogRead(PIN_BATTERY_LEVEL) / fromVtoBit;
}

void batteryVoltageCompensation(){
  //The battery voltage is needed for compensation.
  batteryVoltage = batteryVoltage * 0.92 + analogRead(PIN_BATTERY_LEVEL) / fromVtoBit * 0.08;

  //Turn on the led if battery voltage is too low.
  if(batteryVoltage < minBatteryVoltageInBits) ledcWrite(pwmLedChannel, MAX_DUTY_CYCLE);
}
