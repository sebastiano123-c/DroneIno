void initBattery(){
  analogSetWidth(adcBits);          // means 2^10=1024 width (analogSetWidth can go from 9-12 (default 12))
  getBatteryVoltage();
}

int getBatteryVoltage(){
  batteryVoltage = analogRead(PIN_BATTERY_LEVEL) / fromVtoBit;
}

void batteryVoltageCompensation(){
  //The battery voltage is needed for compensation.
  batteryVoltage = batteryVoltage * 0.92 + analogRead(PIN_BATTERY_LEVEL) / fromVtoBit * 0.08;

  //Turn on the led if battery voltage is to low.
  if(batteryVoltage < 1000 && batteryVoltage > 600) ledcWrite(pwmLedChannel, MAX_DUTY_CYCLE);
}
