void getBatteryVoltage(){
  batteryVoltage = (analogRead(PIN_BATTERY_LEVEL) + 65) * 1.2317;//(float)analogRead(PIN_BATTERY_LEVEL) * fromVoltToTick;
  Serial.print("Battery voltage: ");
  Serial.print(batteryVoltage);
  Serial.println("V");
}

void batteryVoltageCompensation(){
  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //0.09853 = 0.08 * 1.2317.
  batteryVoltage = batteryVoltage * 0.92 + (analogRead(PIN_BATTERY_LEVEL) + 65) * 0.09853;

  //Turn on the led if battery voltage is to low.
  if(batteryVoltage < 1000 && batteryVoltage > 600) digitalWrite(PIN_BATTERY_LED, HIGH);
}
