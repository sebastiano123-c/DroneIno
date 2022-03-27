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
 * @brief Get the initial value of the battery voltage.
 */
void initBattery(){
  
  analogSetWidth(adcBits);                                      // set 2^10=1024 width (analogSetWidth can go from 9-12 (default 12))
  
  readBatteryVoltage();                                          // get battery voltage

  #if DEBUG == true 
    Serial.print("initBattery: OK; voltage: ");
    Serial.println(batteryVoltage);
  #endif
  
}


/** 
 * @brief Compensate the ESCs pulses with battery voltage.
 */
void readBatteryVoltage(){
 
  // The battery voltage is needed for compensation.
  pinPulseWidth = (float)analogRead(PIN_BATTERY_LEVEL);
  pinPulseVoltage = pinPulseWidth * fromWidthToV;
  // pinPulseVoltage = pinPulseVoltage * 0.92 + 
  //                   (float)adc1_get_raw(PIN_BATTERY_LEVEL) * fromWidthToV * 0.08;     // smooth readings
  batteryVoltage = pinPulseVoltage / totalDrop;
  batteryPercentage = batteryVoltage / MAX_BATTERY_VOLTAGE * 100;

  //Turn on the led if battery voltage is too low.
  if(batteryVoltage < MIN_BATTERY_VOLTAGE) ledcWrite(pwmLedBatteryChannel, MAX_DUTY_CYCLE);
  else ledcWrite(pwmLedBatteryChannel, 0);

  #if DEBUG
    printBatteryVoltage();
  #endif
}


/**
 * @brief Prints battery readings
 * 
 */
void printBatteryVoltage(){
  // Serial.print(analogRead(PIN_BATTERY_LEVEL));
  Serial.printf("pinPulseWidth: %f,  pinPulseVoltage: %f,  batteryVoltage: %f V, batteryPercentage: %f\n", pinPulseWidth, pinPulseVoltage, batteryVoltage, batteryPercentage);
}
