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
   
  pinPulseWidth = (float)analogRead(PIN_BATTERY_LEVEL);          // get pulse width on pin
  batteryVoltage = pinPulseWidth * fromWidthToV;                // convert pulse width to vols

  #if DEBUG == true 
    Serial.print("initBattery: OK; voltage: ");
    Serial.println(batteryVoltage);
  #endif
  
}


/** 
 * @brief Compensate the ESCs pulses with battery voltage.
 */
void readBatteryVoltage(){
 
  // get battery voltage
  pinPulseWidth = (float)analogRead(PIN_BATTERY_LEVEL);

  // smooth readings
  batteryVoltage = batteryVoltage * 0.98 + pinPulseWidth * fromWidthToV * 0.02;

  // get battery percentage
  batteryPercentage = batteryVoltage / MAX_BATTERY_VOLTAGE * 100;

  //Turn on the led if battery voltage is too low.
  if(batteryVoltage <= WARNING_BATTERY_VOLTAGE){
    DANGEROUS_BATTERY_LEVEL = 1;
    ledcWrite(pwmLedBatteryChannel, MAX_DUTY_CYCLE);
  }

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
  Serial.printf("pinPulseWidth: %f,  batteryVoltage: %f V, TOTAL_DROP: %f, batteryPercentage: %f\n", pinPulseWidth, batteryVoltage, TOTAL_DROP, batteryPercentage);
}
