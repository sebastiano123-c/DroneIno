/**
 * @file Initialize.ino   
 * @author @sebastiano123-c
 * @brief Some routines used in the setup().
 * @version 0.1
 * @date 2022-03-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/** 
 * @brief Initilizes the EEPROM.
 */
void initEEPROM(){
 
  EEPROM.begin(EEPROM_SIZE);
  vTaskDelay(50/portTICK_PERIOD_MS);

  //if(DEBUG) printEEPROM();

  for(start = 0; start <= 35; start++) eepromData[start] = EEPROM.read(start);
 
  //Check the EEPROM signature to make sure that the setup program is executed.
  while(eepromData[33] != 'J' || eepromData[34] != 'M' || eepromData[35] != 'B') vTaskDelay(10/portTICK_PERIOD_MS);

}

/** 
 * @brief Fills the trims values using the EEPROM saved values. 
 */
void configureReceiverTrims(){

  for (start = 1; start <= 4; start++)
  {
    byte channel = eepromData[start + 23] & 0b00000111;                   //What channel corresponds with the specific function

    if(eepromData[start + 23] & 0b10000000) trimCh[start].reverse = 1;    //Reverse channel when most significant bit is set
    else trimCh[start].reverse = 0;                                       //If the most significant is not set there is no reverse

    trimCh[start].low = (eepromData[channel * 2 + 15] << 8) | eepromData[channel * 2 + 14];  //Store the low value for the specific receiver input channel
    trimCh[start].center = (eepromData[channel * 2 - 1] << 8) | eepromData[channel * 2 - 2]; //Store the center value for the specific receiver input channel
    trimCh[start].high = (eepromData[channel * 2 + 7] << 8) | eepromData[channel * 2 + 6];   //Store the high value for the specific receiver input channel

  }
  
  trimCh[5] = {reverse : 0b00000000, low : 1000, center: 1500, high : 2000};

}

/**
 * @brief Setup the WIRE communication.
 */
void setupWireI2C(){


  // setup wire
  Wire.setClock(WIRE_CLOCK);
  Wire.begin(PIN_SDA, PIN_SCL);
  vTaskDelay(40/portTICK_PERIOD_MS);
  
}
 
/** 
 * @brief Definition of all the pinModes.
 */
void setupPins(){
 
  // LED pinmode
  ledcSetup(pwmLedChannel, freq, resolution);                                       // battery led
  ledcSetup(pwmLedFlyChannel, freq, resolution);                                    // fly led
  ledcAttachPin(PIN_BATTERY_LED, pwmLedChannel);
  ledcAttachPin(PIN_SECOND_LED, pwmLedFlyChannel);


  // ESCs pinmode  
  //     ledc ESC PWM setups
  ledcSetup(pwmChannel1, freq, resolution);                                         // ESC 1 pinmode
  ledcSetup(pwmChannel2, freq, resolution);                                         // ESC 2 pinmode 
  ledcSetup(pwmChannel3, freq, resolution);                                         // ESC 3 pinmode 
  ledcSetup(pwmChannel4, freq, resolution);                                         // ESC 4 pinmode 

  //      attach pin to channel
  ledcAttachPin(PIN_ESC_1, pwmChannel1);
  ledcAttachPin(PIN_ESC_2, pwmChannel2);
  ledcAttachPin(PIN_ESC_3, pwmChannel3);
  ledcAttachPin(PIN_ESC_4, pwmChannel4);


  // RECEIVER pinmode
  pinMode(PIN_RECEIVER_1, INPUT_PULLUP);
  pinMode(PIN_RECEIVER_2, INPUT_PULLUP);
  pinMode(PIN_RECEIVER_3, INPUT_PULLUP);
  pinMode(PIN_RECEIVER_4, INPUT_PULLUP);
  pinMode(PIN_RECEIVER_5, INPUT_PULLUP);
  
  //       event change detector
  attachInterrupt(digitalPinToInterrupt(PIN_RECEIVER_1), myISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RECEIVER_2), myISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RECEIVER_3), myISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RECEIVER_4), myISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RECEIVER_5), myISR, CHANGE);

  // remove the motor beep
  ledcWrite(pwmChannel1, 1000);
  ledcWrite(pwmChannel2, 1000);
  ledcWrite(pwmChannel3, 1000);
  ledcWrite(pwmChannel4, 1000);


  // BATTERY LEVEL pinmode
  pinMode(PIN_BATTERY_LEVEL, INPUT);

}

/** 
 * @brief Fancy introduction with logo and author.
 */
void intro(){

  vTaskDelay(50/portTICK_PERIOD_MS);
  Serial.println();
  Serial.println();
  Serial.println("    |                      |   ");
  Serial.println("    |                      |   ");
  Serial.println(" -------               --------");
  Serial.println("    |\                    /|   ");
  Serial.println("    | \                  / |   ");
  Serial.println("       \                /      ");
  Serial.println("         ---------------       "); 
  Serial.println("         |  DRONEINO!  |       ");
  Serial.println("         ---------------       "); 
  Serial.println("       /                \      ");
  Serial.println("    | /                  \ |   ");
  Serial.println("    |/                    \|   ");
  Serial.println(" -------               --------");
  Serial.println("    |                      |   ");
  Serial.println("    |                      |   ");
  vTaskDelay(1000/portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("@Author: Sebastiano Cocchi");
  Serial.println();
  vTaskDelay(1000/portTICK_PERIOD_MS);

}

/** 
 * @brief Print all the EEPROM data. 
 */
void printEEPROM(){

  for (int i = 0; i < EEPROM_SIZE; i++){
    Serial.println(eepromData[i]);
  }
}
