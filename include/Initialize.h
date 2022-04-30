/**
 * @file Initialize.h   
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
  
  trimCh[0] = {reverse : 0b00000000, low : 1000, center: 1500, high : 2000};

}

/**
 * @brief Setup the WIRE communication.
 */
void setupWireI2C(){

  // setup wire for
  // Wire.begin(21, 22);
  Wire.begin(PIN_SDA, PIN_SCL);
  vTaskDelay(40/portTICK_PERIOD_MS);
  Wire.setClock(WIRE_CLOCK);        
  
}


/** 
 * @brief Definition of all the pinModes.
 */
void setupPins(){
 
  // LED pinmode
  ledcSetup(pwmLedChannel, freq, resolution);                                       // battery led
  ledcSetup(pwmLedBatteryChannel, freq, resolution);                                // fly led
  ledcAttachPin(PIN_BATTERY_LED, pwmLedChannel);
  ledcAttachPin(PIN_SECOND_LED, pwmLedBatteryChannel);


  // ESCs 
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_ESC_1 ); 
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PIN_ESC_2 );
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, PIN_ESC_3 );
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, PIN_ESC_4 );

  mcpwm_config_t pwm_config = {};
  pwm_config.frequency = 500;                                                           //frequency = 500Hz, i.e. for every motor time period is 2ms
  pwm_config.cmpr_a = 0;                                                                //duty cycle of PWMxA = 0
  pwm_config.cmpr_b = 0;                                                                //duty cycle of PWMxb = 0
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);                                 //Configure PWM0A timer 0 with above settings
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);                                 //Configure PWM0A timer 1 with above settings
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);                                 //Configure PWM0A timer 1 with above settings
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);                                 //Configure PWM0A timer 1 with above settings

  // remove the motor beep
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1000);
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 1000);
  mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 1000);
  mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, 1000);


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


  // BATTERY LEVEL pinmode
  pinMode(PIN_BATTERY_LEVEL, INPUT);

}

/** 
 * @brief Fancy introduction with logo and author.
 */
void intro(){

  vTaskDelay(1500/portTICK_PERIOD_MS);
  Serial.println();
  Serial.println();
  Serial.print("    |                      |   \n");
  Serial.print("    |                      |   \n");
  Serial.print(" -------               --------\n");
  Serial.print("    |\\                   /|   \n");
  Serial.print("    | \\                 / |   \n");
  Serial.print("       \\               /      \n");
  Serial.print("         ---------------       \n"); 
  Serial.print("         |  DRONEINO!  |       \n");
  Serial.print("         ---------------       \n"); 
  Serial.print("       /                \\     \n");
  Serial.print("    | /                  \\ |   \n");
  Serial.print("    |/                    \\|   \n");
  Serial.print(" -------               --------\n");
  Serial.print("    |                      |   \n");
  Serial.print("    |                      |   \n");
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
   //print eeprom data on serial
  Serial.println("");
  Serial.println("EEPROM data:");
  Serial.println("");
  for (int i = 0; i < EEPROM_SIZE; i++){
    Serial.printf("EEPROM[%i] => %i\n", i, eepromData[i]);
  }
}
