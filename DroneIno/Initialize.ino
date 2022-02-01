// Initialize 
// @author: Sebastiano Cocchi

void initialize(){
  if(DEBUG) Serial.begin(BAUD_RATE);

  EEPROM.begin(EEPROM_SIZE);
  vTaskDelay(50/portTICK_PERIOD_MS);

  if(DEBUG) intro();

  for(start = 0; start <= 35; start++) eepromData[start] = EEPROM.read(start);
 
  if(DEBUG) printEEPROM();

  // fill the configured values for the trims    
  for (start = 1; start <= 4; start++){
    byte channel = eepromData[start + 23] & 0b00000111;                   //What channel corresponds with the specific function
    if(eepromData[start + 23] & 0b10000000) trimCh[start].reverse = 1;    //Reverse channel when most significant bit is set
    else trimCh[start].reverse = 0;                                       //If the most significant is not set there is no reverse

    trimCh[start].low = (eepromData[channel * 2 + 15] << 8) | eepromData[channel * 2 + 14];    //Store the low value for the specific receiver input channel
    trimCh[start].center = (eepromData[channel * 2 - 1] << 8) | eepromData[channel * 2 - 2];   //Store the center value for the specific receiver input channel
    trimCh[start].high = (eepromData[channel * 2 + 7] << 8) | eepromData[channel * 2 + 6];     //Store the high value for the specific receiver input channel
  }

  //Set start back to zero.
  start = 0;                                                                
       
  // pinmode
  setupPins();
   
  //Start the I2C as master.
  setupGyroscope();                                                         

  //Check the EEPROM signature to make sure that the setup program is executed.
  while(eepromData[33] != 'J' || eepromData[34] != 'M' || eepromData[35] != 'B') vTaskDelay(10/portTICK_PERIOD_MS);

  //The flight controller needs the MPU-6050 with gyro and accelerometer
  //If setup is completed without MPU-6050 stop the flight controller program  
  if(eepromData[31] == 2 || eepromData[31] == 3) vTaskDelay(10/portTICK_PERIOD_MS);

  //Set the specific gyro registers.  
  setGyroscopeRegisters();                                                     

  // turn off second led
  ledcWrite(pwmLedFlyChannel, 0);

  // few seconds for calibrating the gyroscope
  calibrateGyroscope();

  // check Pressure
  checkAltitudeSensor();

  // wait until the rx is connected
  if(!DEBUG) waitController();

  //Set start back to 0.
  start = 0;                                                                

  //Load the battery voltage to the battery_voltage variable.
  initBattery();

  //Set the timer for the next loop.
  loopTimer = micros();                                            

  //When everything is done, turn off the led.
  ledcWrite(pwmLedChannel, 0);                               //Turn off the warning led.                                               
}

void waitController(){
  //Wait until the receiver is active and the throtle is set to the lower position.
  while(receiverInputChannel3 < 990 || receiverInputChannel3 > 1020 || receiverInputChannel4 < 1400){
    receiverInputChannel3 = convertReceiverChannel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
    receiverInputChannel4 = convertReceiverChannel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
    start ++;                                                          //While waiting increment start whith every loop.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
    ledcWrite(pwmChannel1, MAX_DUTY_CYCLE);
    ledcWrite(pwmChannel2, MAX_DUTY_CYCLE);
    ledcWrite(pwmChannel3, MAX_DUTY_CYCLE);
    ledcWrite(pwmChannel4, MAX_DUTY_CYCLE);
    
    vTaskDelay(1/portTICK_PERIOD_MS);                                                //Wait 1000us.
    ledcWrite(pwmChannel1, HALF_DUTY_CYCLE);
    ledcWrite(pwmChannel2, HALF_DUTY_CYCLE);
    ledcWrite(pwmChannel3, HALF_DUTY_CYCLE);
    ledcWrite(pwmChannel4, HALF_DUTY_CYCLE);                                                     //Set digital port 4, 5, 6 and 7 low.
    vTaskDelay(3/portTICK_PERIOD_MS);                                                //Wait 3000us.

    if(start == 125){                                                       //Every 125 loops (500ms).
      if(calInt % 15 == 0) ledcWrite(pwmLedChannel, MAX_DUTY_CYCLE);                //Change the led status to indicate calibration.
      else ledcWrite(pwmLedChannel, 0);
      start = 0;                                                            //Start again at 0.
    }
  }                    
}

void setupPins(){
  // LED 
  ledcSetup(pwmLedChannel, freq, resolution);
  ledcAttachPin(PIN_BATTERY_LED, pwmLedChannel);

  //FLY LED
  ledcSetup(pwmLedFlyChannel, freq, resolution);
  ledcAttachPin(PIN_SECOND_LED, pwmLedFlyChannel);//pwmLedFlyChannel

  // ESCs pinmode  
  //     ledc ESC PWM setups
  ledcSetup(pwmChannel1, freq, resolution);
  ledcSetup(pwmChannel2, freq, resolution);
  ledcSetup(pwmChannel3, freq, resolution);
  ledcSetup(pwmChannel4, freq, resolution);

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
  
  //       event change detector
  attachInterrupt(digitalPinToInterrupt(PIN_RECEIVER_1), myISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RECEIVER_2), myISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RECEIVER_3), myISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RECEIVER_4), myISR, CHANGE);

  //Use the led on the Arduino for startup indication.
  ledcWrite(pwmLedChannel, MAX_DUTY_CYCLE);                                                    //Turn on the warning led.
  ledcWrite(pwmLedFlyChannel, MAX_DUTY_CYCLE); 
}

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

void printEEPROM(){
  for (int i = 0; i < EEPROM_SIZE; i++){
    Serial.println(eepromData[i]);
  }
}
