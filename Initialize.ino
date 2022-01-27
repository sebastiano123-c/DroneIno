// Initialize 
// @author: Sebastiano Cocchi

void initialize(){
  Serial.begin(BAUD_RATE);
  EEPROM.begin(EEPROM_SIZE);
  vTaskDelay(50/portTICK_PERIOD_MS);

  //intro();
  
  for(start = 0; start <= 35; start++) eepromData[start] = EEPROM.read(start);
  start = 0;                                                                //Set start back to zero.
  gyroAddress = eepromData[32];                                           //Store the gyro address in the variable.

  //printEEPROM();

  //Start the I2C as master.
  setupMPU();                                                         
    
  // LED pinmode
  pinMode(PIN_BATTERY_LED, OUTPUT);
  pinMode(PIN_DIGITAL_13, OUTPUT);

  // ESCs pinmode
  pinMode(PIN_ESC_1, OUTPUT);
  pinMode(PIN_ESC_2, OUTPUT);
  pinMode(PIN_ESC_3, OUTPUT);
  pinMode(PIN_ESC_4, OUTPUT);
  
  //     ledc PWM setups
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
  digitalWrite(PIN_BATTERY_LED, HIGH);                                                    //Turn on the warning led.

  //Check the EEPROM signature to make sure that the setup program is executed.
  while(eepromData[33] != 'J' || eepromData[34] != 'M' || eepromData[35] != 'B') vTaskDelay(10/portTICK_PERIOD_MS);

  //The flight controller needs the MPU-6050 with gyro and accelerometer
  //If setup is completed without MPU-6050 stop the flight controller program  
  if(eepromData[31] == 2 || eepromData[31] == 3) vTaskDelay(10/portTICK_PERIOD_MS);

  setGyroscopeRegisters();                                                     //Set the specific gyro registers.

  for (calInt = 0; calInt < 1250 ; calInt ++){                           //Wait 5 seconds before continuing.
    digitalWrite(PIN_ESC_1, HIGH);
    digitalWrite(PIN_ESC_2, HIGH);
    digitalWrite(PIN_ESC_3, HIGH);
    digitalWrite(PIN_ESC_4, HIGH);
    
    vTaskDelay(1/portTICK_PERIOD_MS);                                                //Wait 1000us.
    digitalWrite(PIN_ESC_1, LOW);
    digitalWrite(PIN_ESC_2, LOW);
    digitalWrite(PIN_ESC_3, LOW);
    digitalWrite(PIN_ESC_4, LOW);                                                     //Set digital port 4, 5, 6 and 7 low.
    vTaskDelay(3/portTICK_PERIOD_MS);                                                //Wait 3000us.
  }

  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  for (calInt = 0; calInt < 2000 ; calInt ++){                           //Take 2000 readings for calibration.
    if(calInt % 15 == 0)digitalWrite(PIN_BATTERY_LED, !digitalRead(PIN_BATTERY_LED));                //Change the led status to indicate calibration.
    readGyroscopeStatus();                                                        //Read the gyro output.
    gyroAxisCalibration[1] += gyroAxis[1];                                       //Ad roll value to gyro_roll_cal.
    gyroAxisCalibration[2] += gyroAxis[2];                                       //Ad pitch value to gyro_pitch_cal.
    gyroAxisCalibration[3] += gyroAxis[3];                                       //Ad yaw value to gyro_yaw_cal.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
    digitalWrite(PIN_ESC_1, HIGH);
    digitalWrite(PIN_ESC_2, HIGH);
    digitalWrite(PIN_ESC_3, HIGH);
    digitalWrite(PIN_ESC_4, HIGH);
    
    vTaskDelay(1/portTICK_PERIOD_MS);                                                //Wait 1000us.
    
    digitalWrite(PIN_ESC_1, LOW);
    digitalWrite(PIN_ESC_2, LOW);
    digitalWrite(PIN_ESC_3, LOW);
    digitalWrite(PIN_ESC_4, LOW);                                                     //Set digital port 4, 5, 6 and 7 low.
    vTaskDelay(3/portTICK_PERIOD_MS);                                                //Wait 3000us.
  }
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  gyroAxisCalibration[1] /= 2000;                                                 //Divide the roll total by 2000.
  gyroAxisCalibration[2] /= 2000;                                                 //Divide the pitch total by 2000.
  gyroAxisCalibration[3] /= 2000;                                                 //Divide the yaw total by 2000.
  
  //Wait until the receiver is active and the throtle is set to the lower position.
  while(receiverInputChannel3 < 990 || receiverInputChannel3 > 1020 || receiverInputChannel4 < 1400){
    receiverInputChannel3 = convertReceiverChannel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
    receiverInputChannel4 = convertReceiverChannel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
    start ++;                                                               //While waiting increment start whith every loop.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
    digitalWrite(PIN_ESC_1, HIGH);
    digitalWrite(PIN_ESC_2, HIGH);
    digitalWrite(PIN_ESC_3, HIGH);
    digitalWrite(PIN_ESC_4, HIGH);
    
    vTaskDelay(1/portTICK_PERIOD_MS);                                                //Wait 1000us.
    digitalWrite(PIN_ESC_1, LOW);
    digitalWrite(PIN_ESC_2, LOW);
    digitalWrite(PIN_ESC_3, LOW);
    digitalWrite(PIN_ESC_4, LOW);                                                     //Set digital port 4, 5, 6 and 7 low.
    vTaskDelay(3/portTICK_PERIOD_MS);                                                //Wait 3000us.

    if(start == 125){                                                       //Every 125 loops (500ms).
      digitalWrite(PIN_BATTERY_LED, !digitalRead(PIN_BATTERY_LED));                                   //Change the led status.
      start = 0;                                                            //Start again at 0.
    }
  }
  start = 0;                                                                //Set start back to 0.

  //Load the battery voltage to the battery_voltage variable.
  getBatteryVoltage();

  //Set the timer for the next loop.
  loopTimer = micros();                                            

  //When everything is done, turn off the led.
  digitalWrite(PIN_BATTERY_LED, LOW);                               //Turn off the warning led.                                               
}

void waitController(){
  //Wait until the receiver is active and the throtle is set to the lower position.
  while(receiverInputChannel3 < 990 || receiverInputChannel3 > 1020 || receiverInputChannel4 < 1400){
    receiverInputChannel3 = convertReceiverChannel(3);       //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
    receiverInputChannel4 = convertReceiverChannel(4);       //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
    start ++;                                                //While waiting increment start whith every loop.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
    digitalWrite(PIN_ESC_1, HIGH);
    digitalWrite(PIN_ESC_2, HIGH);
    digitalWrite(PIN_ESC_3, HIGH);
    digitalWrite(PIN_ESC_4, HIGH);

    vTaskDelay(1/portTICK_PERIOD_MS);                          //Wait 1000us.
 
    digitalWrite(PIN_ESC_1, LOW);
    digitalWrite(PIN_ESC_2, LOW);
    digitalWrite(PIN_ESC_3, LOW);
    digitalWrite(PIN_ESC_4, LOW);

    vTaskDelay(3/portTICK_PERIOD_MS);                          //Wait 3 milliseconds before the next loop.
    if(start == 125){                                          //Every 125 loops (500ms).
      digitalWrite(PIN_BATTERY_LED, !digitalRead(PIN_BATTERY_LED));//blink
      start = 0;                                               //Start again at 0.
    }
  }                                                             
}

void intro(){
  vTaskDelay(500/portTICK_PERIOD_MS);
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
  vTaskDelay(1500/portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("@Author: Sebastiano Cocchi");
  Serial.println();
  vTaskDelay(1500/portTICK_PERIOD_MS);

}

void printEEPROM(){
  for (int i = 0; i < EEPROM_SIZE; i++){
    Serial.println(eepromData[i]);
  }
}
