/**
 * @file receiverRoutines.ino
 * @author @sebastiano123-c
 * @brief Receiver routines.
 * @version 0.1
 * @date 2022-03-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/**
 * @brief Check if the receiver values are valid within 10 seconds.
 * 
 */
void waitForReceiver(){
 while(receiverInputChannel3 < 990 || receiverInputChannel3 > 1020 || receiverInputChannel4 < 1400)
  {
    receiverInputChannel3 = convertReceiverChannel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
    receiverInputChannel4 = convertReceiverChannel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us

  }   

  Serial.println("Receiver OK!");
}

/**
 * @brief Convert the receiver signal.
 * 
 * This part converts the actual receiver signals to a standardized 1000 – 2000us value.
 * The stored data in the EEPROM is used.
 * 
 * @param function 
 * @return int 
 */
int convertReceiverChannel(byte function){
  byte channel, reverse;                                                       //First we declare some local variables
  int low, center, high, actual;
  int difference;

  channel = eepromData[function + 23] & 0b00000111;                           //What channel corresponds with the specific function
  if(eepromData[function + 23] & 0b10000000)reverse = 1;                      //Reverse channel when most significant bit is set
  else reverse = 0;                                                            //If the most significant is not set there is no reverse

  actual = receiverInput[channel];                                            //Read the actual receiver value for the corresponding function
  low = (eepromData[channel * 2 + 15] << 8) | eepromData[channel * 2 + 14];  //Store the low value for the specific receiver input channel
  center = (eepromData[channel * 2 - 1] << 8) | eepromData[channel * 2 - 2]; //Store the center value for the specific receiver input channel
  high = (eepromData[channel * 2 + 7] << 8) | eepromData[channel * 2 + 6];   //Store the high value for the specific receiver input channel

  if(actual < center){                                                         //The actual receiver value is lower than the center value
    if(actual < low)actual = low;                                              //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center - actual) * (long)500) / (center - low);       //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 + difference;                                  //If the channel is reversed
    else return 1500 - difference;                                             //If the channel is not reversed
  }
  else if(actual > center){                                                                        //The actual receiver value is higher than the center value
    if(actual > high)actual = high;                                            //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center) * (long)500) / (high - center);      //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 - difference;                                  //If the channel is reversed
    else return 1500 + difference;                                             //If the channel is not reversed
  }
  else return 1500;
}

/**
 * @brief Prints the signals.
 * 
 */
void printSignals(){
  Serial.print("Start:");
  Serial.print(start);

  Serial.print("  Throttle:");
  if(receiverInputChannel3 - 1480 < 0)Serial.print("vvv");
  else if(receiverInputChannel3 - 1520 > 0)Serial.print("^^^");
  else Serial.print("-+-");
  Serial.print(receiverInputChannel3);

  Serial.print("  Yaw:");
  if(receiverInputChannel4 - 1480 < 0)Serial.print("<<<");
  else if(receiverInputChannel4 - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.println(receiverInputChannel4);
  
  Serial.print("  Roll:");
  if(receiverInputChannel1 - 1480 < 0)Serial.print("<<<");
  else if(receiverInputChannel1 - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.print(receiverInputChannel1);

  Serial.print("  Pitch:");
  if(receiverInputChannel2 - 1480 < 0)Serial.print("^^^");
  else if(receiverInputChannel2 - 1520 > 0)Serial.print("vvv");
  else Serial.print("-+-");
  Serial.print(receiverInputChannel2);
}

/**
 * @brief Main routine used in the sketch.
 * 
 */
void rFunction(){
      loopCounter ++;                                                                    //Increase the loopCounter variable.
    receiverInputChannel1 = convertReceiverChannel(1);                           //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
    receiverInputChannel2 = convertReceiverChannel(2);                           //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
    receiverInputChannel3 = convertReceiverChannel(3);                           //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
    receiverInputChannel4 = convertReceiverChannel(4);                           //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.

    if(loopCounter == 125){                                                            //Print the receiver values when the loopCounter variable equals 250.
      printSignals();                                                                  //Print the receiver values on the serial monitor.
      loopCounter = 0;                                                                 //Reset the loopCounter variable.
    }

    //For starting the motors: throttle low and yaw left (step 1).
    if(receiverInputChannel3 < 1050 && receiverInputChannel4 < 1050)start = 1;
    //When yaw stick is back in the center position start the motors (step 2).
    if(start == 1 && receiverInputChannel3 < 1050 && receiverInputChannel4 > 1450)start = 2;
    //Stopping the motors: throttle low and yaw right.
    if(start == 2 && receiverInputChannel3 < 1050 && receiverInputChannel4 > 1950)start = 0;

    esc_1 = 1000;                                                                       //Set the pulse for ESC 1 to 1000us.
    esc_2 = 1000;                                                                       //Set the pulse for ESC 1 to 1000us.
    esc_3 = 1000;                                                                       //Set the pulse for ESC 1 to 1000us.
    esc_4 = 1000;                                                                       //Set the pulse for ESC 1 to 1000us.
    escPulseOutput();                                                                 //Send the ESC control pulses.
}
