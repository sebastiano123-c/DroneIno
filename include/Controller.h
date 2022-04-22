/**
 * @file Controller.h
 * @author @sebastiano123-c
 * @brief Radiocommander control routines.
 * 
 * The radio controller, for example the FlySky, works as PWM signals with variable time widths in the range 1000-2000us.
 * The convertReceiverChannel() routine converts the input signal into the range 1000-2000us.
 * waitController() routine is called in the setup() function and waits until DroneIno recognizes a readable radio controller input.
 * 
 * 
 * 
 * 
 * 
 * 
 * @version 0.1
 * @date 2022-02-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */



/**
 * @brief Converts the receiver input signal.
 * 
 * @param ch 
 * @return int
 */
int convertReceiverChannel(byte ch){

  int difference;

  if(trimCh[ch].actual < trimCh[ch].center){            //The actual receiver value is lower than the center value
    //Limit the lowest value to the value that was detected during setup
    if(trimCh[ch].actual < trimCh[ch].low)trimCh[ch].actual = trimCh[ch].low;

    //Calculate the scale of the actual value to a 1000 - 2000us value
    difference = ((long)(trimCh[ch].center - trimCh[ch].actual) * (long)500) / (trimCh[ch].center - trimCh[ch].low);
    
    switch (trimCh[ch].reverse){
      case 1: // reversed
        return 1500 + difference;
        break;
      
      default: // NOT reversed
        return 1500 - difference;
    }
  }
  else if(trimCh[ch].actual > trimCh[ch].center){     //The actual receiver value is higher than the center value

    //Limit the lowest value to the value that was detected during setup
    if(trimCh[ch].actual > trimCh[ch].high)trimCh[ch].actual = trimCh[ch].high;

    //Calculate and scale the actual value to a 1000 - 2000us value
    difference = ((long)(trimCh[ch].actual - trimCh[ch].center) * (long)500) / (trimCh[ch].high - trimCh[ch].center);
    
    switch (trimCh[ch].reverse){
      case 1:
        return 1500 - difference;
        break;
      
      default:
        return 1500 + difference;
    } 
  }
  else return 1500;
}


/** 
 * @brief Wait until the receiver is active and the throttle is set to the lower position.
 */
void waitController(){

  #if UPLOADED_SKETCH == SETUP
    byte zero = 0;
    unsigned long timeLast = 20; // s
    Serial.printf("Wait for receiver, you have %lus from now ", timeLast);
    Serial.println(F("Please, check that the Receiver has at least 3.3V..."));
    timer = millis() + timeLast*1000;
    while(timer > millis() && zero < 15){
      if(trimCh[1].actual < 2100 && trimCh[1].actual > 900) zero |= 0b00000001;
      if(trimCh[2].actual < 2100 && trimCh[2].actual > 900) zero |= 0b00000010;
      if(trimCh[3].actual < 2100 && trimCh[3].actual > 900) zero |= 0b00000100;
      if(trimCh[4].actual < 2100 && trimCh[4].actual > 900) zero |= 0b00001000;
      delay(500);
      Serial.print(F("."));
    }
    if(zero == 0){
      error = 1;
      Serial.println(F("."));
      Serial.println(F("No valid receiver signals found!!! (ERROR 1)"));
    }
    else Serial.printf(" OK (zero=%i)\n", zero);

  #else

    while(receiverInputChannel3 < 980 && receiverInputChannel3 > 1020 && receiverInputChannel4 < 1400)
    {
  
      receiverInputChannel3 = convertReceiverChannel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
      receiverInputChannel4 = convertReceiverChannel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
      
        start ++;                                                          //While waiting increment start with every loop.

        switch (start)
        {
        case 125:

          ledcWrite(pwmLedChannel, MAX_DUTY_CYCLE);                        //Change the led status to indicate calibration.
          start = 0;  
          break;
        
        default:
          ledcWrite(pwmLedChannel, 0);
          break;
        }
      
    }   
  #endif
}
