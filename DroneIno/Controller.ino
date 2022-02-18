/** 
 * @file Controller.ino
 * @author @sebastiano123-c
 */

void waitController(){
  /* 
  * @brief wait until the receiver is active and the throttle is set to the lower position
  */

  while(receiverInputChannel3 < 990 || receiverInputChannel3 > 1020 || receiverInputChannel4 < 1400)
  {
    receiverInputChannel3 = convertReceiverChannel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
    receiverInputChannel4 = convertReceiverChannel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us

    start ++;                                                          //While waiting increment start whith every loop.

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
}

int convertReceiverChannel(byte ch){
  /* 
  * @brief convert the input RX signal
  * @param ch channel that has to be converted
  */
  int difference;

  if(trimCh[ch].actual < trimCh[ch].center){            //The actual receiver value is lower than the center value
    //Limit the lowest value to the value that was detected during setup
    if(trimCh[ch].actual < trimCh[ch].low)trimCh[ch].actual = trimCh[ch].low;

    //Calculate and scale theactual value to a 1000 - 2000us value
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