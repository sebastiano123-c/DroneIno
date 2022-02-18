/** 
 * ISR
 * @author @sebastiano123-c
 */

void myISR(){
  /** 
   * @brief measures the receiver input signal length 
   * @todo use pwm to read the receivers signals, avoid digitalRead
   */

  currentTime = micros();

  // channel 1                                               
  if(digitalRead(PIN_RECEIVER_1) == HIGH){                                  //Is RECEIVER input high?
      if(lastChannel1 == 0){                                                //Input 8 changed from 0 to 1.
          lastChannel1 = 1;                                                   //Remember current input state.
          timer1 = currentTime;                                               //Set timer1 to currentTime.
      }
  }
  else if(lastChannel1 == 1){                                             //Input 8 is not high and changed from 1 to 0.
      lastChannel1 = 0;                                                     //Remember current input state.
      trimCh[1].actual = currentTime - timer1;                             //Channel 1 is currentTime - timer1.
  }


  // channel 2                                              
  if(digitalRead(PIN_RECEIVER_2) == HIGH){                                 //Is RECEIVER input high?
      if(lastChannel2 == 0){                                                //Input 9 changed from 0 to 1.
          lastChannel2 = 1;                                                   //Remember current input state.
          timer2 = currentTime;                                               //Set timer2 to currentTime.
      }
  }
  else if(lastChannel2 == 1){                                             //Input 9 is not high and changed from 1 to 0.
      lastChannel2 = 0;                                                     //Remember current input state.
      trimCh[2].actual = currentTime - timer2;                             //Channel 2 is currentTime - timer2.
  }


  // channel 3                                              
  if(digitalRead(PIN_RECEIVER_3) == HIGH){                                 //Is RECEIVER input high?
      if(lastChannel3 == 0){                                                //Input 10 changed from 0 to 1.
          lastChannel3 = 1;                                                   //Remember current input state.
          timer3 = currentTime;                                               //Set timer3 to currentTime.
      }
  }
  else if(lastChannel3 == 1){                                             //Input 10 is not high and changed from 1 to 0.
      lastChannel3 = 0;                                                     //Remember current input state.
      trimCh[3].actual = currentTime - timer3;                             //Channel 3 is currentTime - timer3.
  }


  // channel 4                                              
  if(digitalRead(PIN_RECEIVER_4) == HIGH){                                 //Is RECEIVER input high?
      if(lastChannel4 == 0){                                                //Input 11 changed from 0 to 1.
          lastChannel4 = 1;                                                   //Remember current input state.
          timer4 = currentTime;                                               //Set timer4 to currentTime.
      }
  }
  else if(lastChannel4 == 1){                                             //Input 11 is not high and changed from 1 to 0.
      lastChannel4 = 0;                                                     //Remember current input state.
      trimCh[4].actual = currentTime - timer4;                             //Channel 4 is currentTime - timer4.
  }

  
  // channel 5                                              
  if(digitalRead(PIN_RECEIVER_5) == HIGH){                          //Is RECEIVER input high?
      if(lastChannel5 == 0){                                                //Input 11 changed from 0 to 1.
          lastChannel5 = 1;                                                   //Remember current input state.
          timer5 = currentTime;                                               //Set timer4 to currentTime.
      }
  }
  else if(lastChannel5 == 1){                                             //Input 11 is not high and changed from 1 to 0.
      lastChannel5 = 0;                                                     //Remember current input state.
      trimCh[5].actual = currentTime - timer5;                             //Channel 4 is currentTime - timer4.
  }
}
