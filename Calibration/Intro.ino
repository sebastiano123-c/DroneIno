void intro(){

  delay(500);

  Serial.println("DroneIno Calibration\n");
  
   Serial.println(" Dial these on the serial:");
   Serial.println(  " r: print receiver signals.");
   Serial.println(  " a: print quadcopter angles.");
   Serial.println(  " e: print EEPROM.");
   Serial.println(  " 1: check rotation / vibrations for motor 1 (right front CCW).");
   Serial.println(  " 2: check rotation / vibrations for motor 2 (right rear CW).");
   Serial.println(  " 3: check rotation / vibrations for motor 3 (left rear CCW).");
   Serial.println(  " 4: check rotation / vibrations for motor 4 (left front CW).");
   Serial.println(  " 5: check vibrations for all motors together.");
   Serial.println();
}

void setPins(){
  pinMode(PIN_ESC_1, OUTPUT);
  pinMode(PIN_ESC_2, OUTPUT);
  pinMode(PIN_ESC_3, OUTPUT);
  pinMode(PIN_ESC_4, OUTPUT);
  pinMode(PIN_BATTERY_LED, OUTPUT);
  pinMode(PIN_DIGITAL_13, OUTPUT);

  //event change detector
  pinMode(PIN_RECEIVER_1, INPUT_PULLUP);
  pinMode(PIN_RECEIVER_2, INPUT_PULLUP);
  pinMode(PIN_RECEIVER_3, INPUT_PULLUP);
  pinMode(PIN_RECEIVER_4, INPUT_PULLUP);
  
  //       event change detector
  attachInterrupt(digitalPinToInterrupt(PIN_RECEIVER_1), myISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RECEIVER_2), myISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RECEIVER_3), myISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RECEIVER_4), myISR, CHANGE);
}


void printEEPROM(){
   //print eeprom data on serial
  Serial.println("==================================");
  Serial.println("EEPROM data:");
  Serial.println("");
  for (int i = 0; i < EEPROM_SIZE; i++){
    Serial.println(eepromData[i]);
  }
  data = 0;
}
