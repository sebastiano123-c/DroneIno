
void sendTelemetry(){
 /* 
  * @brief Send Events to the Web Server with the Sensor Readings
  * @note refresh rate browser is about 20 Hertz (i.e. they refresh the screen about 20 times per second) - i.e. every 50 milliseconds
  */

  if(SUART.available()){
    readDataTransfer();
  }

  events.send("ping",NULL,millis());

  events.send(String(anglePitch).c_str(),"anglePitch",millis());
  events.send(String(angleRoll).c_str(),"angleRoll",millis());
  events.send(String(flightMode).c_str(),"flightMode",millis());
  events.send(String(batteryPercentage).c_str(),"battery",millis());
  events.send(String(altitudeMeasure).c_str(),"altitude",millis());
}



void writeDataTransfer(){

  // fill data structure before send
  dataController[0] = PID_P_GAIN_ROLL;
  dataController[1] = PID_I_GAIN_ROLL;
  dataController[2] = PID_D_GAIN_ROLL;
  dataController[3] = PID_P_GAIN_PITCH;
  dataController[4] = PID_I_GAIN_PITCH;
  dataController[5] = PID_D_GAIN_PITCH;
  dataController[6] = PID_P_GAIN_YAW;
  dataController[7] = PID_I_GAIN_YAW;
  dataController[8] = PID_D_GAIN_YAW;
  
  // print in csv format
  int i=0;
  for(i = 0; i < dataControllerSize - 1; i++){
    SUART.printf("%.6f,", dataController[i]);
  }
  SUART.printf("%.6f\n", dataController[dataControllerSize - 1]);

}


void readDataTransfer(){

  // declair index array
  int indices[dataTransferSize-1];
  String str = "";
  
  // read from serial
  //Serial.printf("I'm reading...\n");
  str = SUART.readStringUntil('\n');
  //Serial.println(str);

  // find posistion of the last > and the last <
  int posStart = str.lastIndexOf('<') + 1;
  
  // find positions of ","
  int i = 0;
  indices[i] = str.indexOf(',', posStart);
  for( i = 1; i < dataTransferSize - 1; i++){
    indices[i] = str.indexOf(',', indices[i-1]+1);
  }

  // substring the data and convert it to floats
  i = 0;
  dataTransfer[i] = str.substring(posStart, indices[i]).toFloat();
  for( i = 1; i < dataTransferSize - 1; i++){
    dataTransfer[i] = str.substring(indices[i-1] + 1, indices[i]).toFloat();
  }
  dataTransfer[dataTransferSize - 1] = str.substring(indices[dataTransferSize - 2] + 1 ).toFloat();
 
  // fill data structure after reaceiving
  angleRoll = dataTransfer[0];
  anglePitch = dataTransfer[1];
  flightMode = dataTransfer[2];
  batteryPercentage = dataTransfer[3];
  altitudeMeasure = dataTransfer[4];

//  // print in csv format   
//  for(int i = 0; i < dataTransferSize; i++){
//    Serial.printf("%.6f\n", dataTransfer[i]);
//  }
}