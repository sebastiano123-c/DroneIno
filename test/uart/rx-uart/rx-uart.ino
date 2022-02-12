#include<HardwareSerial.h>

HardwareSerial SUART(1); 

float angleRoll = 1;
float anglePitch = 2;
float flightMode = 3;
float batteryPercentage = 4;
float altitudeMeasure = 5;

int delayTime = 3000; // ms

const int dataTransferSize = 5;
float dataTransfer[dataTransferSize];

void setup()
{
  Serial.begin(115200);
  SUART.begin(115200, SERIAL_8N1, 16, 17);
  delay(30);
}

void loop()
{

  angleRoll = angleRoll + 1;
  anglePitch = anglePitch + 1.3;

  writeDataTransfer();
  
  if(SUART.available()){
    readDataTransfer();
  }

 delay(delayTime);
}

void writeDataTransfer(){

  Serial.printf("I'm sending...\n");

  // fill data structure before send
  dataTransfer[0] = angleRoll;
  dataTransfer[1] = anglePitch;
  dataTransfer[2] = flightMode;
  dataTransfer[3] = batteryPercentage;
  dataTransfer[4] = altitudeMeasure;

//  // clear output
//  while(Serial.available()){
//    SUART.read();
//  }
  
  // print in csv format
  int i=0;
  for(i = 0; i < dataTransferSize - 1; i++){
    SUART.printf("%.6f,", dataTransfer[i]);
  }
  SUART.printf("%.6f\n", dataTransfer[dataTransferSize - 1]);

  // print
  checkMessage();
}


void readDataTransfer(){

  // declair index array
  int indices[dataTransferSize-1];
  String str = "";
  String strLast = "x";
  
  // read from serial
  Serial.printf("I'm reading...\n");

//  // tx can be faster, thus serial has many lines. This catches the last one
//  while(strLast.length() != 0){                     // Last is zero iff there are no lines
//    str = strLast;                                  // str is the last before the while condition is false
//    strLast = SUART.readStringUntil('\n');          // this get the following line strLast
//    Serial.println(strLast);
//  }
//  Serial.println(str);
  str = SUART.readStringUntil('\n');

  // find positions of ","
  //Serial.println("indices...");
  int i = 0;
  indices[i] = str.indexOf(',');
  for( i = 1; i < dataTransferSize-1; i++){
    indices[i] = str.indexOf(',', indices[i-1]+1);
  }

  // substring the data and convert it to floats
  i = 0;
  dataTransfer[i] = str.substring(0, indices[i]).toFloat();
  //Serial.printf("%i: %f \n", i, dataTransfer[i]);
  for( i = 1; i < dataTransferSize - 1; i++){
    dataTransfer[i] = str.substring(indices[i-1] + 1, indices[i]).toFloat();
    //Serial.printf("%i: %f \n", i, dataTransfer[i]);
  }
  dataTransfer[dataTransferSize - 1] = str.substring(indices[dataTransferSize - 2] +1 ).toFloat();
  //Serial.printf("%i: %f %s \n", indices[dataTransferSize - 2], dataTransfer[dataTransferSize - 1], str.substring(39+1 ));

  // fill data structure after reaceiving
//  angleRoll = dataTransfer[0];
//  anglePitch = dataTransfer[1];
//  flightMode = dataTransfer[2];
//  batteryPercentage = dataTransfer[3];
//  altitudeMeasure = dataTransfer[4];

  // print
  checkMessage();
}


void checkMessage(){
  // print in csv format   
  for(int i = 0; i < dataTransferSize; i++){
    Serial.printf("%.6f\n", dataTransfer[i]);
  }
 
}
