#include<HardwareSerial.h>

HardwareSerial SUART(1); 

float angleRoll = 1;
float anglePitch = 2;
float flightMode = 3;
float batteryPercentage = 4;
float altitudeMeasure = 5;

int delayTime = 4; // ms
int sendTelemetryLoopN = 0;

// telemetry
const int dataTransferSize = 5;
float dataTransfer[dataTransferSize];

// controller
const int dataControllerSize = 9;
float dataController[dataControllerSize];

// PID:
//              (ROLL)
float PID_P_GAIN_ROLL            = 1.3;                      //Gain setting for the roll P-controller (1.3)
float PID_I_GAIN_ROLL            = 0.001;                  //Gain setting for the roll I-controller  (0.0002)
float PID_D_GAIN_ROLL            = 8.0;                     //Gain setting for the roll D-controller (10.0)

//              (PITCH)
float PID_P_GAIN_PITCH           = PID_P_GAIN_ROLL;          //Gain setting for the pitch P-controller
float PID_I_GAIN_PITCH           = PID_I_GAIN_ROLL;          //Gain setting for the pitch I-controller
float PID_D_GAIN_PITCH           = PID_D_GAIN_ROLL;          //Gain setting for the pitch D-controller

//              (YAW)
float PID_P_GAIN_YAW             = 2.2;                      //Gain setting for the pitch P-controller. (2.0)
float PID_I_GAIN_YAW             = 0.01;                     //Gain setting for the pitch I-controller. (0.04)
float PID_D_GAIN_YAW             = 0.0;                      //Gain setting for the pitch D-controller. (0.0)

void setup()
{
  Serial.begin(115200);
  SUART.begin(115200, SERIAL_8N1, 26, 25);
  delay(30);
}

void loop()
{
  angleRoll = angleRoll + 0.01;
  anglePitch = anglePitch + 0.013;

  switch(sendTelemetryLoopN){
    case 1:                      // sendTelemetryLoopN * 4 ms
      writeDataTransfer();
      sendTelemetryLoopN = 0;
      break;
    default:
      sendTelemetryLoopN++;
  }
  

  if(SUART.available()){
    readDataTransfer();
  }
  
 delay(delayTime);
}

void writeDataTransfer(){

  //Serial.printf("I'm sending...\n");

  // fill data structure before send
  dataTransfer[0] = angleRoll;
  dataTransfer[1] = anglePitch;
  dataTransfer[2] = flightMode;
  dataTransfer[3] = batteryPercentage;
  dataTransfer[4] = altitudeMeasure;
  
  // print in csv format
  int i=0;
  SUART.printf("<%.6f,", dataTransfer[i]);
  for(i = 1; i < dataTransferSize - 1; i++){
    SUART.printf("%.6f,", dataTransfer[i]);
  }
  SUART.printf("%.6f>\n", dataTransfer[dataTransferSize - 1]);

  // print
  //checkMessage();
}


void readDataTransfer(){

  // declair index array
  int indices[dataControllerSize-1];
  String str;
  
  // read from serial
  Serial.printf("I'm reading...\n");
  str = SUART.readStringUntil('\n');

  // find positions of ","
  //Serial.println("indices...");
  int i = 0;
  indices[i] = str.indexOf(',');
  for( i = 1; i < dataControllerSize-1; i++){
    indices[i] = str.indexOf(',', indices[i-1]+1);
  }

  // substring the data and convert it to floats
  i = 0;
  dataController[i] = str.substring(0, indices[i]).toFloat();
  //Serial.printf("%i: %f \n", i, dataController[i]);
  for( i = 1; i < dataControllerSize - 1; i++){
    dataController[i] = str.substring(indices[i-1] + 1, indices[i]).toFloat();
    //Serial.printf("%i: %f \n", i, dataController[i]);
  }
  dataController[dataControllerSize - 1] = str.substring(indices[dataControllerSize - 2] +1 ).toFloat();
  //Serial.printf("%i: %f %s \n", indices[dataControllerSize - 2], dataController[dataControllerSize - 1], str.substring(39+1 ));

  // fill data structure after receiving
  PID_P_GAIN_ROLL = dataController[0];
  PID_I_GAIN_ROLL = dataController[1];
  PID_D_GAIN_ROLL = dataController[2];
  PID_P_GAIN_PITCH = dataController[3];
  PID_I_GAIN_PITCH = dataController[4];
  PID_D_GAIN_PITCH = dataController[5];
  PID_P_GAIN_YAW = dataController[6];
  PID_I_GAIN_YAW = dataController[7];
  PID_D_GAIN_YAW = dataController[8];

  // print in csv format   
  for(int i = 0; i < dataControllerSize; i++){
    Serial.printf("%.6f\n", dataController[i]);
  }
}


void checkMessage(){
  // print in csv format   
  for(int i = 0; i < dataTransferSize; i++){
    Serial.printf("%.6f\n", dataTransfer[i]);
  }
}
