#include "telemetry.h"

const int dataControllerSize = 12;
float dataController[dataControllerSize];

const int dataTransferSize = 5;
float dataTransfer[dataTransferSize];

// serial
HardwareSerial SUART(1); 

void beginUARTCOM(){
    SUART.begin(115200, SERIAL_8N1, 3, 1); // RX, TX
}

void updatePID(){
  // fill data structure before send
  dataController[0] = PID_P_GAIN_ROLL;
  dataController[1] = PID_I_GAIN_ROLL;
  dataController[2] = PID_D_GAIN_ROLL;
  dataController[3] = PID_P_GAIN_YAW;
  dataController[4] = PID_I_GAIN_YAW;
  dataController[5] = PID_D_GAIN_YAW;
  dataController[6] = GYROSCOPE_ROLL_FILTER;
  dataController[7] = GYROSCOPE_ROLL_CORR;
  dataController[8] = GYROSCOPE_PITCH_CORR;
  dataController[9] = PID_P_GAIN_ALTITUDE;
  dataController[10] = PID_I_GAIN_ALTITUDE;
  dataController[11] = PID_D_GAIN_ALTITUDE;
}

void updateTelemetry(){
  // fill data structure after receiving
  rollAngle = dataTransfer[0];
  pitchAngle = dataTransfer[1];
  flightMode = dataTransfer[2];
  batteryPercentage = dataTransfer[3];
  altitudeMeasure = dataTransfer[4];
}

void writeDataTransfer(){

  //Serial.printf("I'm sending...\n"); 
  updatePID();

  // print in csv format
  int i=0;
  for(i = 0; i < dataControllerSize - 1; i++){
    SUART.printf("%.6f,", dataController[i]);
  }
  SUART.printf("%.6f\n", dataController[dataControllerSize - 1]);
}


void readDataTransfer(){
    if(SUART.available()){
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
        
        updateTelemetry();
        //  // print in csv format   
        //  for(int i = 0; i < dataTransferSize; i++){
        //    Serial.printf("%.6f\n", dataTransfer[i]);
        //  }
    }
}