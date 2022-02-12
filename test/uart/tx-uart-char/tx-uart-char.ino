//Receive with start- and end-markers

const byte numChars = 64;
char receivedChars[numChars];

boolean newData = false;
HardwareSerial SUART(1);

float angleRoll = 1;
float anglePitch = 2;
float flightMode = 3;
float batteryPercentage = 4;
float altitudeMeasure = 5;

int timeDelay = 4; // ms

// telemetry
const int dataTransferSize = 5;
float dataTransfer[dataTransferSize];

// controller
const int dataControllerSize = 9;
float dataController[dataControllerSize];



void setup() {
    Serial.begin(115200);
    Serial.println("<Arduino is ready>");

    SUART.begin(115200, SERIAL_8N1, 16, 17);

    delay(30);
}

void loop() {

  angleRoll = angleRoll+ 0.1;
  
    writeDataTransfer();
    
    delay(timeDelay);

}

void writeDataTransfer(){

  Serial.printf("I'm sending...\n");

  // fill data structure before send
  dataTransfer[0] = angleRoll;
  dataTransfer[1] = anglePitch;
  dataTransfer[2] = flightMode;
  dataTransfer[3] = batteryPercentage;
  dataTransfer[4] = altitudeMeasure;
  
  // print in csv format
  if(SUART.available()){
    int i=0;
    String str;
    str = "<" + String(dataTransfer[i]) + ",";
    //SUART.printf("<%.6f,", dataTransfer[i]);
    for(i = 1; i < dataTransferSize - 1; i++){
      //SUART.printf("%.6f,", dataTransfer[i]);
      str = str + String(dataTransfer[i]) + ",";
    }
    //SUART.printf("%.6f>", dataTransfer[dataTransferSize - 1]);
    str = str + String(dataTransfer[i]) + ">";
    SUART.println(str);
  }
  
  // print
  checkMessage();
}

void checkMessage(){
  // print in csv format   
  for(int i = 0; i < dataTransferSize; i++){
    Serial.printf("%.6f\n", dataTransfer[i]);
  }
 
}
