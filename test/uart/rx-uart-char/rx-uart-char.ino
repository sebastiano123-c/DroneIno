//Receive with start- and end-markers
#include <string>


const byte numChars = 64;
char receivedChars[numChars];

boolean newData = false;
HardwareSerial SUART(1);

int timeDelay = 2000;

void setup() {
    Serial.begin(115200);
    Serial.println("<Arduino is ready>");

    SUART.begin(115200, SERIAL_8N1, 16, 17);
}

void loop() {
    receiveWithStartEndMarkers();
    showNewData();

    delay(timeDelay);
}

void receiveWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    const char startMarker = '<';
    const char endMarker = '>';
    char rc;
 
    while (SUART.available() > 0 && newData == false) {
        rc = SUART.read();

        switch (recvInProgress) {

          case true:
            switch (rc) {
              case endMarker:              // message terminates "<abcdef>"
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
                break;

//               case startMarker:           // message is corrupted "<abc<efg"
//                 if(ndx != 0){
//                    char receivedChars[numChars];
//                    return;
//                 }
//                 break;
                
              default:
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }

           break;

           default:
            recvInProgress = true;
        }
    }
}

void showNewData() {    
    if (newData == true) {
        Serial.print("This just in ... ");
        Serial.println(receivedChars);
        newData = false;
    }
}
