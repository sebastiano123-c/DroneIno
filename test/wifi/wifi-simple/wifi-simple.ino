/*
  WiFiTelemetry
  @author @sebastiano123-c
*/

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include<HardwareSerial.h>


// serial
HardwareSerial SUART(1); 

float angleRoll = 1;
float anglePitch = 2;
float flightMode = 3;
float batteryPercentage = 4;
float altitudeMeasure = 5;

int delayTime = 200; // ms

const int dataTransferSize = 5;
float dataTransfer[dataTransferSize];


// server
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

// wifi
const char *ssid = "DroneInoTelemetry";
const char *password = "DroneIno";

// roll
const char* P_ROLL_GET = "rollP";
const char* I_ROLL_GET = "rollI";
const char* D_ROLL_GET = "rollD";

// pitch
const char* P_PITCH_GET = "pitchP";
const char* I_PITCH_GET = "pitchI";
const char* D_PITCH_GET = "pitchD";

// yaw
const char* P_YAW_GET = "yawP";
const char* I_YAW_GET = "yawI";
const char* D_YAW_GET = "yawD";

// telemetry
const char* TELEMETRY_PITCH_ANGLE = "pitchAngle";
const char* TELEMETRY_ROLL_ANGLE = "rollAngle";
const char* TELEMETRY_FLIGHT_MODE = "flightMode";

// vars
int i = 0;

// timer variables
unsigned long lastTime = 0;  
unsigned long lastTime1 = 0; 
unsigned long timerDelay = 500;

// HTML web page to handle 3 input fields (input1, input2, input3)
const char* index_html = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>DroneInoTelemetry</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    html {
      font-family: Arial; 
      display: inline-block; 
      text-align: center;
    }
    p { 
      font-size: 1.2rem;
    }
    body {  
      margin: 0;
    }
    .topnav { 
      overflow: hidden; 
      background-color: #50B8B4; 
      color: white; 
      font-size: 1rem; 
    }
    .content { 
      padding: 20px; 
    }
    .card { 
      background-color: white; 
      box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5); 
    }
    .cards { 
      max-width: 800px; 
      margin: 0 auto; 
      display: grid; 
      grid-gap: 2rem; 
      grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); }
    .reading { 
      font-size: 1.4rem;  
    }
  </style>
  </head>
  <body>
  <div class="topnav">
    <h1>DroneInoTelemetry</h1>
  </div>
  <div class="content">
    <div class="cards">
      <div class="card">
        <p> pitch</p>
        <p><span class="reading"><span id="anglePitch">%PITCHANGLE%</span> &deg;C</span></p>
      </div>
      <div class="card">
        <p> roll</p>
        <p><span class="reading"><span id="angleRoll">%ROLLANGLE%</span> &deg;</span></p>
      </div>
      <div class="card">
        <p> flight mode</p>
        <p><span class="reading"><span id="flightMode">%FLIGHTMODE%</span></span></p>
      </div>
      <div class="card">
        <p> battery</p>
        <p><span class="reading"><span id="battery">%BATTERY%</span></span></p>
      </div>
      <div class="card">
        <p> altitude</p>
        <p><span class="reading"><span id="altitude">%ALTITUDE%</span></span></p>
      </div>
    </div>
  </div><br>
<!--h1 >PID values</h1>
  <h2 >  Roll</h2>
  <form action="/get">
    P: <input type="text" name="rollP">
    <input type="submit" value="Submit">
  </form><br>
  <form action="/get">
    I: <input type="text" name="rollI">
    <input type="submit" value="Submit">
  </form><br>
  <form action="/get">
    D: <input type="text" name="rollD">
    <input type="submit" value="Submit">
  </form><br><hl>
    <h2 >  Pitch</h2>
  <form action="/get">
    P: <input type="text" name="pitchP">
    <input type="submit" value="Submit">
  </form><br>
  <form action="/get">
    I: <input type="text" name="pitchI">
    <input type="submit" value="Submit">
  </form><br>
  <form action="/get">
    D: <input type="text" name="pitchD">
    <input type="submit" value="Submit">
  </form><br><hl>
    <h2 >  Yaw</h2>
  <form action="/get">
    P: <input type="text" name="yawP">
    <input type="submit" value="Submit">
  </form><br>
  <form action="/get">
    I: <input type="text" name="yawI">
    <input type="submit" value="Submit">
  </form><br>
  <form action="/get">
    D: <input type="text" name="yawD">
    <input type="submit" value="Submit">
  </form><br><hl-->
   <script  type="text/javascript">
    if (!!window.EventSource) {
      var source = new EventSource('/events');
      source.addEventListener('open', function(e) {
        console.log("Events Connected");
      }, false);
      source.addEventListener('error', function(e) {
        if (e.target.readyState != EventSource.OPEN) {
          console.log("Events Disconnected");
        }
      }, false);
      source.addEventListener('message', function(e) {
        //console.log("message", e.data);
      }, false);
      source.addEventListener('anglePitch', function(e) {
        //console.log("anglePitch", e.data);
        document.getElementById("anglePitch").innerHTML = e.data;
      }, false);
      source.addEventListener('angleRoll', function(e) {
        //console.log("angleRoll", e.data);
        document.getElementById("angleRoll").innerHTML = e.data;
      }, false);
      source.addEventListener('flightMode', function(e) {
        //console.log("flightMode", e.data);
        document.getElementById("flightMode").innerHTML = e.data;
      }, false);
      source.addEventListener('battery', function(e) {
        //console.log("battery", e.data);
        document.getElementById("battery").innerHTML = e.data;
      }, false);
      source.addEventListener('altitude', function(e) {
        //console.log("altitude", e.data);
        document.getElementById("altitude").innerHTML = e.data;
      }, false);
    }
</script>
</body></html>)rawliteral";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

String processor(const String& var){
  readDataTransfer();
  //Serial.println(var);
  if(var == "PITCHANGLE"){
    return String(anglePitch);
  }
  else if(var == "ROLLANGLE"){
    return String(angleRoll);
  }
  else if(var == "FLIGHTMODE"){
    return String(flightMode);
  }
  else if(var == "BATTERY"){
    return String(batteryPercentage);
  }
  else if(var == "ALTITUDE"){
    return String(altitudeMeasure);
  }
}

void setup(){

  // SERIAL
  Serial.begin(115200);
  SUART.begin(115200, SERIAL_8N1, 16, 17);


  // WIFI
  WiFi.softAP(ssid, password);
  
  Serial.println();
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());
 
  // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });
//  
//  // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
//  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
//    String inputMessage;
//    String inputParam;
//    float inputValue;
//    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
//    if (request->hasParam(P_ROLL_GET)) {
//      inputMessage = request->getParam(P_ROLL_GET)->value();
//      inputParam = P_ROLL_GET;
//    }
//    // GET input2 value on <ESP_IP>/get?input2=<inputMessage>
//    else if (request->hasParam(I_ROLL_GET)) {
//      inputMessage = request->getParam(I_ROLL_GET)->value();
//      inputParam = I_ROLL_GET;
//    }
//    // GET input3 value on <ESP_IP>/get?input3=<inputMessage>
//    else if (request->hasParam(D_ROLL_GET)) {
//      inputMessage = request->getParam(D_ROLL_GET)->value();
//      inputParam = D_ROLL_GET;
//    }
//    // picth
//    else if (request->hasParam(P_PITCH_GET)) {
//      inputMessage = request->getParam(P_PITCH_GET)->value();
//      inputParam = P_PITCH_GET;
//    }
//    else if (request->hasParam(I_PITCH_GET)) {
//      inputMessage = request->getParam(I_PITCH_GET)->value();
//      inputParam = I_PITCH_GET;
//    }
//    else if (request->hasParam(D_PITCH_GET)) {
//      inputMessage = request->getParam(D_PITCH_GET)->value();
//      inputParam = D_PITCH_GET;
//    }
//    // yaw
//    else if (request->hasParam(P_YAW_GET)) {
//      inputMessage = request->getParam(P_YAW_GET)->value();
//      inputParam = P_YAW_GET;
//    }
//    else if (request->hasParam(I_YAW_GET)) {
//      inputMessage = request->getParam(I_YAW_GET)->value();
//      inputParam = I_YAW_GET;
//    }
//    else if (request->hasParam(D_YAW_GET)) {
//      inputMessage = request->getParam(D_YAW_GET)->value();
//      inputParam = D_YAW_GET;
//    }
//    else {
//      inputMessage = "No message sent";
//      inputParam = "none";
//    }
//    request->send_P(200, "text/html", index_html, processor);
//                                     
//  });

  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);

  server.onNotFound(notFound);
  server.begin();
}
 
void loop(){
//    Serial.printf("pitchAngle = %.2f ºC \n", pitchAngle);
//    Serial.printf("rollAngle = %.2f \n", rollAngle);
//    Serial.printf("flightMode = %i hPa \n", flightMode);
//    Serial.println();

  sendTelemetry();

  delay(delayTime);
  
}

void sendTelemetry(){
  // Send Events to the Web Server with the Sensor Readings
  readDataTransfer();

  events.send("ping",NULL,millis());

  events.send(String(anglePitch).c_str(),"anglePitch",millis());
  events.send(String(angleRoll).c_str(),"angleRoll",millis());
  events.send(String(flightMode).c_str(),"flightMode",millis());
  events.send(String(batteryPercentage).c_str(),"battery",millis());
  events.send(String(altitudeMeasure).c_str(),"altitudeMeasure",millis());
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
 // checkMessage();
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
 angleRoll = dataTransfer[0];
 anglePitch = dataTransfer[1];
 flightMode = dataTransfer[2];
 batteryPercentage = dataTransfer[3];
 altitudeMeasure = dataTransfer[4];

  // print
 //checkMessage();
}
