/*
  WiFiTelemetry
  @author @sebastiano123-c
*/
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "esp_camera.h"
#include <WiFi.h>

#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#include "camera_pins.h"

// serial
HardwareSerial SUART(1); 

float angleRoll = 1;
float anglePitch = 2;
float flightMode = 3;
float batteryPercentage = 4;
float altitudeMeasure = 5;

int delayTime = 20; // ms

// telemetry
const int dataTransferSize = 5;
float dataTransfer[dataTransferSize];

// controller
const int dataControllerSize = 9;
float dataController[dataControllerSize];


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

// vars
int i = 0;

// timer variables
unsigned long lastTime           = 0;  
unsigned long timerDelay         = 500;

// server
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");


const char* index_html(){
  String indexHTMLHead = R"rawliteral(<!DOCTYPE HTML><html><head>
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
      max-width: 500px; 
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
    .pid-input{
      max-width: 50px;
    }
    .pid-label{
      background-color:teal;
      color: white;
    }
    </style>
  </head>)rawliteral";
  String indexHTMLBody = indexHTMLHead + R"rawliteral(<body>
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
      <div class="topnav">
          <h1>PID values</h1>
      </div>
      <div class="content">
          <div class="cards">
              <div class="card">
                  <p >  Pitch/Roll </p>
                  <form action="get">
                      P: <label class="pid-label" id="rollPVal">)rawliteral";
  String rollPInput = indexHTMLBody + String(PID_P_GAIN_ROLL, 4);
  String indexHTMLBody1 = rollPInput + R"rawliteral(</label><input class="pid-input" type="text" id="rollPInput" name="rollP">
                      <input type="submit" id="rollP" onclick="showInput(this);" value="set">
                  </form>
                  <form action="get">
                      I: <label class="pid-label" id="rollIVal">)rawliteral";
  String rollIInput = indexHTMLBody1 + String(PID_I_GAIN_ROLL, 4);
  String indexHTMLBody2 = rollIInput + R"rawliteral(</label><input class="pid-input" type="text" id="rollIInput" name="rollI">
                      <input type="submit" id="rollI" onclick="showInput(this);" value="set">
                  </form>
                  <form action="get">
                      D: <label class="pid-label" id="rollDVal">)rawliteral";
  String rollDInput = indexHTMLBody2 + String(PID_D_GAIN_ROLL, 4);
  String indexHTMLBody3 = rollDInput + R"rawliteral(</label><input class="pid-input" type="text" id="rollDInput" name="rollD">
                      <input type="submit" id="rollD" onclick="showInput(this);" value="set">
                  </form>
              </div>
              <!--div class="card">
                  <p >  Pitch </p>
                  <form action="get">
                      P: <label class="pid-label" id="pitchPVal">%PP%</label><input class="pid-input" type="text" id="pitchPInput" name="pitchP">
                      <input type="submit" id="pitchP" onclick="showInput(this);" value="set">
                  </form>
                  <form action="get">
                      I: <label class="pid-label" id="pitchIVal">%IP%</label><input class="pid-input" type="text" id="pitchIInput" name="pitchI">
                      <input type="submit" id="pitchI" onclick="showInput(this);" value="set">
                  </form>
                  <form action="get">
                      D: <label class="pid-label" id="pitchDVal">%DP%</label><input class="pid-input" type="text" id="pitchDInput" name="pitchD">
                      <input type="submit" id="pitchD" onclick="showInput(this);" value="set">
                  </form>
              </div-->
              <div class="card">
                  <p >  Yaw </p>
                  <form action="get">
                      P: <label class="pid-label" id="yawPVal">)rawliteral";
  String yawPInput = indexHTMLBody3 + String(PID_P_GAIN_YAW, 4);
  String indexHTMLBody4 = yawPInput + R"rawliteral(</label><input class="pid-input" type="text" id="yawPInput" name="yawP">
                      <input type="submit" id="yawP" onclick="showInput(this);" value="set">
                  </form>
                  <form action="get">
                      I: <label class="pid-label" id="yawIVal">)rawliteral";
  String yawIInput = indexHTMLBody4 + String(PID_I_GAIN_YAW, 4);
  String indexHTMLBody5 = yawIInput + R"rawliteral(</label><input class="pid-input" type="text" id="yawIInput" name="yawI">
                      <input type="submit" id="yawI" onclick="showInput(this);" value="set">
                  </form>
                  <form action="get">
                      D: <label class="pid-label" id="yawDVal">)rawliteral";
  String yawDInput = indexHTMLBody5 + String(PID_D_GAIN_YAW, 4);
  String indexHTMLBody6 = yawDInput + R"rawliteral(</label><input class="pid-input" type="text" id="yawDInput" name="yawD">
                      <input type="submit" id="yawD" onclick="showInput(this);" value="set">
                  </form>
              </div>
          </div>
      </div>
      <script  type="text/javascript">
        function showInput(elem) {
          document.getElementById(elem.id+"Val").innerHTML = document.getElementById(elem.id+"Input").value;
        }

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
  
  return indexHTMLBody6.c_str();
}

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

String processor(const String& var){
   if(SUART.available()){
    readDataTransfer();
  }
  
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
  // serial
  //Serial.begin(115200);
  SUART.begin(115200, SERIAL_8N1, 3, 1); // RX TX

  // camera
    camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    //Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

  // wifi
  WiFi.softAP(ssid, password);

//  Serial.println();
//  Serial.print("IP address: ");
//  Serial.println(WiFi.softAPIP());
 
   // Send web page with input fields to client
   server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
     request->send_P(200, "text/html", index_html(), processor);
   });
   
   // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
   server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
     String inputMessage;
     String inputParam;
     // pitch/roll
     if (request->hasParam(P_ROLL_GET)) {
       inputMessage = request->getParam(P_ROLL_GET)->value();
       inputParam = P_ROLL_GET;
       PID_P_GAIN_ROLL = inputMessage.toFloat();
       PID_P_GAIN_PITCH = inputMessage.toFloat();
     }
     // 
     else if (request->hasParam(I_ROLL_GET)) {
       inputMessage = request->getParam(I_ROLL_GET)->value();
       inputParam = I_ROLL_GET;
       PID_I_GAIN_ROLL = inputMessage.toFloat();
       PID_I_GAIN_PITCH = inputMessage.toFloat();
     }
     // 
     else if (request->hasParam(D_ROLL_GET)) {
       inputMessage = request->getParam(D_ROLL_GET)->value();
       inputParam = D_ROLL_GET;
       PID_D_GAIN_ROLL = inputMessage.toFloat();
       PID_D_GAIN_PITCH = inputMessage.toFloat();
     }
     // yaw
     else if (request->hasParam(P_YAW_GET)) {
       inputMessage = request->getParam(P_YAW_GET)->value();
       inputParam = P_YAW_GET;
       PID_P_GAIN_YAW = inputMessage.toFloat();
     }
     else if (request->hasParam(I_YAW_GET)) {
       inputMessage = request->getParam(I_YAW_GET)->value();
       inputParam = I_YAW_GET;
       PID_I_GAIN_YAW = inputMessage.toFloat();
     }
     else if (request->hasParam(D_YAW_GET)) {
       inputMessage = request->getParam(D_YAW_GET)->value();
       inputParam = D_YAW_GET;
       PID_D_GAIN_YAW = inputMessage.toFloat();
     }
     else {
       inputMessage = "No message sent";
       inputParam = "none";
     }
     //Serial.printf("\n%.4f ", inputMessage.toFloat());
     //Serial.println(inputParam);
     request->send_P(200, "text/html", index_html(), processor);

      writeDataTransfer();
   });


  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      //Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);


  server.onNotFound(notFound);
  server.begin();
}
 
void loop(){
  sendTelemetry();

  //delay(delayTime);
}


void sendTelemetry(){
  // Send Events to the Web Server with the Sensor Readings
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

  //Serial.printf("I'm sending...\n");

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
  int posEnd = str.lastIndexOf('>') - 1;
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
