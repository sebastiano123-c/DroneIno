/*
  WiFiPID
  @author @sebastiano123-c

  Actually, it is not a telemetry.
  I thought it could be, but the ping time was an eternity with respect to the 4us rate.
  Here there are the functions that permits you to set the PID very easily.
  It uses the ESP32 access point 192.168.4.1
  
*/

const char* index_html(){
  /* 
  * @brief html source of the web page
  */
  String indexHTMLHead = R"rawliteral(<!DOCTYPE HTML><html><head>
      <title>DroneInoPID</title>
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
    </script>
    </body></html>)rawliteral";
  
  return indexHTMLBody6.c_str();
}

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}


String processor(const String& var){
  /* 
  * @brief substittue the HTML tag with the variable
  */
  
  if (var == "PITCHANGLE")      return String(anglePitch);
  else if( var == "ROLLANGLE")  return String(angleRoll);
  else if( var == "FLIGHTMODE") return String(flightMode);
  else if( var == "BATTERY")    return String(batteryPercent);
/*   else if( var == "PR")         return String(PID_P_GAIN_ROLL);
  else if( var == "IR")         return String(PID_I_GAIN_ROLL);
  else if( var == "DR")         return String(PID_D_GAIN_ROLL);
  else if( var == "PP")         return String(PID_P_GAIN_PITCH);
  else if( var == "IP")         return String(PID_I_GAIN_PITCH);
  else if( var == "DP")         return String(PID_D_GAIN_PITCH);
  else if( var == "PY")         return String(PID_P_GAIN_YAW);
  else if( var == "IY")         return String(PID_I_GAIN_YAW);
  else if( var == "DY")         return String(PID_D_GAIN_YAW); */
}

void setupWiFiTelemetry(){
  /* 
  * @brief setup the wifi server AP (generated by the ESP)
  * @link 192.168.4.1 @endlink 
  */

  WiFi.softAP(ssid, password);

  // Serial.printf("\nIP address: " + WiFi.softAPIP() + "\n");
  
// Send web page with input fields to client
   server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
     request->send_P(200, "text/html", index_html());
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
     request->send_P(200, "text/html", index_html());                         
   });

/*   // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events); */

  server.onNotFound(notFound);
  server.begin();
}
 
void sendWiFiTelemetry(){
    /* 
    * @brief Send Events to the Web Server with the Sensor Readings
    * @deprecated not used any more, maybe in the future I will exploit some other features using wifi
    * @note refresh rate browser is about 20 Hertz (i.e. they refresh the screen about 20 times per second) - i.e. every 50 milliseconds
    */
    ledcWrite(pwmLedChannel, 0);
 /*   switch (refreshCounter){

      case 10: // after 40ms 
         events.send("ping" ,NULL, millis());
         break;
      case 20:  // after 80ms 
         events.send(String(anglePitch).c_str(), "anglePitch", millis());
         break;
      case 30: // after 120ms 
         events.send(String(angleRoll).c_str(), "angleRoll", millis());
         break;
      case 40: // after 160ms 
         events.send(String(flightMode).c_str(), "flightMode", millis());
         break;
      case 50: // after 200ms 
         events.send(String(batteryPercent).c_str(), "battery", millis());
          refreshCounter = 0;
         break;
       case 60: // after 240ms 
         events.send(String(PID_P_GAIN_ROLL).c_str(), "rollPVal", millis());
         events.send(String(PID_I_GAIN_ROLL).c_str(), "rollIVal", millis());
         events.send(String(PID_D_GAIN_ROLL).c_str(), "rollDVal", millis());
         events.send(String(PID_P_GAIN_PITCH).c_str(), "pitchPVal", millis());
         events.send(String(PID_I_GAIN_PITCH).c_str(), "pitchIVal", millis());
         events.send(String(PID_D_GAIN_PITCH).c_str(), "pitchDVal", millis());
         events.send(String(PID_P_GAIN_YAW).c_str(), "yawPVal", millis());
         events.send(String(PID_I_GAIN_YAW).c_str(), "yawIVal", millis());
         events.send(String(PID_D_GAIN_YAW).c_str(), "yawDVal", millis());
         refreshCounter = 0;
         break; 
         case refreshRate:
          events.send("ping" ,NULL, millis());
          events.send(String(anglePitch).c_str(), "anglePitch", millis());
          events.send(String(angleRoll).c_str(), "angleRoll", millis());
          events.send(String(flightMode).c_str(), "flightMode", millis());
          events.send(String(batteryPercent).c_str(), "battery", millis());
          refreshCounter = 0;
          break;
          
      default:
        refreshCounter += 1;

    } */
}
