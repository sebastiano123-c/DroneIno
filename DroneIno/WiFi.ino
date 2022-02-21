/**
 * @file WiFi.ino
 * @author @sebastiano123-c
 *
 * @note Actually, it is not a telemetry if WIFI_TELEMETRY is defined NATIVE.
 * I thought it could be, but the ping time was an eternity with respect to the 4us rate.
 * Here there are the functions that permits you to set the PID very easily.
 * It uses the ESP32 access point 192.168.4.1
 * 
 */


#if WIFI_TELEMETRY == 'NATIVE'

  void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
  }
  
  const char* index_html(){
    /* 
     * @brief html source of the web page
     */
  
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
    String rollIInput = indexHTMLBody1 + String(PID_I_GAIN_ROLL, 5);
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
                </div>)rawliteral";
  
  String indexHTMLBody7 = indexHTMLBody6 + R"rawliteral(<div class="card">
                <p >  Gyroscope </p>
                <form action="get">
                  P/R filter: <label class="pid-label" id="filterPitchRollVal">)rawliteral";
  String filterPitchRollHTML = indexHTMLBody7 + String(GYROSCOPE_ROLL_FILTER, 4);
  String indexHTMLBody8 = filterPitchRollHTML + R"rawliteral((&#37;)</label><input class="pid-input" type="text" id="filterPitchRollInput" name="filterPitchRoll">
                  <input type="submit" id="filterPitchRoll" onclick="showInput(this);" value="set">
                </form> 
                <form action="get">
                  roll corr.: <label class="pid-label" id="correctionRollVal">)rawliteral";
  String corrRollHTML = indexHTMLBody8 + String(GYROSCOPE_ROLL_CORR, 4);
  String indexHTMLBody9 = corrRollHTML + R"rawliteral((&deg;)</label><input class="pid-input" type="text" id="correctionRollInput" name="correctionRoll">
                  <input type="submit" id="correctionRoll" onclick="showInput(this);" value="set">
                </form> 
                <form action="get">
                  pitch corr.: <label class="pid-label" id="correctionPitchVal">)rawliteral";
  String corrPitchHTML = indexHTMLBody9 + String(GYROSCOPE_PITCH_CORR, 4);
  String indexHTMLBody10 = corrPitchHTML + R"rawliteral((&deg;)</label><input class="pid-input" type="text" id="correctionPitchInput" name="correctionPitch">
                  <input type="submit" id="correctionPitch" onclick="showInput(this);" value="set">
                </form> 
              </div>)rawliteral";
  String indexHTMLBody11 = indexHTMLBody10 + R"rawliteral(<div class="card">
                <p >  Altitude </p>
                <form action="get">
                  P: <label class="pid-label" id="altitudePVal">)rawliteral";
  String altitudePHTML = indexHTMLBody11 + String(PID_P_GAIN_ALTITUDE, 4);
  String indexHTMLBody12 = altitudePHTML + R"rawliteral(</label><input class="pid-input" type="text" id="altitudePInput" name="altitudeP">
                  <input type="submit" id="altitudeP" onclick="showInput(this);" value="set">
                </form> 
                <form action="get">
                  I: <label class="pid-label" id="altitudeIVal">)rawliteral";
  String altitudeIHTML = indexHTMLBody12 + String(PID_I_GAIN_ALTITUDE, 4);
  String indexHTMLBody13 = altitudeIHTML + R"rawliteral(</label><input class="pid-input" type="text" id="altitudeIInput" name="altitudeI">
                  <input type="submit" id="altitudeI" onclick="showInput(this);" value="set">
                </form> 
                <form action="get">
                  D: <label class="pid-label" id="altitudeDVal">)rawliteral";
  String altitudeDHTML = indexHTMLBody13 + String(PID_D_GAIN_ALTITUDE, 4);
  String indexHTMLBody14 = altitudeDHTML + R"rawliteral(</label><input class="pid-input" type="text" id="altitudeDInput" name="altitudeD">
                  <input type="submit" id="altitudeD" onclick="showInput(this);" value="set">
                </form> 
              </div>)rawliteral";
  String indexHTMLBody15 = indexHTMLBody14 + R"rawliteral(</div>
        <p> Don't forget these values! Copy them in the Constant.h file from <button id="copyToConstants" onclick="copyToConstants();">here</button> </p>
        </div>
        <script  type="text/javascript">
          function showInput(elem) {
            document.getElementById(elem.id+"Val").innerHTML = document.getElementById(elem.id+"Input").value;
          }
          // alert setup values
          function copyToConstants(){
            var stringToPrint = "";
            stringToPrint += "PID_P_GAIN_ROLL            = " + document.getElementById("rollPVal").innerHTML + ";\n";
            stringToPrint += "PID_I_GAIN_ROLL            = " + document.getElementById("rollIVal").innerHTML + ";\n";
            stringToPrint += "PID_D_GAIN_ROLL            = " + document.getElementById("rollDVal").innerHTML + ";\n\n";
            stringToPrint += "PID_P_GAIN_YAW             = " + document.getElementById("yawPVal").innerHTML + ";\n";
            stringToPrint += "PID_I_GAIN_YAW             = " + document.getElementById("yawIVal").innerHTML + ";\n";
            stringToPrint += "PID_D_GAIN_YAW             = " + document.getElementById("yawDVal").innerHTML + ";\n\n";
            stringToPrint += "PID_P_GAIN_ALTITUDE        = " + document.getElementById("altitudePVal").innerHTML + ";\n";
            stringToPrint += "PID_I_GAIN_ALTITUDE        = " + document.getElementById("altitudeIVal").innerHTML + ";\n";
            stringToPrint += "PID_D_GAIN_ALTITUDE        = " + document.getElementById("altitudeDVal").innerHTML + ";\n\n";
            stringToPrint += "GYROSCOPE_ROLL_FILTER      = " + document.getElementById("filterPitchRollVal").innerHTML + ";\n";
            stringToPrint += "GYROSCOPE_ROLL_CORR        = " + document.getElementById("correctionRollVal").innerHTML + ";\n";
            stringToPrint += "GYROSCOPE_PITCH_CORR       = " + document.getElementById("correctionPitchVal").innerHTML + ";\n";
            alert(stringToPrint);
          }
        </script>
      </body></html>)rawliteral";
    
    return indexHTMLBody15.c_str();
  }
  
  
  String processor(const String& var){
    /* 
     * @brief substittue the HTML tag with the variable
     */
    
    if (var == "PITCHANGLE")      return String(anglePitch);
    else if( var == "ROLLANGLE")  return String(angleRoll);
    else if( var == "FLIGHTMODE") return String(flightMode);
    else if( var == "BATTERY")    return String(batteryPercent);
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
       request->send(200, "text/html", index_html());
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
      // gyroscope
       else if (request->hasParam("filterPitchRoll")) {
         inputMessage = request->getParam("filterPitchRoll")->value();
         inputParam = "filterPitchRoll";
         GYROSCOPE_ROLL_FILTER = inputMessage.toFloat();
         GYROSCOPE_PITCH_FILTER = inputMessage.toFloat();
       }
       else if (request->hasParam("correctionRoll")) {
         inputMessage = request->getParam("correctionRoll")->value();
         inputParam = "correctionRoll";
         GYROSCOPE_ROLL_CORR = inputMessage.toFloat();
       }
       else if (request->hasParam("correctionPitch")) {
         inputMessage = request->getParam("correctionPitch")->value();
         inputParam = "correctionPitch";
         GYROSCOPE_PITCH_CORR = inputMessage.toFloat();
       }
      // altitude
       else if (request->hasParam("altitudeP")) {
         inputMessage = request->getParam("altitudeP")->value();
         inputParam = "altitudeP";
         PID_P_GAIN_ALTITUDE = inputMessage.toFloat();
       }
       else if (request->hasParam("altitudeI")) {
         inputMessage = request->getParam("altitudeI")->value();
         inputParam = "altitudeI";
         PID_I_GAIN_ALTITUDE = inputMessage.toFloat();
       }
       else if (request->hasParam("altitudeD")) {
         inputMessage = request->getParam("altitudeD")->value();
         inputParam = "altitudeD";
         PID_D_GAIN_ALTITUDE = inputMessage.toFloat();
       }
       else {
         inputMessage = "No message sent";
         inputParam = "none";
       }
       if(DEBUG) Serial.printf("wifi command: %s\n",inputMessage);
       if(DEBUG) Serial.printf("wifi tag id: %s\n",inputParam);
  
       request->send(200, "text/html", index_html());                         
     });
  
    server.onNotFound(notFound);
    server.begin();
  }
  
  
  void sendWiFiTelemetry(){
      /* 
       * @brief Send Events to the Web Server with the Sensor Readings
       * @deprecated not used any more, maybe in the future I will exploit some other features using wifi
       * @note refresh rate browser is about 20 Hertz (i.e. they refresh the screen about 20 times per second) - i.e. every 50 milliseconds
       */
      return;
  }
  


#elif WIFI_TELEMETRY == 'ESP_CAM'
  
  void setupWiFiTelemetry(){
      SUART.begin(115200, SERIAL_8N1, PIN_RX1, PIN_TX1);
      // Serial.println("SUART enabled");
  }
  
  void sendWiFiTelemetry(){
    /* 
     * @brief sends telemetry via uart to esp cam, and reads the incoming messages from the esp cam
     */
    // sending to esp cam
    switch(refreshCounter){
      case 1:                      // refreshCounter * 4 ms
        writeDataTransfer();
        refreshCounter = 0;
        break;
      default:
        refreshCounter++;
    }
    
    // reading from esp cam
    if(SUART.available()){
      readDataTransfer();
    }
  }
  
  
  void writeDataTransfer(){
  
    //if(DEBUG) Serial.printf("I'm sending...\n");
  
    // fill data structure before send
    dataTransfer[0] = angleRoll;
    dataTransfer[1] = anglePitch;
    dataTransfer[2] = flightMode;
    dataTransfer[3] = batteryPercent;
    dataTransfer[4] = actualPressure;
    
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
    if(DEBUG) Serial.printf("I'm reading...\n");
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
    
    PID_P_GAIN_PITCH = PID_P_GAIN_ROLL;
    PID_I_GAIN_PITCH = PID_I_GAIN_ROLL;
    PID_D_GAIN_PITCH = PID_D_GAIN_ROLL;
    
    PID_P_GAIN_YAW = dataController[3];
    PID_I_GAIN_YAW = dataController[4];
    PID_D_GAIN_YAW = dataController[5];

    GYROSCOPE_ROLL_FILTER = dataController[6];
    GYROSCOPE_PITCH_FILTER = GYROSCOPE_ROLL_FILTER;
    GYROSCOPE_ROLL_CORR = dataController[7];
    GYROSCOPE_PITCH_CORR = dataController[8];
  
    PID_P_GAIN_ALTITUDE = dataController[9];
    PID_I_GAIN_ALTITUDE = dataController[10];
    PID_D_GAIN_ALTITUDE = dataController[11];


    
    // print in csv format   
    //for(int i = 0; i < dataControllerSize; i++){
    //  Serial.printf("%.6f\n", dataController[i]);
    //}
  }
  
  
  void checkMessage(){
    // print in csv format   
    for(int i = 0; i < dataTransferSize; i++){
      Serial.printf("%.6f\n", dataTransfer[i]);
    }
  }

#endif
