/**
 * @file WiFi.h
 * @author your name (you@domain.com)
 * @brief WiFi network routines for telemetry and PID settingusing web apps.
 *
 * Define in the Config.h file the macro WIFI_TELEMETRY as:
 *
 *   @li NATIVE: if you don't have a ESP32-CAM. This will use the ESP32 native
 * wifi to easily set PID parameters;
 *   @li ESP_CAM: allow you to use an ESP32-CAM mounting a OV2640 camera as a
 * telemetry system.
 *
 * NATIVE it is not actually a proper telemetry system since it allows only to
 * change PID very easily and on the fly, but not to see telemetry data. It is
 * so because the ping time between the message sent and the received positive
 * response is an eternity (like 10ms) with respect to the 4ms rate.
 *
 * ESP_CAM, connected to DroneIno accordingly to the documentation on GitHub,
 * provides a full telemetry system, a video streaming and the possibility to
 * set PID parameters back to DroneIno.
 *
 * Both uses,
 *  network name "DroneInoTelemetry"
 *  passwork "DroneIno"
 *  server IP "192.168.4.1"
 *
 * After connecting to DroneInoTelemetry network, dial "192.168.4.1" on your
 * browser.
 *
 * @version 0.1
 * @date 2022-03-01
 *
 * @copyright Copyright (c) 2022
 *
 */

#if WIFI_TELEMETRY == OFF

void setupWiFiTelemetry() { return; }
void sendWiFiTelemetry() { return; }

#elif WIFI_TELEMETRY == NATIVE

/**
 *    (ESP32 BROWSER TAGS)
 */
const char *P_ROLL_GET = "rollP";
const char *I_ROLL_GET = "rollI";
const char *D_ROLL_GET = "rollD";
const char *P_YAW_GET = "yawP";
const char *I_YAW_GET = "yawI";
const char *D_YAW_GET = "yawD";
const char *FILTER_P_R_GET = "filterPitchRoll";
const char *ROLL_CORR_GET = "correctionRoll";
const char *PITCH_CORR_GET = "correctionPitch";
const char *P_ALTITUDE_GET = "altitudeP";
const char *I_ALTITUDE_GET = "altitudeI";
const char *D_ALTITUDE_GET = "altitudeD";

/**
 * @brief Not found response.
 *
 * @param request
 */
void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

/**
 * @brief HTML source of the web page.
 *
 * @return const char*
 */
const char *index_html() {

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
  String indexHTMLBody1 =
      rollPInput +
      R"rawliteral(</label><input class="pid-input" type="text" id="rollPInput" name="rollP">
                        <input type="submit" id="rollP" onclick="showInput(this);" value="set">
                    </form>
                    <form action="get">
                        I: <label class="pid-label" id="rollIVal">)rawliteral";
  String rollIInput = indexHTMLBody1 + String(PID_I_GAIN_ROLL, 5);
  String indexHTMLBody2 =
      rollIInput +
      R"rawliteral(</label><input class="pid-input" type="text" id="rollIInput" name="rollI">
                        <input type="submit" id="rollI" onclick="showInput(this);" value="set">
                    </form>
                    <form action="get">
                        D: <label class="pid-label" id="rollDVal">)rawliteral";
  String rollDInput = indexHTMLBody2 + String(PID_D_GAIN_ROLL, 4);
  String indexHTMLBody3 =
      rollDInput +
      R"rawliteral(</label><input class="pid-input" type="text" id="rollDInput" name="rollD">
                        <input type="submit" id="rollD" onclick="showInput(this);" value="set">
                    </form>
                </div>
                <div class="card">
                    <p >  Yaw </p>
                    <form action="get">
                        P: <label class="pid-label" id="yawPVal">)rawliteral";
  String yawPInput = indexHTMLBody3 + String(PID_P_GAIN_YAW, 4);
  String indexHTMLBody4 =
      yawPInput +
      R"rawliteral(</label><input class="pid-input" type="text" id="yawPInput" name="yawP">
                        <input type="submit" id="yawP" onclick="showInput(this);" value="set">
                    </form>
                    <form action="get">
                        I: <label class="pid-label" id="yawIVal">)rawliteral";
  String yawIInput = indexHTMLBody4 + String(PID_I_GAIN_YAW, 4);
  String indexHTMLBody5 =
      yawIInput +
      R"rawliteral(</label><input class="pid-input" type="text" id="yawIInput" name="yawI">
                        <input type="submit" id="yawI" onclick="showInput(this);" value="set">
                    </form>
                    <form action="get">
                        D: <label class="pid-label" id="yawDVal">)rawliteral";
  String yawDInput = indexHTMLBody5 + String(PID_D_GAIN_YAW, 4);
  String indexHTMLBody6 =
      yawDInput +
      R"rawliteral(</label><input class="pid-input" type="text" id="yawDInput" name="yawD">
                        <input type="submit" id="yawD" onclick="showInput(this);" value="set">
                    </form>
                </div>)rawliteral";

  String indexHTMLBody7 = indexHTMLBody6 + R"rawliteral(<div class="card">
                <p >  Gyroscope </p>
                <form action="get">
                  P/R filter: <label class="pid-label" id="filterPitchRollVal">)rawliteral";
  String filterPitchRollHTML =
      indexHTMLBody7 + String(GYROSCOPE_ROLL_FILTER, 4);
  String indexHTMLBody8 =
      filterPitchRollHTML +
      R"rawliteral((&#37;)</label><input class="pid-input" type="text" id="filterPitchRollInput" name="filterPitchRoll">
                  <input type="submit" id="filterPitchRoll" onclick="showInput(this);" value="set">
                </form> 
                <form action="get">
                  roll corr.: <label class="pid-label" id="correctionRollVal">)rawliteral";
  String corrRollHTML = indexHTMLBody8 + String(GYROSCOPE_ROLL_CORR, 4);
  String indexHTMLBody9 =
      corrRollHTML +
      R"rawliteral((&deg;)</label><input class="pid-input" type="text" id="correctionRollInput" name="correctionRoll">
                  <input type="submit" id="correctionRoll" onclick="showInput(this);" value="set">
                </form> 
                <form action="get">
                  pitch corr.: <label class="pid-label" id="correctionPitchVal">)rawliteral";
  String corrPitchHTML = indexHTMLBody9 + String(GYROSCOPE_PITCH_CORR, 4);
  String indexHTMLBody10 =
      corrPitchHTML +
      R"rawliteral((&deg;)</label><input class="pid-input" type="text" id="correctionPitchInput" name="correctionPitch">
                  <input type="submit" id="correctionPitch" onclick="showInput(this);" value="set">
                </form> 
              </div>)rawliteral";
  String indexHTMLBody11 = indexHTMLBody10 + R"rawliteral(<div class="card">
                <p >  Altitude </p>
                <form action="get">
                  P: <label class="pid-label" id="altitudePVal">)rawliteral";
  String altitudePHTML = indexHTMLBody11 + String(PID_P_GAIN_ALTITUDE, 4);
  String indexHTMLBody12 =
      altitudePHTML +
      R"rawliteral(</label><input class="pid-input" type="text" id="altitudePInput" name="altitudeP">
                  <input type="submit" id="altitudeP" onclick="showInput(this);" value="set">
                </form> 
                <form action="get">
                  I: <label class="pid-label" id="altitudeIVal">)rawliteral";
  String altitudeIHTML = indexHTMLBody12 + String(PID_I_GAIN_ALTITUDE, 4);
  String indexHTMLBody13 =
      altitudeIHTML +
      R"rawliteral(</label><input class="pid-input" type="text" id="altitudeIInput" name="altitudeI">
                  <input type="submit" id="altitudeI" onclick="showInput(this);" value="set">
                </form> 
                <form action="get">
                  D: <label class="pid-label" id="altitudeDVal">)rawliteral";
  String altitudeDHTML = indexHTMLBody13 + String(PID_D_GAIN_ALTITUDE, 4);
  String indexHTMLBody14 =
      altitudeDHTML +
      R"rawliteral(</label><input class="pid-input" type="text" id="altitudeDInput" name="altitudeD">
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

/**
 * @brief Substittue the HTML tag with the variable.
 *
 * @param var
 * @return String
 */
String processor(const String &var) {

  if (var == "PITCHANGLE")
    return String(anglePitch);
  else if (var == "ROLLANGLE")
    return String(angleRoll);
  else if (var == "FLIGHTMODE")
    return String(flightMode);
  else if (var == "BATTERY")
    return String(batteryPercentage);
}

/**
 * @brief Setup the wifi server AP (generated by the ESP32).
 * @note Web app link: 192.168.4.1
 *
 */
void setupWiFiTelemetry() {

  WiFi.softAP(ssid, password);

  // Serial.printf("\nIP address: " + WiFi.softAPIP() + "\n");

  // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", index_html());
  });

  // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
  server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request) {
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
    } else if (request->hasParam(I_YAW_GET)) {
      inputMessage = request->getParam(I_YAW_GET)->value();
      inputParam = I_YAW_GET;
      PID_I_GAIN_YAW = inputMessage.toFloat();
    } else if (request->hasParam(D_YAW_GET)) {
      inputMessage = request->getParam(D_YAW_GET)->value();
      inputParam = D_YAW_GET;
      PID_D_GAIN_YAW = inputMessage.toFloat();
    }
    // gyroscope
    else if (request->hasParam(FILTER_P_R_GET)) {
      inputMessage = request->getParam(FILTER_P_R_GET)->value();
      inputParam = FILTER_P_R_GET;
      GYROSCOPE_ROLL_FILTER = inputMessage.toFloat();
      GYROSCOPE_PITCH_FILTER = inputMessage.toFloat();
    } else if (request->hasParam(ROLL_CORR_GET)) {
      inputMessage = request->getParam(ROLL_CORR_GET)->value();
      inputParam = ROLL_CORR_GET;
      GYROSCOPE_ROLL_CORR = inputMessage.toFloat();
    } else if (request->hasParam(PITCH_CORR_GET)) {
      inputMessage = request->getParam(PITCH_CORR_GET)->value();
      inputParam = PITCH_CORR_GET;
      GYROSCOPE_PITCH_CORR = inputMessage.toFloat();
    }
    // altitude
    else if (request->hasParam(P_ALTITUDE_GET)) {
      inputMessage = request->getParam(P_ALTITUDE_GET)->value();
      inputParam = P_ALTITUDE_GET;
      PID_P_GAIN_ALTITUDE = inputMessage.toFloat();
    } else if (request->hasParam(I_ALTITUDE_GET)) {
      inputMessage = request->getParam(I_ALTITUDE_GET)->value();
      inputParam = I_ALTITUDE_GET;
      PID_I_GAIN_ALTITUDE = inputMessage.toFloat();
    } else if (request->hasParam(D_ALTITUDE_GET)) {
      inputMessage = request->getParam(D_ALTITUDE_GET)->value();
      inputParam = D_ALTITUDE_GET;
      PID_D_GAIN_ALTITUDE = inputMessage.toFloat();
    } else {
      inputMessage = "No message sent";
      inputParam = "none";
    }
    if (DEBUG)
      Serial.printf("wifi command: %s\n", inputMessage.c_str());
    if (DEBUG)
      Serial.printf("wifi tag id: %s\n", inputParam.c_str());

    request->send(200, "text/html", index_html());
  });

  server.onNotFound(notFound);
  server.begin();
}

/**
 * @brief Updates '\events' of the web server with the sensor readings.
 * @note refresh rate browser is about 20 Hertz (i.e. they refresh the screen
 * about 20 times per second) - i.e. every 50 milliseconds, thus for flight
 * stability, with NATIVE WiFi, no telemetry is available.
 *
 * @return * void
 */
void sendWiFiTelemetry() { return; }

#elif WIFI_TELEMETRY == ESP_CAM

HardwareSerial SUART(2);

/**
 * @brief Setup the UART communication with ESP32-CAM.
 * @note Web app link: 192.168.4.1
 *
 */
void setupWiFiTelemetry() {
  SUART.begin(WIFI_BAUD_RATE, SERIAL_8N1, PIN_RX1, PIN_TX1);

#if DEBUG
  Serial.println("SUART enabled");
#endif
}

/**
 * @brief Writes on the UART serial the telemetry.
 *
 */
void writeDataTransfer() {

  // if(DEBUG) Serial.printf("I'm sending...\n");

  // declarations
  const char *stringToPrint = "";
  static char staticCharToPrint[numChars];
  char *sptr = staticCharToPrint;

  // fill data structure before send
  sptr += sprintf(sptr, "<%.3f,", angleRoll);//PGainRoll);
  sptr += sprintf(sptr, "%.3f,", angleRoll);//PGainYaw);
  sptr += sprintf(sptr, "%x,", flightMode);
  sptr += sprintf(sptr, "%.1f,", batteryVoltage);
  sptr += sprintf(sptr, "%.1f,", altitudeMeasure);
  sptr += sprintf(sptr, "%i,", esc1); // trimCh[1].actual);
  sptr += sprintf(sptr, "%i,", esc2); // trimCh[2].actual);
  sptr += sprintf(sptr, "%i,", esc3); // trimCh[4].actual);
  sptr += sprintf(sptr, "%i,", esc4); // trimCh[3].actual);
  sptr += sprintf(sptr, "%.1f,", (float)gyroTemp / 340. + 36.53f);
  sptr += sprintf(sptr, "%.7f,", latitudeGPS);
  sptr += sprintf(sptr, "%.7f,", longitudeGPS);
  sptr += sprintf(sptr, "%s\n>", timeUTC);

  // close the string
  *sptr++ = 0;

  // print in csv format
  stringToPrint = (const char *)staticCharToPrint;

  SUART.print(stringToPrint);

  // print
  #if DEBUG && defined(DEBUG_WIFI_SEND)
    Serial.printf(" I'm sending: %s", stringToPrint);
  #endif
}


/**
 * @brief Decode the incoming message in the form < a, b, ... >.
 *
 */
void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static int ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (SUART.available() && newData == false) {
    rc = SUART.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

/**
 * @brief COnverts the message < a, b, ... > into variables.
 *
 */
void parseData() { // split the data into its parts

  char *strtokIndx; // this is used by strtok() as an index

  // fill data structure after receiving
    strtokIndx = strtok(tempChars,","); PGainRoll     = atof(strtokIndx);
    strtokIndx = strtok(NULL, ",");     IGainRoll     = atof(strtokIndx);
    strtokIndx = strtok(NULL, ",");     DGainRoll     = atof(strtokIndx);

    PGainPitch    = PGainRoll;
    IGainPitch    = IGainRoll;
    DGainPitch    = DGainRoll;

    strtokIndx = strtok(NULL, ",");     PGainYaw      = atof(strtokIndx);
    strtokIndx = strtok(NULL, ",");     IGainYaw      = atof(strtokIndx);
    strtokIndx = strtok(NULL, ",");     DGainYaw      = atof(strtokIndx);

    strtokIndx = strtok(NULL, ",");     GYROSCOPE_ROLL_FILTER = atof(strtokIndx);
    GYROSCOPE_PITCH_FILTER  = GYROSCOPE_ROLL_FILTER;
    strtokIndx = strtok(NULL, ",");     GYROSCOPE_ROLL_CORR = atof(strtokIndx);
    strtokIndx = strtok(NULL, ",");     GYROSCOPE_PITCH_CORR  = atof(strtokIndx);

    strtokIndx = strtok(NULL, ",");     PGainAltitude  = atof(strtokIndx);
    strtokIndx = strtok(NULL, ",");     IGainAltitude  = atof(strtokIndx);
    strtokIndx = strtok(NULL, ",");     DGainAltitude  = atof(strtokIndx);

    
    // signal the right reception of the message
    ledcWrite(pwmLedChannel, abs((long)ledcRead(pwmLedChannel)-MAX_DUTY_CYCLE));
    delay(10);
    ledcWrite(pwmLedChannel, abs((long)ledcRead(pwmLedChannel)-MAX_DUTY_CYCLE));


    #if DEBUG && defined(DEBUG_WIFI_REC)
      Serial.printf("...........\n I'm reading: %s \n PID_ro (%f, %f, %f), PID_pi (%f, %f, %f), PID_ya (%f, %f, %f), corr(%f, %f, %f) PID_al(%f,%f,%f)\n", receivedChars, PID_P_GAIN_ROLL, PID_I_GAIN_ROLL, PID_D_GAIN_ROLL, PID_P_GAIN_PITCH, PID_I_GAIN_PITCH, PID_D_GAIN_PITCH,PID_P_GAIN_YAW, PID_I_GAIN_YAW, PID_D_GAIN_YAW, GYROSCOPE_ROLL_FILTER, GYROSCOPE_ROLL_CORR, GYROSCOPE_PITCH_CORR, PID_P_GAIN_ALTITUDE, PID_I_GAIN_ALTITUDE, PID_D_GAIN_ALTITUDE);
    #endif

}


/**
 * @brief Read from the UART serial the PID parameters.
 *
 */
void readDataTransfer() {

  // decode message
  recvWithStartEndMarkers();

  if (newData == true) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0

    parseData();      // convert message to variables
    newData = false;
  }
}

/**
 * @brief Sends telemetry via uart to ESP32-CAM, and reads the incoming messages
 * from the ESP32-CAM.
 *
 */
void sendWiFiTelemetry() {

  // sending to ESP32-CAM
  switch (refreshCounter) {
  case 20: // refreshCounter * 4 ms
    writeDataTransfer();
    refreshCounter = 0;
    break;
  default:
    refreshCounter++;
    break;
  }

  // reading from ESP32-CAM
  readDataTransfer();
}

#endif
