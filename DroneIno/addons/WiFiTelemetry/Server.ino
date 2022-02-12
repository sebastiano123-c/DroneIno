/* 
 * Server
 * @author @sebastiano123-c
 * @note server functions
 */

void notFound(AsyncWebServerRequest *request) {
  /* 
   * @brief send "not found" if the server is not found
   */
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
  else if( var == "ALTITUDE")   return String(altitudeMeasure);
}


void readFromClientOnGet(){
  /* 
   * @brief get the value of the input form submitted on the client
   */

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
}