/* 
 * Index
 * @author @sebastiano123-c
 * @note this file contains the html of the browser 
 */

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
