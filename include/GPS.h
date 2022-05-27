/**
 * @file GPS.h
 * @author @sebastiano123-c
 * @brief GPS routines.
 * @version 0.1
 * @date 2022-02-28
 * 
 * @copyright Copyright (c) 2022
 * 
 * The GPS communication works via RX/TX serial communication.
 * GPS prints on this serial strings with the informations using the NMEA protocol, for example
 * 
 *    $GPGGA,155902.00,4501.502642,N,01434.102644,E,1,12,0.7,15.0,M,44.0,M,,*78
 * 
 * The following code reads this string using a char array whose index represents
 *    @li 0: '$', signalling the beginning of the string
 *    @li 1-6: string type
 *    @li 7-16: UTC time
 *    @li 17-25: latitude in degrees
 *    @li 30: l8titude dir
 *    @li 31-43: longitude in degrees
 *    @li 45: longitude dir
 *    @li @todo finishing this list
 * 
 *  $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
 * 
 *   1    = UTC of Position
 *   2    = Latitude
 *   3    = N or S
 *   4    = Longitude
 *   5    = E or W
 *   6    = GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
 *   7    = Number of satellites in use [not those in view]
 *   8    = Horizontal dilution of position
 *   9    = Antenna altitude above/below mean sea level (geoid)
 *   10   = Meters  (Antenna height unit)
 *   11   = Geoidal separation (Diff. between WGS-84 earth ellipsoid and
 *          mean sea level.  -=geoid is below WGS-84 ellipsoid)
 *   12   = Meters  (Units of geoidal separation)
 *   13   = Age in seconds since last update from diff. reference station
 *   14   = Diff. reference station ID#
 *   15   = Checksum
 * 
 * @link http://aprs.gids.nl/nmea/ @endlink
 */

// new serial
HardwareSerial SerialGPS(1);

/**
 * @brief Setup function for the GPS
 * 
 */
void setupGPS(void) {

  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, PIN_RX2, PIN_TX2);
  delay(250);

//   //Disable GPGSV messages by using the ublox protocol.
//   uint8_t Disable_GPGSV[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
//   SerialGPS.write(Disable_GPGSV, 11);
//   delay(350);   //A small delay is added to give the GPS some time to respond @ 9600bps.
//   //Set the refresh rate to 5Hz by using the ublox protocol.
//   uint8_t Set_to_5Hz[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
//   SerialGPS.write(Set_to_5Hz, 14);
//   delay(350);   //A small delay is added to give the GPS some time to respond @ 9600bps.
//   //Set the baud rate to 57.6kbps by using the ublox protocol.
//   uint8_t Set_to_57kbps[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
//                                0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0xE1
//                               };
//   SerialGPS.write(Set_to_57kbps, 28);
//   delay(200);

//   SerialGPS.begin(57600);
//   delay(200);
}

void readGPSSerialLine(){
    char read_serial_byte = SerialGPS.read();                                              // new serial byte

    if (read_serial_byte == '$') {                                                         // $ signals the first char of the string
      for (GPSStringCounter = 0; GPSStringCounter < 100; GPSStringCounter ++) {            // clear data
        GPSString[GPSStringCounter] = '-';                                                 // writing - 
      }
      GPSStringCounter = 0;                                                                // reset the counter o start writing at the begin of the array
    }
    else                                                                                   // else, it is not at beginning
        if (GPSStringCounter < 100)                                                        // increment the counter
            GPSStringCounter ++;

    GPSString[GPSStringCounter] = read_serial_byte;                                        // write the new received byte to the new position in the array

    if (read_serial_byte == '*')                                                           // if the message is at its end (see NMEA protocol)
        NMEANewline = 1;                                                                   // then there will be a new line
}

void printRawGPSSerialLine(){
    Serial.println(SerialGPS.readStringUntil('\n'));
}

void calculateGPSTimeUTC(){
  // instantiate objs
  static char charGPS[30];                                              // the char array that will be filled
  char* ptr = charGPS;                                                  // pointer to the char array

  // add UTC time zone
  int hourUTC = ((int)GPSString[7] - 48) * 10 + (int)GPSString[8] - 48; // create the hour integer
  hourUTC += UTC_TIME_ZONE;                                             // add time zone

  // fill char array
  ptr += sprintf(ptr, "%i:%c%c:%c%c.%c%c", hourUTC,                     // hh: hours
                                           GPSString[9],                // mm: minutes
                                           GPSString[10],
                                           GPSString[11],               // ss: seconds
                                           GPSString[12],
                                           GPSString[14],               // ff: milliseconds
                                           GPSString[15]);
  *ptr++ = 0;                                                           // char array must terminate with the null char

  timeUTC = (const char*)charGPS;                                       // convert char array to const char*
}

void calculateLatLonGPSGA(){

    latActualGPS = ((int)GPSString[19] - 48) *  (long)10000000;                              // filter the minutes for the GGA line multiplied by factors 10, ...
    latActualGPS += ((int)GPSString[20] - 48) * (long)1000000;                                                                                       
    latActualGPS += ((int)GPSString[22] - 48) * (long)100000;                                                                                        
    latActualGPS += ((int)GPSString[23] - 48) * (long)10000;                                                                                         
    latActualGPS += ((int)GPSString[24] - 48) * (long)1000;                                                                                          
    latActualGPS += ((int)GPSString[25] - 48) * (long)100;                                                                                           
    latActualGPS += ((int)GPSString[26] - 48) * (long)10;                                                                                            
    latActualGPS /= (long)6;                                                                 // convert minutes to degrees: divide the minutes by 6.
    latActualGPS += ((int)GPSString[17] - 48) *  (long)100000000;                            // add the degrees multiplied by 10.
    latActualGPS += ((int)GPSString[18] - 48) *  (long)10000000;                             
    latActualGPS /= 10;                                                                      // divide everything by 10.

    lonActualGPS = ((int)GPSString[33] - 48) *  (long)10000000;                              // filter the minutes for the GGA line multiplied by 10,...
    lonActualGPS += ((int)GPSString[34] - 48) * (long)1000000;                                                                                      
    lonActualGPS += ((int)GPSString[36] - 48) * (long)100000;                                                                                       
    lonActualGPS += ((int)GPSString[37] - 48) * (long)10000;                                                                                        
    lonActualGPS += ((int)GPSString[38] - 48) * (long)1000;                                                                                         
    lonActualGPS += ((int)GPSString[39] - 48) * (long)100;                                                                                          
    lonActualGPS += ((int)GPSString[40] - 48) * (long)10;                                                                                           
    lonActualGPS /= (long)6;                                                                 // convert minutes to degrees we need to divide the minutes by 6.
    lonActualGPS += ((int)GPSString[30] - 48) * (long)1000000000;                            // add the degrees multiplied by 10, ...
    lonActualGPS += ((int)GPSString[31] - 48) * (long)100000000;                             
    lonActualGPS += ((int)GPSString[32] - 48) * (long)10000000;                              
    lonActualGPS /= 10;                                                                      // divide everything by 10.

    if (GPSString[28] == 'N')latNorth = 1;                                                   // when flying north of the equator the latNorth variable set to 1
    else latNorth = 0;                                                                       // when flying south of the equator the latNorth variable set to 0

    if (GPSString[42] == 'E')lonEast = 1;                                                    // when flying east of the prime meridian the lonEast variable set to 1
    else lonEast = 0;                                                                        // when flying west of the prime meridian the lonEast variable set to 0

    GPSSatNumber = ((int)GPSString[46] - 48) * (long)10;                                     // filter the number of satellites from the GGA line
    GPSSatNumber += (int)GPSString[47] - 48;                                                 // filter the number of satellites from the GGA line

    if (latGPSPrevious == 0 && lonGPSPrevious == 0) {                                        // if no readings are found for GPS 
        latGPSPrevious = latActualGPS;                                                         // set the latGPSPrevious variable to the latActualGPS variable
        lonGPSPrevious = lonActualGPS;                                                         // set the lonGPSPrevious variable to the lonActualGPS variable
    }

    latLoop = (float)(latActualGPS - latGPSPrevious) / 10.0;                              
    lonLoop = (float)(lonActualGPS - lonGPSPrevious) / 10.0;                              

    longLatGPS = latGPSPrevious;                                                              
    longLonGPS = lonGPSPrevious;                                                              

    // set the next loop.
    latGPSPrevious = latActualGPS;                                                                 
    lonGPSPrevious = lonActualGPS;                                                                 

    // the GPS is set to a 5Hz refresh rate. Between every 2 GPS measurements, 9 GPS values are simulated.
    GPSAddCounter = 5;                                                                       // set the GPSAddCounter variable to 5 as a count down loop timer
    newGPSDataCounter = 9;                                                                   // set to to 9, simulated values between 2 GPS measurements
    latGPSAdd = 0;                                                                           // reset the latGPSAdd variable
    lonGPSAdd = 0;                                                                           // reset the lonGPSAdd variable
    newGPSDataAvailable = 1;                                                                 // set to indicate that there is new data available

    // create float variables
    latitudeGPS = (float)latActualGPS/1e6;
    longitudeGPS = (float)lonActualGPS/1e6;
}

void calculatePIDFromGPS(){
    if (GPSSatNumber < 8)
        ledcWrite(pwmLedChannel, abs(MAX_DUTY_CYCLE - (int)ledcRead(pwmLedChannel)));                // change the LED on the STM32 to indicate GPS reception.

    else ledcWrite(pwmLedChannel, 0);                                                           // turn the LED on the STM solid on (LED function is inverted)

    timerGPS = millis();                                                                        // reset the GPS timer.
    newGPSDataAvailable = 0;                                                                    // reset the newGPSDataAvailable variable

    if (flightMode >= 3 && waypointGPS == 0) {                                                 // if the flight mode is 3 (GPS hold) and no waypoints are set
      waypointGPS = 1;                                                                         // waypoints are set
      longLatWaypoint = longLatGPS;                                                            // remember the current latitude as GPS hold waypoint
      longLonWaypoint = longLonGPS;                                                            // remember the current longitude as GPS hold waypoint
    }

    if (flightMode >= 3 && waypointGPS == 1) {                                                          //If the GPS hold mode and the waypoints are stored.
      //GPS stick move adjustments
      if (flightMode == 3 /*&& takeoffDetected == 1*/) {
        if (!latNorth) {
          latGPSAdjust += 0.0015 * (((trimCh[2].actual - 1500) * cos(GPSManAdjustHeading * 0.017453)) + ((trimCh[1].actual - 1500) * cos((GPSManAdjustHeading - 90) * 0.017453))); //South correction
        }
        else {
          latGPSAdjust -= 0.0015 * (((trimCh[2].actual - 1500) * cos(GPSManAdjustHeading * 0.017453)) + ((trimCh[1].actual - 1500) * cos((GPSManAdjustHeading - 90) * 0.017453))); //North correction
        }

        if (!lonEast) {
          lonGPSAdjust -= (0.0015 * (((trimCh[1].actual - 1500) * cos(GPSManAdjustHeading * 0.017453)) + ((trimCh[2].actual - 1500) * cos((GPSManAdjustHeading + 90) * 0.017453)))) / cos(((float)longLatGPS / 1000000.0) * 0.017453); //West correction
        }

        else {
          lonGPSAdjust += (0.0015 * (((trimCh[1].actual - 1500) * cos(GPSManAdjustHeading * 0.017453)) + ((trimCh[2].actual - 1500) * cos((GPSManAdjustHeading + 90) * 0.017453)))) / cos(((float)longLatGPS / 1000000.0) * 0.017453); //East correction
        }
      }

      if (latGPSAdjust > 1) {
        longLatWaypoint ++;
        latGPSAdjust --;
      }
      if (latGPSAdjust < -1) {
        longLatWaypoint --;
        latGPSAdjust ++;
      }

      if (lonGPSAdjust > 1) {
        longLonWaypoint ++;
        lonGPSAdjust --;
      }
      if (lonGPSAdjust < -1) {
        longLonWaypoint --;
        lonGPSAdjust ++;
      }

      GPSLonError = longLonWaypoint - longLonGPS;                                                         //Calculate the latitude error between waypoint and actual position.
      GPSLatError = longLatGPS - longLatWaypoint;                                                         //Calculate the longitude error between waypoint and actual position.

      GPSLatAvarage -=  GPSLatRotatingMem[ GPSRotatingMemLocation];                         //Subtract the current memory position to make room for the new value.
      GPSLatRotatingMem[ GPSRotatingMemLocation] = GPSLatError - GPSLatErrorPrevious;          //Calculate the new change between the actual pressure and the previous measurement.
      GPSLatAvarage +=  GPSLatRotatingMem[ GPSRotatingMemLocation];                         //Add the new value to the long term avarage value.

      GPSLonAvarage -=  GPSLonRotatingMem[ GPSRotatingMemLocation];                         //Subtract the current memory position to make room for the new value.
      GPSLonRotatingMem[ GPSRotatingMemLocation] = GPSLonError - GPSLonErrorPrevious;          //Calculate the new change between the actual pressure and the previous measurement.
      GPSLonAvarage +=  GPSLonRotatingMem[ GPSRotatingMemLocation];                         //Add the new value to the long term avarage value.
      GPSRotatingMemLocation++;                                                                        //Increase the rotating memory location.
      if ( GPSRotatingMemLocation == 35) GPSRotatingMemLocation = 0;                                //Start at 0 when the memory location 35 is reached.

      GPSLatErrorPrevious = GPSLatError;                                                             //Remember the error for the next loop.
      GPSLonErrorPrevious = GPSLonError;                                                             //Remember the error for the next loop.

      //Calculate the GPS pitch and roll correction as if the nose of the multicopter is facing north.
      //The Proportional part = (float)GPSLatError * GPS_P_GAIN.
      //The Derivative part = (float)GPSLatAvarage * GPS_D_GAIN.
      GPSPitchAdjustNorth = (float)GPSLatError * PGainGPS + (float)GPSLatAvarage * DGainGPS;
      GPSRollAdjustNorth = (float)GPSLonError * PGainGPS + (float)GPSLonAvarage * DGainGPS;

      if (!latNorth)GPSPitchAdjustNorth *= -1;                                                   //Invert the pitch adjustmet because the quadcopter is flying south of the equator.
      if (!lonEast)GPSRollAdjustNorth *= -1;                                                     //Invert the roll adjustmet because the quadcopter is flying west of the prime meridian.

      //Because the correction is calculated as if the nose was facing north, we need to convert it for the current heading.
      GPSRollAdjust = ((float)GPSRollAdjustNorth * cos(gyroAxis[3] * 0.017453)) + ((float)GPSPitchAdjustNorth * cos((gyroAxis[3] - 90) * 0.017453));
      GPSPitchAdjust = ((float)GPSPitchAdjustNorth * cos(gyroAxis[3] * 0.017453)) + ((float)GPSRollAdjustNorth * cos((gyroAxis[3] + 90) * 0.017453));

      //Limit the maximum correction to 300. This way we still have full control with the pitch and roll stick on the transmitter.
      if (GPSRollAdjust > 300) GPSRollAdjust = 300;
      if (GPSRollAdjust < -300) GPSRollAdjust = -300;
      if (GPSPitchAdjust > 300) GPSPitchAdjust = 300;
      if (GPSPitchAdjust < -300) GPSPitchAdjust = -300;
    }
}

#if GPS == BN_880

  void readGPS(void) {

    if (GPSAddCounter >= 0) GPSAddCounter --;

    // read byte-to-byte the serial information from the GPS 
    while (SerialGPS.available() && NMEANewline == 0)                                        
        readGPSSerialLine();

    // if a new line is found from the previous reading
    if (NMEANewline == 1) {                                                         

        NMEANewline = 0;                                                                        // reset line counters

        // if no GPS fix or latitude/longitude information available
        if (GPSString[4] == 'L' && GPSString[5] == 'L' && GPSString[7] == ',') {                

        ledcWrite(pwmLedChannel, abs(MAX_DUTY_CYCLE - ledcRead(pwmLedChannel)));              // led blink to signal
        
        longLatGPS = 0;
        longLonGPS = 0;
        latGPSPrevious = 0;
        lonGPSPrevious = 0;
        GPSSatNumber = 0;

        }

        // if the line starts with GA and if there is a GPS fix we can scan the line for the latitude, longitude and number of satellites.
        if (GPSString[4] == 'G' && GPSString[5] == 'A' && (GPSString[44] == '1' || GPSString[44] == '2')){
          calculateGPSTimeUTC();
          calculateLatLonGPSGA();
        }

        // if the line starts with SA and if there is a GPS fix we can scan the line for the fix type (none, 2D or 3D).
        if (GPSString[4] == 'S' && GPSString[5] == 'A')
            GPSFixType = (int)GPSString[9] - 48;

    }

    //After 5 program loops 5 x 4ms = 20ms the GPSAddCounter is 0
    if (GPSAddCounter == 0 && newGPSDataCounter > 0) {                              // if GPSAddCounter is 0 and there are new GPS simulations needed
        newGPSDataAvailable = 1;                                                      // set the newGPSDataAvailable to indicate there is new data available
        newGPSDataCounter --;                                                         // decrement the newGPSDataCounter so there will be only 9 simulations
        GPSAddCounter = 5;                                                            // set the GPSAddCounter variable to 5 as a count down loop timer

        latGPSAdd += latLoop;                                                         // add the simulated part to a buffer float variable because the longLatGPS can only hold integers.
        if (abs(latGPSAdd) >= 1) {                                                    // if the absolute value of latGPSAdd is larger then 1
            longLatGPS += (int)latGPSAdd;                                             // increment the latGPSAdd value with the latGPSAdd value as an integer
            latGPSAdd -= (int)latGPSAdd;                                              // subtract the latGPSAdd value as an integer so the decimal value remains.
        }

        lonGPSAdd += lonLoop;                                                         // add the simulated part to a buffer float variable because the longLatGPS can only hold integers.
        if (abs(lonGPSAdd) >= 1) {                                                    // if the absolute value of lonGPSAdd is larger then 1
            longLonGPS += (int)lonGPSAdd;                                             // increment the lonGPSAdd value with the lonGPSAdd value as an integer
            lonGPSAdd -= (int)lonGPSAdd;                                              // subtract the lonGPSAdd value as an integer so the decimal value remains.
        }
    }

    // if there is a new set of GPS data available
    if (newGPSDataAvailable) {                                                                    
        calculatePIDFromGPS();
    }

    // if the timer is exceeded the GPS signal is missing
    if (timerGPS + 1000 < millis()) {                                                             
        if (flightMode >= 3 && start > 0) {                                                         //If flight mode is set to 3 (GPS hold).
        flightMode = 2;                                                                           //Set the flight mode to 2.
        error = 4;                                                                                //Output an error.
        }
    }

    // if the GPS hold mode is disabled and the waypoints are set
    if (flightMode < 3 && waypointGPS > 0) {                                                              
          GPSRollAdjust = 0;                                                                                  //Reset the GPSRollAdjust variable to disable the correction.
          GPSPitchAdjust = 0;                                                                                 //Reset the GPSPitchAdjust variable to disable the correction.
          if (waypointGPS == 1) {                                                                              //If the waypoints are stored
          GPSRotatingMemLocation = 0;                                                                      //Set the GPSRotatingMemLocation to zero so we can empty the
          waypointGPS = 2;                                                                                   //Set the waypointGPS variable to 2 as an indication that the buffer is not cleared.
          }
          GPSLonRotatingMem[ GPSRotatingMemLocation] = 0;                                                 //Reset the current GPSLonRotatingMem location.
          GPSLatRotatingMem[ GPSRotatingMemLocation] = 0;                                                 //Reset the current GPSLonRotatingMem location.
          GPSRotatingMemLocation++;                                                                          //Increment the GPSRotatingMemLocation variable for the next loop.
          if (GPSRotatingMemLocation == 36) {                                                                //If the GPSRotatingMemLocation equals 36, all the buffer locations are cleared.
          waypointGPS = 0;                                                                                   //Reset the waypointGPS variable to 0.
          //Reset the variables that are used for the D-controller.
          GPSLatErrorPrevious = 0;
          GPSLonErrorPrevious = 0;
          GPSLatAvarage = 0;
          GPSLonAvarage = 0;
          GPSRotatingMemLocation = 0;
          //Reset the waypoints.
          longLatWaypoint = 0;
          longLonWaypoint = 0;
          }
      }
  }

#elif GPS == OFF

  void readGPS(){
    return;
  }

#else 
  #error "\n Error: Invalid GPS token "
#endif

void printGPS(){//696581
    Serial.printf("time: %s, lat: %f, lon: %f \n", timeUTC, latitudeGPS, longitudeGPS);
}