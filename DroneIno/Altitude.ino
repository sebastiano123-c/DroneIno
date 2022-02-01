void checkAltitudeSensor(){
  bool statusBMP = bmp.begin(ALTITUDE_SENSOR_ADDRESS);
  if (!statusBMP) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    ledcWrite(pwmLedChannel, MAX_DUTY_CYCLE);
    while (1) vTaskDelay(10/portTICK_PERIOD_MS);
  }
  ledcWrite(pwmLedChannel, 0);

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  pressure = bmp.readPressure();
  vTaskDelay(10/portTICK_PERIOD_MS);

  if(DEBUG) Serial.println("checkAltitudeSensor: OK");
}

void readPresTempAlt() {
	//Print values to serial monitor:
	Serial.print(F("Pressure: "));
  Serial.print(bmp.readPressure());
  Serial.print(" Pa");
  Serial.print("\t");
  Serial.print(("Temp: "));
  Serial.print(bmp.readTemperature());
  Serial.print(" oC");
	Serial.print("\t");
  Serial.print("Altimeter: ");
  Serial.print(bmp.readAltitude (1050.35)); // this should be adjusted to your local forcase
  Serial.println(" m");
}

void calculateAltitudeHold(){
  barometerCounter ++;

  //Every time this function is called the barometerCounter variable is incremented. This way a specific action
  //is executed at the correct moment. This is needed because requesting data from the MS5611 takes around 9ms to complete.

  switch (barometerCounter)
  {                                              //When the barometerCounter variable is 1.
   case 1:
    //Get pressure data
    pressure = bmp.readPressure();
  
    //To get a smoother pressure value we will use a 20 location rotating memory.
    pressureTotalAvarage -= pressureRotatingMem[pressureRotatingMemLocation];                          //Subtract the current memory position to make room for the new value.
    pressureRotatingMem[pressureRotatingMemLocation] = pressure;                                                //Calculate the new change between the actual pressure and the previous measurement.
    pressureTotalAvarage += pressureRotatingMem[pressureRotatingMemLocation];                          //Add the new value to the long term avarage value.
    pressureRotatingMemLocation++;                                                                         //Increase the rotating memory location.
    if (pressureRotatingMemLocation == 20)pressureRotatingMemLocation = 0;                              //Start at 0 when the memory location 20 is reached.
    actualPressureFast = (float)pressureTotalAvarage / 20.0;                                              //Calculate the average pressure of the last 20 pressure readings.

    //To get better results we will use a complementary fillter that can be adjusted by the fast average.
    actualPressureSlow = actualPressureSlow * (float)0.985 + actualPressureFast * (float)0.015;
    actualPressureDiff = actualPressureSlow - actualPressureFast;                                       //Calculate the difference between the fast and the slow avarage value.
    if (actualPressureDiff > 8)actualPressureDiff = 8;                                                    //If the difference is larger then 8 limit the difference to 8.
    if (actualPressureDiff < -8)actualPressureDiff = -8;                                                  //If the difference is smaller then -8 limit the difference to -8.
    //If the difference is larger then 1 or smaller then -1 the slow average is adjuste based on the error between the fast and slow average.
    if (actualPressureDiff > 1 || actualPressureDiff < -1)actualPressureSlow -= actualPressureDiff / 6.0;
    actualPressure = actualPressureSlow;                                                                   //The actualPressure is used in the program for altitude calculations.
    break;

   case 2:                                                              //When the barometer counter is 3
    barometerCounter = 0;                                              //Set the barometer counter to 0 for the next measurements.
    //In the following part a rotating buffer is used to calculate the long term change between the various pressure measurements.
    //This total value can be used to detect the direction (up/down) and speed of the quadcopter and functions as the D-controller of the total PID-controller.
    if (manualAltitudeChange == 1)pressureParachutePrevious = actualPressure * 10;                       //During manual altitude change the up/down detection is disabled.
    parachuteThrottle -= parachuteBuffer[parachuteRotatingMemLocation];                                  //Subtract the current memory position to make room for the new value.
    parachuteBuffer[parachuteRotatingMemLocation] = actualPressure * 10 - pressureParachutePrevious;   //Calculate the new change between the actual pressure and the previous measurement.
    parachuteThrottle += parachuteBuffer[parachuteRotatingMemLocation];                                  //Add the new value to the long term avarage value.
    pressureParachutePrevious = actualPressure * 10;                                                       //Store the current measurement for the next loop.
    parachuteRotatingMemLocation++;                                                                        //Increase the rotating memory location.
    if (parachuteRotatingMemLocation == 30)parachuteRotatingMemLocation = 0;                            //Start at 0 when the memory location 20 is reached.

    //If the altitude hold function is disabled some variables need to be reset to ensure a bumpless start when the altitude hold function is activated again.
    switch (flightMode)
    {

    case 1:
      if(pidAltitudeSetpoint != 0){
        pidAltitudeSetpoint = 0;                                                     //Reset the PID altitude setpoint.
        pidOutputAltitude = 0;                                                       //Reset the output of the PID controller.
        pidIMemAltitude = 0;                                                        //Reset the I-controller.
        manualThrottle = 0;                                                           //Set the manualThrottle variable to 0 .
        manualAltitudeChange = 1;                                                    //Set the manualAltitudeChange to 1.
      }
      break;
    
    default: //If the quadcopter is in altitude mode and flying.

      if (pidAltitudeSetpoint == 0) pidAltitudeSetpoint = actualPressure;       //If not yet set, set the PID altitude setpoint.
      //When the throttle stick position is increased or decreased the altitude hold function is partially disabled.
      //The manualAltitudeChange variable will indicate if the altitude of the quadcopter is changed by the pilot.
      manualAltitudeChange = 0;                                                    //Preset the manualAltitudeChange variable to 0.
      manualThrottle = 0;                                                           //Set the manualThrottle variable to 0.
      if (throttle > 1600) {                                                         //If the throttle is increased above 1600us (60%).
        manualAltitudeChange = 1;                                                  //Set the manualAltitudeChange variable to 1 to indicate that the altitude is adjusted.
        pidAltitudeSetpoint = actualPressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manualThrottle = (throttle - 1600) / 3;                                     //To prevent very fast changes in hight limit the function of the throttle.
      }
      if (throttle < 1400) {                                                         //If the throttle is lowered below 1400us (40%).
        manualAltitudeChange = 1;                                                  //Set the manualAltitudeChange variable to 1 to indicate that the altitude is adjusted.
        pidAltitudeSetpoint = actualPressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manualThrottle = (throttle - 1400) / 5;                                     //To prevent very fast changes in hight limit the function of the throttle.
      }

      //Calculate the PID output of the altitude hold.
      pidAltitudeInput = actualPressure;                                          //Set the setpoint (pidAltitudeInput) of the PID-controller.
      pidErrorTemp = pidAltitudeInput - pidAltitudeSetpoint;                   //Calculate the error between the setpoint and the actual pressure value.

      //To get better results the P-gain is increased when the error between the setpoint and the actual pressure value increases.
      //The variable pidErrorGainAltitude will be used to adjust the P-gain of the PID-controller.
      pidErrorGainAltitude = 0;                                                   //Set the pidErrorGainAltitude to 0.
      if (pidErrorTemp > 10 || pidErrorTemp < -10) {                             //If the error between the setpoint and the actual pressure is larger than 10 or smaller then -10.
        pidErrorGainAltitude = (abs(pidErrorTemp) - 10) / 20.0;                 //The positive pidErrorGainAltitude variable is calculated based based on the error.
        if (pidErrorGainAltitude > 3)pidErrorGainAltitude = 3;                 //To prevent extreme P-gains it must be limited to 3.
      }

      //In the following section the I-output is calculated. It's an accumulation of errors over time.
      //The time factor is removed as the program loop runs at 250Hz.
      pidIMemAltitude += (PID_I_GAIN_ALTITUDE / 100.0) * pidErrorTemp;
      if (pidIMemAltitude > PID_MAX_ALTITUDE)pidIMemAltitude = PID_MAX_ALTITUDE;
      else if (pidIMemAltitude < PID_MAX_ALTITUDE * -1)pidIMemAltitude = PID_MAX_ALTITUDE * -1;
      //In the following line the PID-output is calculated.
      //P = (PID_P_GAIN_ALTITUDE + pidErrorGainAltitude) * pidErrorTemp.
      //I = pidIMemAltitude += (PID_I_GAIN_ALTITUDE / 100.0) * pidErrorTemp (see above).
      //D = PID_D_GAIN_ALTITUDE * parachuteThrottle.
      pidOutputAltitude = (PID_P_GAIN_ALTITUDE + pidErrorGainAltitude) * pidErrorTemp + pidIMemAltitude + PID_D_GAIN_ALTITUDE * parachuteThrottle;
      //To prevent extreme PID-output the output must be limited.
      if (pidOutputAltitude > PID_MAX_ALTITUDE)pidOutputAltitude = PID_MAX_ALTITUDE;
      else if (pidOutputAltitude < PID_MAX_ALTITUDE * -1)pidOutputAltitude = PID_MAX_ALTITUDE * -1;
    }
    break;

  default: // if barometerCounter is neither 1 or 2
    barometerCounter = 1;
  }
}
