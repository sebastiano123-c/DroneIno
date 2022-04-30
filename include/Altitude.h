/**
 * @file Altitude.h
 * @author @sebastiano123-c
 * @brief Pressure readings routines.
 * 
 * Depending on the ALTITUDE_SENSOR macro value:
 * 
 *  @li BMP280: uses the I2C communication WITHOUT using the ADAFRUIT library to better performances;
 *  @li BME280*: not yet implemented;
 *  @li OFF: no altitude sensor, so there is no pressure data acquisition.
 * 
 * @note For now, the only sensor available is the BMP280, otherwise these routines return void.
 * 
 * @version 0.1
 * @date 2022-02-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

uint32_t sampleNumber = 5;
uint8_t sampleCounter = 0;
float samplePressure = 0.0f;

#if ALTITUDE_SENSOR == BMP280


  /**
   * @brief Writes a message to the I2C designed address.
   * 
   * @param reg_address 
   * @param msg 
   */
  void writeRegister(uint8_t reg_address, uint8_t msg){

    Wire.beginTransmission(ALTITUDE_SENSOR_ADDRESS);
    Wire.write(reg_address);
    Wire.write(msg);
    error = Wire.endTransmission();    

  }


  /**
   * @brief Read barometer data from I2C communication.
   * 
   */
  void readTrim(){

    uint8_t data[32], i = 0;                                    // prepare the array

    Wire.beginTransmission(ALTITUDE_SENSOR_ADDRESS);            // start the acquisition from I2C
    Wire.write(0x88);
    Wire.endTransmission();
    Wire.requestFrom(ALTITUDE_SENSOR_ADDRESS, 24);              // read the communication from the altitude sensor

    while(Wire.available()){                                    // fill the data array
      data[i] = Wire.read();
      i++;
    }

    Wire.beginTransmission(ALTITUDE_SENSOR_ADDRESS);    
    Wire.write(0xA1);                          
    Wire.endTransmission();                    
    Wire.requestFrom(ALTITUDE_SENSOR_ADDRESS,1);      

    data[i] = Wire.read();                     
    i++;                                       

    Wire.beginTransmission(ALTITUDE_SENSOR_ADDRESS);
    Wire.write(0xE1);
    Wire.endTransmission();
    Wire.requestFrom(ALTITUDE_SENSOR_ADDRESS,7);      

    while(Wire.available()){
        data[i] = Wire.read();
        i++;    
    }

    dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];
    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11]<< 8) | data[10];
    dig_P4 = (data[13]<< 8) | data[12];
    dig_P5 = (data[15]<< 8) | data[14];
    dig_P6 = (data[17]<< 8) | data[16];
    dig_P7 = (data[19]<< 8) | data[18];
    dig_P8 = (data[21]<< 8) | data[20];
    dig_P9 = (data[23]<< 8) | data[22];
    dig_H1 = data[24];
    dig_H2 = (data[26]<< 8) | data[25];
    dig_H3 = data[27];
    dig_H4 = (data[28]<< 4) | (0x0F & data[29]);
    dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F); 
    dig_H6 = data[31];             

  }
  
  
  /**
   * @brief Check if the altitude sensor is connected with I2C.
   * 
   */
  void checkAltitudeSensor(){

    writeRegister(0xF4, 0x27);
    writeRegister(0xF5, 0xA0);

    while(error != 0){
      Serial.printf("ALTITUDE SENSOR ERROR, not found\n");
      ledcWrite(pwmLedChannel, abs(MAX_DUTY_CYCLE - (int)ledcRead(pwmLedChannel)));
    }

    readTrim();                                     

    #if(DEBUG)
      Serial.println("\nAltitude sensor: OK");
    #endif
  }
  

  /**
   * @brief Calibrates temperature readings.
   * 
   * @param adc_T 
   * @return signed long int
   */
  signed long int calibration_T(signed long int adc_T){

    signed long int var1, var2, T;
    var1 = ((((adc_T >> 3) - ((signed long int)dig_T1<<1))) * ((signed long int)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T>>4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;

    tFine = var1 + var2;
    T = (tFine * 5 + 128) >> 8;
    return T; 
  }

  /**
   * @brief Calibrates pressure readings.
   * 
   * @param adc_P 
   * @return unsigned long int 
   */
  unsigned long int calibration_P(signed long int adc_P)
  {
    signed long int var1, var2;
    unsigned long int P;
    var1 = (((signed long int)tFine)>>1) - (signed long int)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int)dig_P6);
    var2 = var2 + ((var1*((signed long int)dig_P5))<<1);
    var2 = (var2>>2)+(((signed long int)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((signed long int)dig_P2) * var1)>>1))>>18;
    var1 = ((((32768+var1))*((signed long int)dig_P1))>>15);
    if (var1 == 0)
    {
        return 0;
    }    
    P = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
    if(P<0x80000000)
    {
       P = (P << 1) / ((unsigned long int) var1);   
    }
    else
    {
        P = (P / (unsigned long int)var1) * 2;    
    }
    var1 = (((signed long int)dig_P9) * ((signed long int)(((P>>3) * (P>>3))>>13)))>>12;
    var2 = (((signed long int)(P>>2)) * ((signed long int)dig_P8))>>13;
    P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
    return P;
  }

/**
   * @brief Reads pressure data
   * 
   */
  void readPressureData(){

    uint32_t barometerData[8];

    Wire.beginTransmission(ALTITUDE_SENSOR_ADDRESS);
    Wire.write(0xF7);
    Wire.endTransmission();
    Wire.requestFrom(ALTITUDE_SENSOR_ADDRESS, 8);

    int i = 0;

    while(Wire.available()){
        barometerData[i] = Wire.read();
        i++;
    }

    presRaw = (barometerData[0] << 12) | (barometerData[1] << 4) | (barometerData[2] >> 4);
    tempRaw = (barometerData[3] << 12) | (barometerData[4] << 4) | (barometerData[5] >> 4);

    tempCal = calibration_T(tempRaw);
    pressCal = calibration_P(presRaw);
    temperature = (double)tempCal / 100.0;
    
    pressure = (double)pressCal / 100.0;
    
    altitudeMeasure = 44330 * (1 - pow( ((float)pressure/PRESSURE_SEA_LEVEL), (1/5.255)) );

  }

  
  /**
   * @brief Sample the pressure readings.
   * 
   */
  void samplePressureReadings(){

    samplePressure += pressure;                                                 // increment the readings
    sampleCounter = sampleCounter + 1;                                          // increment counter
    
    if(sampleCounter == sampleNumber){                                          // after 10 samplings
      pressureSampled = samplePressure/sampleNumber;                            // create the mean value
      sampleCounter = 0;                                                        // reset
      samplePressure = 0.0f;
    }

  }

  /**
   * @brief Filter barometer readings for a smoother flight.
   * 
   */
  void smoothPressureReadings(){
    //To get a smoother pressure value we will use a 20 location rotating memory.
    pressureTotalAvarage -= pressureRotatingMem[pressureRotatingMemLocation];                          //Subtract the current memory position to make room for the new value.
    pressureRotatingMem[pressureRotatingMemLocation] = pressure;                                      //Calculate the new change between the actual pressure and the previous measurement.
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

    //In the following part a rotating buffer is used to calculate the long term change between the various pressure measurements.
    //This total value can be used to detect the direction (up/down) and speed of the quadcopter and functions as the D-controller of the total PID-controller.
    if (manualAltitudeChange == 1)pressureParachutePrevious = actualPressure * 10;                       //During manual altitude change the up/down detection is disabled.

    parachuteThrottle -= parachuteBuffer[parachuteRotatingMemLocation];                                  //Subtract the current memory position to make room for the new value.
    parachuteBuffer[parachuteRotatingMemLocation] = actualPressure * 10 - pressureParachutePrevious;   //Calculate the new change between the actual pressure and the previous measurement.
    parachuteThrottle += parachuteBuffer[parachuteRotatingMemLocation];                                  //Add the new value to the long term avarage value.
    pressureParachutePrevious = actualPressure * 10;                                                       //Store the current measurement for the next loop.
    parachuteRotatingMemLocation++;                                                                        //Increase the rotating memory location.

    if (parachuteRotatingMemLocation == 30)parachuteRotatingMemLocation = 0;                            //Start at 0 when the memory location 20 is reached.
  }
#else

  void checkAltitudeSensor(){
    return;
  }
  void readTrim(){
    return;
  }
  void writeRegister(){
    return;
  }
  void calibration_T(){
    return;
  }
  void calibration_P(){
    return;
  }
  void smoothPressureReadings(){
    return;
  }
  void samplePressureReadings(){
    return;
  }
  void readPressureData(){ 
    return;
  }

#endif

/**
 * @brief  Transforms the barometer readings into the PID output pulses.
 * 
 * When the throttle stick position is increased or decreased the altitude hold function is partially disabled.
 * The manualAltitudeChange variable will indicate if the altitude of the quadcopter is changed by the pilot.
 * 
 * @param pressureForPID pressure value used for the PID calculations
 */
void calculateAltitudeAdjustmentPID(float pressureForPID){// 
  
  manualAltitudeChange = 0;                                                    //Preset the manualAltitudeChange variable to 0.
  manualThrottle = 0;                                                          //Set the manualThrottle variable to 0.

  if (throttle > 1600) {                                                       //If the throttle is increased above 1600us (60%).
    manualAltitudeChange = 1;                                                  //Set the manualAltitudeChange variable to 1 to indicate that the altitude is adjusted.
    pidAltitudeSetpoint = pressureForPID;                                      //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
    manualThrottle = (throttle - 1600) / 3;                                    //To prevent very fast changes in hight limit the function of the throttle.
  }
  if (throttle < 1400) {                                                       //If the throttle is lowered below 1400us (40%).
    manualAltitudeChange = 1;                                                  //Set the manualAltitudeChange variable to 1 to indicate that the altitude is adjusted.
    pidAltitudeSetpoint = pressureForPID;                                      //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
    manualThrottle = (throttle - 1400) / 5;                                    //To prevent very fast changes in hight limit the function of the throttle.
  }

  //Calculate the PID output of the altitude hold.
  pidAltitudeInput = pressureForPID;                                           //Set the setpoint (pidAltitudeInput) of the PID-controller.
  pidErrorTemp = pidAltitudeInput - pidAltitudeSetpoint;                       //Calculate the error between the setpoint and the actual pressure value.

  //To get better results the P-gain is increased when the error between the setpoint and the actual pressure value increases.
  //The variable pidErrorGainAltitude will be used to adjust the P-gain of the PID-controller.
  pidErrorGainAltitude = 0;                                                    //Set the pidErrorGainAltitude to 0.
  if (pidErrorTemp > 10 || pidErrorTemp < - 10) {                              //If the error between the setpoint and the actual pressure is larger than 10 or smaller then -10.
    pidErrorGainAltitude = (abs(pidErrorTemp) - 10) / 20.0;                    //The positive pidErrorGainAltitude variable is calculated based based on the error.
    if (abs(pidErrorGainAltitude -3.0) > 1e-9)pidErrorGainAltitude = 3;        //To prevent extreme P-gains it must be limited to 3.
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
  if (pidOutputAltitude > PID_MAX_ALTITUDE) pidOutputAltitude = PID_MAX_ALTITUDE;
  else if (pidOutputAltitude < PID_MAX_ALTITUDE * -1) pidOutputAltitude = PID_MAX_ALTITUDE * -1;
}

/**
 * @brief Main routine governing the barometer acquisitions.
 * 
 */
void calculateAltitudeHold(){

  barometerCounter ++;                                                                    // the barometric readings happen in two subsequent loops

  switch (barometerCounter){                                                                                  

    case 1:                                                                                // when the barometerCounter variable is 1.

      readPressureData();                                                                  // get pressure data
      // SEBA__________  05/04/2022
      samplePressureReadings();                                                            // remove the spikes from the barometer data
      break;

    case 2:                                                                                // when the barometer counter is 2   

      //If the altitude hold function is disabled some variables need to be reset to ensure a bumpless start when the altitude hold function is activated again.
      switch (flightMode) {

        case 1:                                                                            // if the altitude hold is disabled

          if(abs(pidAltitudeSetpoint) > 1e-8){                                             // if pidAltitudeSetpoint != 0
            pidAltitudeSetpoint = 0;                                                       // reset the PID altitude setpoint.
            pidOutputAltitude = 0;                                                         // reset the output of the PID controller.
            pidIMemAltitude = 0;                                                           // reset the I-controller.
            manualThrottle = 0;                                                            // set the manualThrottle variable to 0 .
            manualAltitudeChange = 1;                                                      // set the manualAltitudeChange to 1.
          }

          break;
        
        default:                                                                           // if the quadcopter is in altitude mode and flying.

          if (abs(pidAltitudeSetpoint) < 1e-8) pidAltitudeSetpoint = pressureSampled;      // if not yet set, set the PID altitude setpoint.

          calculateAltitudeAdjustmentPID(pressureSampled);                                 // calculate PID for altitude hold
        
      }

      barometerCounter = 0;                                                                // set the barometer counter to 0 for the next measurements.

      break;

    default:                                                                               // if barometerCounter is neither 1 or 2

      barometerCounter = 1;

  }
}


/**
 * @brief Print the barometric readings
 * 
 */
void printBarometer(){
  Serial.printf("pressure: %f,  altitude: %f, temp: %f\n", pressure, altitudeMeasure, temperature);
}