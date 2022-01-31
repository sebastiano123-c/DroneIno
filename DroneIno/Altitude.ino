void checkAltitudeSensor(){
  bool statusBMP = bmp.begin(BMP280_I2C_ADDRESS);
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
