/*	Arduino Tutorial - Testing the Adafruits BMP280 Presure & Temperature Sensor
	I2C Connection: Vin: 5V/3V, GND: GND, SCK: A5 (SCL), SDI: A4 (SDA) 
	Dev: Michalis Vasilakis // Date: 8/3/206 // Info: www.ardumotive.com		*/
#include "BMP280.h"

void readPresTempAlt(Adafruit_BMP280 bmp) {
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