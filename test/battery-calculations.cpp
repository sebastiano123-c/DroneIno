#include <iostream>
#include <math.h>
using namespace std;

const int maxBatteryLevel = 11100;                         // your battery voltage
const int minBatteryLevel = 5000;                          // minimum voltage above which your battery must not go
const int BoardLimitVoltage = 3300;                        // your board maximum pin voltage (ex. esp32 has 3.3V, arduino uno 5V)


const int DIODE_DROP = 600;                                // diode tension drop, generally 0.7V         
float res1 = 2.5;
float res2 = 1.;
float totalDrop = res2 / (res1 + res2);


// voltage is read using pin 
uint8_t adcBits = 12;                                      // (bits) of width when measuring the voltage
float maximumWidth = pow(2., (float)adcBits);
float fromVtoWidth = maximumWidth / BoardLimitVoltage;

float voltagePartitor(int volt){
    return (float)(volt-DIODE_DROP) * totalDrop;
}

float fromWidthToVPin(int width, float corr = 1.){
    return (float)width / (fromVtoWidth * corr);
}

float fromWidthToVBattery(int width, float corr = 1.){
    return (float)width / (totalDrop * fromVtoWidth * corr) + DIODE_DROP;
}

int main(){

    // float correction = BoardLimitVoltage/voltagePartitor(maxBatteryLevel);

    // cout << " MAX vout: " << 10.4*totalDrop << " V \n";
    // cout << "analogRead \t voltage @ pin \t\t battery voltage  \n";

    // for(int pinWidth = 600; pinWidth <= (int)maximumWidth; pinWidth = pinWidth + 100)
    // {
    //     float voltPin = fromWidthToVPin(pinWidth, correction)/1000;
    //     float voltBattery = fromWidthToVBattery(pinWidth, correction)/1000;

    //     if ( abs(voltBattery - (float)minBatteryLevel/1000) < .1 )
    //         cout << "------------------------------------------------- LOWER LIMIT\n";     
    //     cout << pinWidth << " \t \t " << voltPin << "V \t\t " << voltBattery << " V\t" << "\n";
    //     if ( abs(voltBattery - (float)minBatteryLevel/1000) < .1 )
    //         cout << "-------------------------------------------------\n";
    // }


    const int DIODE_DROP          = 700;                                   //generally it is -0.7V
    float res3                    = 2.5;                                   // resistance between Vin and vout
    float res2                    = 1.;                                    // load resistance
    float totalDrop               = res2 / (res2 + res3);                  // IMPORTANT: this is in my case, you have to calculate YOUR total drop

    // digital read bits accuracy
    uint8_t adcBits                 = 12;                                    // (bits) of width when measuring the voltage
    float maximumWidth              = pow(2., (float)adcBits)-1;             // maximum width that the pin can read

    // battery calculations
    float fromVtoWidth              = maximumWidth / (float)BoardLimitVoltage;
    float maxBatteryLevelDropped    = (float)(maxBatteryLevel-DIODE_DROP) * totalDrop;
    float correctionBattery         = (float)BoardLimitVoltage/maxBatteryLevelDropped;
    float minBatteryLevelThreshold  = ((float)minBatteryLevel-(float)DIODE_DROP) * totalDrop * correctionBattery;

    cout << "\n" << correctionBattery << "\t" << maxBatteryLevelDropped*correctionBattery << "\n";
}