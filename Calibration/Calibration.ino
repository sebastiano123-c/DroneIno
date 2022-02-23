///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////
//Safety note
///////////////////////////////////////////////////////////////////////////////////////
//Always remove the propellers and stay away from the motors unless you 
//are 100% certain of what you are doing.
///////////////////////////////////////////////////////////////////////////////////////

// Calibration
// @author: Sebastiano Cocchi
//
// Dial these on the serial
//  r = print receiver signals.
//  a = print quadcopter angles.
//  1 = check rotation / vibrations for motor 1 (right front CCW).
//  2 = check rotation / vibrations for motor 2 (right rear CW).
//  3 = check rotation / vibrations for motor 3 (left rear CCW).
//  4 = check rotation / vibrations for motor 4 (left front CW).
//  5 = check vibrations for all motors together.

#include <Arduino.h>
#include <Wire.h>                                    //Include the Wire.h library so we can communicate with the gyro.
#include <EEPROM.h>                                  //Include the EEPROM.h library so we can store information onto the EEPROM

#define BAUD_RATE 115200 // 57600  
#define EEPROM_SIZE 36
#define WIRE_CLOCK 400000L

// ESC
#define PIN_ESC_1 17
#define PIN_ESC_2 16
#define PIN_ESC_3 27
#define PIN_ESC_4 14

// RECEIVER
#define PIN_RECEIVER_1 12
#define PIN_RECEIVER_2 13
#define PIN_RECEIVER_3 5
#define PIN_RECEIVER_4 23

// LED
#define PIN_BATTERY_LED 19 
#define PIN_DIGITAL_13 18 

// BATTERY LEVEL
#define PIN_BATTERY_LEVEL 4

// PROXIMITY_SENSOR
#define PIN_PROXIMITY_SENSOR_ECHO 26
#define PIN_PROXIMITY_SENSOR_TRIG 25

// ALTITUDE_SENSOR
#define PIN_ALTITUDE_SENSOR_SDA 36
#define PIN_ALTITUDE_SENSOR_SCL 39

//Declaring global variables
byte lastChannel1, lastChannel2, lastChannel3, lastChannel4;
byte eepromData[36], start, data;
boolean new_function_request,firstAngle;
volatile int receiverInputChannel1, receiverInputChannel2, receiverInputChannel3, receiverInputChannel4;
int esc_1, esc_2, esc_3, esc_4;
int counterChannel1, counterChannel2, counterChannel3, counterChannel4;
int receiverInput[5];
int loopCounter, gyroAddress, vibrationCounter;
long accX, accY, accZ, accTotalVector[20], accAvVector, vibrationTotalResult;
unsigned long timerChannel1, timerChannel2, timerChannel3, timerChannel4, escTimer, escLoopTimer;
unsigned long zeroTimer, timer1, timer2, timer3, timer4, currentTime;

//double gyroAxis[2], gyroAxis[1], gyroAxis[3];
float angleRollAcc, anglePitchAcc, anglePitch, angleRoll;
int calInt;
int16_t accAxis[4], gyroAxis[4], temperature;
int32_t gyroAxisCal[4], accAxisCcal[4];

//fromATmega32ToEsp32
float fromATmega32ToEsp32 = 240 / 16; //  (esp32 MCU speed) / (ATmega32 MCU speed ) in MHz
float accCorrection = 1; //  (esp32 MCU speed) / (ATmega32 MCU speed ) in MHz
float gyroCorrection = 1; //  (esp32 MCU speed) / (ATmega32 MCU speed ) in MHz

//gyro contant for communication
const int gyroFrequency = 250; // (Hz)
const float gyroSensibility = 65.5; //
float travelCoeff = 1/((float)gyroFrequency * gyroSensibility);
float convDegToRad = 180.0 / PI;
float travelCoeffToRad = travelCoeff / convDegToRad;

//Setup routine
void setup(){
  //begin
  Serial.begin(BAUD_RATE);                                                                  //Start the serial port.
  EEPROM.begin(EEPROM_SIZE);
  vTaskDelay(500/portTICK_PERIOD_MS);

  //fill EEPROM data array
  for(data = 0; data < EEPROM_SIZE; data++) eepromData[data] = EEPROM.read(data);               //Read EEPROM for faster data access

  gyroAddress = 0x68;//eepromData[32];                                                       //Store the gyro address in the variable.

  setupMPU();                                                                                 //setup MPU 6050

  //print eeprom data on serial
  Serial.println("==================================");
  Serial.println("EEPROM data:");
  Serial.println("");
  for (int i = 0; i < EEPROM_SIZE; i++){
    Serial.println(eepromData[i]);
  }

  //Arduino Uno pins default to inputs, so they don't need to be explicitly declared as inputs.
  pinMode(PIN_ESC_1, OUTPUT);
  pinMode(PIN_ESC_2, OUTPUT);
  pinMode(PIN_ESC_3, OUTPUT);
  pinMode(PIN_ESC_4, OUTPUT);
  pinMode(PIN_BATTERY_LED, OUTPUT);
  pinMode(PIN_DIGITAL_13, OUTPUT);

  //event change detector
  pinMode(PIN_RECEIVER_1, INPUT_PULLUP);
  pinMode(PIN_RECEIVER_2, INPUT_PULLUP);
  pinMode(PIN_RECEIVER_3, INPUT_PULLUP);
  pinMode(PIN_RECEIVER_4, INPUT_PULLUP);
  
  //       event change detector
  attachInterrupt(digitalPinToInterrupt(PIN_RECEIVER_1), myISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RECEIVER_2), myISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RECEIVER_3), myISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RECEIVER_4), myISR, CHANGE);
  set_gyro_registers();                                                                 //Set the specific gyro registers.

  //Check the EEPROM signature to make sure that the setup program is executed.
  while(eepromData[33] != 'J' || eepromData[34] != 'M' || eepromData[35] != 'B'){
    vTaskDelay(500/portTICK_PERIOD_MS);                                                                         //Wait for 500ms.
    digitalWrite(PIN_BATTERY_LED, !digitalRead(PIN_BATTERY_LED));                                                 //Change the led status to indicate error.
  }

  //wait_for_receiver();                                                                  //Wait until the receiver is active.
  
  zeroTimer = micros();                                                                //Set the zeroTimer for the first loop.
  while(Serial.available())data = Serial.read();                                        //Empty the serial buffer.
  data = 0;                                                                             //Set the data variable back to zero.
  Serial.println("END SETUP");
}

//Main program loop
void loop(){
  while(zeroTimer + 4000 > micros());                                                  //Start the pulse after 4000 micro seconds.
  zeroTimer = micros();                                                                //Reset the zero timer.

  if(Serial.available() > 0){
    data = Serial.read();                                                               //Read the incoming byte.
    vTaskDelay(100/portTICK_PERIOD_MS);                                                                         //Wait for any other bytes to come in
    while(Serial.available() > 0)loopCounter = Serial.read();                          //Empty the Serial buffer.
    new_function_request = true;                                                        //Set the new request flag.
    loopCounter = 0;                                                                   //Reset the loopCounter variable.
    calInt = 0;                                                                        //Reset the calInt variable to undo the calibration.
    start = 0;                                                                          //Set start to 0.
    firstAngle = false;                                                                //Set firstAngle to false.
    //Confirm the choice on the serial monitor.
    if(data == 'r') Serial.println("Reading receiver signals.");
    if(data == 'a') Serial.println("Print the quadcopter angles.");
    if(data == 'a') Serial.println("Gyro calibration starts in 2 seconds (don't move the quadcopter).");
    if(data == '1') Serial.println("Test motor 1 (right front CCW.)");
    if(data == '2') Serial.println("Test motor 2 (right rear CW.)");
    if(data == '3') Serial.println("Test motor 3 (left rear CCW.)");
    if(data == '4') Serial.println("Test motor 4 (left front CW.)");
    if(data == '5') Serial.println("Test all motors together");

    //Let's create a small delay so the message stays visible for 2.5 seconds.
    //We don't want the ESC's to beep and have to send a 1000us pulse to the ESC's.
    for(vibrationCounter = 0; vibrationCounter < 625; vibrationCounter++){           //Do this loop 625 times
      vTaskDelay(3/portTICK_PERIOD_MS);                                                                         //Wait 3000us.
      esc_1 = 1000;                                                                     //Set the pulse for ESC 1 to 1000us.
      esc_2 = 1000;                                                                     //Set the pulse for ESC 2 to 1000us.
      esc_3 = 1000;                                                                     //Set the pulse for ESC 3 to 1000us.
      esc_4 = 1000;                                                                     //Set the pulse for ESC 4 to 1000us.
      esc_pulse_output();                                                               //Send the ESC control pulses.
    }
    vibrationCounter = 0;                                                              //Reset the vibrationCounter variable.
  }

  receiverInputChannel3 = convert_receiverChannel(3);                               //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
  if(receiverInputChannel3 < 1025)new_function_request = false;                      //If the throttle is in the lowest position set the request flag to false.


  ////////////////////////////////////////////////////////////////////////////////////////////
  //Run the ESC calibration program to start with.
  ////////////////////////////////////////////////////////////////////////////////////////////
  if(data == 0 && new_function_request == false){                                       //Only start the calibration mode at first start. 
    receiverInputChannel3 = convert_receiverChannel(3);                             //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
    esc_1 = receiverInputChannel3;                                                   //Set the pulse for motor 1 equal to the throttle channel.
    esc_2 = receiverInputChannel3;                                                   //Set the pulse for motor 2 equal to the throttle channel.
    esc_3 = receiverInputChannel3;                                                   //Set the pulse for motor 3 equal to the throttle channel.
    esc_4 = receiverInputChannel3;                                                   //Set the pulse for motor 4 equal to the throttle channel.
    esc_pulse_output();                                                                 //Send the ESC control pulses.
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  //When user sends a 'r' print the receiver signals.
  ////////////////////////////////////////////////////////////////////////////////////////////
  if(data == 'r'){
    rFunction();
  }

  ///////////////////////////////////////////////////////////////////////////////////////////
  //When user sends a '1, 2, 3, 4 or 5 test the motors.
  ////////////////////////////////////////////////////////////////////////////////////////////
  if(data == '1' || data == '2' || data == '3' || data == '4' || data == '5'){          //If motor 1, 2, 3 or 4 is selected by the user.
    escFunction();
  }
  ///////////////////////////////////////////////////////////////////////////////////////////
  //When user sends a 'a' display the quadcopter angles.
  ////////////////////////////////////////////////////////////////////////////////////////////
  if(data == 'a'){
    checkGyro();
  }
}


void myISR(){
      currentTime = micros();
      //Channel 1=========================================
      if(digitalRead(PIN_RECEIVER_1) == HIGH){                                                     //Is input 8 high?
          if(lastChannel1 == 0){                                                //Input 8 changed from 0 to 1.
              lastChannel1 = 1;                                                   //Remember current input state.
              timer1 = currentTime;                                               //Set timer1 to currentTime.
          }
      }
      else if(lastChannel1 == 1){                                             //Input 8 is not high and changed from 1 to 0.
          lastChannel1 = 0;                                                     //Remember current input state.
          receiverInput[1] = currentTime - timer1;                             //Channel 1 is currentTime - timer1.
      }
      //Channel 2=========================================
      if(digitalRead(PIN_RECEIVER_2) == HIGH){                                                    //Is input 9 high?
          if(lastChannel2 == 0){                                                //Input 9 changed from 0 to 1.
              lastChannel2 = 1;                                                   //Remember current input state.
              timer2 = currentTime;                                               //Set timer2 to currentTime.
          }
      }
      else if(lastChannel2 == 1){                                             //Input 9 is not high and changed from 1 to 0.
          lastChannel2 = 0;                                                     //Remember current input state.
          receiverInput[2] = currentTime - timer2;                             //Channel 2 is currentTime - timer2.
      }
      //Channel 3=========================================
      if(digitalRead(PIN_RECEIVER_3) == HIGH){                                                    //Is input 10 high?
          if(lastChannel3 == 0){                                                //Input 10 changed from 0 to 1.
              lastChannel3 = 1;                                                   //Remember current input state.
              timer3 = currentTime;                                               //Set timer3 to currentTime.
          }
      }
      else if(lastChannel3 == 1){                                             //Input 10 is not high and changed from 1 to 0.
          lastChannel3 = 0;                                                     //Remember current input state.
          receiverInput[3] = currentTime - timer3;                             //Channel 3 is currentTime - timer3.
      }
      //Channel 4=========================================
      if(digitalRead(PIN_RECEIVER_4) == HIGH){                                                    //Is input 11 high?
          if(lastChannel4 == 0){                                                //Input 11 changed from 0 to 1.
              lastChannel4 = 1;                                                   //Remember current input state.
              timer4 = currentTime;                                               //Set timer4 to currentTime.
          }
      }
      else if(lastChannel4 == 1){                                             //Input 11 is not high and changed from 1 to 0.
          lastChannel4 = 0;                                                     //Remember current input state.
          receiverInput[4] = currentTime - timer4;                             //Channel 4 is currentTime - timer4.
      }
  }
