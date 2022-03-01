/**
 * @file Setup.ino
 * @author @sebastiano123-c
 * @brief Setup sketch to check DroneIno sensors, motors, receiver and leds.
 * @version 0.1
 * @date 2022-02-28
 *
* 
 * TERMS OF USE
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * 
 * @note Always remove the propellers and stay away from the motors unless you are 100% certain of what you are doing.
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <Arduino.h>
#include <Wire.h>               
#include <EEPROM.h>             

// BAUD RATE
#define BAUD_RATE 115200

// EEPROM size
#define EEPROM_SIZE 36

// WIRE clock
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
#define PIN_BATTERY_LEVEL 2

// PROXIMITY_SENSOR
#define PIN_PROXIMITY_SENSOR_ECHO 26
#define PIN_PROXIMITY_SENSOR_TRIG 25

// ALTITUDE_SENSOR
#define PIN_ALTITUDE_SENSOR_SDA 36
#define PIN_ALTITUDE_SENSOR_SCL 39

//Declaring Global Variables
byte lastChannel1, lastChannel2, lastChannel3, lastChannel4;
byte lowByte, highByte, type, gyroAddress, error, clockspeedOk;
byte channel1Assign, channel2Assign, channel3Assign, channel4Assign;
byte rollAxis, pitchAxis, yawAxis;
byte receiverCheckByte, gyroCheckByte;
volatile int receiverInputChannel1, receiverInputChannel2, receiverInputChannel3, receiverInputChannel4;
int16_t centerChannel1, centerChannel2, centerChannel3, centerChannel4;
int16_t highChannel1, highChannel2, highChannel3, highChannel4;
int16_t lowChannel1, lowChannel2, lowChannel3, lowChannel4;
int16_t address, calInt;
unsigned long timer, timer1, timer2, timer3, timer4, currentTime;
int16_t gyroPitch, gyroRoll, gyroYaw;
int16_t gyroRollCal, gyroPitchCal, gyroYawCal;

//Setup routine
void setup(){
    Serial.begin(BAUD_RATE);      //Start the serial connetion @ 57600bps
    EEPROM.begin(EEPROM_SIZE);
    delay(1000);

    //setup MPU 6050
    Wire.setClock(WIRE_CLOCK);
    Wire.begin();                                                //Start the I2C as master
    Wire.beginTransmission(0x68);                        //Start communication with the MPU-6050.
    int error = Wire.endTransmission();                              //End the transmission and register the exit status.
    while (error != 0) {                                          //Stay in this loop because the MPU-6050 did not respond.
      error = 1;                                                  //Set the error status to 1.
      delay(4);                                                   //Simulate a 250Hz refresc rate as like the main loop.
      Serial.println("MPU6050 ERROR.");
    }
    
    pinMode(PIN_BATTERY_LED, OUTPUT);

    attachInterrupt(PIN_RECEIVER_1, myISR, CHANGE);
    attachInterrupt(PIN_RECEIVER_2, myISR, CHANGE);
    attachInterrupt(PIN_RECEIVER_3, myISR, CHANGE);
    attachInterrupt(PIN_RECEIVER_4, myISR, CHANGE);

    vTaskDelay(250 / portTICK_PERIOD_MS);               //Give the gyro time to start 
}

//Main program
void loop(){
  
  Serial.println(F(""));
  Serial.println(F("==================================================="));
  Serial.println(F("System check"));
  Serial.println(F("==================================================="));
  vTaskDelay(300 / portTICK_PERIOD_MS);
  Serial.println(F("Checking I2C clock speed."));
  vTaskDelay(300 / portTICK_PERIOD_MS);
  
  Wire.setClock(400000L);                      //Set the I2C clock speed to 400kHz.
  
  #if F_CPU == 16000000L          //If the clock speed is 16MHz include the next code line when compiling
    clockspeedOk = 1;            //Set clockspeedOk to 1
  #endif                          //End of if statement
  
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("Transmitter setup"));
    Serial.println(F("==================================================="));
    vTaskDelay(300 / portTICK_PERIOD_MS);
    Serial.print(F("Checking for valid receiver signals."));
    //Wait 10 seconds until all receiver inputs are valid
    wait_for_receiver();
    Serial.println(F(""));
  }
  //Quit the program in case of an error
  if(error == 0){
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.println(F("Place all sticks and subtrims in the center position within 5 seconds."));
    for(int i = 4;i > 0;i--){
      delay(1000);
      Serial.print(i);
      Serial.print(" ");
    }
    Serial.println(" ");
    //Store the central stick positions
    centerChannel1 = receiverInputChannel1;
    centerChannel2 = receiverInputChannel2;
    centerChannel3 = receiverInputChannel3;
    centerChannel4 = receiverInputChannel4;
    Serial.println(F(""));
    Serial.println(F("Center positions stored."));
    Serial.print(F("Digital input 08 = "));
    Serial.println(receiverInputChannel1);
    Serial.print(F("Digital input 09 = "));
    Serial.println(receiverInputChannel2);
    Serial.print(F("Digital input 10 = "));
    Serial.println(receiverInputChannel3);
    Serial.print(F("Digital input 11 = "));
    Serial.println(receiverInputChannel4);
    Serial.println(F(""));
    Serial.println(F(""));
  }
  if(error == 0){  
    Serial.println(F("Move the throttle stick to full throttle and back to center"));
    //Check for throttle movement
    check_receiver_inputs(1);
    Serial.print(F("Throttle is connected to digital input "));
    Serial.println((channel3Assign & 0b00000111) + 7);
    if(channel3Assign & 0b10000000)Serial.println(F("Channel inverted = yes"));
    else Serial.println(F("Channel inverted = no"));
    wait_sticks_zero();
    
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Move the roll stick to simulate left wing up and back to center"));
    //Check for throttle movement
    check_receiver_inputs(2);
    Serial.print(F("Roll is connected to digital input "));
    Serial.println((channel1Assign & 0b00000111) + 7);
    if(channel1Assign & 0b10000000)Serial.println(F("Channel inverted = yes"));
    else Serial.println(F("Channel inverted = no"));
    wait_sticks_zero();
  }
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Move the pitch stick to simulate nose up and back to center"));
    //Check for throttle movement
    check_receiver_inputs(3);
    Serial.print(F("Pitch is connected to digital input "));
    Serial.println((channel2Assign & 0b00000111) + 7);
    if(channel2Assign & 0b10000000)Serial.println(F("Channel inverted = yes"));
    else Serial.println(F("Channel inverted = no"));
    wait_sticks_zero();
  }
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Move the yaw stick to simulate nose right and back to center"));
    //Check for throttle movement
    check_receiver_inputs(4);
    Serial.print(F("Yaw is connected to digital input "));
    Serial.println((channel4Assign & 0b00000111) + 7);
    if(channel4Assign & 0b10000000)Serial.println(F("Channel inverted = yes"));
    else Serial.println(F("Channel inverted = no"));
    wait_sticks_zero();
  }
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Gently move all the sticks simultaneously to their extends"));
    Serial.println(F("When ready put the sticks back in their center positions"));
    //Register the min and max values of the receiver channels
    register_min_max();
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("High, low and center values found during setup"));
    Serial.print(F("Digital input 08 values:"));
    Serial.print(lowChannel1);
    Serial.print(F(" - "));
    Serial.print(centerChannel1);
    Serial.print(F(" - "));
    Serial.println(highChannel1);
    Serial.print(F("Digital input 09 values:"));
    Serial.print(lowChannel2);
    Serial.print(F(" - "));
    Serial.print(centerChannel2);
    Serial.print(F(" - "));
    Serial.println(highChannel2);
    Serial.print(F("Digital input 10 values:"));
    Serial.print(lowChannel3);
    Serial.print(F(" - "));
    Serial.print(centerChannel3);
    Serial.print(F(" - "));
    Serial.println(highChannel3);
    Serial.print(F("Digital input 11 values:"));
    Serial.print(lowChannel4);
    Serial.print(F(" - "));
    Serial.print(centerChannel4);
    Serial.print(F(" - "));
    Serial.println(highChannel4);
    Serial.println(F("Move stick 'nose up' and back to center to continue"));
    check_to_continue();
  }
    
  if(error == 0){
    //What gyro is connected
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("Gyro search"));
    Serial.println(F("==================================================="));
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    Serial.println(F("Searching for MPU-6050 on address 0x68/104"));
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if(search_gyro(0x68, 0x75) == 0x68){
      Serial.println(F("MPU-6050 found on address 0x68"));
      type = 1;
      gyroAddress = 0x68;
    }
    
    if(type == 0){
      Serial.println(F("No gyro device found!!! (ERROR 3)"));
      error = 1;
    }
    
    else{
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      Serial.println(F(""));
      Serial.println(F("==================================================="));
      Serial.println(F("Gyro register settings"));
      Serial.println(F("==================================================="));
      start_gyro(); //Setup the gyro for further use
    }
  }
  
  //If the gyro is found we can setup the correct gyro axes.
  if(error == 0){
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("Gyro calibration"));
    Serial.println(F("==================================================="));
    Serial.println(F("Don't move the quadcopter!! Calibration starts in 1 seconds"));
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.println(F("Calibrating the gyro, this will take +/- 8 seconds"));
    Serial.print(F("Please wait"));
    //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
    for (calInt = 0; calInt < 2000 ; calInt ++){              //Take 2000 readings for calibration.
      if(calInt % 100 == 0)Serial.print(F("."));                //Print dot to indicate calibration.
      gyro_signalen();                                           //Read the gyro output.
      gyroRollCal += gyroRoll;                                //Ad roll value to gyroRollCal.
      gyroPitchCal += gyroPitch;                              //Ad pitch value to gyroPitchCal.
      gyroYawCal += gyroYaw;                                  //Ad yaw value to gyroYawCal.
      vTaskDelay(4 / portTICK_PERIOD_MS);                                                  //Wait 3 milliseconds before the next loop.
    }
    //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
    gyroRollCal /= 2000;                                       //Divide the roll total by 2000.
    gyroPitchCal /= 2000;                                      //Divide the pitch total by 2000.
    gyroYawCal /= 2000;                                        //Divide the yaw total by 2000.
    
    //Show the calibration results
    Serial.println(F(""));
    Serial.print(F("Axis 1 offset="));
    Serial.println(gyroRollCal);
    Serial.print(F("Axis 2 offset="));
    Serial.println(gyroPitchCal);
    Serial.print(F("Axis 3 offset="));
    Serial.println(gyroYawCal);
    Serial.println(F(""));
    
    Serial.println(F("==================================================="));
    Serial.println(F("Gyro axes configuration"));
    Serial.println(F("==================================================="));
    
    //Detect the left wing up movement
    Serial.println(F("Lift the left side of the quadcopter to a 45 degree angle within 10 seconds"));
    //Check axis movement
    check_gyro_axes(1);
    
    if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(rollAxis & 0b00000011);
      if(rollAxis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));
      Serial.println(F("Put the quadcopter back in its original position"));
      Serial.println(F("Move stick 'nose up' and back to center to continue"));
      check_to_continue();

      //Detect the nose up movement
      Serial.println(F(""));
      Serial.println(F(""));
      Serial.println(F("Lift the nose of the quadcopter to a 45 degree angle within 10 seconds"));
      //Check axis movement
      check_gyro_axes(2);
    }
    if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(pitchAxis & 0b00000011);
      if(pitchAxis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));
      Serial.println(F("Put the quadcopter back in its original position"));
      Serial.println(F("Move stick 'nose up' and back to center to continue"));
      check_to_continue();
      
      //Detect the nose right movement
      Serial.println(F(""));
      Serial.println(F(""));
      Serial.println(F("Rotate the nose of the quadcopter 45 degree to the right within 10 seconds"));
      //Check axis movement
      check_gyro_axes(3);
    }
    if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(yawAxis & 0b00000011);
      if(yawAxis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));
      Serial.println(F("Put the quadcopter back in its original position"));
      Serial.println(F("Move stick 'nose up' and back to center to continue"));
      check_to_continue();
    }
  }
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("LED test"));
    Serial.println(F("==================================================="));
    digitalWrite(PIN_BATTERY_LED, HIGH);
    Serial.println(F("The LED should now be lit"));
    Serial.println(F("Move stick 'nose up' and back to center to continue"));
    check_to_continue();
    digitalWrite(PIN_BATTERY_LED, LOW);
  }
  
  Serial.println(F(""));
  
  if(error == 0){
    Serial.println(F("==================================================="));
    Serial.println(F("Final setup check"));
    Serial.println(F("==================================================="));
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if(receiverCheckByte == 0b00001111){
      Serial.println(F("Receiver channels ok"));
    }
    else{
      Serial.println(F("Receiver channel verification failed!!! (ERROR 6)"));
      error = 1;
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if(gyroCheckByte == 0b00000111){
      Serial.println(F("Gyro axes ok"));
    }
    else{
      Serial.println(F("Gyro exes verification failed!!! (ERROR 7)"));
      Serial.println(gyroCheckByte);
      error = 1;
    }
  }     
  
  if(error == 0){
    //If all is good, store the information in the EEPROM
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("Storing EEPROM information"));
    Serial.println(F("==================================================="));
    Serial.println(F("Writing EEPROM"));
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.println(F("Done!"));
    EEPROM.write(0, centerChannel1 & 0b11111111);
    EEPROM.write(1, centerChannel1 >> 8);
    EEPROM.write(2, centerChannel2 & 0b11111111);
    EEPROM.write(3, centerChannel2 >> 8);
    EEPROM.write(4, centerChannel3 & 0b11111111);
    EEPROM.write(5, centerChannel3 >> 8);
    EEPROM.write(6, centerChannel4 & 0b11111111);
    EEPROM.write(7, centerChannel4 >> 8);
    EEPROM.write(8, highChannel1 & 0b11111111);
    EEPROM.write(9, highChannel1 >> 8);
    EEPROM.write(10, highChannel2 & 0b11111111);
    EEPROM.write(11, highChannel2 >> 8);
    EEPROM.write(PIN_BATTERY_LED, highChannel3 & 0b11111111);
    EEPROM.write(13, highChannel3 >> 8);
    EEPROM.write(14, highChannel4 & 0b11111111);
    EEPROM.write(15, highChannel4 >> 8);
    EEPROM.write(16, lowChannel1 & 0b11111111);
    EEPROM.write(17, lowChannel1 >> 8);
    EEPROM.write(18, lowChannel2 & 0b11111111);
    EEPROM.write(19, lowChannel2 >> 8);
    EEPROM.write(20, lowChannel3 & 0b11111111);
    EEPROM.write(21, lowChannel3 >> 8);
    EEPROM.write(22, lowChannel4 & 0b11111111);
    EEPROM.write(23, lowChannel4 >> 8);
    EEPROM.write(24, channel1Assign);
    EEPROM.write(25, channel2Assign);
    EEPROM.write(26, channel3Assign);
    EEPROM.write(27, channel4Assign);
    EEPROM.write(28, rollAxis);
    EEPROM.write(29, pitchAxis);
    EEPROM.write(30, yawAxis);
    EEPROM.write(31, type);
    EEPROM.write(32, gyroAddress);
    //Write the EEPROM signature
    EEPROM.write(33, 'J'); 
    EEPROM.write(34, 'M');
    EEPROM.write(35, 'B');
    //Commit changes
    EEPROM.commit();
    
    //To make sure evrything is ok, verify the EEPROM data.
    Serial.println(F("Verify EEPROM data"));
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if(centerChannel1 != ((EEPROM.read(1) << 8) | EEPROM.read(0)))error = 1;
    if(centerChannel2 != ((EEPROM.read(3) << 8) | EEPROM.read(2)))error = 1;
    if(centerChannel3 != ((EEPROM.read(5) << 8) | EEPROM.read(4)))error = 1;
    if(centerChannel4 != ((EEPROM.read(7) << 8) | EEPROM.read(6)))error = 1;
    
    if(highChannel1 != ((EEPROM.read(9) << 8) | EEPROM.read(8)))error = 1;
    if(highChannel2 != ((EEPROM.read(11) << 8) | EEPROM.read(10)))error = 1;
    if(highChannel3 != ((EEPROM.read(13) << 8) | EEPROM.read(12)))error = 1;
    if(highChannel4 != ((EEPROM.read(15) << 8) | EEPROM.read(14)))error = 1;
    
    if(lowChannel1 != ((EEPROM.read(17) << 8) | EEPROM.read(16)))error = 1;
    if(lowChannel2 != ((EEPROM.read(19) << 8) | EEPROM.read(18)))error = 1;
    if(lowChannel3 != ((EEPROM.read(21) << 8) | EEPROM.read(20)))error = 1;
    if(lowChannel4 != ((EEPROM.read(23) << 8) | EEPROM.read(22)))error = 1;
    
    if(channel1Assign != EEPROM.read(24))error = 1;
    if(channel2Assign != EEPROM.read(25))error = 1;
    if(channel3Assign != EEPROM.read(26))error = 1;
    if(channel4Assign != EEPROM.read(27))error = 1;
    
    if(rollAxis != EEPROM.read(28))error = 1;
    if(pitchAxis != EEPROM.read(29))error = 1;
    if(yawAxis != EEPROM.read(30))error = 1;
    if(type != EEPROM.read(31))error = 1;
    if(gyroAddress != EEPROM.read(32))error = 1;
    
    if('J' != EEPROM.read(33))error = 1;
    if('M' != EEPROM.read(34))error = 1;
    if('B' != EEPROM.read(35))error = 1;
  
    if(error == 1)Serial.println(F("EEPROM verification failed!!! (ERROR 5)"));
    else Serial.println(F("Verification done"));

    
    Serial.println("");
    Serial.println("========================================");    
    Serial.println("EEPROM DATA");   
    Serial.println(); 
    for(int i = 0; i < EEPROM_SIZE; i++){
      Serial.println(EEPROM.read(i));
    }
    
  }
  
  
  if(error == 0){
    Serial.println(F("Setup is finished."));
    Serial.println(F("You can now calibrate the esc's and upload the YMFC-AL code."));
  }
  else{
   Serial.println(F("The setup is aborted due to an error."));
   Serial.println(F("Check the Q and A page of the YMFC-AL project on:"));
   Serial.println(F("www.brokking.net for more information about this error."));
  }
  while(1);
}

//Search for the gyro and check the Who_am_I register
byte search_gyro(int gyroAddress, int who_am_i){
  Wire.beginTransmission(gyroAddress);
  Wire.write(who_am_i);
  Wire.endTransmission();
  Wire.requestFrom(gyroAddress, 1);
  timer = millis() + 100;
  while(Wire.available() < 1 && timer > millis());
  lowByte = Wire.read();
  address = gyroAddress;
  return lowByte;
}

void start_gyro(){
  //Setup the L3G4200D or L3GD20H
  if(type == 2 || type == 3){
    Wire.beginTransmission(address);                             //Start communication with the gyro with the address found during search
    Wire.write(0x20);                                            //We want to write to register 1 (20 hex)
    Wire.write(0x0F);                                            //Set the register bits as 00001111 (Turn on the gyro and enable all axis)
    Wire.endTransmission();                                      //End the transmission with the gyro

    Wire.beginTransmission(address);                             //Start communication with the gyro (adress 1101001)
    Wire.write(0x20);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address, 1);                                //Request 6 bytes from the gyro
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x20 is set to:"));
    Serial.println(Wire.read(),BIN);

    Wire.beginTransmission(address);                             //Start communication with the gyro  with the address found during search
    Wire.write(0x23);                                            //We want to write to register 4 (23 hex)
    Wire.write(0x90);                                            //Set the register bits as 10010000 (Block Data Update active & 500dps full scale)
    Wire.endTransmission();                                      //End the transmission with the gyro
    
    Wire.beginTransmission(address);                             //Start communication with the gyro (adress 1101001)
    Wire.write(0x23);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address, 1);                                //Request 6 bytes from the gyro
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x23 is set to:"));
    Serial.println(Wire.read(),BIN);

  }
  //Setup the MPU-6050
  if(type == 1){
    
    Wire.beginTransmission(address);                             //Start communication with the gyro
    Wire.write(0x6B);                                            //PWR_MGMT_1 register
    Wire.write(0x00);                                            //Set to zero to turn on the gyro
    Wire.endTransmission();                                      //End the transmission
    
    Wire.beginTransmission(address);                             //Start communication with the gyro
    Wire.write(0x6B);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address, 1);                                //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x6B is set to:"));
    Serial.println(Wire.read(),BIN);
    
    Wire.beginTransmission(address);                             //Start communication with the gyro
    Wire.write(0x1B);                                            //GYRO_CONFIG register
    Wire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                      //End the transmission
    
    Wire.beginTransmission(address);                             //Start communication with the gyro (adress 1101001)
    Wire.write(0x1B);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address, 1);                                //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x1B is set to:"));
    Serial.println(Wire.read(),BIN);

  }
}

void gyro_signalen(){
  if(type == 2 || type == 3){
    Wire.beginTransmission(address);                             //Start communication with the gyro
    Wire.write(168);                                             //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address, 6);                                //Request 6 bytes from the gyro
    while(Wire.available() < 6);                                 //Wait until the 6 bytes are received
    lowByte = Wire.read();                                       //First received byte is the low part of the angular data
    highByte = Wire.read();                                      //Second received byte is the high part of the angular data
    gyroRoll = ((highByte<<8)|lowByte);                         //Multiply highByte by 256 (shift left by 8) and ad lowByte
    if(calInt == 2000)gyroRoll -= gyroRollCal;               //Only compensate after the calibration
    lowByte = Wire.read();                                       //First received byte is the low part of the angular data
    highByte = Wire.read();                                      //Second received byte is the high part of the angular data
    gyroPitch = ((highByte<<8)|lowByte);                        //Multiply highByte by 256 (shift left by 8) and ad lowByte
    if(calInt == 2000)gyroPitch -= gyroPitchCal;             //Only compensate after the calibration
    lowByte = Wire.read();                                       //First received byte is the low part of the angular data
    highByte = Wire.read();                                      //Second received byte is the high part of the angular data
    gyroYaw = ((highByte<<8)|lowByte);                          //Multiply highByte by 256 (shift left by 8) and ad lowByte
    if(calInt == 2000)gyroYaw -= gyroYawCal;                 //Only compensate after the calibration
  }
  if(type == 1){
    Wire.beginTransmission(address);                             //Start communication with the gyro
    Wire.write(0x43);                                            //Start reading @ register 43h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address, 6);                                 //Request 6 bytes from the gyro
    while(Wire.available() < 6);                                 //Wait until the 6 bytes are received
    gyroRoll = Wire.read()<<8|Wire.read();                        //Read high and low part of the angular data
    if(calInt == 2000)gyroRoll -= gyroRollCal;               //Only compensate after the calibration
    gyroPitch = Wire.read()<<8|Wire.read();                       //Read high and low part of the angular data
    if(calInt == 2000)gyroPitch -= gyroPitchCal;             //Only compensate after the calibration
    gyroYaw = Wire.read()<<8|Wire.read();                         //Read high and low part of the angular data
    if(calInt == 2000)gyroYaw -= gyroYawCal;                 //Only compensate after the calibration
  }
}

//Check if a receiver input value is changing within 30 seconds
void check_receiver_inputs(byte movement){
  byte trigger = 0;
  int pulse_length;
  timer = millis() + 30000;
  while(timer > millis() && trigger == 0){
    vTaskDelay(250 / portTICK_PERIOD_MS);
    if(receiverInputChannel1 > 1750 || receiverInputChannel1 < 1250){
      trigger = 1;
      receiverCheckByte |= 0b00000001;
      pulse_length = receiverInputChannel1;
    }
    if(receiverInputChannel2 > 1750 || receiverInputChannel2 < 1250){
      trigger = 2;
      receiverCheckByte |= 0b00000010;
      pulse_length = receiverInputChannel2;
    }
    if(receiverInputChannel3 > 1750 || receiverInputChannel3 < 1250){
      trigger = 3;
      receiverCheckByte |= 0b00000100;
      pulse_length = receiverInputChannel3;
    }
    if(receiverInputChannel4 > 1750 || receiverInputChannel4 < 1250){
      trigger = 4;
      receiverCheckByte |= 0b00001000;
      pulse_length = receiverInputChannel4;
    } 
  }
  if(trigger == 0){
    error = 1;
    Serial.println(F("No stick movement detected in the last 30 seconds!!! (ERROR 2)"));
  }
  //Assign the stick to the function.
  else{
    if(movement == 1){
      channel3Assign = trigger;
      if(pulse_length < 1250)channel3Assign += 0b10000000;
    }
    if(movement == 2){
      channel1Assign = trigger;
      if(pulse_length < 1250)channel1Assign += 0b10000000;
    }
    if(movement == 3){
      channel2Assign = trigger;
      if(pulse_length < 1250)channel2Assign += 0b10000000;
    }
    if(movement == 4){
      channel4Assign = trigger;
      if(pulse_length < 1250)channel4Assign += 0b10000000;
    }
  }
}

void check_to_continue(){
  byte continue_byte = 0;
  while(continue_byte == 0){
    if(channel2Assign == 0b00000001 && receiverInputChannel1 > centerChannel1 + 150)continue_byte = 1;
    if(channel2Assign == 0b10000001 && receiverInputChannel1 < centerChannel1 - 150)continue_byte = 1;
    if(channel2Assign == 0b00000010 && receiverInputChannel2 > centerChannel2 + 150)continue_byte = 1;
    if(channel2Assign == 0b10000010 && receiverInputChannel2 < centerChannel2 - 150)continue_byte = 1;
    if(channel2Assign == 0b00000011 && receiverInputChannel3 > centerChannel3 + 150)continue_byte = 1;
    if(channel2Assign == 0b10000011 && receiverInputChannel3 < centerChannel3 - 150)continue_byte = 1;
    if(channel2Assign == 0b00000100 && receiverInputChannel4 > centerChannel4 + 150)continue_byte = 1;
    if(channel2Assign == 0b10000100 && receiverInputChannel4 < centerChannel4 - 150)continue_byte = 1;
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  wait_sticks_zero();
}

//Check if the transmitter sticks are in the neutral position
void wait_sticks_zero(){
  byte zero = 0;
  while(zero < 15){
    if(receiverInputChannel1 < centerChannel1 + 20 && receiverInputChannel1 > centerChannel1 - 20)zero |= 0b00000001;
    if(receiverInputChannel2 < centerChannel2 + 20 && receiverInputChannel2 > centerChannel2 - 20)zero |= 0b00000010;
    if(receiverInputChannel3 < centerChannel3 + 20 && receiverInputChannel3 > centerChannel3 - 20)zero |= 0b00000100;
    if(receiverInputChannel4 < centerChannel4 + 20 && receiverInputChannel4 > centerChannel4 - 20)zero |= 0b00001000;
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

//Check if the receiver values are valid within 10 seconds
void wait_for_receiver(){
  byte zero = 0;
  timer = millis() + 10000;
  while(timer > millis() && zero < 15){
    if(receiverInputChannel1 < 2100 && receiverInputChannel1 > 900)zero |= 0b00000001;
    if(receiverInputChannel2 < 2100 && receiverInputChannel2 > 900)zero |= 0b00000010;
    if(receiverInputChannel3 < 2100 && receiverInputChannel3 > 900)zero |= 0b00000100;
    if(receiverInputChannel4 < 2100 && receiverInputChannel4 > 900)zero |= 0b00001000;
    vTaskDelay(500 / portTICK_PERIOD_MS);
    Serial.print(F("."));
  }
  if(zero == 0){
    error = 1;
    Serial.println(F("."));
    Serial.println(F("No valid receiver signals found!!! (ERROR 1)"));
  }
  else Serial.println(F(" OK"));
}

//Register the min and max receiver values and exit when the sticks are back in the neutral position
void register_min_max(){
  byte zero = 0;
  lowChannel1 = receiverInputChannel1;
  lowChannel2 = receiverInputChannel2;
  lowChannel3 = receiverInputChannel3;
  lowChannel4 = receiverInputChannel4;
  while(receiverInputChannel1 < centerChannel1 + 20 && receiverInputChannel1 > centerChannel1 - 20)delay(250);
  Serial.println(F("Measuring endpoints...."));
  while(zero < 15){
    if(receiverInputChannel1 < centerChannel1 + 20 && receiverInputChannel1 > centerChannel1 - 20)zero |= 0b00000001;
    if(receiverInputChannel2 < centerChannel2 + 20 && receiverInputChannel2 > centerChannel2 - 20)zero |= 0b00000010;
    if(receiverInputChannel3 < centerChannel3 + 20 && receiverInputChannel3 > centerChannel3 - 20)zero |= 0b00000100;
    if(receiverInputChannel4 < centerChannel4 + 20 && receiverInputChannel4 > centerChannel4 - 20)zero |= 0b00001000;
    if(receiverInputChannel1 < lowChannel1)lowChannel1 = receiverInputChannel1;
    if(receiverInputChannel2 < lowChannel2)lowChannel2 = receiverInputChannel2;
    if(receiverInputChannel3 < lowChannel3)lowChannel3 = receiverInputChannel3;
    if(receiverInputChannel4 < lowChannel4)lowChannel4 = receiverInputChannel4;
    if(receiverInputChannel1 > highChannel1)highChannel1 = receiverInputChannel1;
    if(receiverInputChannel2 > highChannel2)highChannel2 = receiverInputChannel2;
    if(receiverInputChannel3 > highChannel3)highChannel3 = receiverInputChannel3;
    if(receiverInputChannel4 > highChannel4)highChannel4 = receiverInputChannel4;
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

//Check if the angular position of a gyro axis is changing within 10 seconds
void check_gyro_axes(byte movement){
  byte triggerAxis = 0;
  float gyroAngleRoll, gyroAnglePitch, gyroAngleYaw;
  //Reset all axes
  gyroAngleRoll = 0;
  gyroAnglePitch = 0;
  gyroAngleYaw = 0;
  gyro_signalen();
  timer = millis();// + 10000; // milliseconds 
  int frequencyLoop = 250; // (Hz)
  //Serial.print(int());
  int delayMS = 4;
  float incrementCoefficient = 1 / ( 250 *65.5 );  // 0.0000611 = 1 / 65.5 (LSB deg/s) / 250(Hz)
  int angleLimit = 30;

  while(millis() - timer <= 1e4 && gyroAngleRoll > -angleLimit && gyroAngleRoll < angleLimit && gyroAnglePitch > -angleLimit && gyroAnglePitch < angleLimit && gyroAngleYaw > -angleLimit && gyroAngleYaw < angleLimit){
   
    gyro_signalen();

    gyroAngleRoll += gyroRoll * incrementCoefficient;          
    gyroAnglePitch += gyroPitch * incrementCoefficient;
    gyroAngleYaw += gyroYaw * incrementCoefficient;
  

    // --- debug
//    Serial.print(millis()-timer);
//    Serial.print(" ms, ");
//    Serial.print(" R: ");
//    Serial.print(gyroAngleRoll);
//    Serial.print(" P: ");
//    Serial.print(gyroAnglePitch);
//    Serial.print(" Y: ");
//    Serial.println(gyroAngleYaw);
    vTaskDelay(delayMS / portTICK_PERIOD_MS); //Loop is running @ 250(Hz). +/-300us is used for communication with the gyro

    //delayMicroseconds(3700);
  }

  //Assign the moved axis to the corresponding function (pitch, roll, yaw)
  if((gyroAngleRoll < -angleLimit || gyroAngleRoll > angleLimit) && gyroAnglePitch > -angleLimit && gyroAnglePitch < angleLimit && gyroAngleYaw > -angleLimit && gyroAngleYaw < angleLimit){
    gyroCheckByte |= 0b00000001;
    if(gyroAngleRoll < 0)triggerAxis = 0b10000001;
    else triggerAxis = 0b00000001;
  }
  if((gyroAnglePitch < -angleLimit || gyroAnglePitch > angleLimit) && gyroAngleRoll > -angleLimit && gyroAngleRoll < angleLimit && gyroAngleYaw > -angleLimit && gyroAngleYaw < angleLimit){
    gyroCheckByte |= 0b00000010;
    if(gyroAnglePitch < 0)triggerAxis = 0b10000010;
    else triggerAxis = 0b00000010;
  }
  if((gyroAngleYaw < -angleLimit || gyroAngleYaw > angleLimit) && gyroAngleRoll > -angleLimit && gyroAngleRoll < angleLimit && gyroAnglePitch > -angleLimit && gyroAnglePitch < angleLimit){
    gyroCheckByte |= 0b00000100;
    if(gyroAngleYaw < 0)triggerAxis = 0b10000011;
    else triggerAxis = 0b00000011;
  }
  
  if(triggerAxis == 0){
    error = 1;
    Serial.println(F("No angular motion is detected in the last 10 seconds!!! (ERROR 4)"));
  }
  else
  if(movement == 1) rollAxis = triggerAxis;
  if(movement == 2) pitchAxis = triggerAxis;
  if(movement == 3) yawAxis = triggerAxis;
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
          receiverInputChannel1 = currentTime - timer1;                             //Channel 1 is currentTime - timer1.
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
          receiverInputChannel2 = currentTime - timer2;                             //Channel 2 is currentTime - timer2.
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
          receiverInputChannel3 = currentTime - timer3;                             //Channel 3 is currentTime - timer3.
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
          receiverInputChannel4 = currentTime - timer4;                             //Channel 4 is currentTime - timer4.
      }
  }

//Intro subroutine
void intro(){
  Serial.println(F("==================================================="));
  vTaskDelay(1500 / portTICK_PERIOD_MS);
  Serial.println(F(""));
  Serial.println(F("      DroneIno ESP32"));
  vTaskDelay(300 / portTICK_PERIOD_MS);
  Serial.println(F(""));
  Serial.println(F(""));
}
