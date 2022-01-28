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

// DroneIno32
// @author: Sebastiano Cocchi

#include "Config.h"
#include "src/Models.h"
#include <Arduino.h>
#include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
#include <EEPROM.h>                        //Include the EEPROM.h library so we can store information onto the EEPROM
#include <SPI.h>                           
#include "Globals.h"

// #if ALTITUDE_SENSOR == BMP280
//     Adafruit_BMP280 bmp; // I2C 
// #endif

void setup(){
  initialize();                            //function at initialize.ino                                    
}

// Loop
void loop(){

  // #if ALTITUDE_SENSOR == BMP290
  //   readPresTempAlt();
  // #endif

  // #if PROXIMITY_SENSOR == HCSR04
  //   dangerAlert();
  // #endif

  //read gyro
  readGyroscopeStatus();                   // see Gyro.ino

  //calculate the gyro values
  calculateAnglePRY();                     // see Gyro.ino

  //print gyro status
  //printGyroscopeStatus();                // see Gyro.ino

  //For starting the motors: throttle low and yaw left (step 1).
  if(receiverInputChannel3 < 1050 && receiverInputChannel4 < 1050) start = 1;//When yaw stick is back in the center position start the motors (step 2).
  if(start == 1 && receiverInputChannel3 < 1050 && receiverInputChannel4 > 1450)   droneStart();
  if(start == 2 &&  receiverInputChannel3 < 1050 &&  receiverInputChannel4 > 1950) start = 0; //Stopping the motors: throttle low and yaw right.

  // calculate PID
  calculatePID();                           // see PID.ino

  // the battery voltage can affect the efficiency 
  //batteryVoltageCompensation();

  // create ESC pulses
  setEscPulses();                           // see ESC.ino
}
