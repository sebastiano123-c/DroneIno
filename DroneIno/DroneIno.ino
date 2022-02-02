
// DroneIno32
// @author: Sebastiano Cocchi

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

#include <Arduino.h>
#include <Wire.h>                          
#include <EEPROM.h>     
#include "Config.h"
#include "src/Models.h"
#include "Constants.h"            
#include "Globals.h"

void setup(){
  // init function
  initialize();                                        // function at initialize.ino                                    
}

void loop(){                                           // loop runs at 250Hz => each loop last 4000us
  // select mode
  if      (trimCh[5].actual < 1050) flightMode = 1;    // SWC UP: no mode on, (only auto leveling if enabled)
  else if (trimCh[5].actual < 1550 &&
           trimCh[5].actual > 1450) flightMode = 2;    // SWC CENTER: altitude hold 
  else if (trimCh[5].actual < 2050 &&
           trimCh[5].actual > 1950) flightMode = 3;    // SWC DOWN: GPS*
  
  // starting the quadcopter
  if(receiverInputChannel3 < 1050 &&                   // to start the motors: throttle low and yaw left (step 1).
     receiverInputChannel4 < 1050) start = 1;          
  if(start                == 1    &&                   // when yaw stick is back in the center position start the motors (step 2).
     receiverInputChannel3 < 1050 && 
     receiverInputChannel4 > 1450) droneStart();       
  if(start                == 2    &&                   // stopping the motors: throttle low and yaw right.
     receiverInputChannel3 < 1050 &&  
     receiverInputChannel4 > 1950) start = 0;          

  // calculate the gyroscope values for pitch, roll and yaw
  calculateAnglePRY();                                 // see Gyro.ino

  //convert the signal of the rx
  convertAllSignals();                                 // see ESC.ino
  
  // calculate the altitude hold pressure parameters
  calculateAltitudeHold();                             // see Altitude.ino

  // calculate PID values
  calculatePID();                                      // see PID.ino

  // battery voltage can affect the efficiency 
  batteryVoltageCompensation();                        // see Battery.ino

  // create ESC pulses
  setEscPulses();                                      // see ESC.ino

  // finish the loop
  if(micros() - loopTimer > 4050) //ledcWrite(pwmLedChannel, MAX_DUTY_CYCLE);
  {
    ledcWrite(pwmLedChannel, MAX_DUTY_CYCLE);                           //Turn on the LED if the loop time exceeds 4050us.
    if(DEBUG) {
      Serial.print("DANGER: LOOP TIMER > 4us: ");
      Serial.println(micros() - loopTimer );
    }
  }
  else  ledcWrite(pwmLedChannel, 0);
  
  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while(micros() - loopTimer < 4000) ledcWrite(pwmLedFlyChannel, MAX_DUTY_CYCLE);  //We wait until 4000us are passed.
  ledcWrite(pwmLedFlyChannel, 0);
  loopTimer = micros();                                                 //Set the timer for the next loop.
}
