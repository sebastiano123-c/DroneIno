
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

void loop(){
  //calculate the gyro values for pitch, roll and yaw
  calculateAnglePRY();                                 // see Gyro.ino

  // starting sequence
  if(receiverInputChannel3 < 1050 &&                   // to start the motors: throttle low and yaw left (step 1).
     receiverInputChannel4 < 1050) start = 1;          
  if(start                == 1    &&                   // when yaw stick is back in the center position start the motors (step 2).
     receiverInputChannel3 < 1050 && 
     receiverInputChannel4 > 1450) droneStart();       
  if(start                == 2    &&                   // stopping the motors: throttle low and yaw right.
     receiverInputChannel3 < 1050 &&  
     receiverInputChannel4 > 1950) start = 0;          

  // calculate PID values
  calculatePID();                                      // see PID.ino

  // battery voltage can affect the efficiency 
  batteryVoltageCompensation();                        // see Battery.ino

  // create ESC pulses
  setEscPulses();                                      // see ESC.ino
}
