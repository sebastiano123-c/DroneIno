#include "AutoLevel.h"

void autoLevelLoop(AutoLeveling myAutoLev){
    // calculate the initial parameters
    myAutoLev.setAutoLevelParameters();

    //For starting the motors: throttle low and yaw left (step 1).
    if(myAutoLev.receiverInputChannel3 < 1050 && myAutoLev.receiverInputChannel4 < 1050) myAutoLev.start = 1;

    //When yaw stick is back in the center position start the motors (step 2).
    if(myAutoLev.start == 1 && myAutoLev.receiverInputChannel3 < 1050 && myAutoLev.receiverInputChannel4 > 1450) myAutoLev.startAutoLeveling();

    //Stopping the motors: throttle low and yaw right.
    if(myAutoLev.start == 2 &&  myAutoLev.receiverInputChannel3 < 1050 &&  myAutoLev.receiverInputChannel4 > 1950) myAutoLev.start = 0;

    //set PID parameters
    myAutoLev.setPID();

    // calculate PID
    myAutoLev.calculatePID();

    // the battery voltage can affect the efficiency 
    myAutoLev.batteryVoltageCompensation();

    // create ESC pulses
    myAutoLev.setEscPulses();
}