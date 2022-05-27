<!-- image -->
<pre align=center>
    |                      |   
    |                      |   
 ---4---                ---1---
    |\                    /|   
    | \                  / |   
       \                /      
         ---------------       
         |  DRONEINO!  |       
         ---------------       
       /                \      
    | /                  \ |   
    |/                    \|   
 ---3---                ---2---
    |                      |   
    |                      |   
</pre>

<!-- title -->
<h1 align="center">
  <b> 
    DroneIno - Full documentation
  </b>
</h1>
<h3 align="center"> 
  <i>
    Arduino code for ESP32 based DIY quadcopter drones.
  </i>
</h3>
<br>

# <ins>**_Disclaimer_**</ins>
The device you are programming can be dangerous, use this code if you are an expert at your own risk.
Thus, *test it first without propellers*.

Read carefully this documentation before starting to fly.
Make sure to do all the passages described here below.

# **Table of contents**
- [<ins>**_Disclaimer_**</ins>](#insdisclaimerins)
- [**Table of contents**](#table-of-contents)
- [**Usage**](#usage)
- [**Abstract**](#abstract)
- [**Features**](#features)
  - [**Selecting flight mode**](#selecting-flight-mode)
- [**Documentation**](#documentation)
  - [**ESP32 D1 R32 - Circuit scheme**](#esp32-d1-r32---circuit-scheme)
  - [**ESP32 DevKit V1 - Circuit scheme**](#esp32-devkit-v1---circuit-scheme)
  - [**Hardware**](#hardware)
  - [**Pinmap**](#pinmap)
  - [**Configuration**](#configuration)
  - [**Setup**](#setup)
  - [**Calibration**](#calibration)
    - [**ESCs calibration**](#escs-calibration)
    - [**Propellers calibration**](#propellers-calibration)
  - [**Flight controller**](#flight-controller)
    - [**Start and stop commands**](#start-and-stop-commands)
  - [**PID manual tuning**](#pid-manual-tuning)
    - [**Step 1: reset variables**](#step-1-reset-variables)
    - [**Step 2: yaw tuning**](#step-2-yaw-tuning)
    - [**Step 3: roll/pitch tuning**](#step-3-rollpitch-tuning)
  - [**PID auto-tuning**](#pid-auto-tuning)
    - [**How auto-tuning works: back propagation (BP) neural network (NN)**](#how-auto-tuning-works-back-propagation-bp-neural-network-nn)
- [**DroneInoTelemetry web app**](#droneinotelemetry-web-app)
  - [**Connection using only ESP32**](#connection-using-only-esp32)
  - [**Connection using ESP32-CAM**](#connection-using-esp32-cam)
    - [**SD card for data storage**](#sd-card-for-data-storage)
- [**Altitude hold**](#altitude-hold)
- [**GPS**](#gps)
- [**Appendix A: previous version**](#appendix-a-previous-version)
- [**Appendix B: another circuit scheme**](#appendix-b-another-circuit-scheme)
- [**Appendix C: components voltages**](#appendix-c-components-voltages)
- [**Author**](#author)

# **Usage**
Clone this repo
<pre><code>git clone git@github.com:sebastiano123-c/DroneIno.git
</code></pre>
or download the .zip file.

# **Abstract**
Based on the [YMFC](https://github.com/F4b1-/YMFC-AL-Flight-Controller-improved) project, DroneIno allows you to control a quadcopter with:
* ESP32 D1 R32 board, like [this](https://github.com/sebastiano123-c/Motorize-a-1980-telescope/blob/main/Setup/D1%20R32%20Board%20Pinout.pdf);
* gyroscope MPU6050;
* radio controller and a receiver (_e.g._ FlySky);
* 11.1V 3S >20C LiPo battery (2200mAh or 3800mAh);
* four 30A ESCs;
* four brushless DC motors (around 1000KV).
  
Optional addons:
* ESP32-CAM;
* BMP280 pressure sensor.

To build DroneIno I followed these [videos](https://www.youtube.com/watch?v=XFxqFQwRumc&list=PL0K4VDicBzsibZqfa42DVxC8CGCMB7G2G) explaining how to build the YMFC and I recommend you to do the same.

<!-- <pre>
CIRCUITAL SCHEMATIC HERE 
</pre> -->
<!-- 
# Boards
- ESP32 D1 R32 -->

# **Features**
1) Auto-leveling
2) Altitude hold
3) GPS (**_NOT TO USE AT THE MOMENT_**)
4) WiFi telemetry system
5) SD flight data storage
<!-- - : the drone corrects spurious drifts using the gyroscope signals -->

## **Selecting flight mode**
Set the SWC switch of the transmitter to channel 5 and connect the receiver channel to the desired pin (mine is the GPIO 4, see the circuit scheme):
* SWC up: flight mode selected is only auto-leveling;
* SWC center: flight mode selected is altitude hold (and auto-leveling if enabled);
* SWC down: flight mode selected is GPS (**_NOT TO USE AT THE MOMENT_**).


# **Documentation**
I first start drawing the circuital scheme for:
* [ESP32 D1 R32](https://github.com/sebastiano123-c/Motorize-a-1980-telescope/blob/main/Setup/D1%20R32%20Board%20Pinout.pdf) (_note: the Wemos D1 R32 has shown problems with UART communication, so I recommend to choose an AZ-Delivery board_);
* [ESP32 DevKit](https://www.espressif.com/en/products/devkits/esp32-devkitc).

_ALERT READ CAREFULLY_
Some boards come with no protection circuit.
Thus, if you power your board via both USB and externally using a power supply, you will fry your board.
To eradicate this issue from the roots, I recommend to use separate connectors: one for the motors and
one for the board.
So, when calibrating the propellers, connect the battery to the motor BUT NOT TO THE BOARD.

## **ESP32 D1 R32 - Circuit scheme**
The power is supplied by the 11.1V 2200mAh (3800 should be better) 3s 20C LiPo battery.
The battery powers the motors and the ESP32.
Battery voltage is measured by GPIO-39 using a voltage divider with R1 = 5.1 kOhm and R2 = 1.55 kOhm.
Take a look at the [batteryCalc](https://github.com/sebastiano123-c/DroneIno/tree/main/test/batteryCalc.cpp) file to calculate the voltage divider.

Consider that each of the 3 cells of the battery can have a maximum voltage of 4.2V, thus the total battery voltage should be 4.2V*3 = 12.6V.
**Warning:** *Do not discharge the battery's cells under 3V, thus never let our battery go under the 3V*3 = 9V.*

ESP32 power is supplied from a voltage regulator (Vreg) connected to the battery.
Set the maximum output voltage to 11.5V or so.
The reason is that ESP32 VIN+ should never be supplied more than 12V.
*Please note that a circuit involving a diode (lighter than a Vreg) is reported in Appendix B.*

The resistance R3 = 330 Ohm is used for the LED. 
<pre>
    LEGEND:                                                 +-----+-------R1-----+     
    X-  = disconnected                                      |     |              |                      
    -+- = sold cables                                       |     R2             |         _____________________ 
    -|- = not touching cables                               |  +--|------Vreg----+-----++==|+VCC|11.1V, >= 20C | 
    === = 3-5A cables                                       |  |  +----------------++==||==|-GND|2200mAh, 3s   | 
                                                            |  |  |                ||  ||  ### LiPo BATTERY ####
                 +---------------------------------+        |  |  |   _________    ||  ||              _____    
                 | +-------------------------------|--------|--|--|-->|°INPUT |====||==||=============/     \
                 | | +-----------------------------|-----+  |  |  | X-|+VIN   |====||==||=============| M1  |
                 | | | +---------------------------|---+ |  |  |  +-->|-GND   |====||==||=============\ CCW /
        ______   | | | | +-------------------------|-+ | |  |  |  |   |      +|====||==++                    
+------>|SDA°|   | | | | |   _____________________ | | | |  |  |  |   |      -|====++  ||                    
| +---->|SCL°|   | | | | |   |°IO03         IO39°|<|-|-|-|--+  |  |   ##ESC-1##    ||  ||                    
| |     |VCC+|<--------------------------+  IO38°| | | | |     |  |   _________    ||  ||              _____  
| |   +>|GND-|   | | | | |   |°IO26      |  IO34°| | | | +-----|--|-->|°INPUT |====||==||=============/     \
| |   | #baro#   | | | | |   |°IO25      |  IO04°|<+ | |       |  | X-|+VIN   |====||==||=============| M2  |
| |   |          | | | | +-->|°IO17      |  IO02°|   | |       |  +-->|-GND   |====||==||=============\ CW  /
| |   |          | | | +---->|°IO16      |       |   | |       |  |   |      +|====||==++                    
| |   |          | | +------>|°IO27      |   VIN+|<--|-|-------+  |   |      -|====++  ||                    
| |   |  _______ | +-------->|°IO14      |   GND-|<--|-|----------+   ##ESC-2##    ||  ||                    
| |   |  | CH5°|<+           |           |   GND-|   | |          |   _________    ||  ||              _____ 
| |   |  | CH1°|<----------->|°IO12      |    5V°|<+ | +----------|-->|°INPUT |====||==||=============/     \
| |   |  | CH2°|<----------->|°IO13      +-->3V3°| | |            | X-|+VIN   |====||==||=============| M3  |
| |   |  | CH3°|<----------->|°IO05          RST°| | |            +-->|-GND   |====||==||=============\ CCW /
| |   |  | CH4°|<----------->|°IO23           5V°| | |            |   |      +|====||==++                    
| | +-|->| VIN+|       +---->|°IO19  (not in  OD°| | |            |   |      -|====++  ||                    
| | | +->| GND-|       R3 +->|°IO18    scale)    | | |            |   ##ESC-3##    ||  ||                    
| | | |  ##RX ##       |  |  |-GND               | | |            |   _________    ||  ||              _____ 
| | | |  _______     (LED)|  |°RST               | | +------------|-->|°INPUT |====||==||=============/     \
+-|-|-|->| SDA°|<------|--|->|°SDA (GPIO 21)     | |              | X-|+VIN   |====||==||=============| M4  |
  +-|-|->| SCL°|<------|--|->|°SCL (GPIO 22)     | |              +-->|-GND   |====||==||=============\ CW  /
    +-|->| VCC+|       |  |  #### ESP32 D1 R32 ### |              |   |      +|====||==++                  
    | +->| GND-|       |  R3                       |              |   |      -|====++                        
    | |  #gyro #       | (LEDf)                    |              |   ##ESC-4##                              
    | +----------------+--+------------------------|--------------+                                            
    +----------------------------------------------+                                                                       
</pre>

## **ESP32 DevKit V1 - Circuit scheme**
The ESP32 DevKit seems to offer a more compact solution, if you are looking to compactness.
The battery specifications are the same as before.
Here I use a 1n4007 diode D1, which drops the battery voltage about 0.7V.
The Vin pin is the 5V pin. This can be problem if you need 5V for some component.
In this case, you can also add a voltage regulator (Vreg) after the diode and tune it to give 5V.
I recommend to use the voltage divider composed by the resistances R1 = 5.32kOhm (series of a 5.1kOhm and 0.22Ohm) and R2 = 2kOhm.
With this you will obtain a maximum input voltage of 3.25V.
Otherwise, take a look at the [batteryCalc](https://github.com/sebastiano123-c/DroneIno/tree/main/test/batteryCalc.cpp) file to calculate the voltage divider.
For the LEDs, R3 = 330Ohm.

_Note_: with this scheme BE VERY CAREFUL, when connecting the USB cable disconnect the battery otherwise the battery will be damaged.
<pre>
    LEGEND:                                                 +---+-----R1-----+                                   
    X-  = disconnected                                      |   |            |                                    
    -+- = sold cables                                       |   R2           |           ____________________ 
    -|- = not touching cables                               | +-|------------+--D1---++==| + |11.1V, >= 20C | 
    === = 3-5A cables                                       | | +----------------++==||==| - |2200mAh, 3s   | 
   - \- = switch                                            | | |                ||  ||  ### LiPo BATTERY ###
                                                            | | |   _________    ||  ||              _____    
                   +----------------------------------------|-|-|-->|°INPUT |====||==||=============/     \
                   | +------------------------------------+ | | | X-|+VIN   |====||==||=============| M1  |
                   | | +--------------------------------+ | | | +-->|-GND   |====||==||=============\ CCW /
                   | | | +----------------------------+ | | | | |   |      +|====||==++                    
                   | | | |   ____________________     | | | | | |   |      -|====++  ||                    
 +-----------------|-|-|-|-->|°3V3          GND°|<----|-|-|-|-|-+   ##ESC-1##    ||  ||                    
 |   _______       | | | |   |°RST    IO23(SCL)°|<--+ | | | | | |   _________    ||  ||              _____  
 |   | CH1°|<------|-|-|-|-->|°IO36        IO22°|   | | | +-|-|-|-->|°INPUT |====||==||=============/     \
 |   | CH2°|<------|-|-|-|-->|°IO39   IO01(TX0)°|   | | |   | | | X-|+VIN   |====||==||=============| M2  |
 |   | CH3°|<------|-|-|-|-->|°IO34   IO03(RX0)°|   | | |   | | +-->|-GND   |====||==||=============\ CW  /
 |   | CH4°|<------|-|-|-|-->|°IO35   IO21(SDA)°|<+ | | |   | | |   |      +|====||==++                    
 |   | CH5°|<------|-|-|-|-->|°IO32         GND°| | | | |   | | |   |      -|====++  ||                    
 +-->| 3V3+|       | | | +-->|°IO33        IO19°| | | | |   | | |   ##ESC-2##    ||  ||                    
 |   | GND-|<---+  | | +---->|°IO25        IO18°| | | | |   | | |   _________    ||  ||              _____ 
 |   ##RX ##    |  | +------>|°IO26        IO05°| | | | +---|-|-|-->|°INPUT |====||==||=============/     \
 |      ______  |  +-------->|°IO27   IO17(TX1)°| | | |     | | | X-|+VIN   |====||==||=============| M3  |
 | +--->|SCL°|  +-+    +---->|°IO14   IO16(RX1)°| | | |     | | +-->|-GND   |====||==||=============\ CCW /
 | | +->|SDA°|    |    |  +->|°IO12        IO04°| | | |     | | |   |      +|====||==++                    
 +-|-|->|3V3+|    +----|--|->|°GND         IO00°| | | |     | | |   |      -|====++  ||                    
 | | |  |GND-|<---+    R3 |  |°IO13 <-------------|-|-|-----+ | |   ##ESC-3##    ||  ||                    
 | | |  #baro#    |    |  |  |°IO09        IO15°| | | |       | |   _________    ||  ||              _____ 
 | | |   ______   |  (LED)|  |°IO10        IO08°| | | +-------|-|-->|°INPUT |====||==||=============/     \
 | | +-->|SDA°|   |    |  |  |°IO11        IO07°| | |         | | X-|+VIN   |====||==||=============| M4  |
 | +-|-->|SCL°|   |    |  |  |°5V <---------------|-|---- \---+ +-->|-GND   |====||==||=============\ CW  /
 +-|-|-->|3V3+|   |    |  R3 ### ESP32 DevKit ### | |               |      +|====||==++                     
   | |   |GND-|<--+    |  |                       | |               |      -|====++                        
   | |   #gyro#        | (LED)                    | |               ##ESC-4##                              
   | +----------------+---+-----------------------+ |                                                          
   +------------------------------------------------+                                                           
</pre>

After building the circuit, place it on the drone and proceed with the following passages.

## **Hardware**
For each circuit I provide a [circuit-housing](https://github.com/sebastiano123-c/DroneIno/tree/main/DroneIno/hardware) to be printed using a 3D printer for drones of te type F450. 

## **Pinmap**
Check that your pinmap corresponds to the one defined in the [pinmap](https://github.com/sebastiano123-c/DroneIno/tree/main/DroneIno/include/pinmaps) folder, otherwise change it.

## **Configuration**
Take a look at the [Config.h](https://github.com/sebastiano123-c/DroneIno/tree/main/DroneIno/src/Config.h) file where you can set the hardware components, the battery minimum voltage, the sensors you use and others.

## **Setup**
Write "UPLOADED_SKETCH  SETUP" in the [Config.h](https://github.com/sebastiano123-c/DroneIno/tree/main/DroneIno/src/Config.h) and upload the sketch to your board.
This step, battery remains still disconnected.

## **Calibration**
If the setup sketch succeeds, write "UPLOADED_SKETCH  CALIBRATION" in the [Config.h](https://github.com/sebastiano123-c/DroneIno/tree/main/DroneIno/src/Config.h) and upload the sketch.
To check the propellers direction and the propeller calibration you can connect the battery to the motors.

_Note_: DO NOT connect the battery to the board (but only to the motors) in this configuration since the USB is connected. 

Send the following characters to the serial monitor:
* **r** to check if the transmitter signal is decoded correctly; move the trim and check that:
  * throttle: low 1000us| full    2000us;
  * roll:     left 1000us| right 2000us;
  * pitch:    left 1000us| right 2000us;
  * yaw:      left 1000us| right 2000us;
* **g** to verify that the gyroscope is set up correctly (the angles should be between approximately [-45°, 45°]):
  * nose down: negative angle;
  * left down: negative angle;
  * yaw left: negative angle;
  * nose up: positive angle;
  * left up: positive angle;
  * yaw right: positive angle;
* **1**, **2**, **3** or **4** to check the rotation direction (see figure at the top):
  * 1: CCW;
  * 2: CW;
  * 3: CCW;
  * 4: CW;  
* **5** to use all the motors.

If the motors rotate in the opposite direction, just exchange one of the three cables between the ESC and the motor.

*Note:* if CALIBRATION sketch is uploaded, you can also check the readings of:
* the altitude sensor (send 'a' on serial);
* battery readings (send 'b');
* blinking leds (send 'l');
* printing the EEPROM memory (send 'e');
* GPS (send 's').

### **ESCs calibration**
Calibrate the ESCs without propellers.
You'd be better to find out how to calibrate your ESCs.
Usually this is done as in [this](https://www.youtube.com/watch?v=l8rjjvAZvHM) video.

### **Propellers calibration**
With calibration sketch still uploaded, calibrate the propellers.
Send the number of the motor to the serial and read the accelerometer measurements which are printed on the serial.
Try to lower these numbers by adding some scotch.

## **Flight controller**
Finally, write "UPLOADED_SKETCH  FLIGHT_CONTROLLER" in the [Config.h](https://github.com/sebastiano123-c/DroneIno/tree/main/DroneIno/src/Config.h) and upload the sketch.

<ins>_Try first without the propellers!_</ins>

### **Start and stop commands**
Turn on the battery and the RC-controller, and wait until all the calibration is done (signaled by the led blinking).
Move the left stick:
* to the bottom-left to **start** the motors;
* to the botom-right to **stop** the motors.

If this is done correctly, move to the following step.
Otherwise, you may have problem in the RC signals, thus check it using the CALIBRATION.

## **PID manual tuning**
PID may be different from case to case and plays a very important role in the flight stability.
PID controls:
- roll movement;
- pitch movement;
- yaw movement.

<ins>_In the next steps hold the quadcopter with your hand._</ins>
  
### **Step 1: reset variables**
Go in the [Globals.h](https://github.com/sebastiano123-c/DroneIno/blob/main/DroneIno/Globals.h) file and set
<pre>
PID_P_GAIN_ROLL             0.0                     
PID_I_GAIN_ROLL             0.0                    
PID_D_GAIN_ROLL             0.0           
PID_MAX_ROLL                400

PID_P_GAIN_YAW              0.0                     
PID_I_GAIN_YAW              0.0                    
PID_D_GAIN_YAW              0.0        
PID_MAX_YAW                 400  
</pre>

### **Step 2: yaw tuning**
Increment the yaw **P** parameter in steps of 0.5, upload the sketch and start the drone. Hold _**firmly**_ in your hand the copter, possibly stay on a soft surface, like a carpet.
Move the throttle until it seems to take fly and test its movements while the drone is still in safe in your hand.
Increment the **P** until the drone scrambles. Reduce the **P** at the value it acts quietly.

Do the same with the **I**, incrementing in steps of 0.005.

### **Step 3: roll/pitch tuning**
This part must be done in an open space, like your garden.
You can try to fly your drone and look at its behavior.

<ins>_Stay low, the throttle can act violently at this point_</ins>

Increment the roll **P** parameter in steps of 0.5, upload the sketch and start the drone. 

You may observe that the copter acts unpredictably.
Increment the value until this behavior reduces.

Do the same with the **D** and **I**, incrementing in steps of 0.002.

After a while you may observe that the behavior depends widely on the PID parameters, and you may find your own way to set properly these parameters.

## **PID auto-tuning**
_NOW ON DEVELOPING_
If you are tired of manually tune the PIDs of the quadcopter you should try the auto-tuning PID feature. (I recommend you to set at least once the PID manually to see the behaviour of each parameter).

Procedure: Upload the CALIBRATION sketch (as usual by `#define UPLOADED_SKETCH CALIBRATION` in the config.h).
After the initial dialogue, press `p` on the serial monitor. It will print the values of the PID provided by the neural network (NN).
Now, roll and rotate with your hands (_the motors must be TURNED OFF_) the drone around the three axis to train the NN. 
You will see that the parameters change. 
Do that until the integrative parameters for both the PIDs are less than 0.09.

Pressing ENTER the loop stops and the weights of the NN are saved in the flash memory.
On the serial will appear the estimated weights and biases. If they are ALL zeroes the NN has faced a gradient vanishing problem, so you have to try other `learningRate` and `momentumFactor` (defined in the Globals.h) and retry the training.

Once you have calculated your weights, set in the config.h file AUTOTUNE_PID_GYROSCOPE to true and upload the FLIGHT_CONTROLLER sketch.
The weights are automatically taken from the memory, so you do not need to train the NN anymore.
Now DroneIno can fly.

### **How auto-tuning works: back propagation (BP) neural network (NN)**
There are two NN, one for the roll (pitch) and one for the yaw.
I have used the back propagation ([BP](https://ieeexplore.ieee.org/abstract/document/8711864?casa_token=nNSTnIzV2pgAAAAA:gUgufp3ZQUK4LI0KIL-O82qP37pqxUCWW9m2jjZNu-OwocdGd5XzIk15_gERvh70LP_ESG8)) online gradient descent learning method.

(_TODO: spiegazione_)



# **DroneInoTelemetry web app**
DroneInoTelemetry is a web app that makes everything simple and easy reach.
Use it to fine-tune your PID or fix gyroscope set point and altitude hold PID parameters.

Using this app, you can adjust on the fly these parameters and much more.

## **Connection using only ESP32**
Exploiting the ESP32 native WiFi access point (AP), DroneInoTelemetry web app promises to be very smart in terms of time savings and feedback.

After connecting to DroneInoTelemetry network using the password `DroneIno`, dial in your browser's search bar `292.168.4.1`.
That's it.
You don't have to download anything, it's just there.

## **Connection using ESP32-CAM**
Now on developing, the ESP32-CAM promises to be the best solution for the telemetry.
<!-- Without it, it is only possible to set PID parameters. -->
With the ESP32-CAM, the flying experience will be much more interactive.
See the complete [esp32-cam-telemetry](https://github.com/sebastiano123-c/Esp32-cam-telemetry) repo for more information.

Features:
- on flight camera streaming;
- set camera settings;
- set PID parameters;
- pitch, roll, altitude pressure, flight mode and battery telemetry system.

First of all, upload the [WiFiTelemetry](https://github.com/sebastiano123-c/DroneIno/tree/main/WiFiTelemetry) sketch to your ESP32-CAM.

Then, power the ESP32-CAM with the 5V pin of the ESP32 board and connect the ESP32-CAM GND to the one of the ESP32.
The two boards can talk to each other using the UART communication reached by connecting:
- GPIO_3 pin of the ESP32-CAM to the GPIO_25 of the ESP32 board;
- GPIO_1 pin of the ESP32-CAM to the GPIO_26 of the ESP32 board.

Place ESP32-CAM on the drone and start the drone.
After connecting to DroneInoTelemetry network using the password `DroneIno`, dial in your browser's search bar `292.168.4.1`.
You will find the main menu with:
- PID parameters;
- camera settings;
- handle camera buttons.
Tap the **turn on** button to start the streaming.

If you want to save *automatically* the PID parameters, you'll need to insert a SD card to store them, see the following paragraph.

_Note 2:_ DroneIno signals the correct interpretation of the message with a led blinking.

### **SD card for data storage**
You can also save your flight data on a SD card. All you need is:
- a SD card formatted in FAT32 to put into your ESP32-CAM;
- create a folder named `src`;
- in `src` create a file called `config.txt` in which you need to write (the order is important!) only the values of:
```
PID_P_GAIN_ROLL
PID_I_GAIN_ROLL
PID_D_GAIN_ROLL
PID_P_GAIN_PITCH
PID_I_GAIN_PITCH
PID_D_GAIN_PITCH
PID_P_GAIN_YAW
PID_I_GAIN_YAW
PID_D_GAIN_YAW
GYROSCOPE_ROLL_FILTER
GYROSCOPE_ROLL_CORR
GYROSCOPE_PITCH_CORR
PID_P_GAIN_ALTITUDE
PID_I_GAIN_ALTITUDE
PID_D_GAIN_ALTITUDE
```
_Note 1:_ GYROSCOPE_ROLL_FILTER, GYROSCOPE_ROLL_CORR and GYROSCOPE_PITCH_CORR can be initially set to
```
0.9996
0
0
```
Then, place DroneIno onto a 0° horizontal surface.
After the drone initialization, depending on the value of the pitch and roll you read on the telemetry web app, change the value of the roll correction and pitch correction.
<!-- These adjustments seem to be crucial to prevent drifts when the drone is hovering. -->

SD card feature is recommended because when you change some settings in the web app, the `src\config.txt` file will be automatically updated.
With SD card your flight data, such as roll angle, pitch angles, battery and altitude will be save in a file in the folder `data\flight_i.csv`, which is automatically created.
In the future, in the `data` folder, there will the possibility to save also videos and photos.

The directory structure is
<pre>
\src
    config.txt
\data
    \videos
    \images
    \flightData
        flight_1.csv
        flight_2.csv
             :
        flight_n.csv
</pre>


# **Altitude hold**
Altitude hold function uses barometric data from the barometer sensor, for example the BMP280.
Look at the circuit scheme to connect the barometer and the SWC channel of the radio-controller to the board.
When DroneIno is flying, switch to the middle the SWC lever to activate altitude hold.

Battery goes down when motors are in use, causing a little performances loss.
DroneIno uses the `batteryCurvePendency` variable (defined in `include\Globals.h`) to correct this behavior.
In the first attempts using altitude hold, you may notice that DroneIno loses, or undesirably gains, altitude.
Tuning `batteryCurvePendency` you may find the expected altitude hold behavior.


# **GPS**
**_NOT TO USE AT THE MOMENT_**
**Now on developing**, DroneIno provides GPS for:
- flight adjustements;
- if you use ESP32-CAM telemetry, cool plots showing the flight route of DroneIno.

# **Appendix A: previous version**
The first version of DroneIno is [here](https://github.com/sebastiano123-c/DroneIno/tree/main/DroneIno/test/DroneIno.zip).
It is intended to work with Arduino IDE.

# **Appendix B: another circuit scheme**
In the previous DroneIno version, I used the following circuit scheme which involves a diode (and not a voltage regulator).
This solution provide lighter drone weight and will supply 12.6V - 0.7V = 11.9V to the board.
Otherwise, I wanted to have a 'modulate' tension supplier, in order to test the minimum tension one can give to the board.

The resistances R2 = 2.22kOhms and R1 = 5.1kOhms and the diode D1 (1N4001 or similar) handle the current going through the board.
Remember that the diodes typically drop the voltage about 0.7V.
D1 ensures that the PC is safe while both battery and pc are connected to the ESP32.

<pre>
    LEGEND:                                                 +-----+--R1--+     
    X-  = disconnected                                      |     |      |                      
    -+- = sold cables                                       |     R2     |                 _____________________ 
    -|- = not touching cables                               |  +--|------+---D1--------++==|+VCC|11.1V, >= 20C | 
    === = 3-5A cables                                       |  |  +----------------++==||==|-GND|2200mAh, 3s   | 
                                                            |  |  |                ||  ||  #### LiPo BATTERY ###
                 +---------------------------------+        |  |  |   _________    ||  ||              _____    
                 | +-------------------------------|--------|--|--|-->|°INPUT |====||==||=============/     \
                 | | +-----------------------------|-----+  |  |  | X-|+VIN   |====||==||=============| M1  |
                 | | | +---------------------------|---+ |  |  |  +-->|-GND   |====||==||=============\ CCW /
        ______   | | | | +-------------------------|-+ | |  |  |  |   |      +|====||==++                    
+------>|SDA°|   | | | | |   _____________________ | | | |  |  |  |   |      -|====++  ||                    
| +---->|SCL°|   | | | | |   |°IO03         IO39°|<|-|-|-|--+  |  |   ##ESC-1##    ||  ||                    
| |     |VCC+|<--------------------------+  IO38°| | | | |     |  |   _________    ||  ||              _____  
| |   +>|GND-|   | | | | |   |°IO26      |  IO34°| | | | +-----|--|-->|°INPUT |====||==||=============/     \
| |   | #baro#   | | | | |   |°IO25      |  IO04°|<+ | |       |  | X-|+VIN   |====||==||=============| M2  |
| |   |          | | | | +-->|°IO17      |  IO02°|   | |       |  +-->|-GND   |====||==||=============\ CW  /
| |   |          | | | +---->|°IO16      |       |   | |       |  |   |      +|====||==++                    
| |   |          | | +------>|°IO27      |   VIN+|<--|-|-------+  |   |      -|====++  ||                    
| |   |  _______ | +-------->|°IO14      |   GND-|<--|-|----------+   ##ESC-2##    ||  ||                    
| |   |  | CH5°|<+           |           |   GND-|   | |          |   _________    ||  ||              _____ 
| |   |  | CH1°|<----------->|°IO12      |    5V°|<+ | +----------|-->|°INPUT |====||==||=============/     \
| |   |  | CH2°|<----------->|°IO13      +-->3V3°| | |            | X-|+VIN   |====||==||=============| M3  |
| |   |  | CH3°|<----------->|°IO05          RST°| | |            +-->|-GND   |====||==||=============\ CCW /
| |   |  | CH4°|<----------->|°IO23           5V°| | |            |   |      +|====||==++                    
| | +-|->| VIN+|       +---->|°IO19  (not in  OD°| | |            |   |      -|====++  ||                    
| | | +->| GND-|       R3 +->|°IO18    scale)    | | |            |   ##ESC-3##    ||  ||                    
| | | |  ##RX ##       |  |  |-GND               | | |            |   _________    ||  ||              _____ 
| | | |  _______     (LED)|  |°RST               | | +------------|-->|°INPUT |====||==||=============/     \
+-|-|-|->| SDA°|<------|--|->|°SDA (GPIO 21)     | |              | X-|+VIN   |====||==||=============| M4  |
  +-|-|->| SCL°|<------|--|->|°SCL (GPIO 22)     | |              +-->|-GND   |====||==||=============\ CW  /
    +-|->| VCC+|       |  |  #### ESP32 D1 R32 ### |              |   |      +|====||==++                  
    | +->| GND-|       |  R3                       |              |   |      -|====++                        
    | |  #gyro #       | (LEDf)                    |              |   ##ESC-4##                              
    | +----------------+--+------------------------|--------------+                                            
    +----------------------------------------------+                                                                       
</pre>

# **Appendix C: components voltages**
<pre>
Name | type | voltage (V) 
--- | --- | --- | ---
ESP32 Vin | board | 3.3 - 11.7
ESP32 pin | board pin | 3.3
MPU6050 | gyro | 5
BMP280 | baro | 3.3
FSi6 | RC receiv. | 4.8 - 6 
BN 880 | GPS | 3.3 - 5
</pre>

# **Author**
Sebastiano Cocchi