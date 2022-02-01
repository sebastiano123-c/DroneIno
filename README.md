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
    DroneIno
  </b>
</h1>
<h3 align="center"> 
  <i>
    Arduino code for ESP32 based DIY quadcopter drones.
  </i>
</h3>
<br>

<h2>
<b><ins><i>Disclaimer</i></ins></b>
</h2>
The code is not well tested yet, use it if you are an expert at your own risk.

Test it first without propellers.

Read carefully this brief documentation before starting to fly.
Make sure to do all the passages described here below.

# **Table of contents**
- [**Table of contents**](#table-of-contents)
- [**Usage**](#usage)
- [**Description**](#description)
- [**Features**](#features)
- [**Documentation**](#documentation)
  - [**Circuit scheme**](#circuit-scheme)
  - [**Pinmap**](#pinmap)
  - [**Configuration**](#configuration)
  - [**Setup**](#setup)
  - [**Calibration**](#calibration)
    - [**ESCs calibration**](#escs-calibration)
    - [**Propellers calibration**](#propellers-calibration)
  - [**Flight controller**](#flight-controller)
  - [**PID tuning**](#pid-tuning)
    - [**Step 1: reset variables**](#step-1-reset-variables)
    - [**Step 2: yaw tuning**](#step-2-yaw-tuning)
    - [**Step 3: roll/pitch tuning**](#step-3-rollpitch-tuning)
- [**Roadmap**](#roadmap)

# **Usage**
Clone this repo
<pre><code>git clone git@github.com:sebastiano123-c/DroneIno.git
</code></pre>
or download the .zip file.

# **Description**
Based on the [YMFC-AL](https://github.com/F4b1-/YMFC-AL-Flight-Controller-improved) project, DroneIno allows you to control a quadcopter with:
* ESP32 D1 R32 board;
* gyroscope MPU6050;
* radio controller and a receiver (_e.g._ FlySky);
* 11.1V 3S >20C LiPo battery (2200mAh or 3800mAh);
* four 30A ESCs;
* four brushless DC motors (around 1000KV).
To build DroneIno I followed these [videos](https://www.youtube.com/watch?v=XFxqFQwRumc&list=PL0K4VDicBzsibZqfa42DVxC8CGCMB7G2G) explaining how to build the YMFC and I recommend you to do the same.

<!-- <pre>
CIRCUITAL SCHEMATIC HERE 
</pre> -->
<!-- 
# Boards
- ESP32 D1 R32 -->

# **Features**
- autoleveling
- altitude hold (developing9
<!-- - : the drone corrects spurious drifts using the gyroscope signals -->

# **Documentation**
## **Circuit scheme**
The power is supplied by the 11.1V 2200mAh 3s 20C LiPo battery.
The battery powers the motors and the ESP32.
The resistances (Ohms) R2 = 1K and R3 = 1.5K and the diode D1 (1N4001 or similar) handle the current going through the board.
D1 ensures that the PC is safe while both battery and pc are connected to the ESP32.
The resistance R1 = 330 is used for the LED. 
<pre>
LEGEND:                                             +-----+--R3--+     
X-  = disconnected                                  |     |      |                      
-+- = sold cables                                   |     R1     |                   __________________ 
-|- = not touching cables                           |  +--|------+---D1--------++====|+VCC|11.1V, >= 20C | 
=== = 3-5A cables                                   |  |  +----------------++==||====|-GND|2200mAh, 3s   | 
                                                    |  |  |                ||  ||    ## LiPo BATTERY ## 
                                                    |  |  |   _________    ||  ||                       
           +----------------------------------------|--|--|-->|°INPUT |====||==||=============/     \
           | +-----------------------------------+  |  |  | X-|+VIN   |====||==||=============| M1  |
           | | +-------------------------------+ |  |  |  +-->|-GND   |====||==||=============\ CCW /
           | | | +---------------------------+ | |  |  |  |   |      +|====||==++                    
           | | | |   _____________________   | | |  |  |  |   |      -|====++  ||                    
           | | | |   |°IO03         IO39°|   | | |  |  |  |   ##ESC-1##    ||  ||                    
           | | | |   |°IO01         IO38°|   | | |  |  |  |   _________    ||  ||                    
           | | | |   |°IO26         IO34°|   | | +--|--|--|-->|°INPUT |====||==||=============/     \
           | | | |   |°IO25         IO04°|   | |    |  |  | X-|+VIN   |====||==||=============| M2  |
           | | | +-->|°IO17         IO02°|<--|-|----+  |  +-->|-GND   |====||==||=============\ CW  /
           | | +---->|°IO16              |   | |       |  |   |      +|====||==++                    
           | +------>|°IO27          VIN+|<--|-|-------+  |   |      -|====++  ||                    
           +-------->|°IO14          GND-|<--|-|----------+   ##ESC-2##    ||  ||                    
     _______         |               GND-|   | |          |   _________    ||  ||                    
     | CH1°|<------->|°IO12           5V°|<+ | +----------|-->|°INPUT |====||==||=============/     \
     | CH2°|<------->|°IO13          3V3°| | |            | X-|+VIN   |====||==||=============| M3  |
     | CH3°|<------->|°IO05          RST°| | |            +-->|-GND   |====||==||=============\ CCW /
     | CH4°|<------->|°IO23           5V°| | |            |   |      +|====||==++                    
+--->| VIN+|    +--->|°IO19           OD°| | |            |   |      -|====++  ||                    
| +->| GND-|    R1   |°IO18              | | |            |   ##ESC-3##    ||  ||                    
| |  ##RX ##    |    |-GND   (not in     | | |            |   _________    ||  ||                    
| |  _______  (LED)  |°RST     scale)    | | +------------|-->|°INPUT |====||==||=============/     \
| |  | SDA°|<---|--->|°SDA               | |              | X-|+VIN   |====||==||=============| M4  |
| |  | SCL°|<---|--->|°SCL               | |              +-->|-GND   |====||==||=============\ CW  /
+-|->| VCC+|    |    #### ESP32 D1 R32 ### |              |   |      +|====||==++                  
| +->| GND-|    |                          |              |   |      -|====++                        
| |  #gyro #    |                          |              |   ##ESC-4##                              
| +-------------+--------------------------|--------------+                                            
+------------------------------------------+                                                                                     
</pre>
After building the circuit, place it on the drone and proceed with the following passages.

## **Pinmap**
Check that your pinmap corresponds to the one defined in the [pinmap](https://github.com/sebastiano123-c/DroneIno/tree/main/DroneIno/src/pinmaps) folder, otherwise change it.

## **Configuration**
Take a look at the [Config.h](https://github.com/sebastiano123-c/DroneIno/tree/main/DroneIno/Config.h) file where you can adjust the PID parameters and others constants.

## **Setup**
Upload the [setup](https://github.com/sebastiano123-c/DroneIno/tree/main/Setup) sketch to your board and run it.

## **Calibration**
If the setup sketch exits with succeed, upload the [calibration](https://github.com/sebastiano123-c/DroneIno/tree/main/Calibration) sketch. Send the following characters to the serial monitor:
* **r** to check if the transmitter signal is decoded correctly; move the trim and check that:
  * throttle: low 1000us| full    2000us;
  * roll:     left 1000us| right 2000us;
  * pitch:    left 1000us| right 2000us;
  * yaw:      left 1000us| right 2000us;
* **a** to verify that the gyroscope is set up correctly (the angles should be between approximately [-45°, 45°]):
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

### **ESCs calibration**
Calibrate the ESCs without propellers.
You'd be better to find out how to calibrate your ESCs.
Usually this is done as in [this](https://www.youtube.com/watch?v=l8rjjvAZvHM) video.

### **Propellers calibration**
With calibration sketch still uploaded, calibrate the propellers.
Send the number of the motor to the serial and read the accelerometer measurements which are printed on the serial.
Try to lower these numbers by adding some scotch.

## **Flight controller**
Finally, upload the [DroneIno](https://github.com/sebastiano123-c/DroneIno/tree/main/DroneIno) flight-controller.

<ins>_Try first without the propellers!_</ins>

## **PID tuning**
PID may be different from case to case and plays a very important role in the flight stability.
PID controls:
- roll movement ;
- pitch movement;
- yaw movement.

<ins>_In the next steps hold the quadcopter with your hand._</ins>
  
### **Step 1: reset variables**
Go in the Config.h file and set
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
Increment the yaw **D** parameter in steps of 2.0, upload the sketch and start the drone. Hold _**firmly**_ in your hand the copter, possibly stay on a soft surface, like a carpet.
Move the throttle until it seems to take fly and test its movements while the drone is still in safe in your hand.
Increment the **D** until the drone scrambles. Reduce the **D** at the value it acts quietly.

Do the same with the **P**, incrementing in steps of 0.5.

### **Step 3: roll/pitch tuning**
This part must be done in an open space, like your garden.
You can try to fly your drone and look at its behavior.

<ins>_Stay low, the throttle can act violently at this point_</ins>

Increment the roll **D** parameter in steps of 2.0, upload the sketch and start the drone. 

You may observe that the copter acts unpredictably.
Increment the value until this behavior reduces.

Do the same with the **P**, incrementing in steps of 0.2.

After a while you may observe that the behaviour depends widely on the PID parameters, and you may find your own way to set properly these parameters.

# **Roadmap**
As one can see in the pinmap folder, I am planning to test it on other boards, but at the moment this works only for ESP32.

Future improvements:
- use other boards
- altitude hold
- GPS
- Gimbal CAM
