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
    Arduino code for DIY quadcopter drones using ESP32 board.
  </i>
</h3>
<br>

# Disclaimer
The code is not well tested yet, use it if you are an expert at your own risk.
Test it first without propellers.

# Table of contents
- [Disclaimer](#disclaimer)
- [Table of contents](#table-of-contents)
- [Description](#description)
- [Features](#features)
- [Documentation](#documentation)
  - [Pinmap](#pinmap)
  - [Configuration](#configuration)
  - [Setup](#setup)
  - [Calibration](#calibration)
    - [ESCs calibration](#escs-calibration)
    - [Propellers calibration](#propellers-calibration)
  - [Flight controller](#flight-controller)
- [Roadmap](#roadmap)

# Description
Based on the [YMFC-AL](https://github.com/F4b1-/YMFC-AL-Flight-Controller-improved) project, DroneIno allows you to control a quadcopter with:
* ESP32 D1 R32;
* a gyroscope MPU6050;
* a radio controller and a receiver (_e.g._ FlySky);
* 11.1V 3C LiPo battery (2200mAh or 3800mAh);
* four 30A ESCs;
* four brushless motors (around 1000KV).

<!-- <pre>
CIRCUITAL SCHEMATIC HERE 
</pre> -->
<!-- 
# Boards
- ESP32 D1 R32 -->

# Features
- Autoleveling: the drone corrects spurious drifts using the gyroscope signals.

# Documentation
## Pinmap
Check that your pinmap correspond to the predefined [pinmap.ESP32.h](https://github.com/sebastiano123-c/DroneIno/tree/main/src/pinmaps/pinmap.ESP32.h), otherwise change it.

## Configuration
Take a look at the [Config.h](https://github.com/sebastiano123-c/DroneIno/tree/main/Config.h) file where you can adjust the PID parameters and others constants.

## Setup
Upload the [setup](https://github.com/sebastiano123-c/DroneIno/tree/main/Setup) sketch to your board and run it.

## Calibration
If the setup sketch exits with succeed, upload the [calibration](https://github.com/sebastiano123-c/DroneIno/tree/main/Calibration) sketch. Send the following characters to the serial monitor:
* **r** to check if the transmitter signal is decoded correctly; move the trim and check that:
  * throttle: down 1000us| up    2000us;
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

### ESCs calibration
Calibrate the ESCs without propellers.
You'd be better to find out how to calibrate your ESCs.
Usually this is done as in [this](https://www.youtube.com/watch?v=l8rjjvAZvHM) video.

### Propellers calibration
With calibration sketch calibrate the propellers.
Send the number of the motor to the serial and read the accelerometer measurements which are printed on the serial.
Try to lower these numbers by adding some scotch.

## Flight controller
Finally, upload the [DroneIno](https://github.com/sebastiano123-c/DroneIno/tree/main/DroneIno.ino) flight-controller.

# Roadmap
As one can see in the pinmap folder, I am planning to test it on other boards, but at the moment this works only for ESP32.

Future improvements:
- use other boards
- altitude hold
- GPS
- Gimbal CAM
