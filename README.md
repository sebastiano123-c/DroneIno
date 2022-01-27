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
<h1 align="center"> 
DroneIno
</h1>
<h3 align="center"> 
Arduino code for DIY quadcopter drones using ESP32 board.
</h3>
<br>

# Disclaimer
The code is not well tested yet, use it only if you are an expert.

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
- Autoleveling

# Documentation
## Pinmap
Pinmap is declaired [here](https://github.com/sebastiano123-c/DroneIno/tree/main/src).
Write your Config.h file.

## Setup
Upload the [setup](https://github.com/sebastiano123-c/DroneIno/tree/main/Setup) sketch to your board and run it.

## Calibration
If it succeeds, upload the [calibration](https://github.com/sebastiano123-c/DroneIno/tree/main/Calibration) sketch. Sending the following characters to the serial monitor allows you to
* **r** to check if the transmitter signal is decoded correctly; move the trim and check that:
  * throttle: dowm 1000us| up    2000us;
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
* **1**, **2**, **3** or **4** to check the rotation direction (see figure below):
  * 1: CCW;
  * 2: CW;
  * 3: CCW;
  * 4: CW;  
* **5** to use all the motors.
 
### ESCs calibration
Calibrate the ESCs without propellers.
You'd be better to find out how to calibrate your ESCs.
Usually it is done like [this](https://www.youtube.com/watch?v=l8rjjvAZvHM).

### Propellers calibration
With calibration sketch calibrate the propellers.
Send the number of the motor to the serial and read the accelerometer measurements which are printed on the serial.
Try to lower these numbers by adding some scotch.

# Roadmap
Future improvements which are not yet implemented:
- use other boards
- altitude hold
- GPS
- Gimbal CAM
