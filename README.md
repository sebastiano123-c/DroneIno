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
The device you are programming can be dangerous, use this code if you are an expert at your own risk.
Thus, *test it firstly without propellers*.

Read carefully this [documentation](https://github.com/sebastiano123-c/DroneIno/tree/main/docs) before starting to fly.
Make sure to do all the passages described there.


# **Usage**
Clone this repo
<pre><code>git clone git@github.com:sebastiano123-c/DroneIno.git
</code></pre>
or download the .zip file.

# **Description**
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

The code is intended to be used with PlatformIO.

## **Features**
                                    1) Auto-leveling
                                    2) Altitude hold
                                    3) WiFi telemetry system
                                    4) SD flight data storage

_Important:_ otherwise telemetry system is implemented using the native ESP32 WiFi, I strongly recommend to add DroneIno the [Esp32-cam-telemetry](https://github.com/sebastiano123-c/Esp32-cam-telemetry) for astonishing results (including video streaming and SD card storage).

# **Documentation**
The circuit scheme and the detailed construction steps are reported in the full [documentation](https://github.com/sebastiano123-c/DroneIno/tree/main/docs). 
I describe the main features of DroneIno.

## **Auto-leveling**
DroneIno automatically corrects spurious drifts using the gyroscope signals.

## **Altitude hold**
Altitude hold function uses barometric data of the barometer sensor, for example the BMP280.

## **DroneInoTelemetry web app**
DroneInoTelemetry is a web app that makes everything simple and easy reach.
Use it to fine-tune your PID or fix gyroscope set point and altitude hold PID parameters.

Using this app, you can adjust on the fly these parameters and much more.

## **Connection using only ESP32**
Exploiting the ESP32 native WiFi access point (AP), DroneInoTelemetry web app promises to be very smart in terms of time savings and feedback.

After connecting to DroneInoTelemetry network using the password `DroneIno`, dial in your browser's search bar `292.168.4.1`.
That's it.
You don't have to download anything, it's just there.

## **Connection using ESP32-CAM**
ESP32-CAM is best solution for the telemetry.
<!-- Without it, it is only possible to set PID parameters. -->
With the ESP32-CAM, the flying experience is much more interactive including:
- on flight camera streaming;
- tune camera settings;
- set PID parameters;
- pitch, roll, altitude pressure, flight mode and battery telemetry system.

See the complete [esp32-cam-telemetry](https://github.com/sebastiano123-c/Esp32-cam-telemetry) repo for more information.

### **SD card for data storage**
With esp32-cam-telemetry, you can also save your flight data on a SD card.
All you need is a SD card formatted in FAT32 to put into your ESP32-CAM.


# **Roadmap**
Future improvements:
- GPS;
- flight planned;
- following me flight;
- Gimbal CAM;
- use other boards.

# **Previous version**
The first version of DroneIno is [here](https://github.com/sebastiano123-c/DroneIno/tree/main/DroneIno/test/DroneIno.zip).
It is intended to work with Arduino IDE.

# **Author**
Sebastiano Cocchi