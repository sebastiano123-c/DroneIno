/**
 *
 *
 *                       **********************************
 *                       *       Configuration file       *
 *                       **********************************
 *
 *             Config.h is intended to fit as your drone developing.
 *
 *
 *
 *
 *                           SKETCH CONFIGURATION:
 *
 * Accordingly to the documentation, after you building the hardware, set the proper
 * variable as BOARD, BAUD_RATE and EEPROM size in the sketch configuration.
 * Then:
 *       @li define UPLOADED_SKETCH as SETUP, upload the sketch and follow the
 *           instruction appearing in the serial monitor;
 *       @li define UPLOADED_SKETCH as CALIBRATION, upload the sketch and write,
 *           using the commands suggested in the documentation, the sensor or the
 *           the part of the drone you want to check.
 *           Propellers calibration.
 *           The readings will be printed on the Web Serial (see docs for connection).
 *           Dial, one at a time, in the web serial 1, 2, 3 and 4 (each number corresponds
 *           to a specifics motor:
 *                      1, right front CCW,
 *                      2, right rear CW,
 *                      3, left rear CCW,
 *                      4, left front CW,
 *                      5, all motors together.
 *           On the web serial will compare the accelerometer readings.
 *           The lower these number, the stabler the props.
 *           Use them to calibrate (i.e. add scotch) to the propeller in order to make
 *           them more balanced;
 *       @li define UPLOADED_SKETCH as FLIGHT_CONTROLLER and upload the sketch.
 *           DEBUG mode enables the Serial for debugging, otherwise, no serial monitor.
 *           To start the drone: move the left stick to the bottom left.
 *           To stop the drone: move the left stick to the bottom right.
 *
 *
 *
 *                              AUTOLEVELING:
 *
 * Set to true, if you want to compensate the drone oscillations, accordingly to the
 * gyroscope readings.
 * Set to false, if you are not intend to autolevel.
 *
 *
 *
 *                                ALTITUDE:
 *
 * Set the name of the barometer sensor you are using.
 * For now, only BMP280 is allowed.
 *
 *
 *
 *                                  GPS:
 *
 * Set GPS type, if you got, or leave OFF if you are not using a GPS at all.
 * I recommend to use a baud rate about 9600, it is sufficient.
 * Set you current UTC time zone (e.g. for UTC+2 set 2) to adjust the time readings
 * to your region.
 *
 *
 *
 *                                BATTERY:
 *
 * In this part you set the battery specs, the resistances R1 and R2 you have used for
 * the voltage divider, and more.
 * Battery readings can be useful to compensate the voltage lowering, but, until now, this
 * feature is not stable, thus leave BATTERY_COMPENSATION false.
 *
 *
 *
 *                               TELEMETRY:
 *
 * DroneIno uses WiFi for telemetry.
 * It involves the NATIVE ESP32 WiFi access point (AP). Using this, you can easily set PID
 * parameters for the PID adjustment (without continuing stopping and uploading the code).
 * Remember to write the PID changing on a paper or whatever. Indeed, when you turn off the drone,
 * the PID parameter changed are NOT saved anywhere.
 * On the other hand, as said in the documentation, it is preferable to configure an
 * external ESP32-CAM. This choice will bring:
 *      @li video streaming;
 *      @li SD card storage for PID parameter savings and flight data recordings.
 *
 *
 *
 *                             TERMS OF USE:
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 *
 *
 *               ! ALWAYS TRY FIRST WITHOUT PROPELLERS FIRST !
 *
 *
 *
 * @file Config.h
 * @author @sebastiano123-c
 * @brief Configuration file.
 * @note * = developing
 * @note ** = not available at the moment
 * @version 1.1
 * @date 2022-03-30
 * @copyright Copyright (c) 2022
 */



/**
 * ----------------------------------------------------------------------------------------------------------------------------
 *  SKETCH CONFIGURATION:
 *
 *      (MICROCONTROLLER BOARD)
 *      Accordingly to the documentation (\docs\README.md), two microcontroller boards are now available: ESP32 D1 R32, which
 *      has the same shape as the Arduino Uno board but using an ESP32-WROOM-32 processor, and ESP32 Dev Kit, which comes with
 *      the typical ESP32 shape.
 */
#define PINMAP                      ESP32_DEVKIT              // (OFF**, ATMEGA32**, ESP32_D1_R32, ESP32_DEVKIT)
/**
 *      (SKETCH TO BE UPLOADED)
 *      Defines the compiler which sketch is to be uploaded.
 *      Following the list:
 *          1) SETUP               tells the compiler to use the setup sketch,
 *          2) CALIBRATION         ... calibration sketch,
 *              -- ALERT READ CAREFULLY --
 *              Some boards come with no protection circuit.
 *              Thus, if you power your board via both USB and externally using a power supply, you will fry your board.
 *              To eradicate this issue from the roots, I recommend to use separate connectors: one for the motors and
 *              one for the board.
 *              So, when calibrating the propellers, connect the battery to the motor BUT NOT TO THE BOARD.
 *          3) FLIGHT_CONTROLLER   ... flight controller
 *      and uploading the sketch each time, you first begin with SETUP.
 *      This way you will calibrate RC-controller and more, and this will save information into EEPROM.
 *      Then, upload with CALIBRATION and calibrate ESC and check that each components behaves in the correct way.
 *      Finally upload with FLIGHT_CONTROLLER and start to fly.
 */
#define UPLOADED_SKETCH             SETUP
// #define UPLOADED_SKETCH             CALIBRATION
// #define UPLOADED_SKETCH             FLIGHT_CONTROLLER
/**
 *      (MOTOR PULSE PROVIDER)
 *      Motor pulses are nothing more than a time in us sent to the ESCs.
 *      There are mainly two ways in ESP32 to it: using MCPWM library or LEDC library.
 *      LEDC is the most secure, is the one I have used until now. On the other hand, MCPWM seems to be
 *      designed properly for motor control. So far, I noticed that MCPWM, in some ways, has a bigger energy cost
 *      than LEDC (researches are now on the way).
 */
// #define MOTOR_PULSE_BY_MCPWM              // NOT TESTED YET
#define MOTOR_PULSE_BY_LEDC
/**
 *      (DEBUG MODE)
 *      Works only when the UPLOADED_SKETCH is FLIGHT_CONTROLLER.
 *      If true, some serial prints are enabled.
 *      Otherwise, it is not good to set DEBUG true when flying.
 *
 *      Then uncomment the section you want to debug for the serial print.
 */
#define DEBUG                      false                    // (true, false) true enable serial prints for debugging
// #define DEBUG_GYRO
// #define DEBUG_BATTERY
// #define DEBUG_ESC
// #define DEBUG_AUTOPID
// #define DEBUG_PID
// #define DEBUG_PID_SIGNALS
// #define DEBUG_WIFI_SEND
// #define DEBUG_WIFI_REC
/**
 *      (SKETCH CONSTANTS)
 */
#define BAUD_RATE                   115200                   // (9600, 57600, 115200)
#define WIRE_CLOCK                  400000L                  // (100000L, 400000L) 400000L is suggested
#define EEPROM_SIZE                 36                       // EEPROM size for the byte variables


/**
 * ----------------------------------------------------------------------------------------------------------------------------
 *  AUTO-LEVELING:
 *
 *      If true, the drone will adjust its flying accordingly to the gyroscope readings.
 *      If false, disables every auto-leveling compensation.
 *      Till now, MPU6050 is the only gyroscope admitted on DroneIno.
 */
#define AUTO_LEVELING               true                     // (true, false)
#define GYROSCOPE                   MPU6050                  // (MPU6050) unique for now



/**
 * -----------------------------------------------------------------------------------------------------------
 * MANUAL PID:
 * 
 * 
 *    @brief PID parameters for roll, pitch, yaw and altitude.
 *    @note Roll and pitch parameters have the same values.
 * 
 *    P = proportional -> P_output = (gyro - receiver) * P_gain
 *    I = integral     -> I_output = I_output + (gyro - receiver) * I_gain
 *    D = derivative   -> D_output = (gyro - receiver - (gyro_prev - receiver_prev) ) * D_gain 
 * 
 *  
 *      ROLL
 */
#define PID_P_GAIN_ROLL             0.656f                    //Gain setting for the roll P-controller (1.3)
#define PID_I_GAIN_ROLL             0.022f                    //Gain setting for the roll I-controller  (0.04)
#define PID_D_GAIN_ROLL             0.772f                    //Gain setting for the roll D-controller (18.0)
#define PID_MAX_ROLL                400                       //Maximum output of the PID-controller   (+/-)
/**
 *      PITCH
 */                                             
#define PID_P_GAIN_PITCH            PID_P_GAIN_ROLL           //Gain setting for the pitch P-controller
#define PID_I_GAIN_PITCH            PID_I_GAIN_ROLL           //Gain setting for the pitch I-controller
#define PID_D_GAIN_PITCH            PID_D_GAIN_ROLL           //Gain setting for the pitch D-controller
#define PID_MAX_PITCH               PID_MAX_ROLL              //Maximum output of the PID-controller   (+/-)
/**
 *      YAW
 */                                       
#define PID_P_GAIN_YAW              6.5f                      //Gain setting for the pitch P-controller. (4.0)
#define PID_I_GAIN_YAW              0.022f                    //Gain setting for the pitch I-controller. (0.02)
#define PID_D_GAIN_YAW              0.0f                      //Gain setting for the pitch D-controller. (0.0)
#define PID_MAX_YAW                 400                       //Maximum output of the PID-controller     (+/-)
/**
 *      ALTITUDE
 */
#define PID_P_GAIN_ALTITUDE         1.4f                      //Gain setting for the altitude P-controller (default = 1.4).
#define PID_I_GAIN_ALTITUDE         0.3f                      //Gain setting for the altitude I-controller (default = 0.2).
#define PID_D_GAIN_ALTITUDE         0.75f                     //Gain setting for the altitude D-controller (default = 0.75).
#define PID_MAX_ALTITUDE            400                       //Maximum output of the PID-controller (+/-).
/**
 *      GPS 
 */
#define PID_P_GAIN_GPS              2.7f                      //Gain setting for the GPS P-controller (default = 2.7).
#define PID_D_GAIN_GPS              6.5f                      //Gain setting for the GPS D-controller (default = 6.5).



/**
 * ----------------------------------------------------------------------------------------------------------------------------
 * AUTOMATIC PID:
 * 
 *      Auto-tune PID allow DroneIno to adjust on run the PID parameters to best fit the environmental changes.
 *      If false, the following PID values are set. If true the drone auto calibrates the parameters
 */
#define AUTOTUNE_PID_GYROSCOPE      false                      // (false, true), "true" is now on testing.



/**
 * ----------------------------------------------------------------------------------------------------------------------------
 *  ALTITUDE:
 *
 *      Altitude readings are relevant for the 'altitude hold' feature.
 *      Leave OFF if you don't have any pressure sensor installed on DroneIno.
 *      BMP280 is the only sensor I have tested and, probably, not the most sensitive one.
 *      In the future, I will test other sensors.
 */
#define ALTITUDE_SENSOR             BMP280                   // (OFF, BMP280*, BME280**)



/**
 * ----------------------------------------------------------------------------------------------------------------------------
 *  GPS:
 *
 *      GPS sensor is useful to recover the latitude and longitude position of DroneIno.
 *      GPSs talk with DroneIno using a serial communication (TX/RX).
 *      String printed on serial are then read by the board and converted into information in the file GPS.h.
 *      Leave OFF if you don't have a GPS installed.
 *      Till now I have tested the Beitian BN 880 (which also incorporates the compass).
 *      For what I know, BN 880 should be very similar to the Ublox M8N.
 */
#define GPS                         OFF                   // (OFF, BN_880*)
#define GPS_BAUD                    9600                   // (9600, 57600, 115200) 9600 should be ok
#define UTC_TIME_ZONE               2                        // (0-23) Put your time zone here, for example 2 stands for UTC+2



/**
 * ----------------------------------------------------------------------------------------------------------------------------
 *  BATTERY:
 *
 *      (BATTERY SPECS)
 *      I use a 3s LiPo Battery with 11.1V, 2200mAh capacity and 50C discharge rate
 *      For 3s batteries, the danger voltage is 3*3 V = 9 V, but it is preferable
 *      to stop flying before reaching 9V.
 *      The warning battery voltage defines the value under which a led starts blinking.
 */
#define BATTERY_NUMBER_OF_CELLS     3                        // (V) battery nominal maximum voltage (use ONLY 11.1V batteries)
#define WARNING_BATTERY_VOLTAGE     10.00                    // (V) when drone reaches this value, it will not take off
/**
 *      (BATTERY_EMERGENCY_STOP)
 *      If the battery reaches the danger WARNING_BATTERY_VOLTAGE, at the next take off the motors will not run.
 *      This feature will reduce battery damages.
 *      Otherwise, this feature may signal a battery low level while, when DroneIno is still at ground, the battery level is above
 *      the WARNING_BATTERY_VOLTAGE.
 *      This is so because battery typically requires more power during the ascension.
 */
#define BATTERY_EMERGENCY_STOP      false
/**
 *      (COMPENSATION*)
 *      When flying, the motors rotating cause a voltage diminution.
 *      If true, BATTERY_COMPENSATION will compensate this behavior.
 *
 *      WARNING: THIS FEATURE IS DISABLED DUE TO INCORRECTNESS, PLEASE LEAVE false.
 *
 */
#define BATTERY_COMPENSATION        false                     // (true, false) IMPORTANT: leave false for now
/**
 *      (VOLTAGE DIVIDER)
 *      The voltage divider is:
 *            Vin --- res 1 ---+--- res2 --- GND
 *                             |
 *                           V_pin
 *
 *      @note choose res1 and res2 so that the maximum V_pin is less than BOARD_MAXIMUM_VOLTAGE
 *
 *      If you apply a diode D1 before the voltage divider, as for the ESP32 DevKit circuit (see \docs),
 *      write DIODE_DROP 0.7. Otherwise DIODE_DROP 0.
 */
#define RESISTANCE_1                5.100                    // (kOhm) the resistance before V_pin
#define RESISTANCE_2                1.22                     // (kOhm) the resistance after V_pin
#define DIODE_DROP                  0.70                     // (V) voltage drop due to the diode. If not used put it to 0
#define TOTAL_DROP                  RESISTANCE_2 / (RESISTANCE_1 + RESISTANCE_2)


/**
 * ----------------------------------------------------------------------------------------------------------------------------
 *  TELEMETRY:
 *
 *      (WIFI)
 *      Using WiFi a telemetry system is done, making everything simple and easy reach.
 *      Use it to fine-tune your PID or fix gyroscope set point and altitude hold PID parameters.
 *      Using this you can adjust on the fly these parameters and much more:
 *          *) OFF, no WiFi created;
 *          *) NATIVE, uses the ESP32 wifi AP;
 *          *) ESP_CAM, uses the ESP32CAM wifi.
 *      See https://github.com/sebastiano123-c/Esp32-cam-telemetry for more details.
 */
#define WIFI_TELEMETRY              OFF                   // (OFF, NATIVE, ESP_CAM) set NATIVE if you don't have an ESP32-CAM
#define WIFI_BAUD_RATE              115200                    // (9600, 57600, 115200)
