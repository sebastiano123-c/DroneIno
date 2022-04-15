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
 *           using the commads suggested in the documentation, the sensor or the 
 *           the part of the drone you want to check.
 *           Finally, dial, one at a time, in the serial 1, 2, 3 and 4; The number 
 *           showing in the monitor represent the accelerometer readings.
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
 * the PID parameterchanged are NOT saved anywhere.
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
 * @version 0.1
 * @date 2022-03-30
 * @copyright Copyright (c) 2022
 */



/**
 * ----------------------------------------------------------------------------------------------------------------------------
 *  SKETCH CONFIGURATION:
 * 
 *      (MICROCONTROLLER BOARD)
 */
#define PINMAP                      ESP32                    // (OFF**, ATMEGA32**, ESP32) OFF if the board is Arduino Uno (directly write pin)
/**
 *      (SKETCH TO BE UPLOADED)
 *      Defines the compiler which sketch is to be uploaded.
 *      Following the list:
 *          1) SETUP               tells the compiler to use the setup sketch,
 *          2) CALIBRATION         ... calibration sketch,
 *          3) FLIGHT_CONTROLLER   ... flight controller
 *      and uploading the sketch each time, you first begin with SETUP.
 *      This way you will calibrate RC-controller and more, and this will save information into EEPROM.
 *      Then, upload with CALIBRATION and calibrate ESC and check that each components behaves in the correct way.
 *      Finally upload with FLIGHT_CONTROLLER and start to fly.
 */
#define UPLOADED_SKETCH             SETUP        // (SETUP, CALIBRATION, FLIGHT_CONTROLLER) 
/**
 *      (DEBUG MODE)
 *      If true, some serial prints are enabled.
 *      Otherwise, it is not auspicable to set DEBUG true when flying.
 */
#define DEBUG                       false                    // (true, false) true enable serial prints for debugging
/**
 *      (SKETCH CONSTANTS)
 */
#define BAUD_RATE                   115200                   // (9600, 57600, 115200)
#define WIRE_CLOCK                  400000L                  // (100000L, 400000L) 400000L is suggested
#define EEPROM_SIZE                 36                       // EEPROM size for the allocatable memory



/**
 * ----------------------------------------------------------------------------------------------------------------------------
 *  AUTO LEVELING
 * 
 *      If true, the drone will adjust its flying accordingly to the gyroscope readings.
 *      If false, disables every autoleveling compensation.
 *      Till now, MPU6050 is the only gyroscope admitted on DroneIno.
 */
#define AUTO_LEVELING               true                     // (true, false) 
#define GYROSCOPE                   MPU6050                  // (MPU6050) unique for now



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
 *      For what I know, BN 880 should be very similar to the Ubox M8N.
 */
#define GPS                         BN_880                   // (OFF, BN_880*)
#define GPS_BAUD                    115200                   // (9600, 57600, 115200) 9600 should be ok
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
 *      When flying, the motors rotating cause a voltage diminuition.
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
 */
#define RESISTANCE_1                5.11                     // (kOhm) the resistance before V_pin
#define RESISTANCE_2                1.55                     // (kOhm) the resistance after V_pin
#define TOTAL_DROP                  RESISTANCE_2 / (RESISTANCE_1 + RESISTANCE_2)


/**
 * ----------------------------------------------------------------------------------------------------------------------------
 *  TELEMETRY:
 * 
 *      (WIFI)
 *      Using WiFi a telemtry system is done, making everything simple and easy reach.
 *      Use it to fine-tune your PID or fix gyroscope set point and altitude hold PID parameters.
 *      Using this you can adjust on the fly these parameters and much more:
 *          *) NATIVE uses the ESP32 wifi AP,
 *          *) ESP_CAM uses the ESP32CAM wifi.
 *      See https://github.com/sebastiano123-c/Esp32-cam-telemetry for more details.
 */
#define WIFI_TELEMETRY              ESP_CAM                   // (NATIVE, ESP_CAM) set NATIVE if you don't have an ESP32-CAM



//      (SENSORS)
// #define PROXIMITYSENSOR             OFF                      // (OFF, HCSR04*)
// #define DANGER_DISTANCE             40                       // (cm) Proximity sensor distance under which is considered a danger zone
