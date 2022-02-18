/** 
 * Esp32-cam telemtry
 * @author @sebastiano123-c
 * @brief wifi based telemtry system using your esp3-cam
 * @note board: AI thinker
 * @todo include pid elements as status
 */
#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>

#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#include "camera_pins.h"
#include "app_httpd.h"
#include "telemetry.h"

const char* ssid = "DroneInoTelemetry";
const char* password = "DroneIno";

const int timeDelay = 1;

//              (ROLL)
float PID_P_GAIN_ROLL            = 1.3;                      //Gain setting for the roll P-controller (1.3)
float PID_I_GAIN_ROLL            = 0.001;                  //Gain setting for the roll I-controller  (0.0002)
float PID_D_GAIN_ROLL            = 8.0;                     //Gain setting for the roll D-controller (10.0)
                                             
//              (PITCH)                                             
float PID_P_GAIN_PITCH           = PID_P_GAIN_ROLL;          //Gain setting for the pitch P-controller
float PID_I_GAIN_PITCH           = PID_I_GAIN_ROLL;          //Gain setting for the pitch I-controller
float PID_D_GAIN_PITCH           = PID_D_GAIN_ROLL;          //Gain setting for the pitch D-controller
                                              
//              (YAW)                                             
float PID_P_GAIN_YAW             = 2.2;                      //Gain setting for the pitch P-controller. (2.0)
float PID_I_GAIN_YAW             = 0.01;                     //Gain setting for the pitch I-controller. (0.04)
float PID_D_GAIN_YAW             = 0.0;                      //Gain setting for the pitch D-controller. (0.0)

//              (GYROSCOPE)
float GYROSCOPE_ROLL_FILTER      = .9996;                      // read your gyroscope data after the calibration, try different values and choose the best one
float GYROSCOPE_ROLL_CORR        = -.30;                      // (0.) after set GYROSCOPE_ROLL_FILTER, put here the angle roll you read eneabling DEBUG
float GYROSCOPE_PITCH_CORR       = -1.65;                     // (-1.65.) after set GYROSCOPE_PITCH_FILTER, put here the angle pitch you read eneabling DEBUG

//              (ALTITUDE)                                                                                          
float PID_P_GAIN_ALTITUDE        = 2.24;                     //Gain setting for the pitch P-controller. (2.0)
float PID_I_GAIN_ALTITUDE        = 0.11;                     //Gain setting for the pitch I-controller. (0.04)
float PID_D_GAIN_ALTITUDE        = 3.0;                      //Gain setting for the pitch D-controller. (0.0)
                                             
float rollAngle                  = 0.45;
float pitchAngle                 = 5.;
float flightMode                 = 1;
float batteryPercentage          = .7;
float altitudeMeasure            = 1000.;

void setup() {
  // Serial.begin(115200);
  // Serial.setDebugOutput(true);
  // Serial.println();

  beginUARTCOM();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    // Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

  
  // Serial.println("");
  // Serial.println("Starting AP...");
  
  WiFi.softAP(ssid, password);
  delay(30);
  
  // Serial.println("");
  // Serial.println("WiFi connected");

  // Serial.print("Camera Ready! Use 'http://");
  // Serial.print(WiFi.softAPIP());
  // Serial.println("' to connect");

  startCameraServer();

}

void loop() {

  readDataTransfer();

  delay(timeDelay);
}
