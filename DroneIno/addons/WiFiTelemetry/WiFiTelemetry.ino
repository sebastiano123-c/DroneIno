/* 
 * WiFiTelemetry
 * @author @sebastiano123-c
 * @note this sketch should be uploaded directly to the esp32 cam
 */

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include "../../Config.h"
#include "../../Constants.h"
#include "../../Globals.h"
#include "../../src/Models.h"

int delayTime = 200; // ms

void setup(){
  // serials
  //Serial.begin(BAUD_RATE);
  SUART.begin(BAUD_RATE, SERIAL_8N1, PIN_RX0, PIN_TX0); // RX TX


  WiFi.softAP(ssid, password);

  // Serial.printf("\nIP address: " + WiFi.softAPIP() + "\n");
  
  // Send web page with input fields to client
   server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
     request->send(200, "text/html", index_html());
   });
   
   // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
   server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
     readFromClientOnGet();
     
     writeDataTransfer();

     request->send(200, "text/html", index_html());                         
   });

  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);

  // if not found
  server.onNotFound(notFound);
  server.begin();
}

void loop(){
  sendTelemetry();

  //delay(delayTime);
}
