/**
 * @file wifi_telemetry.native.h
 * @author @sebastiano123-c
 * @brief Defines the server port and the events location for the native ESP32 WiFi connection.
 * @version 0.1
 * @date 2022-02-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// server
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");