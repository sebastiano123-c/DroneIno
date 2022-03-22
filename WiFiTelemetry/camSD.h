#pragma once

#include "FS.h" 
#include "SD_MMC.h" 
#include "telemetry.h"
#include "time.h"

extern uint8_t isConnectedSD;
extern const char* logFileName;

extern void setupSD();
extern void readConfigFile(fs::FS &fs);
extern void writeFile(fs::FS &fs, const char * path, const char * message);
extern void appendFile(fs::FS &fs, const char * path, const char * message);
extern void writeDataLogFlight(fs::FS &fs);
extern void updateConfigFile(fs::FS &fs);
