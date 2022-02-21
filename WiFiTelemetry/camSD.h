#pragma once

#include "FS.h" 
#include "SD_MMC.h" 
#include "telemetry.h"
#include "time.h"

extern uint8_t isConnectedSD;

extern void setupSD();
extern void writeDataLogFlight(fs::FS &fs);
extern void updateConfigFile(fs::FS &fs);
