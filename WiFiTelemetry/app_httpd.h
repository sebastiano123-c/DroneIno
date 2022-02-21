
#pragma once

#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "camera_index.h"
#include "Arduino.h"

#include <stdarg.h>
#include <stdio.h>

#include "fb_gfx.h"
#include "fd_forward.h"
#include "fr_forward.h"

#include "telemetry.h"
#include "camSD.h"

void startCameraServer();
