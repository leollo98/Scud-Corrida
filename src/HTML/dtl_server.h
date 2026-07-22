#pragma once

#include "dtl_Page.h"
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <WiFi.h>
#include <algorithm>
#include <Preferences.h>
#include <Update.h>
#include <global.h>

struct LogFile {
  uint32_t size;
  char filename[32];
};

void setupWebServer(AsyncWebServer &server);