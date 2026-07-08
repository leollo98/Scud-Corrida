#pragma once

#include "dtl_Page.h"
#include <ESPAsyncWebServer.h>
#include <SD.h>
#include <AsyncTCP.h>
#include <WiFi.h>
#include <algorithm>

struct LogFile {
  uint32_t number;
  uint32_t size;
  char filename[32];
};

void setupWebServer(AsyncWebServer &server);