#ifndef RECEPTOR_H
#define RECEPTOR_H

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>


void setupReceptor();
void loopReceptor();
#endif