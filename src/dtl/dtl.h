#ifndef DTL_H
#define DTL_H

#include "driver/pcnt.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include "../HTML/dtl_server.h"
#include <Preferences.h>
#include <global.h>


// Configuration constants
constexpr uint8_t SD_CHIP_SELECT_PIN = 5;
constexpr uint8_t PIN_BRAKE_PRESSURE = 34;
constexpr uint8_t SD_FAT_TYPE = 1; //  1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
constexpr uint16_t PAGE_SIZE = 1900; // 2048 com margem de segurança
constexpr uint8_t NUM_PAGES = 2;
constexpr uint16_t FREQUENCY = 100; // Hz
constexpr uint16_t COLECTION_TIME = 1000/FREQUENCY;

// Pulse counter units
#define PCNT_UNIT_RIGHT PCNT_UNIT_0
#define PCNT_UNIT_LEFT PCNT_UNIT_1

void setupDTL();
void loopDTL();

#endif