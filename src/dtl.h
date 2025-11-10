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
#include <espnow_fragmentado.h>

#define chipSelectPin 5 // chip select (CS) do módulo do cartão SD
#define pinPresFreio 34
// #define dev
#define pin
//  1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 1

Adafruit_MPU6050 mpu;
SdFat sd;
SdFile dataFile;
TaskHandle_t Task1;

// mutex
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Contador de pulsos
#define PCNT_UNIT_DIREITA PCNT_UNIT_0
#define PCNT_UNIT_ESQUERDA PCNT_UNIT_1

// SD chip select pin.
const uint8_t SD_CS_PIN = 5;

// Use a large percent of sector size for best performance (512B -> 2K -> 4k).

#define pageSize 1900 // 2048 com margem de segurança
#define numberOfPages 3
bool Copia = false;
bool trava = false;
std::string csv = "";
std::string dadosParaGravar = "";
uint32_t linha = 0;

struct dados {
  float accel[3];
  float gyro[3];
  float tempC;
  uint32_t presFreio = 0;
  uint8_t pulsosRodaDireita = 0;
  uint8_t pulsosRodaEsquerda = 0;
};
dados dataFrame;

uint32_t previousMillis = 0;

void setupDTL();
void loopDTL();

#endif