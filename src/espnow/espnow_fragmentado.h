#ifndef ESPNOW_FRAGMENTADO_H
#define ESPNOW_FRAGMENTADO_H

#include <Arduino.h>
#include <vector>
#include <map>
#include <string>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// Estrutura do pacote fragmentado
struct Packet {
  uint16_t id;
  uint8_t index;
  uint8_t total;
  uint8_t payload[246];
};

// Chave composta (MAC + ID da mensagem)
struct Key {
  std::string mac;
  uint16_t id;

  bool operator<(const Key& other) const {
    if (mac == other.mac) return id < other.id;
    return mac < other.mac;
  }
};

// Envia uma string fragmentada via ESP-NOW
void enviarDadosFragmentado(const std::string& msg);
void enviarComando(const char *comando);

// Callback para reconstrução da mensagem no receptor
void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len);

void setupEspNowSend();
void setupEspNowReceive();
void setupComandos();

#endif  // ESPNOW_FRAGMENTADO_H