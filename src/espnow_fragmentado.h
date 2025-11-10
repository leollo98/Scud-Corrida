#ifndef ESPNOW_FRAGMENTADO_H
#define ESPNOW_FRAGMENTADO_H

#include <Arduino.h>
#include <vector>
#include <map>
#include <string>
#include <esp_now.h>

// Estrutura do pacote fragmentado
struct Packet {
  uint8_t id;
  uint8_t index;
  uint8_t total;
  uint8_t payload[247];
};

// Mapa global que armazena mensagens parciais
extern std::map<uint8_t, std::vector<std::string>> mensagensParciais;

// Envia uma string fragmentada via ESP-NOW
void enviarDadosFragmentado(const std::string& msg);

// Callback para reconstrução da mensagem no receptor
void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len);

#endif  // ESPNOW_FRAGMENTADO_H