#include "espnow_fragmentado.h"

void enviarDadosFragmentado(const std::string& msg) {
  static uint8_t msg_id = 0; // contador pra identificar mensagens únicas
  msg_id++;

  size_t totalSize = msg.size();
  const size_t maxPayload = 247;
  uint8_t totalPackets = (totalSize + maxPayload - 1) / maxPayload;

  for (uint8_t i = 0; i < totalPackets; i++) {
    Packet p{};
    p.id = msg_id;
    p.index = i;
    p.total = totalPackets;

    size_t start = i * maxPayload;
    size_t len = std::min(maxPayload, totalSize - start);
    memcpy(p.payload, msg.data() + start, len);

    esp_err_t result = esp_now_send(nullptr, reinterpret_cast<uint8_t*>(&p), len + 3);
    if (result != ESP_OK) {
      Serial.printf("Erro ao enviar pacote %d/%d (%d)\n", i + 1, totalPackets, result);
    }
    delay(2); // dá tempo pro rádio
  }
}

void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len < 4) return;

  const Packet* p = reinterpret_cast<const Packet*>(incomingData);

  mensagensParciais[p->id].resize(p->total);
  mensagensParciais[p->id][p->index] = std::string((char*)p->payload, len - 3);

  // verifica se todos chegaram
  bool completa = true;
  for (uint8_t i = 0; i < p->total; i++) {
    if (mensagensParciais[p->id][i].empty()) {
      completa = false;
      break;
    }
  }

  if (completa) {
    std::string mensagem;
    for (auto& parte : mensagensParciais[p->id])
      mensagem += parte;

    Serial.println(mensagem.c_str());
    mensagensParciais.erase(p->id); // limpa memória
  }
}