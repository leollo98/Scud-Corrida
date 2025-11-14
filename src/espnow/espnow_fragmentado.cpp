#include "espnow_fragmentado.h"

// Lista de MACs conhecidos pra identificar carro 1 e carro 2
std::vector<std::string> macsConhecidos;
// Armazena as partes recebidas
std::map<Key, std::vector<std::string>> mensagensParciais;
uint8_t mac_receptor[6] = {0x0c, 0xb8, 0x15, 0xd7, 0x85, 0x80};

void readMacAddr(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}

#ifdef MODO_DTL

#include "../dtl/dtl.h"

void enviarDadosFragmentado(const std::string &msg) {
  static uint16_t msg_id = 0; // contador pra identificar mensagens únicas
  msg_id++;

  size_t totalSize = msg.size();
  const size_t maxPayload = 246;
  uint8_t totalPackets = (totalSize + maxPayload - 1) / maxPayload;

  for (uint8_t i = 0; i < totalPackets; i++) {
    Packet p{};
    p.id = msg_id;
    p.index = i;
    p.total = totalPackets;

    size_t start = i * maxPayload;
    size_t len = std::min(maxPayload, totalSize - start);
    memcpy(p.payload, msg.data() + start, len);

    esp_err_t result =
        esp_now_send(nullptr, reinterpret_cast<uint8_t *>(&p), len + 4);
    if (result != ESP_OK) {
      Serial.printf("Erro ao enviar pacote %d/%d (%d)\n", i + 1, totalPackets,
                    result);
    }
  }
}

void setupEspNowSend() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao inicializar ESP-NOW");
    return;
  }
  

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac_receptor, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (!esp_now_is_peer_exist(peer.peer_addr)) {
    if (esp_now_add_peer(&peer) != ESP_OK) {
      Serial.println("Erro ao adicionar peer");
    }
  }
}


#else

#include "../receptor/receptor.h"

void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len < 5)
    return; // precisa ter no mínimo cabeçalho

  // Converte o MAC pra string legível
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0],
           mac[1], mac[2], mac[3], mac[4], mac[5]);
  std::string macStrKey(macStr);

  // Adiciona à lista e mantém ordenado
  if (std::find(macsConhecidos.begin(), macsConhecidos.end(), macStrKey) ==
      macsConhecidos.end()) {
    macsConhecidos.push_back(macStrKey);
    std::sort(macsConhecidos.begin(), macsConhecidos.end());
  }

  const Packet *p = reinterpret_cast<const Packet *>(incomingData);
  Key key{macStrKey, p->id};

  // Garante espaço no buffer
  mensagensParciais[key].resize(p->total);
  size_t payloadLen = len - 4;
  mensagensParciais[key][p->index] =
      std::string((char *)p->payload, payloadLen);

  // Verifica se todos os pacotes chegaram
  bool completa = true;
  for (uint8_t i = 0; i < p->total; i++) {
    if (mensagensParciais[key][i].empty()) {
      completa = false;
      break;
    }
  }

  if (completa) {
    std::string mensagem;
    for (auto &parte : mensagensParciais[key])
      mensagem += parte;

    // Identifica se é carro 1 ou 2
    int numeroCarro = 0;
    if (macStrKey == macsConhecidos.front())
      numeroCarro = 1;
    else if (macsConhecidos.size() > 1 && macStrKey == macsConhecidos.back())
      numeroCarro = 2;
    else
      numeroCarro = 0; // qualquer outro que apareça

    Serial.printf("%d - %s\n", numeroCarro, mensagem.c_str());

    std::string mensagemFinal = std::to_string(numeroCarro) + " - " + mensagem;

    mensagensParciais.erase(key); // limpa memória
  }
}

void setupEspNowReceive() {
  WiFi.mode(WIFI_STA);
  readMacAddr();
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao iniciar ESP-NOW");
    esp_restart();
  }

  esp_now_register_recv_cb(onReceive);
  Serial.println("Receptor pronto");
}

#endif