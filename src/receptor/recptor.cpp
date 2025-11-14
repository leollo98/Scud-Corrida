#include "../espnow/espnow_fragmentado.h"
#include "receptor.h"


void sendWeb(uint8_t j, uint16_t i) {
  std::string mensagemFinal;
  mensagemFinal = std::to_string(j) + " - " ;
  mensagemFinal += std::to_string(millis()) + ",";
  mensagemFinal += std::to_string(sin(0+i*2.0*3.14/200.0)+10.0) + ",";
  mensagemFinal += std::to_string(sin(1+i*2.0*3.14/200.0)-1.0) + ",";
  mensagemFinal += std::to_string(sin(2+i*2.0*3.14/200.0)+2.0) + ",";
  mensagemFinal += std::to_string(sin(0.5+i*2.0*3.14/200.0)+0.05) + ",";
  mensagemFinal += std::to_string(sin(1.5+i*2.0*3.14/200.0)+0.15) + ",";
  mensagemFinal += std::to_string(sin(2.5+i*2.0*3.14/200.0)-0.05) + ",";
  mensagemFinal += std::to_string(temperatureRead()) + ",";
  mensagemFinal += "0,0,0";
  Serial.println(mensagemFinal.c_str());
}

void setupReceptor() {
  Serial.begin(115200);
  setupEspNowReceive();
}

void loopReceptor() {
  #ifdef DADOS_SIMULADOS
  for (uint16_t i = 0; i < 200; i++)
  {
    sendWeb(1,i);
    sendWeb(2,i*2);
    delay(100);
  }
  #endif
}

