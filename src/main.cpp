#include "main.h"

void setup() {
  // Chama o setup correto
  #ifdef MODO_DTL
    setupDTL();
  #else
    setupReceptor();
  #endif
}

void loop() {
  // Chama o loop correto
  #ifdef MODO_DTL
    loopDTL();
  #else
    loopReceptor();
  #endif
}