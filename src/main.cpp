#include <Arduino.h>

// Inclui os arquivos .h baseado na flag
#ifdef MODO_DTL
  #include "dtl.h"
#elif defined(MODO_RECEPTOR)
  #include "receptor.h"
#else
  #error "Nenhum modo definido! Defina no platformio.ini"
#endif

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