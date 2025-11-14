#ifndef MAIN_H
#define MAIN_H

// Inclui os arquivos .h baseado na flag
#ifdef MODO_DTL
  #include "dtl/dtl.h"
#elif defined(MODO_RECEPTOR)
  #include "receptor/receptor.h"
#else
  #error "Nenhum modo definido! Defina no platformio.ini"
#endif


#endif