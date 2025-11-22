#ifndef GAME_CONTROL_H
#define GAME_CONTROL_H

#include <Arduino.h>

// Procesar comandos recibidos por RF
void procesarComando(char receivedData[], bool& stop_state, unsigned long& gol1,
                     unsigned long& gol2, int& contador_seg, int minutos);

// Actualizar el cronómetro
void actualizarCronometro(unsigned long& tiempoejec, bool stop_state,
                          int& contador_seg, int minutos);

#endif
