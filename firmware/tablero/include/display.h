#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include "SevSegShift.h"

// Inicializar el display
void inicializarDisplay(SevSegShift& sevseg);

// Actualizar el display con goles y tiempo
void actualizarDisplay(SevSegShift& sevseg, unsigned long gol1, unsigned long gol2, int contador_seg);

#endif
