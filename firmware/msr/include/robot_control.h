#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <Arduino.h>

// Funciones de movimiento
void moverAdelante();
void moverAtras();
void girarIzquierda();
void girarDerecha();
void detenerMovimiento();

// Funciones de actuadores
void activarSolenoide();
void activarMotorDC();
void detenerMotorDC();

// Funciones de sistema
void powerOff();
void ejecutarComando(char comando);

#endif
