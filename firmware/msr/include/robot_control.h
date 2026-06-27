#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <Arduino.h>

// Funciones de movimiento
void moverAdelante();
void moverAtras();
void girarIzquierda();
void girarDerecha();
void detenerMovimiento();
void setMotorSpeeds(int16_t leftSpeed, int16_t rightSpeed);

// Funciones de actuadores
void solenoidFire(unsigned long now);    // inicia disparo NO bloqueante
void solenoidUpdate(unsigned long now);  // baja el pin del solenoide tras TIEMPO_PATEO_MS
void setDribblerSpeed(uint8_t pwm);

// Funciones de sistema
void powerOff();
void ejecutarComando(char comando);

#endif
