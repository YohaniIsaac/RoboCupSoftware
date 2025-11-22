#include "robot_control.h"
#include "config.h"
#include <SoftPWM.h>

// Variables externas (definidas en main.cpp)
extern unsigned long tiempoInicio;

void moverAdelante() {
  SoftPWMSet(MOTOR_1A_PIN, 0);
  SoftPWMSet(MOTOR_1B_PIN, VELOCIDAD_ADELANTE_IZQ);
  SoftPWMSet(MOTOR_2A_PIN, VELOCIDAD_ADELANTE_DER);
  SoftPWMSet(MOTOR_2B_PIN, 0);
}

void moverAtras() {
  SoftPWMSet(MOTOR_1A_PIN, VELOCIDAD_ATRAS_IZQ);
  SoftPWMSet(MOTOR_1B_PIN, 0);
  SoftPWMSet(MOTOR_2A_PIN, 0);
  SoftPWMSet(MOTOR_2B_PIN, VELOCIDAD_ATRAS_DER);
}

void girarIzquierda() {
  SoftPWMSet(MOTOR_1A_PIN, VELOCIDAD_GIRO_LENTO);
  SoftPWMSet(MOTOR_1B_PIN, 0);
  SoftPWMSet(MOTOR_2A_PIN, VELOCIDAD_GIRO_RAPIDO);
  SoftPWMSet(MOTOR_2B_PIN, 0);
}

void girarDerecha() {
  SoftPWMSet(MOTOR_1A_PIN, 0);
  SoftPWMSet(MOTOR_1B_PIN, VELOCIDAD_GIRO_RAPIDO);
  SoftPWMSet(MOTOR_2A_PIN, 0);
  SoftPWMSet(MOTOR_2B_PIN, VELOCIDAD_GIRO_LENTO);
}

void detenerMovimiento() {
  SoftPWMSet(MOTOR_1A_PIN, 0);
  SoftPWMSet(MOTOR_1B_PIN, 0);
  SoftPWMSet(MOTOR_2A_PIN, 0);
  SoftPWMSet(MOTOR_2B_PIN, 0);
}

void activarSolenoide() {
  digitalWrite(SOLENOIDE_PIN, HIGH);
  delay(TIEMPO_PATEO_MS);
  digitalWrite(SOLENOIDE_PIN, LOW);
}

void activarMotorDC() {
  digitalWrite(MOTOR_DC_PIN, HIGH);
}

void detenerMotorDC() {
  digitalWrite(MOTOR_DC_PIN, LOW);
}

void powerOff() {
  digitalWrite(ENCENDIDO_PIN, LOW);
}

void ejecutarComando(char comando) {
  switch (comando) {
    case 'F':  // Adelante
      moverAdelante();
      tiempoInicio = millis();
      break;
    case 'B':  // Atrás
      moverAtras();
      tiempoInicio = millis();
      break;
    case 'L':  // Izquierda
      girarIzquierda();
      tiempoInicio = millis();
      break;
    case 'R':  // Derecha
      girarDerecha();
      tiempoInicio = millis();
      break;
    case 'P':  // Patear
      activarSolenoide();
      break;
    case 'D':  // Activar rodillo
      activarMotorDC();
      tiempoInicio = millis();
      break;
    case 'S':  // Detener rodillo
      detenerMotorDC();
      tiempoInicio = millis();
      break;
    case 'Q':  // Apagar
      powerOff();
      break;
    default:
      detenerMovimiento();
  }
}
