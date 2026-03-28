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
  detenerMotorDC();  // Dribbler requiere señal constante igual que las ruedas
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

void setMotorSpeeds(int16_t leftSpeed, int16_t rightSpeed) {
  // Aplicar calibración individual del robot
  float left_cal = leftSpeed * CALIBRATION_LEFT_FACTOR;
  float right_cal = rightSpeed * CALIBRATION_RIGHT_FACTOR;

  // Aplicar bias solo cuando ambos motores van en la misma dirección (movimiento recto)
  if (abs(leftSpeed - rightSpeed) < 20) {
    float bias_value = CALIBRATION_BIAS * 255.0;
    left_cal += bias_value;
    right_cal -= bias_value;
  }

  // Limitar a rango válido
  left_cal = constrain(left_cal, -255, 255);
  right_cal = constrain(right_cal, -255, 255);

  // Motor izquierdo
  if (left_cal >= 0) {
    SoftPWMSet(MOTOR_1A_PIN, 0);
    SoftPWMSet(MOTOR_1B_PIN, (uint8_t)left_cal);
  } else {
    SoftPWMSet(MOTOR_1A_PIN, (uint8_t)abs(left_cal));
    SoftPWMSet(MOTOR_1B_PIN, 0);
  }

  // Motor derecho
  if (right_cal >= 0) {
    SoftPWMSet(MOTOR_2A_PIN, (uint8_t)right_cal);
    SoftPWMSet(MOTOR_2B_PIN, 0);
  } else {
    SoftPWMSet(MOTOR_2A_PIN, 0);
    SoftPWMSet(MOTOR_2B_PIN, (uint8_t)abs(right_cal));
  }
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
