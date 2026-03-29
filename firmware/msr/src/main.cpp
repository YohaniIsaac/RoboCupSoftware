#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <SoftPWM.h>
#include <PololuBuzzer.h>
#include "config.h"
#include "robot_control.h"

// Variables globales
PololuBuzzer buzzer;
RF24 radio(RF_CE_PIN, RF_CSN_PIN);
const byte address[6] = RF_ADDRESS;
unsigned long tiempoInicio = 0;

void setup() {
  // Configurar pin de encendido
  pinMode(ENCENDIDO_PIN, OUTPUT);
  digitalWrite(ENCENDIDO_PIN, HIGH);

  delay(1000);

  // DEBUG: Indicar ID del robot con buzzer
  // Robot 1 = 1 beep, Robot 2 = 2 beeps, etc.
  for (int i = 0; i < ROBOT_ID; i++) {
    buzzer.playFromProgramSpace(PSTR("!L16 V10 c"));
    delay(200);
  }

  // Configurar pines de actuadores
  pinMode(SOLENOIDE_PIN, OUTPUT);
  // MOTOR_DC_PIN se controla via SoftPWM (inicializado en SoftPWMBegin)
  pinMode(BOTON_ENCENDIDO_PIN, INPUT_PULLUP);

  // Configurar pines de motores
  pinMode(MOTOR_1A_PIN, OUTPUT);
  pinMode(MOTOR_1B_PIN, OUTPUT);
  pinMode(MOTOR_2A_PIN, OUTPUT);
  pinMode(MOTOR_2B_PIN, OUTPUT);

  // Inicializar módulo RF
  radio.begin();
  radio.setDataRate(RF24_2MBPS);  // Misma configuración que tablero
  radio.openReadingPipe(1, address);
  radio.startListening();

  // Sonido de inicio
  buzzer.playFromProgramSpace(PSTR("!L16 V10 cdegreg4"));
  delay(1500);

  // Inicializar PWM por software
  SoftPWMBegin();
}

void loop() {
  // Verificar botón de apagado
  if (digitalRead(BOTON_ENCENDIDO_PIN) == LOW) {
    digitalWrite(ENCENDIDO_PIN, LOW);
  }

  // Procesar comandos RF
  if (radio.available()) {
    // Leer primer byte para determinar tipo de comando
    uint8_t data[5];
    radio.read(&data, sizeof(data));

    // Verificar si es comando de motor (M) o comando discreto
    if (data[0] == 'M') {
      // Comando de motor con velocidades variables
      // data[0] = 'M'
      // data[1] = robot_id
      // data[2] = left_speed (0-255, convertir a -128..127)
      // data[3] = right_speed (0-255, convertir a -128..127)

      int16_t leftSpeed = (int16_t)data[2] - 128;
      int16_t rightSpeed = (int16_t)data[3] - 128;

      setMotorSpeeds(leftSpeed, rightSpeed);
      tiempoInicio = millis();
    } else if (data[0] == 'D') {
      // Comando de dribbler con potencia variable (PWM 0-255)
      // data[0] = 'D'
      // data[1] = robot_id
      // data[2] = potencia PWM (0 = apagado, 255 = máximo)
      uint8_t pwm = data[2];
      setDribblerSpeed(pwm);
      tiempoInicio = millis();
    } else {
      // Comando discreto tradicional (F, B, L, R, P, S, Q)
      char comando = data[0];
      ejecutarComando(comando);
    }
  }

  // Detener movimiento después de la duración configurada
  if (millis() - tiempoInicio >= DURACION_ACCION_MS) {
    detenerMovimiento();
  }
}
