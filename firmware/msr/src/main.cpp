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

  // Configurar pines de actuadores
  pinMode(SOLENOIDE_PIN, OUTPUT);
  pinMode(MOTOR_DC_PIN, OUTPUT);
  pinMode(BOTON_ENCENDIDO_PIN, INPUT_PULLUP);

  // Configurar pines de motores
  pinMode(MOTOR_1A_PIN, OUTPUT);
  pinMode(MOTOR_1B_PIN, OUTPUT);
  pinMode(MOTOR_2A_PIN, OUTPUT);
  pinMode(MOTOR_2B_PIN, OUTPUT);

  // Inicializar módulo RF
  radio.begin();
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
    char comando;
    radio.read(&comando, sizeof(comando));
    ejecutarComando(comando);
  }

  // Detener movimiento después de la duración configurada
  if (millis() - tiempoInicio >= DURACION_ACCION_MS) {
    detenerMovimiento();
  }
}
