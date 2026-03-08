#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "config.h"
#include "protocol.h"
#include "diagnostics.h"

// Variables globales
RF24 radio(RF_CE_PIN, RF_CSN_PIN);

// Direcciones RF para cada dispositivo
const byte addressTablero[6] = "00001";  // Tablero (no cambiar, ya funciona)
const byte addressRobot1[6] = "00002";   // Robot 1
const byte addressRobot2[6] = "00003";   // Robot 2
const byte addressRobot3[6] = "00004";   // Robot 3
const byte addressRobot4[6] = "00005";   // Robot 4

void setup() {
  // Inicializar comunicación serial
  Serial.begin(SERIAL_BAUD);

  // Inicializar módulo RF
  if (!radio.begin()) {
    Serial.println("ERROR: NRF24L01 no detectado!");
    Serial.println("Verifica las conexiones:");
    Serial.println("  CE  -> Pin 10");
    Serial.println("  CSN -> Pin 9");
    Serial.println("  SCK -> Pin 13");
    Serial.println("  MOSI -> Pin 11");
    Serial.println("  MISO -> Pin 12");
    Serial.println("  VCC -> 3.3V");
    Serial.println("  GND -> GND");
    while (1) { delay(1000); }
  }

  radio.setDataRate(RF24_2MBPS);  // Misma configuración que tablero (solo esto)
  radio.stopListening();  // Modo transmisor

  // Mensaje de inicio
  Serial.println("\n========================================");
  Serial.println("  Transmisor RF - RoboCup Soccer");
  Serial.println("========================================");

  printRadioDetails(radio);

  Serial.println("Protocolo:");
  Serial.println("  Robots: R[1-4][F/B/L/R/P/D/S/Q]");
  Serial.println("  Tablero: T[1-5]");
  Serial.println("\nComandos especiales:");
  Serial.println("  ping    - Test de conexión a todos");
  Serial.println("  scan    - Escanear canales RF");
  Serial.println("  info    - Mostrar info del radio");
  Serial.println("\nEsperando comandos...");
}

void loop() {
  if (Serial.available() > 0) {
    String mensaje = Serial.readStringUntil('\n');
    mensaje.trim();

    if (mensaje.length() == 0) {
      return;
    }

    // Comandos especiales de diagnóstico
    if (mensaje == "ping") {
      Serial.println("\n=== Testing Connections ===");
      testConnection(radio, addressTablero, "Tablero");
      testConnection(radio, addressRobot1, "Robot 1");
      testConnection(radio, addressRobot2, "Robot 2");
      testConnection(radio, addressRobot3, "Robot 3");
      testConnection(radio, addressRobot4, "Robot 4");
      Serial.println("===========================\n");
      return;
    }

    if (mensaje == "scan") {
      scanChannels(radio);
      return;
    }

    if (mensaje == "info") {
      printRadioDetails(radio);
      return;
    }

    RobotCommand robotCmd;
    MotorCommand motorCmd;
    TableroCommand tableroCmd;
    MessageType msgType = parseMessage(mensaje, robotCmd, motorCmd, tableroCmd);

    switch (msgType) {
      case MSG_MOTOR_CONTROL: {
        // Seleccionar dirección según el robot
        const byte* targetAddress;
        switch (motorCmd.robotId) {
          case 1: targetAddress = addressRobot1; break;
          case 2: targetAddress = addressRobot2; break;
          case 3: targetAddress = addressRobot3; break;
          case 4: targetAddress = addressRobot4; break;
          default: targetAddress = addressRobot1; break;
        }

        // Cambiar dirección de escritura
        radio.openWritingPipe(targetAddress);

        // Enviar estructura de motor (3 bytes)
        uint8_t data[5];
        data[0] = 'M';  // Identificador de comando de motor
        data[1] = motorCmd.robotId;
        data[2] = (uint8_t)(motorCmd.leftSpeed + 128);   // Convertir -128..127 a 0..255
        data[3] = (uint8_t)(motorCmd.rightSpeed + 128);  // Convertir -128..127 a 0..255
        data[4] = 0;  // Byte de relleno/checksum futuro

        bool success = radio.write(&data, sizeof(data));

        if (success) {
          Serial.print("OK: Robot ");
          Serial.print(motorCmd.robotId);
          Serial.print(" <- M(");
          Serial.print(motorCmd.leftSpeed);
          Serial.print(",");
          Serial.print(motorCmd.rightSpeed);
          Serial.println(")");
        } else {
          Serial.println("ERROR: Fallo al transmitir");
        }
        break;
      }

      case MSG_ROBOT_CONTROL: {
        // Seleccionar dirección según el robot
        const byte* targetAddress;
        switch (robotCmd.robotId) {
          case 1: targetAddress = addressRobot1; break;
          case 2: targetAddress = addressRobot2; break;
          case 3: targetAddress = addressRobot3; break;
          case 4: targetAddress = addressRobot4; break;
          default: targetAddress = addressRobot1; break;
        }

        // Cambiar dirección de escritura
        radio.openWritingPipe(targetAddress);

        // Enviar comando a robot
        bool success = radio.write(&robotCmd.command, sizeof(robotCmd.command));

        if (success) {
          Serial.print("OK: Robot ");
          Serial.print(robotCmd.robotId);
          Serial.print(" <- ");
          Serial.println(robotCmd.command);
        } else {
          Serial.println("ERROR: Fallo al transmitir");
        }
        break;
      }

      case MSG_TABLERO_CONTROL: {
        // Cambiar dirección al tablero
        radio.openWritingPipe(addressTablero);

        // Enviar comando a tablero
        char data[3];
        data[0] = tableroCmd.header;
        data[1] = tableroCmd.command;
        data[2] = tableroCmd.data;

        bool success = radio.write(&data, sizeof(data));

        if (success) {
          Serial.print("OK: Tablero <- G");
          Serial.println(tableroCmd.command);
        } else {
          Serial.println("ERROR: Fallo al transmitir");
        }
        break;
      }

      case MSG_UNKNOWN:
      default:
        Serial.print("ERROR: Comando invalido: ");
        Serial.println(mensaje);
        break;
    }

    // Sin delay: máximo throughput para control PID fluido
    // El rate limiting se maneja desde Python (rf_controller.py)
  }
}
