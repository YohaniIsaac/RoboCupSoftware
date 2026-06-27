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

// Dirección RF de un robot por id (1-4).
const byte* robotAddr(uint8_t id) {
  switch (id) {
    case 1: return addressRobot1;
    case 2: return addressRobot2;
    case 3: return addressRobot3;
    case 4: return addressRobot4;
    default: return addressRobot1;
  }
}

// Lee el ACK-payload de telemetría que el robot adjunta al ACK (si lo hay) y lo imprime
// parseable para Python. Record de 12 B (ver firmware/msr/telemetry.h):
// ['T', level, onMs, offMs, wdtMs, engaged, power, lastEvent, mLo, mHi, dLo, dHi].
void readAckTelemetry(uint8_t robotId) {
  if (!radio.available()) return;
  uint8_t t[32];
  uint8_t len = radio.getDynamicPayloadSize();
  if (len == 0 || len > 32) return;
  radio.read(t, len);
  if (len >= 12 && t[0] == 'T') {
    uint16_t mC = (uint16_t)t[8]  | ((uint16_t)t[9]  << 8);
    uint16_t dC = (uint16_t)t[10] | ((uint16_t)t[11] << 8);
    Serial.print(F("TELEM R")); Serial.print(robotId);
    Serial.print(F(" dbg="));   Serial.print(t[1]);
    Serial.print(F(" cfg="));   Serial.print(t[2]); Serial.print('/'); Serial.print(t[3]); Serial.print('/'); Serial.print(t[4]);
    Serial.print(F(" eng="));   Serial.print(t[5]);
    Serial.print(F(" pwr="));   Serial.print(t[6]);
    Serial.print(F(" ev="));    Serial.print(t[7]);
    Serial.print(F(" m="));     Serial.print(mC);
    Serial.print(F(" d="));     Serial.println(dC);
  }
}

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
  // DPL + ACK payload: recibir telemetría de los robots piggyback en el ACK (D2). Se APAGA
  // temporalmente solo en los writes al tablero (estático, no reflasheable). Ver MSG_TABLERO.
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
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

    // Debug/telemetría (D2): "G,id,level" (set nivel de debug) y "?,id" (consulta de estado).
    // Diagnóstico de baja frecuencia; se manejan acá (como ping/scan/info) sin tocar el protocolo.
    if (mensaje.charAt(0) == 'G') {
      int c1 = mensaje.indexOf(',');
      int c2 = mensaje.indexOf(',', c1 + 1);
      if (c1 > 0 && c2 > 0) {
        int id = mensaje.substring(c1 + 1, c2).toInt();
        int lv = mensaje.substring(c2 + 1).toInt();
        if (id >= 1 && id <= 4 && lv >= 0 && lv <= 255) {
          radio.openWritingPipe(robotAddr(id));
          uint8_t data[5] = {'G', (uint8_t)id, (uint8_t)lv, 0, 0};
          bool ok = radio.write(&data, sizeof(data));
          Serial.print(ok ? F("OK: Robot ") : F("ERROR G Robot "));
          Serial.print(id); Serial.print(F(" <- G(")); Serial.print(lv); Serial.println(F(")"));
          readAckTelemetry(id);
        }
      }
      return;
    }
    if (mensaje.charAt(0) == '?') {
      int c1 = mensaje.indexOf(',');
      if (c1 > 0) {
        int id = mensaje.substring(c1 + 1).toInt();
        if (id >= 1 && id <= 4) {
          radio.openWritingPipe(robotAddr(id));
          uint8_t data[5] = {'?', (uint8_t)id, 0, 0, 0};
          radio.write(&data, sizeof(data));   // 1º: el robot encola su telemetría
          radio.write(&data, sizeof(data));   // 2º: su ACK ya trae el record encolado
          readAckTelemetry(id);
        }
      }
      return;
    }

    RobotCommand robotCmd;
    MotorCommand motorCmd;
    DribblerCommand dribblerCmd;
    DribblerConfigCommand cfgCmd;
    TableroCommand tableroCmd;
    MessageType msgType = parseMessage(mensaje, robotCmd, motorCmd, dribblerCmd, cfgCmd, tableroCmd);

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
        readAckTelemetry(motorCmd.robotId);   // telemetría piggyback en el ACK (si la hay)
        break;
      }

      case MSG_DRIBBLER_CONTROL: {
        // Seleccionar dirección según el robot
        const byte* targetAddress;
        switch (dribblerCmd.robotId) {
          case 1: targetAddress = addressRobot1; break;
          case 2: targetAddress = addressRobot2; break;
          case 3: targetAddress = addressRobot3; break;
          case 4: targetAddress = addressRobot4; break;
          default: targetAddress = addressRobot1; break;
        }

        // Cambiar dirección de escritura
        radio.openWritingPipe(targetAddress);

        // Enviar paquete de 5 bytes: ['D', robot_id, power, 0, 0]
        uint8_t data[5];
        data[0] = 'D';
        data[1] = dribblerCmd.robotId;
        data[2] = dribblerCmd.power;
        data[3] = 0;
        data[4] = 0;

        bool success = radio.write(&data, sizeof(data));

        if (success) {
          Serial.print("OK: Robot ");
          Serial.print(dribblerCmd.robotId);
          Serial.print(" <- D(");
          Serial.print(dribblerCmd.power);
          Serial.println(")");
        } else {
          Serial.println("ERROR: Fallo al transmitir");
        }
        readAckTelemetry(dribblerCmd.robotId);
        break;
      }

      case MSG_DRIBBLER_CONFIG: {
        // Seleccionar dirección según el robot
        const byte* targetAddress;
        switch (cfgCmd.robotId) {
          case 1: targetAddress = addressRobot1; break;
          case 2: targetAddress = addressRobot2; break;
          case 3: targetAddress = addressRobot3; break;
          case 4: targetAddress = addressRobot4; break;
          default: targetAddress = addressRobot1; break;
        }

        // Cambiar dirección de escritura
        radio.openWritingPipe(targetAddress);

        // Enviar paquete de 5 bytes: ['C', robot_id, onMs, offMs, wdtMs]
        uint8_t data[5];
        data[0] = 'C';
        data[1] = cfgCmd.robotId;
        data[2] = cfgCmd.onMs;
        data[3] = cfgCmd.offMs;
        data[4] = cfgCmd.wdtMs;

        bool success = radio.write(&data, sizeof(data));

        if (success) {
          Serial.print("OK: Robot ");
          Serial.print(cfgCmd.robotId);
          Serial.print(" <- C(");
          Serial.print(cfgCmd.onMs);
          Serial.print(",");
          Serial.print(cfgCmd.offMs);
          Serial.print(",");
          Serial.print(cfgCmd.wdtMs);
          Serial.println(")");
        } else {
          Serial.println("ERROR: Fallo al transmitir");
        }
        readAckTelemetry(cfgCmd.robotId);
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
        readAckTelemetry(robotCmd.robotId);
        break;
      }

      case MSG_TABLERO_CONTROL: {
        // Cambiar dirección al tablero
        radio.openWritingPipe(addressTablero);

        // El tablero NO es reflasheable y usa payloads ESTÁTICOS (3 bytes). Apagar DPL solo para
        // este write y reactivarlo después (los robots siguen con DPL para la telemetría).
        radio.disableDynamicPayloads();

        // Enviar comando a tablero
        char data[3];
        data[0] = tableroCmd.header;
        data[1] = tableroCmd.command;
        data[2] = tableroCmd.data;

        bool success = radio.write(&data, sizeof(data));

        radio.enableDynamicPayloads();
        radio.enableAckPayload();

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
