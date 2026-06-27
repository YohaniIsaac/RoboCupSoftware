#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>

// Tipos de mensaje
enum MessageType {
  MSG_ROBOT_CONTROL,      // Comandos discretos: R[ID][CMD]
  MSG_MOTOR_CONTROL,      // Comandos de velocidad variable: M,id,left,right
  MSG_DRIBBLER_CONTROL,   // Comandos de dribbler con PWM: D,id,power
  MSG_DRIBBLER_CONFIG,    // Config de oscilación del dribbler: C,id,onMs,offMs,wdtMs
  MSG_TABLERO_CONTROL,
  MSG_UNKNOWN
};

// Estructura para comandos de robot (discretos)
struct RobotCommand {
  uint8_t robotId;  // 1-4
  char command;     // F, B, L, R, P, D, S, Q
};

// Estructura para comandos de motor (velocidad variable)
struct MotorCommand {
  uint8_t robotId;    // 1-4
  int16_t leftSpeed;  // -255 a 255
  int16_t rightSpeed; // -255 a 255
};

// Estructura para comandos de dribbler (potencia variable)
struct DribblerCommand {
  uint8_t robotId;  // 1-4
  uint8_t power;    // 0-255 PWM
};

// Estructura para config de oscilación del dribbler (persiste en EEPROM del robot)
struct DribblerConfigCommand {
  uint8_t robotId;  // 1-4
  uint8_t onMs;     // ms — fase encendida
  uint8_t offMs;    // ms — fase apagada
  uint8_t wdtMs;    // ms — sin refresco 'D' → apagar
};

// Estructura para comandos de tablero
struct TableroCommand {
  char header;      // 'G'
  uint8_t command;  // 1-5
  uint8_t data;     // Datos adicionales si es necesario
};

// Funciones de procesamiento de protocolo
MessageType parseMessage(const String& msg, RobotCommand& robotCmd, MotorCommand& motorCmd, DribblerCommand& dribblerCmd, DribblerConfigCommand& cfgCmd, TableroCommand& tableroCmd);
bool isValidRobotCommand(char cmd);

#endif
