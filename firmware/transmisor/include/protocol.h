#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>

// Tipos de mensaje
enum MessageType {
  MSG_ROBOT_CONTROL,
  MSG_TABLERO_CONTROL,
  MSG_UNKNOWN
};

// Estructura para comandos de robot
struct RobotCommand {
  uint8_t robotId;  // 1-4
  char command;     // F, B, L, R, P, D, S, Q
};

// Estructura para comandos de tablero
struct TableroCommand {
  char header;      // 'G'
  uint8_t command;  // 1-5
  uint8_t data;     // Datos adicionales si es necesario
};

// Funciones de procesamiento de protocolo
MessageType parseMessage(const String& msg, RobotCommand& robotCmd, TableroCommand& tableroCmd);
bool isValidRobotCommand(char cmd);

#endif
