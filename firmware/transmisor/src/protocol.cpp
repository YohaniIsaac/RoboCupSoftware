#include "protocol.h"
#include "config.h"

bool isValidRobotCommand(char cmd) {
  return (cmd == CMD_FORWARD || cmd == CMD_BACKWARD ||
          cmd == CMD_LEFT || cmd == CMD_RIGHT ||
          cmd == CMD_KICK || cmd == CMD_ROLLER_ON ||
          cmd == CMD_ROLLER_OFF || cmd == CMD_POWER_OFF);
}

MessageType parseMessage(const String& msg, RobotCommand& robotCmd, TableroCommand& tableroCmd) {
  if (msg.length() < 2) {
    return MSG_UNKNOWN;
  }

  char target = msg.charAt(0);

  // Comando para Robot: R[ID][CMD]
  // Ejemplo: "R1F" = Robot 1, Forward
  if (target == 'R' && msg.length() >= 3) {
    char idChar = msg.charAt(1);
    char cmdChar = msg.charAt(2);

    if (idChar >= '1' && idChar <= '4' && isValidRobotCommand(cmdChar)) {
      robotCmd.robotId = idChar - '0';
      robotCmd.command = cmdChar;
      return MSG_ROBOT_CONTROL;
    }
  }

  // Comando para Tablero: T[CMD]
  // Ejemplo: "T1" = Toggle pause, "T2" = Goal team 1
  else if (target == 'T' && msg.length() >= 2) {
    char cmdChar = msg.charAt(1);

    if (cmdChar >= '1' && cmdChar <= '5') {
      tableroCmd.header = 'G';
      tableroCmd.command = cmdChar - '0';
      tableroCmd.data = 0;
      return MSG_TABLERO_CONTROL;
    }
  }

  return MSG_UNKNOWN;
}
