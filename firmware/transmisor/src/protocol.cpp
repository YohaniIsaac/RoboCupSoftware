#include "protocol.h"
#include "config.h"

bool isValidRobotCommand(char cmd) {
  return (cmd == CMD_FORWARD || cmd == CMD_BACKWARD ||
          cmd == CMD_LEFT || cmd == CMD_RIGHT ||
          cmd == CMD_KICK || cmd == CMD_ROLLER_ON ||
          cmd == CMD_ROLLER_OFF || cmd == CMD_POWER_OFF);
}

MessageType parseMessage(const String& msg, RobotCommand& robotCmd, MotorCommand& motorCmd, TableroCommand& tableroCmd) {
  if (msg.length() < 2) {
    return MSG_UNKNOWN;
  }

  char target = msg.charAt(0);

  // Comando para Motor con velocidad variable: M,id,left,right
  // Ejemplo: "M,1,150,100" = Robot 1, Left=150, Right=100
  if (target == 'M') {
    int firstComma = msg.indexOf(',');
    int secondComma = msg.indexOf(',', firstComma + 1);
    int thirdComma = msg.indexOf(',', secondComma + 1);

    if (firstComma > 0 && secondComma > 0 && thirdComma > 0) {
      int robotId = msg.substring(firstComma + 1, secondComma).toInt();
      int leftSpeed = msg.substring(secondComma + 1, thirdComma).toInt();
      int rightSpeed = msg.substring(thirdComma + 1).toInt();

      // Validar rangos (limitado a -127/127 por conversión uint8_t)
      if (robotId >= 1 && robotId <= 4 &&
          leftSpeed >= -127 && leftSpeed <= 127 &&
          rightSpeed >= -127 && rightSpeed <= 127) {
        motorCmd.robotId = robotId;
        motorCmd.leftSpeed = leftSpeed;
        motorCmd.rightSpeed = rightSpeed;
        return MSG_MOTOR_CONTROL;
      }
    }
  }

  // Comando para Robot: R[ID][CMD]
  // Ejemplo: "R1F" = Robot 1, Forward
  else if (target == 'R' && msg.length() >= 3) {
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
