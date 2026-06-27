#include "protocol.h"
#include "config.h"

bool isValidRobotCommand(char cmd) {
  return (cmd == CMD_FORWARD || cmd == CMD_BACKWARD ||
          cmd == CMD_LEFT || cmd == CMD_RIGHT ||
          cmd == CMD_KICK || cmd == CMD_ROLLER_ON ||
          cmd == CMD_ROLLER_OFF || cmd == CMD_POWER_OFF);
}

MessageType parseMessage(const String& msg, RobotCommand& robotCmd, MotorCommand& motorCmd, DribblerCommand& dribblerCmd, DribblerConfigCommand& cfgCmd, TableroCommand& tableroCmd) {
  if (msg.length() < 2) {
    return MSG_UNKNOWN;
  }

  char target = msg.charAt(0);

  // Comando de dribbler con potencia variable: D,id,power
  // Ejemplo: "D,1,150" = Robot 1, Dribbler PWM=150
  if (target == 'D') {
    int firstComma = msg.indexOf(',');
    int secondComma = msg.indexOf(',', firstComma + 1);

    if (firstComma > 0 && secondComma > 0) {
      int robotId = msg.substring(firstComma + 1, secondComma).toInt();
      int power = msg.substring(secondComma + 1).toInt();

      if (robotId >= 1 && robotId <= 4 && power >= 0 && power <= 255) {
        dribblerCmd.robotId = robotId;
        dribblerCmd.power = power;
        return MSG_DRIBBLER_CONTROL;
      }
    }
  }

  // Comando de CONFIG de oscilación del dribbler: C,id,onMs,offMs,wdtMs
  // Ejemplo: "C,1,65,15,150" = Robot 1, on=65ms off=15ms watchdog=150ms (persiste en EEPROM)
  else if (target == 'C') {
    int c1 = msg.indexOf(',');
    int c2 = msg.indexOf(',', c1 + 1);
    int c3 = msg.indexOf(',', c2 + 1);
    int c4 = msg.indexOf(',', c3 + 1);

    if (c1 > 0 && c2 > 0 && c3 > 0 && c4 > 0) {
      int robotId = msg.substring(c1 + 1, c2).toInt();
      int onMs    = msg.substring(c2 + 1, c3).toInt();
      int offMs   = msg.substring(c3 + 1, c4).toInt();
      int wdtMs   = msg.substring(c4 + 1).toInt();

      if (robotId >= 1 && robotId <= 4 &&
          onMs  >= 0 && onMs  <= 255 &&
          offMs >= 0 && offMs <= 255 &&
          wdtMs >= 0 && wdtMs <= 255) {
        cfgCmd.robotId = robotId;
        cfgCmd.onMs    = onMs;
        cfgCmd.offMs   = offMs;
        cfgCmd.wdtMs   = wdtMs;
        return MSG_DRIBBLER_CONFIG;
      }
    }
  }

  // Comando para Motor con velocidad variable: M,id,left,right
  // Ejemplo: "M,1,150,100" = Robot 1, Left=150, Right=100
  else if (target == 'M') {
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
