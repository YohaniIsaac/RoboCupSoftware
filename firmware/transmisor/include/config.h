#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Configuración de comunicación RF
#define RF_CE_PIN 10
#define RF_CSN_PIN 9
#define RF_ADDRESS "00001"

// Configuración serial
#define SERIAL_BAUD 9600  // Compatible con Python SerialManager

// Protocolo de comunicación serial
// Formato: [TARGET][COMMAND][DATA]
// TARGET: 'R' (Robot) o 'T' (Tablero)
// Para robots: R[ID][CMD] donde ID=1-4, CMD=F/B/L/R/P/D/S/Q
// Para tablero: T[CMD][DATA] donde CMD=comando, DATA=datos adicionales

// Comandos para robots
#define CMD_FORWARD 'F'
#define CMD_BACKWARD 'B'
#define CMD_LEFT 'L'
#define CMD_RIGHT 'R'
#define CMD_KICK 'P'
#define CMD_ROLLER_ON 'D'
#define CMD_ROLLER_OFF 'S'
#define CMD_POWER_OFF 'Q'

// Comandos para tablero
#define CMD_TOGGLE_PAUSE 1
#define CMD_GOAL_TEAM1 2
#define CMD_GOAL_TEAM2 3
#define CMD_RESET_GOALS 4
#define CMD_RESET_TIME 5

#endif
