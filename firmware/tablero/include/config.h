#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Configuración de comunicación RF
#define RF_CE_PIN 10
#define RF_CSN_PIN 9
#define RF_ADDRESS "00001"

// Pines de shift registers para displays
#define SHIFT_PIN_DS 2
#define SHIFT_PIN_STCP 3
#define SHIFT_PIN_SHCP 4

// Configuración del juego
#define MINUTOS_JUEGO 15
#define BRILLO_DISPLAY 90

// Comandos RF
#define CMD_TOGGLE_PAUSE 1
#define CMD_GOL_EQUIPO1 2
#define CMD_GOL_EQUIPO2 3
#define CMD_RESET_GOLES 4
#define CMD_RESET_TIEMPO 5

#endif
