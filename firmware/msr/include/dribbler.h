#ifndef DRIBBLER_H
#define DRIBBLER_H

#include <Arduino.h>

// Configuración de oscilación del dribbler (persistida en EEPROM, ver persist.*).
// duty = onMs / (onMs + offMs). El pulsado limita la corriente media del motor del
// rodillo (sin sensor; a stall continuo se quema).
struct DribblerCfg {
  uint8_t onMs;   // ms — duración de la fase encendida
  uint8_t offMs;  // ms — duración de la fase apagada
  uint8_t wdtMs;  // ms — sin refresco 'D' por este tiempo → apagar (fail-safe propio)
};

// Aplica la config (al iniciar desde EEPROM y cuando llega el comando 'C').
void dribblerSetConfig(const DribblerCfg& cfg);

// Comando de runtime. power>0: engancha a esa potencia y arranca/mantiene la oscilación
// (y REFRESCA el watchdog). power==0: apaga de inmediato. 'now' = millis().
void dribblerSet(uint8_t power, unsigned long now);

// Corta el rodillo al instante (para el kick: apagar antes de disparar, sin pico de corriente).
void dribblerKickCut();

// Avanza la máquina: oscilación on/off + watchdog de seguridad. Llamar cada iteración del loop.
void dribblerUpdate(unsigned long now);

// Getters para telemetría (D2).
bool    dribblerEngaged();   // ¿enganchado? (estado, no la fase de oscilación)
uint8_t dribblerPower();     // potencia vigente de la fase ON

#endif
