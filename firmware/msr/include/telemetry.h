#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>
#include "dribbler.h"  // DribblerCfg

// Telemetría del robot para diagnóstico (D2). Registra contadores y el último evento, y arma
// un record de estado que main() encola como ACK-payload del nRF24 (piggyback en el ACK del
// próximo comando → ≈cero airtime). Con nivel 0 (PARTIDO) NO encola nada (cero latencia).
#define TELEMETRY_SIZE 12

void    telemetrySetLevel(uint8_t level);          // 0=off, 1=eventos, 2=verbose
uint8_t telemetryLevel();
void    telemetryConfig(const DribblerCfg& cfg);   // refleja la config vigente en el record
void    telemetryCountM();                         // ++ contador de comandos 'M'
void    telemetryCountD();                         // ++ contador de comandos 'D'
void    telemetrySetEvent(uint8_t evCode);         // último evento; marca pendiente si nivel>=1
void    telemetryForcePending();                   // fuerza encolar (comando de consulta '?')
bool    telemetryTakePending();                    // true UNA vez si hay encole pendiente
uint8_t telemetryBuild(uint8_t* buf);              // llena buf (>=TELEMETRY_SIZE); devuelve len

#endif
