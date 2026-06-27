#ifndef PERSIST_H
#define PERSIST_H

#include <Arduino.h>
#include "dribbler.h"  // DribblerCfg

// Bloque de configuración persistente en EEPROM, versionado y con marca de validez.
// El MAGIC evita leer basura (0xFF) de una EEPROM virgen como si fuera config.
struct PersistCfg {
  uint16_t    magic;    // PERSIST_MAGIC
  uint8_t     version;  // PERSIST_VERSION (para migración futura)
  DribblerCfg drib;     // {onMs, offMs, wdtMs}
};

// Carga la config desde EEPROM. Si MAGIC/VERSION no coinciden (virgen o esquema viejo),
// rellena con los defaults compilados y los escribe. Siempre deja cfg con valores válidos.
void persistLoad(PersistCfg& cfg);

// Persiste la sub-config del dribbler. EEPROM.put usa update por byte internamente: solo
// escribe los bytes que cambian → minimiza el desgaste de la EEPROM (~100k ciclos/celda).
void persistSaveDribbler(const DribblerCfg& drib);

#endif
