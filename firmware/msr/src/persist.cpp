#include "persist.h"
#include "config.h"   // PERSIST_MAGIC/VERSION, DRIBBLER_DEFAULT_*
#include <EEPROM.h>

// La config vive al inicio de la EEPROM (la app no usa otra cosa de EEPROM).
static const int EE_ADDR = 0;

void persistLoad(PersistCfg& cfg) {
  EEPROM.get(EE_ADDR, cfg);
  if (cfg.magic != PERSIST_MAGIC || cfg.version != PERSIST_VERSION) {
    // EEPROM virgen o esquema distinto: escribir defaults y usarlos (config por defecto robusta).
    cfg.magic      = PERSIST_MAGIC;
    cfg.version    = PERSIST_VERSION;
    cfg.drib.onMs  = DRIBBLER_DEFAULT_ON_MS;
    cfg.drib.offMs = DRIBBLER_DEFAULT_OFF_MS;
    cfg.drib.wdtMs = DRIBBLER_DEFAULT_WDT_MS;
    cfg.dbgLevel   = DEBUG_DEFAULT_LEVEL;
    EEPROM.put(EE_ADDR, cfg);  // update por byte: no desgasta si ya estaba igual
  }
}

void persistSaveDebug(uint8_t dbgLevel) {
  PersistCfg cfg;
  EEPROM.get(EE_ADDR, cfg);   // conservar magic/version/dribbler
  // Escribir solo si cambió: si el bloque ya es válido y el nivel coincide, no tocar la EEPROM.
  if (cfg.magic == PERSIST_MAGIC && cfg.version == PERSIST_VERSION
      && cfg.dbgLevel == dbgLevel) {
    return;
  }
  cfg.magic    = PERSIST_MAGIC;
  cfg.version  = PERSIST_VERSION;
  cfg.dbgLevel = dbgLevel;
  EEPROM.put(EE_ADDR, cfg);
}

void persistSaveDribbler(const DribblerCfg& drib) {
  PersistCfg cfg;
  EEPROM.get(EE_ADDR, cfg);   // conservar magic/version/otros campos
  // Escribir solo si la config entrante difiere de la guardada (y el bloque es válido).
  // EEPROM.put ya hace update por byte, pero este guard evita hasta el recorrido de
  // comparación y deja explícita la intención de "no reescribir si no cambió nada".
  if (cfg.magic == PERSIST_MAGIC && cfg.version == PERSIST_VERSION
      && cfg.drib.onMs == drib.onMs && cfg.drib.offMs == drib.offMs
      && cfg.drib.wdtMs == drib.wdtMs) {
    return;
  }
  cfg.magic   = PERSIST_MAGIC;
  cfg.version = PERSIST_VERSION;
  cfg.drib    = drib;
  EEPROM.put(EE_ADDR, cfg);
}
