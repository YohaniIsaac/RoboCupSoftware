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
    EEPROM.put(EE_ADDR, cfg);  // update por byte: no desgasta si ya estaba igual
  }
}

void persistSaveDribbler(const DribblerCfg& drib) {
  PersistCfg cfg;
  EEPROM.get(EE_ADDR, cfg);   // conservar magic/version/otros campos
  cfg.magic   = PERSIST_MAGIC;
  cfg.version = PERSIST_VERSION;
  cfg.drib    = drib;
  EEPROM.put(EE_ADDR, cfg);
}
