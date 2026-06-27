#include "telemetry.h"
#include "config.h"

// Estado de telemetría (mínimo, sin asignación dinámica). El record se arma on-demand.
static uint8_t  s_level     = DEBUG_DEFAULT_LEVEL;
static uint8_t  s_on = 0, s_off = 0, s_wdt = 0;   // espejo de la config vigente para el record
static uint16_t s_mCount    = 0;
static uint16_t s_dCount    = 0;
static uint8_t  s_lastEvent = DBG_EV_NONE;
static bool     s_pending   = false;

void telemetrySetLevel(uint8_t level) { s_level = level; }
uint8_t telemetryLevel() { return s_level; }

void telemetryConfig(const DribblerCfg& cfg) {
  s_on = cfg.onMs; s_off = cfg.offMs; s_wdt = cfg.wdtMs;
}

void telemetryCountM() { s_mCount++; }
void telemetryCountD() { s_dCount++; }

void telemetrySetEvent(uint8_t evCode) {
  s_lastEvent = evCode;
  if (s_level >= 1) s_pending = true;   // nivel 0: no encola → cero latencia en partido
}

void telemetryForcePending() { s_pending = true; }  // '?' encola aunque el nivel sea 0

bool telemetryTakePending() {
  if (!s_pending) return false;
  s_pending = false;
  return true;
}

uint8_t telemetryBuild(uint8_t* buf) {
  buf[0]  = TELEMETRY_MAGIC;
  buf[1]  = s_level;
  buf[2]  = s_on;
  buf[3]  = s_off;
  buf[4]  = s_wdt;
  buf[5]  = dribblerEngaged() ? 1 : 0;
  buf[6]  = dribblerPower();
  buf[7]  = s_lastEvent;
  buf[8]  = (uint8_t)(s_mCount & 0xFF);
  buf[9]  = (uint8_t)(s_mCount >> 8);
  buf[10] = (uint8_t)(s_dCount & 0xFF);
  buf[11] = (uint8_t)(s_dCount >> 8);
  return TELEMETRY_SIZE;
}
