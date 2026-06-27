#include "dribbler.h"
#include "robot_control.h"  // setDribblerSpeed (escribe MOTOR_DC_PIN vía SoftPWM)
#include "config.h"         // DRIBBLER_DEFAULT_*
#include "telemetry.h"      // eventos de telemetría (engage/off)

// Estado interno del único rodillo del robot. Dos conceptos SEPARADOS:
//   s_engaged : ¿debo estar capturando? (estado de alto nivel, lo fija el comando 'D')
//   s_oscOn   : fase actual del pulsado (on/off rápido) DENTRO de estar enganchado
// Apagar el pin en la fase OFF de la oscilación NO es desenganchar: s_engaged sigue true
// y el rodillo vuelve en la próxima fase ON. Solo desengancha 'D' power=0, el kick o el wdt.
static DribblerCfg   s_cfg      = { DRIBBLER_DEFAULT_ON_MS,
                                    DRIBBLER_DEFAULT_OFF_MS,
                                    DRIBBLER_DEFAULT_WDT_MS };
static bool          s_engaged  = false;
static uint8_t       s_power    = 0;      // potencia de la fase ON (50 captura / 30 sostén)
static bool          s_oscOn    = false;  // fase actual de la oscilación
static unsigned long s_oscStart = 0;      // inicio de la fase actual
static unsigned long s_lastFeed = 0;      // último refresco 'D' (alimenta el watchdog propio)

void dribblerSetConfig(const DribblerCfg& cfg) {
  s_cfg = cfg;
}

void dribblerSet(uint8_t power, unsigned long now) {
  if (power == 0) {            // apagado inmediato (cesión / fin de captura)
    if (s_engaged) telemetrySetEvent(DBG_EV_DRIBBLER_OFF);  // solo en la transición
    s_engaged = false;
    s_power   = 0;
    setDribblerSpeed(0);
    return;
  }
  if (!s_engaged) {            // flanco de enganche: arrancar en fase ON para agarrar ya
    s_engaged  = true;
    s_oscOn    = true;
    s_oscStart = now;
    setDribblerSpeed(power);
    telemetrySetEvent(DBG_EV_DRIBBLER_ON);
  } else if (s_oscOn && power != s_power) {  // cambio de potencia (captura↔sostén) en plena fase ON
    setDribblerSpeed(power);
  }
  s_power    = power;
  s_lastFeed = now;            // refresca el watchdog en cada comando 'D'
}

void dribblerKickCut() {
  s_engaged = false;
  s_power   = 0;
  setDribblerSpeed(0);
}

void dribblerUpdate(unsigned long now) {
  if (!s_engaged) return;
  // Watchdog de seguridad PROPIO: si Python dejó de refrescar 'D', apagar (independiente
  // del watchdog de movimiento; el tráfico de 'M' ya NO mantiene vivo el rodillo).
  if (now - s_lastFeed >= s_cfg.wdtMs) {
    telemetrySetEvent(DBG_EV_DRIBBLER_OFF);   // fail-safe: apagado por watchdog
    s_engaged = false;
    s_power   = 0;
    setDribblerSpeed(0);
    return;
  }
  // Oscilación on/off (duty). Cambios de fase por millis(), no bloqueante.
  if (s_oscOn) {
    if (now - s_oscStart >= s_cfg.onMs) {
      s_oscOn    = false;
      s_oscStart = now;
      setDribblerSpeed(0);
    }
  } else {
    if (now - s_oscStart >= s_cfg.offMs) {
      s_oscOn    = true;
      s_oscStart = now;
      setDribblerSpeed(s_power);
    }
  }
}

bool    dribblerEngaged() { return s_engaged; }
uint8_t dribblerPower()   { return s_power; }
