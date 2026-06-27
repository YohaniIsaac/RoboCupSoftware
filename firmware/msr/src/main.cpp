#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <SoftPWM.h>
#include <PololuBuzzer.h>
#include "config.h"
#include "robot_control.h"
#include "dribbler.h"
#include "persist.h"
#include "telemetry.h"

// Variables globales
PololuBuzzer buzzer;
RF24 radio(RF_CE_PIN, RF_CSN_PIN);
const byte address[6] = RF_ADDRESS;
unsigned long tiempoInicio = 0;

void setup() {
  // Configurar pin de encendido
  pinMode(ENCENDIDO_PIN, OUTPUT);
  digitalWrite(ENCENDIDO_PIN, HIGH);

  delay(1000);

  // DEBUG: Indicar ID del robot con buzzer
  // Robot 1 = 1 beep, Robot 2 = 2 beeps, etc.
  for (int i = 0; i < ROBOT_ID; i++) {
    buzzer.playFromProgramSpace(PSTR("!L16 V10 c"));
    delay(200);
  }

  // Configurar pines de actuadores
  pinMode(SOLENOIDE_PIN, OUTPUT);
  // MOTOR_DC_PIN se controla via SoftPWM (inicializado en SoftPWMBegin)
  pinMode(BOTON_ENCENDIDO_PIN, INPUT_PULLUP);

  // Configurar pines de motores
  pinMode(MOTOR_1A_PIN, OUTPUT);
  pinMode(MOTOR_1B_PIN, OUTPUT);
  pinMode(MOTOR_2A_PIN, OUTPUT);
  pinMode(MOTOR_2B_PIN, OUTPUT);

  // Inicializar módulo RF
  radio.begin();
  radio.setDataRate(RF24_2MBPS);  // Misma configuración que tablero
  radio.enableDynamicPayloads();  // requerido por los ACK payloads (telemetría D2)
  radio.enableAckPayload();       // adjuntar estado del robot al ACK (piggyback, ≈cero airtime)
  radio.openReadingPipe(1, address);
  radio.startListening();

  // Sonido de inicio
  buzzer.playFromProgramSpace(PSTR("!L16 V10 cdegreg4"));
  delay(1500);

  // Inicializar PWM por software
  SoftPWMBegin();

  // Cargar la config del dribbler desde EEPROM (o escribir defaults si está virgen) y aplicarla.
  // La oscilación on/off del rodillo la maneja el firmware desde aquí en adelante.
  PersistCfg persistCfg;
  persistLoad(persistCfg);
  dribblerSetConfig(persistCfg.drib);
  telemetrySetLevel(persistCfg.dbgLevel);
  telemetryConfig(persistCfg.drib);
}

void loop() {
  // Verificar botón de apagado
  if (digitalRead(BOTON_ENCENDIDO_PIN) == LOW) {
    digitalWrite(ENCENDIDO_PIN, LOW);
  }

  // Drenar la FIFO RF: procesar TODOS los paquetes encolados en esta iteración (no uno por
  // loop). Con el lazo no bloqueante (el solenoide ya no usa delay) la FIFO de 3 niveles no
  // se desborda aunque lleguen M+D+... casi juntos.
  while (radio.available()) {
    uint8_t data[5];
    radio.read(&data, sizeof(data));
    unsigned long now = millis();

    switch (data[0]) {
      case 'M': {  // Motor: data[2]=left+128, data[3]=right+128
        int16_t leftSpeed  = (int16_t)data[2] - 128;
        int16_t rightSpeed = (int16_t)data[3] - 128;
        setMotorSpeeds(leftSpeed, rightSpeed);
        tiempoInicio = now;          // refresca el watchdog de MOVIMIENTO (solo ruedas)
        telemetryCountM();
        break;
      }
      case 'D': {  // Dribbler: data[2]=potencia (0=apagar ya, >0=enganchar+oscilar)
        dribblerSet(data[2], now);   // el módulo tiene su PROPIO watchdog (no toca tiempoInicio)
        telemetryCountD();
        break;
      }
      case 'C': {  // Config dribbler en runtime: data[2]=onMs, data[3]=offMs, data[4]=wdtMs
        DribblerCfg cfg = { data[2], data[3], data[4] };
        dribblerSetConfig(cfg);
        telemetryConfig(cfg);
        persistSaveDribbler(cfg);    // persiste en EEPROM
        telemetrySetEvent(DBG_EV_CONFIG);
        break;
      }
      case 'G': {  // Debug/telemetría: data[2]=nivel (0=partido,1=eventos,2=verbose). Persiste.
        telemetrySetLevel(data[2]);
        persistSaveDebug(data[2]);
        break;
      }
      case '?': {  // Consulta de telemetría: encolar el estado YA (aunque el nivel sea 0).
        telemetryForcePending();
        break;
      }
      default:     // Comandos discretos (F,B,L,R,P,S,Q); 'P' corta el rodillo y dispara
        ejecutarComando(data[0]);
        break;
    }
  }

  // Telemetría (D2): si hay un record pendiente (evento con nivel>=1, o consulta '?'), encolarlo
  // como ACK-payload — se manda piggyback en el ACK del PRÓXIMO comando. Nivel 0 → nunca encola.
  if (telemetryTakePending()) {
    uint8_t tbuf[TELEMETRY_SIZE];
    uint8_t tlen = telemetryBuild(tbuf);
    radio.writeAckPayload(1, tbuf, tlen);
  }

  unsigned long now = millis();
  // Watchdog de MOVIMIENTO: sin refresco 'M' por DURACION_ACCION_MS → frenar SOLO ruedas.
  if (now - tiempoInicio >= DURACION_ACCION_MS) {
    detenerMovimiento();
  }
  dribblerUpdate(now);    // oscilación on/off + watchdog propio del rodillo
  solenoidUpdate(now);    // baja el pin del solenoide tras TIEMPO_PATEO_MS (no bloqueante)
}
