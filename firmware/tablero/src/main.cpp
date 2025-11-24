#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "SevSegShift.h"
#include "config.h"
#include "display.h"
#include "game_control.h"

// Variables globales
RF24 radio(RF_CE_PIN, RF_CSN_PIN);
const byte address[6] = RF_ADDRESS;
SevSegShift sevseg(SHIFT_PIN_DS, SHIFT_PIN_SHCP, SHIFT_PIN_STCP);

int minutos = MINUTOS_JUEGO;
int contador_seg = minutos * 100;
bool stop_state = HIGH;
unsigned long gol1 = 0;
unsigned long gol2 = 0;

void setup() {
  // Inicializar display
  inicializarDisplay(sevseg);

  // Inicializar módulo RF
  // NOTA: Solo configurar setDataRate, el resto usa defaults de RF24
  // Esto mantiene compatibilidad con todos los dispositivos del sistema
  radio.begin();
  radio.setDataRate(RF24_2MBPS);  // 2Mbps (resto: canal 76, PA max, auto-ack true por default)
  radio.openReadingPipe(1, address);
  radio.startListening();
}

void loop() {
  static unsigned long tiempoejec = millis();

  // Procesar comandos RF
  if (radio.available()) {
    char receivedData[3];
    radio.read(&receivedData, sizeof(receivedData));
    procesarComando(receivedData, stop_state, gol1, gol2, contador_seg, minutos);
  }

  // Actualizar cronómetro
  actualizarCronometro(tiempoejec, stop_state, contador_seg, minutos);

  // Actualizar display
  actualizarDisplay(sevseg, gol1, gol2, contador_seg);
}
