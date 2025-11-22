#include "game_control.h"
#include "config.h"

void procesarComando(char receivedData[], bool& stop_state, unsigned long& gol1,
                     unsigned long& gol2, int& contador_seg, int minutos) {
  if (receivedData[0] == 'G') {
    switch (receivedData[1]) {
      case CMD_TOGGLE_PAUSE:
        stop_state = !stop_state;
        break;
      case CMD_GOL_EQUIPO1:
        gol1++;
        break;
      case CMD_GOL_EQUIPO2:
        gol2++;
        break;
      case CMD_RESET_GOLES:
        gol1 = 0;
        gol2 = 0;
        break;
      case CMD_RESET_TIEMPO:
        contador_seg = minutos * 100;
        break;
    }
  }
}

void actualizarCronometro(unsigned long& tiempoejec, bool stop_state,
                          int& contador_seg, int minutos) {
  if (millis() >= tiempoejec) {
    tiempoejec += 1000;

    if (stop_state == LOW) {
      contador_seg--;

      // Ajustar cuando se llega a X:59 -> X:00
      for (int i = minutos; i >= 0; i--) {
        if (contador_seg == (i * 100) - 1) {
          contador_seg = contador_seg - 40;
        }
      }
    }
  }
}
