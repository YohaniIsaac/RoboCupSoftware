#include "display.h"
#include "config.h"

void inicializarDisplay(SevSegShift& sevseg) {
  byte numDigits = 8;
  byte digitPins[] = {8 + 3, 8 + 2, 8 + 1, 8 + 0, 8 + 4, 8 + 5, 8 + 6, 8 + 7};
  byte segmentPins[] = {0, 1, 2, 3, 4, 5, 6, 7};
  bool resistorsOnSegments = true;
  byte hardwareConfig = COMMON_CATHODE;
  bool updateWithDelays = false;
  bool leadingZeros = true;
  bool disableDecPoint = false;

  sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins,
               resistorsOnSegments, updateWithDelays, leadingZeros, disableDecPoint);
  sevseg.setBrightness(BRILLO_DISPLAY);
}

void actualizarDisplay(SevSegShift& sevseg, unsigned long gol1, unsigned long gol2, int contador_seg) {
  char displayNumber[9];
  sprintf(displayNumber, "%08lu", (gol1 * 1000000) + (gol2 * 10000) + contador_seg);
  sevseg.setChars(displayNumber);
  sevseg.refreshDisplay();
}
