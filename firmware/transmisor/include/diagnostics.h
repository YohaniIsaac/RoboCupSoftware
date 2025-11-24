#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include <Arduino.h>
#include <RF24.h>

// Funciones de diagnóstico
void printRadioDetails(RF24& radio);
bool testConnection(RF24& radio, const byte* address, const char* deviceName);
void scanChannels(RF24& radio);

#endif
