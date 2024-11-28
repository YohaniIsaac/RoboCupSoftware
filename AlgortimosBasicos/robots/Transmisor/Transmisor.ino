#include <SPI.h>
#include <RF24.h>

RF24 radio(10,9); // Pines CE, CSN
const byte address[6] = "00001"; // Dirección del módulo NRF24L01

void setup() {
  Serial.begin(9600); // Iniciar la comunicación serial
  radio.begin();
  radio.openWritingPipe(address);
  radio.stopListening();
}

void loop() {
  if (Serial.available()) {
    char comando = Serial.read(); // Leer el carácter del teclado

    // Enviar el comando al receptor a través del NRF24L01
    radio.write(&comando, sizeof(comando));
  }
}