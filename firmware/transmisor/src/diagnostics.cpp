#include "diagnostics.h"

void printRadioDetails(RF24& radio) {
  Serial.println("\n=== Radio Details ===");

  Serial.print("Channel: ");
  Serial.println(radio.getChannel());

  Serial.print("Data Rate: ");
  Serial.println(radio.getDataRate());

  Serial.print("PA Level: ");
  Serial.println(radio.getPALevel());

  Serial.print("Is Plus Variant: ");
  Serial.println(radio.isPVariant() ? "Yes" : "No");

  Serial.println("=====================\n");
}

bool testConnection(RF24& radio, const byte* address, const char* deviceName) {
  Serial.print("Testing connection to ");
  Serial.print(deviceName);
  Serial.print(" (");

  // Imprimir dirección
  for (int i = 0; i < 5; i++) {
    Serial.print((char)address[i]);
  }
  Serial.println(")...");

  radio.openWritingPipe(address);

  // Enviar byte de test
  char testByte = 'T';
  bool success = radio.write(&testByte, 1);

  if (success) {
    Serial.print("  ✓ ");
    Serial.print(deviceName);
    Serial.println(" responded!");
    return true;
  } else {
    Serial.print("  ✗ ");
    Serial.print(deviceName);
    Serial.println(" NO RESPONSE");
    return false;
  }
}

void scanChannels(RF24& radio) {
  Serial.println("\n=== Scanning RF Channels ===");
  Serial.println("Scanning for activity on all channels...");

  const int num_channels = 126;
  byte values[num_channels];

  // Scan all channels
  for (int i = 0; i < num_channels; i++) {
    radio.setChannel(i);
    radio.startListening();
    delayMicroseconds(128);
    radio.stopListening();

    if (radio.testCarrier()) {
      values[i] = 1;
    } else {
      values[i] = 0;
    }
  }

  // Print results
  Serial.println("\nChannel Activity (X = activity detected):");
  for (int i = 0; i < num_channels; i++) {
    if (i % 16 == 0) {
      Serial.println();
      Serial.print(i);
      Serial.print(": ");
    }
    Serial.print(values[i] ? "X" : ".");
  }
  Serial.println("\n============================\n");

  // Restore to default channel (76)
  radio.setChannel(76);
}
