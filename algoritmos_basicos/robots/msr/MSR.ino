#include <SPI.h>
#include <RF24.h>
#include <SoftPWM.h>
#include <PololuBuzzer.h>

PololuBuzzer buzzer;

RF24 radio(10, 9); // Pines CE, CSN
const byte address[6] = "00001"; // Dirección del módulo NRF24L01

// Definición de pines
const int solenoidePin = A4;
const int motorDCPin = A5;
const int botonEncendidoPin = 4;
const int encendidoPin = 2;

// Definición de pines del puente H MX1508
const int motor1A = A0;
const int motor1B = A1;
const int motor2A = A2;
const int motor2B = A3;

// Definiciones de variables globales para temporización
unsigned long tiempoInicio = 0;
unsigned long duracionAccion = 100; // 100 ms


void setup() {
  pinMode(encendidoPin, OUTPUT);
  digitalWrite(encendidoPin, HIGH); // Encender el circuito

  delay(1000);

  // Configuración de pines
  pinMode(solenoidePin, OUTPUT);
  pinMode(motorDCPin, OUTPUT);
  pinMode(botonEncendidoPin, INPUT_PULLUP);
  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);

  

  // Inicialización del módulo NRF24L01
  radio.begin();
  radio.openReadingPipe(1, address);
  radio.startListening();

  buzzer.playFromProgramSpace(PSTR("!L16 V10 cdegreg4"));
  delay(1500);

 
  // Iniciación de PWM por software
  SoftPWMBegin();


  
}

void loop() {
  // Verificar el estado del botón de encendido
  if (digitalRead(botonEncendidoPin) == LOW) {
    digitalWrite(encendidoPin, LOW); // Apagar el circuito
  }
  // Verificar si se recibe un comando.
  if (radio.available()) {
    char comando;
    radio.read(&comando, sizeof(comando));
    ejecutarComando(comando);
  }

  // Verificar si se ha alcanzado la duración deseada
  if (millis() - tiempoInicio >= duracionAccion) {
    
    detenerMovimiento(); // Detener el movimiento después de la duración deseada
  }
}

void ejecutarComando(char comando) {
  switch (comando) {
    case 'F': // Adelante
      moverAdelante();
      tiempoInicio = millis(); // Iniciar el temporizador   
      break;
    case 'B': // Atrás
      moverAtras();
      tiempoInicio = millis(); // Iniciar el temporizador
      break;
    case 'L': // Izquierda
      girarIzquierda();
      tiempoInicio = millis(); // Iniciar el temporizador
      break;
    case 'R': // Derecha
      girarDerecha();
      tiempoInicio = millis(); // Iniciar el temporizador
      break;
    case 'P': // Patear
      activarSolenoide();
      break;
    case 'D': // Activar el motor DC (rodillo)
      activarMotorDC();
      tiempoInicio = millis(); // Iniciar el temporizador
      break;
    case 'S': // Detener el motor DC (rodillo)
      detenerMotorDC();
      tiempoInicio = millis(); // Iniciar el temporizador
      break;
    case 'Q':
      powerOff();
      break;
    default:
      detenerMovimiento();
  }
}

void moverAdelante() {
  SoftPWMSet(motor1A, 0);  // atras izq
  SoftPWMSet(motor1B, 47); // adelante iqz
  SoftPWMSet(motor2A, 45); // adelante der
  SoftPWMSet(motor2B, 0);  // atras der
  // Ajusta la velocidad y duración según sea necesario
}

void moverAtras() {
  SoftPWMSet(motor1A, 47); // atras izq
  SoftPWMSet(motor1B, 0);  // adelante izq
  SoftPWMSet(motor2A, 0); // adelante der
  SoftPWMSet(motor2B, 45); //atras der
  // Ajusta la velocidad y duración según sea necesario
}

void girarIzquierda() {
  SoftPWMSet(motor1A, 30); // atras izq
  SoftPWMSet(motor1B, 0);  // adelante izq
  SoftPWMSet(motor2A, 40); // adelante der
  SoftPWMSet(motor2B, 0);  // atras izq
  // Ajusta la velocidad y duración según sea necesario
}

void girarDerecha() {
  SoftPWMSet(motor1A, 0); // atras izq
  SoftPWMSet(motor1B, 40); // adelante izq
  SoftPWMSet(motor2A, 0); // adelante der
  SoftPWMSet(motor2B, 30); // atras der
  // Ajusta la velocidd y duración según sea necesario
}

void activarSolenoide() {
  digitalWrite(solenoidePin, HIGH);
  delay(50); // Ajusta el tiempo de pateo según sea necesario
  digitalWrite(solenoidePin, LOW);
}

void activarMotorDC() {
  digitalWrite(motorDCPin, HIGH);
  // No es necesario el delay aquí para mantenerlo activado durante un tiempo específico
}

void detenerMotorDC() {
  digitalWrite(motorDCPin, LOW);
  // No es necesario el delay aquí para mantenerlo activado durante un tiempo específico
}

void powerOff() {  
  digitalWrite(encendidoPin, LOW); // Apaga el robot.
}


void detenerMovimiento() {
  SoftPWMSet(motor1A, 0); 
  SoftPWMSet(motor1B, 0);
  SoftPWMSet(motor2A, 0);
  SoftPWMSet(motor2B, 0);
}