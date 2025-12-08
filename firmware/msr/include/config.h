#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Configuración de comunicación RF
#define RF_CE_PIN 10
#define RF_CSN_PIN 9

// ID del robot (definido por build flag)
#ifndef ROBOT_ID
#define ROBOT_ID 1  // Default si no se especifica
#endif

// Dirección RF única por robot
// Nota: "00001" está reservado para el Tablero
#if ROBOT_ID == 1
#define RF_ADDRESS "00002"
#elif ROBOT_ID == 2
#define RF_ADDRESS "00003"
#elif ROBOT_ID == 3
#define RF_ADDRESS "00004"
#elif ROBOT_ID == 4
#define RF_ADDRESS "00005"
#else
#define RF_ADDRESS "00002"  // Default
#endif

// Pines de control de motores (Puente H MX1508)
#define MOTOR_1A_PIN A0  // Motor izquierdo - atrás
#define MOTOR_1B_PIN A1  // Motor izquierdo - adelante
#define MOTOR_2A_PIN A2  // Motor derecho - adelante
#define MOTOR_2B_PIN A3  // Motor derecho - atrás

// Pines de actuadores
#define SOLENOIDE_PIN A4
#define MOTOR_DC_PIN A5

// Pines de control de sistema
#define BOTON_ENCENDIDO_PIN 4
#define ENCENDIDO_PIN 2

// Configuración de movimiento
#define DURACION_ACCION_MS 100  // Duración de cada acción en milisegundos
#define TIEMPO_PATEO_MS 50      // Duración del solenoide activado

// Velocidades de motores (0-255 para PWM)
#define VELOCIDAD_ADELANTE_IZQ 47
#define VELOCIDAD_ADELANTE_DER 45
#define VELOCIDAD_ATRAS_IZQ 47
#define VELOCIDAD_ATRAS_DER 45
#define VELOCIDAD_GIRO_LENTO 30
#define VELOCIDAD_GIRO_RAPIDO 40

// ========================================================================
// CALIBRACIÓN INDIVIDUAL POR ROBOT
// ========================================================================
// Estos factores corrigen diferencias físicas entre robots
// Valores de 0.0 a 1.0 (1.0 = sin corrección)
// Ajusta estos valores después de calibrar con Python

#if ROBOT_ID == 1
  #define CALIBRATION_LEFT_FACTOR 1.0
  #define CALIBRATION_RIGHT_FACTOR 1.0
  #define CALIBRATION_BIAS 0.0
#elif ROBOT_ID == 2
  #define CALIBRATION_LEFT_FACTOR 1.0
  #define CALIBRATION_RIGHT_FACTOR 1.0
  #define CALIBRATION_BIAS 0.0
#elif ROBOT_ID == 3
  #define CALIBRATION_LEFT_FACTOR 1.0
  #define CALIBRATION_RIGHT_FACTOR 1.0
  #define CALIBRATION_BIAS 0.0
#elif ROBOT_ID == 4
  #define CALIBRATION_LEFT_FACTOR 1.0
  #define CALIBRATION_RIGHT_FACTOR 1.0
  #define CALIBRATION_BIAS 0.0
#else
  #define CALIBRATION_LEFT_FACTOR 1.0
  #define CALIBRATION_RIGHT_FACTOR 1.0
  #define CALIBRATION_BIAS 0.0
#endif

#endif
