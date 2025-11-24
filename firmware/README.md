# Firmware - RoboCup Soccer

Firmware para los componentes físicos del sistema RoboCup Soccer.

## Estructura

```
firmware/
├── msr/          # Robot MSR (control de motores, comunicación RF)
├── tablero/      # Tablero de marcador/cronómetro
└── transmisor/   # Transmisor RF (conectado a PC vía USB)
```

## 🚀 Quick Start

### 1. Flashear Transmisor
```bash
cd firmware/transmisor
pio run -t upload --upload-port /dev/ttyUSB0
```

### 2. Flashear Robots
```bash
cd firmware/msr

# Robot 1 (hace 1 beep al encender)
pio run -e robot1 -t upload --upload-port /dev/ttyUSB0

# Robot 2 (hace 2 beeps al encender)
pio run -e robot2 -t upload --upload-port /dev/ttyUSB0

# Robot 3 y 4 similar...
```

### 3. Probar Comunicación
```bash
cd firmware/transmisor
python test_manual.py

>>> ping    # Verifica conexión a todos los dispositivos
>>> 1f      # Robot 1 adelante
>>> t1      # Tablero toggle pausa
```

## Hardware

- **Placa**: Arduino Nano (ATmega328P) con CH340G
- **Comunicación**: NRF24L01 (RF 2.4GHz)
- **Upload Speed**: 115200 baud (importante para CH340G)
- **MSR**: Puente H MX1508, solenoide, motores DC, buzzer
- **Tablero**: Displays 7 segmentos con shift registers

## Direcciones RF

```
┌────────────┬─────────────┬──────────────────┐
│ Dispositivo│  Dirección  │  Build Flag      │
├────────────┼─────────────┼──────────────────┤
│ Tablero    │   "00001"   │  (fixed)         │
│ Robot 1    │   "00002"   │  -DROBOT_ID=1    │
│ Robot 2    │   "00003"   │  -DROBOT_ID=2    │
│ Robot 3    │   "00004"   │  -DROBOT_ID=3    │
│ Robot 4    │   "00005"   │  -DROBOT_ID=4    │
└────────────┴─────────────┴──────────────────┘
```

**Ver detalles:** [`RF_ADDRESSES.md`](RF_ADDRESSES.md)

## Configuración RF

**Todos los dispositivos usan la misma configuración mínima:**

```cpp
radio.begin();
radio.setDataRate(RF24_2MBPS);  // Solo esto
// Resto usa defaults: canal 76, PA max, auto-ack true
```

Esta configuración está basada en el tablero original que funciona correctamente.

**IMPORTANTE:** El tablero actual tiene chip USB defectuoso, NO reflashearlo. Ya funciona correctamente con esta configuración.

## Build Flags - IDs de Robots

Cada robot tiene un ID único (1-4) definido en tiempo de compilación:

### Flashear Robot con ID específico

```bash
cd firmware/msr

# Robot 1
pio run -e robot1 -t upload --upload-port /dev/ttyUSB0

# Robot 2
pio run -e robot2 -t upload --upload-port /dev/ttyUSB0

# Robot 3
pio run -e robot3 -t upload --upload-port /dev/ttyUSB0

# Robot 4
pio run -e robot4 -t upload --upload-port /dev/ttyUSB0
```

### Verificar ID del Robot

Cuando enciendes un robot, el buzzer emite beeps según su ID:
- Robot 1: 🔊 beep
- Robot 2: 🔊🔊 beep beep
- Robot 3: 🔊🔊🔊 beep beep beep
- Robot 4: 🔊🔊🔊🔊 beep beep beep beep

**Ver más:** [`BUILD_FLAGS.md`](BUILD_FLAGS.md)

## Protocolo de Comandos

### Comandos de Robots

Formato: `R[ID][CMD]`

| Comando | Descripción |
|---------|-------------|
| `R1F` | Robot 1 adelante |
| `R1B` | Robot 1 atrás |
| `R1L` | Robot 1 izquierda |
| `R1R` | Robot 1 derecha |
| `R1P` | Robot 1 patear |
| `R1D` | Robot 1 activar rodillo |
| `R1S` | Robot 1 detener rodillo |
| `R1Q` | Robot 1 apagar |

Cambiar `1` por `2`, `3`, o `4` para otros robots.

### Comandos de Tablero

Formato: `T[CMD]`

| Comando | Descripción |
|---------|-------------|
| `T1` | Toggle pausa/inicio cronómetro |
| `T2` | Gol equipo 1 |
| `T3` | Gol equipo 2 |
| `T4` | Reset goles |
| `T5` | Reset tiempo |

### Comandos de Diagnóstico

| Comando | Descripción |
|---------|-------------|
| `ping` | Verifica conexión a todos los dispositivos |
| `scan` | Escanea canales RF (detecta interferencias) |
| `info` | Muestra configuración del radio |

## Testing Manual

```bash
cd firmware/transmisor
python test_manual.py
```

**Ejemplo de uso:**
```
>>> ping
=== Testing Connections ===
Testing connection to Tablero (00001)...
  ✓ Tablero responded!
Testing connection to Robot 1 (00002)...
  ✓ Robot 1 responded!
===========================

>>> 1f
✓ OK: Robot 1 <- F

>>> t2
✓ OK: Tablero <- G2

>>> help
# Muestra todos los comandos disponibles
```

**Ver más:** [`transmisor/TESTING.md`](transmisor/TESTING.md)

## Uso desde Python

```python
from robot_soccer.controllers.rf_transmitter import RFTransmitter, RobotCommand

# Conectar al transmisor
transmitter = RFTransmitter(port='/dev/ttyUSB0')
transmitter.connect()

# Enviar comandos
transmitter.send_robot_command(1, RobotCommand.FORWARD)
transmitter.send_robot_command(2, RobotCommand.KICK)
transmitter.send_tablero_command(TableroCommand.GOAL_TEAM1)

# Desconectar
transmitter.disconnect()
```

**Ver ejemplo completo:** `src/robot_soccer/controllers/rf_transmitter_example.py`

## Solución de Problemas

### Error: Permission denied en /dev/ttyUSB0

```bash
# Solución permanente
sudo usermod -aG uucp $USER
# Cerrar sesión y volver a entrar

# Solución temporal
sudo chmod 666 /dev/ttyUSB0
```

### Error: avrdude stk500_getsync() not in sync

**Causas:**
1. Puerto incorrecto
2. Velocidad incorrecta (debe ser 115200 para CH340G)
3. Bootloader corrupto (caso del tablero actual)
4. Cable USB defectuoso

**Solución:**
```bash
# Verificar puerto
pio device list

# Intentar re-flashear
pio run -t upload --upload-port /dev/ttyUSB0
```

**Para tablero:** NO intentar flashear si tiene chip USB defectuoso. Ya funciona correctamente.

### Robots no responden a comandos

**1. Verificar con ping:**
```bash
python test_manual.py
>>> ping
```

**2. Si ping falla:**
- Verificar que el robot está encendido
- Verificar conexiones NRF24L01:
  - VCC → 3.3V (NO 5V) ⚠️
  - GND → GND
  - CE → Pin 10
  - CSN → Pin 9
  - SCK → Pin 13
  - MOSI → Pin 11
  - MISO → Pin 12

**3. Si ping funciona pero comandos fallan:**
- Problema en código de parsing
- Verificar formato de comandos

### Compilación lenta

```bash
# Limpiar build cache
cd firmware/msr
pio run -t clean
```

## Comandos PlatformIO Útiles

```bash
# Listar puertos disponibles
pio device list

# Monitor serial (ver logs del Arduino)
pio device monitor --port /dev/ttyUSB0 --baud 9600

# Compilar sin flashear
pio run

# Limpiar build
pio run -t clean

# Ver información detallada
pio run -v

# Buscar librerías
pio lib search "nombre"

# Actualizar plataforma
pio platform update atmelavr
```

## Estructura de Archivos

### MSR (Robot)

```
msr/
├── include/
│   ├── config.h          # Pines, constantes, RF address
│   └── robot_control.h   # Control de motores y actuadores
├── src/
│   ├── main.cpp          # Loop principal y RF
│   └── robot_control.cpp # Implementación control
└── platformio.ini        # Configuración (4 environments: robot1-4)
```

### Tablero

```
tablero/
├── include/
│   ├── config.h          # Pines y constantes
│   ├── display.h         # Control display 7 segmentos
│   └── game_control.h    # Lógica de juego
├── src/
│   ├── main.cpp          # Loop principal y RF
│   ├── display.cpp       # Implementación display
│   └── game_control.cpp  # Implementación lógica juego
└── platformio.ini        # Configuración
```

### Transmisor

```
transmisor/
├── include/
│   ├── config.h          # Pines RF
│   ├── protocol.h        # Parsing de comandos
│   └── diagnostics.h     # Diagnósticos RF
├── src/
│   ├── main.cpp          # Loop principal
│   ├── protocol.cpp      # Implementación protocolo
│   └── diagnostics.cpp   # Implementación diagnósticos
├── platformio.ini        # Configuración
├── test_manual.py        # Testing interactivo
└── TESTING.md            # Guía de testing
```

## Pines

### NRF24L01 (Todos los dispositivos)
- CE: 10
- CSN: 9
- SCK: 13 (SPI)
- MISO: 12 (SPI)
- MOSI: 11 (SPI)
- VCC: 3.3V ⚠️
- GND: GND

### MSR (Robot)
- Motores: A0 (1A), A1 (1B), A2 (2A), A3 (2B)
- Solenoide: A4
- Motor DC (rodillo): A5
- Botón encendido: 4
- Control encendido: 2

### Tablero
- Shift Registers: DS=2, STCP=3, SHCP=4

## Dependencias

### MSR
```ini
lib_deps =
    nRF24/RF24@^1.4.9
    https://github.com/bhagman/SoftPWM.git
    pololu/PololuBuzzer@^1.0.0
```

### Tablero
```ini
lib_deps =
    nRF24/RF24@^1.4.9
    https://github.com/bridystone/SevSegShift.git
```

### Transmisor
```ini
lib_deps =
    nRF24/RF24@^1.4.9
```

## Notas Importantes

### ⚠️ Tablero con Chip USB Defectuoso

El tablero actual tiene un chip CH340G defectuoso que impide flashearlo.

**NO intentar flashear el tablero.**

Ya está flasheado y funciona correctamente. Si necesitas reemplazar la placa en el futuro, el código en `firmware/tablero/` está actualizado y listo.

### ⚠️ NRF24L01 a 3.3V

Los módulos NRF24L01 **DEBEN** alimentarse a 3.3V, NO a 5V.

Usar 5V puede dañar el módulo permanentemente.

### ⚠️ Velocidad de Upload

Para placas con CH340G, usar `upload_speed = 115200` en `platformio.ini`.

Otras velocidades (57600, 19200) pueden causar errores de sincronización.

## Referencias

- [PlatformIO Documentation](https://docs.platformio.org/)
- [RF24 Library](https://github.com/nRF24/RF24)
- [Arduino Nano Pinout](https://content.arduino.cc/assets/Pinout-NANO_latest.pdf)
- [NRF24L01 Datasheet](https://www.sparkfun.com/datasheets/Components/SMD/nRF24L01Pluss_Preliminary_Product_Specification_v1_0.pdf)
