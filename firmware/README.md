# Firmware - RoboCup Soccer

Firmware para los componentes físicos del sistema RoboCup Soccer.

## Estructura

```
firmware/
├── msr/          # Robot MSR (control de motores, comunicación RF)
├── tablero/      # Tablero de marcador/cronómetro
└── transmisor/   # Transmisor RF (conectado a PC vía USB)
```

## Hardware

- **Placa**: Arduino Nano (ATmega328P)
- **Comunicación**: NRF24L01 (RF 2.4GHz)
- **MSR**: Puente H MX1508, solenoide, motores DC
- **Tablero**: Displays 7 segmentos con shift registers
- **Transmisor**: NRF24L01, conexión USB a PC

## Comandos PlatformIO

### Compilar

```bash
# MSR
cd firmware/msr
pio run

# Tablero
cd firmware/tablero
pio run

# Transmisor
cd firmware/transmisor
pio run
```

### Flashear (subir a la placa)

```bash
# MSR
cd firmware/msr
pio run -t upload --upload-port /dev/ttyUSB0

# Tablero
cd firmware/tablero
pio run -t upload --upload-port /dev/ttyUSB0

# Transmisor
cd firmware/transmisor
pio run -t upload --upload-port /dev/ttyUSB0
```

### Listar puertos disponibles

```bash
pio device list
```

### Monitor serial

```bash
# MSR
cd firmware/msr
pio device monitor --port /dev/ttyUSB0

# Tablero
cd firmware/tablero
pio device monitor --port /dev/ttyUSB0
```

### Limpiar build

```bash
cd firmware/msr
pio run -t clean
```

## Flujo de trabajo típico

1. **Conectar Arduino Nano**
2. **Verificar puerto**: `pio device list`
3. **Compilar y flashear**: `cd firmware/msr && pio run -t upload --upload-port /dev/ttyUSB0`
4. **Ver logs** (opcional): `pio device monitor --port /dev/ttyUSB0`

## Proyecto MSR

**Descripción**: Control del robot de fútbol

**Funcionalidades**:
- Recepción de comandos RF (adelante, atrás, izquierda, derecha)
- Control de motores con PWM por software
- Activación de solenoide para patear
- Control de rodillo (motor DC)
- Sistema de apagado por botón

**Librerías**:
- RF24: Comunicación NRF24L01
- SoftPWM: PWM por software para control de motores
- PololuBuzzer: Sonidos de feedback

**Pines**:
- CE: 10, CSN: 9 (NRF24L01)
- Motores: A0, A1, A2, A3 (Puente H)
- Solenoide: A4
- Motor DC: A5
- Botón encendido: 4
- Control encendido: 2

## Proyecto Tablero

**Descripción**: Marcador y cronómetro del juego

**Funcionalidades**:
- Display 8 dígitos: goles + tiempo
- Recepción de comandos RF (goles, pausar, reiniciar)
- Cronómetro regresivo

**Librerías**:
- RF24: Comunicación NRF24L01
- SevSegShift: Control de displays con shift registers

**Pines**:
- CE: 10, CSN: 9 (NRF24L01)
- Shift registers: DS=2, STCP=3, SHCP=4

## Proyecto Transmisor

**Descripción**: Puente entre PC y robots/tablero vía RF

**Funcionalidades**:
- Recibe comandos por serial (USB desde PC)
- Transmite comandos por RF a robots y tablero
- Protocolo simple y extensible

**Librerías**:
- RF24: Comunicación NRF24L01

**Pines**:
- CE: 10, CSN: 9 (NRF24L01)

**Protocolo Serial** (115200 baud):
```
Robots:   R[ID][CMD]    Ejemplo: R1F (Robot 1, Forward)
Tablero:  T[CMD]        Ejemplo: T2  (Goal equipo 1)
```

**Uso desde Python**:
```python
from robot_soccer.controllers.rf_transmitter import RFTransmitter, RobotCommand

transmitter = RFTransmitter(port='/dev/ttyUSB0')
transmitter.connect()
transmitter.send_robot_command(1, RobotCommand.FORWARD)
transmitter.disconnect()
```

Ver ejemplo completo en: `src/robot_soccer/controllers/rf_transmitter_example.py`

**Testing manual (sin Python)**:
```bash
cd firmware/transmisor
python test_manual.py

# O directamente por serial:
echo "R1F" > /dev/ttyUSB0  # Robot 1 adelante
```

Ver guía completa de testing: `firmware/transmisor/TESTING.md`

## Comunicación RF

**Dirección**: `00001`
**Canal**: 76 (2.476 GHz)
**Data Rate**: 2 Mbps

### Protocolo MSR
- `'F'`: Adelante
- `'B'`: Atrás
- `'L'`: Izquierda
- `'R'`: Derecha
- `'P'`: Patear
- `'D'`: Activar rodillo
- `'S'`: Detener rodillo
- `'Q'`: Apagar robot

### Protocolo Tablero
- `{'G', 1}`: Toggle pausa/inicio
- `{'G', 2}`: Gol equipo 1
- `{'G', 3}`: Gol equipo 2
- `{'G', 4}`: Reset goles
- `{'G', 5}`: Reset tiempo

## Solución de problemas

### Puerto serial no disponible
```bash
# Agregar usuario al grupo uucp (Arch Linux)
sudo usermod -aG uucp $USER
# Cerrar sesión y volver a entrar
```

### Error de permisos temporalmente
```bash
sudo chmod 666 /dev/ttyUSB0
```

### Reinstalar dependencias
```bash
cd firmware/msr
pio pkg uninstall
pio pkg install
```

### Ver información detallada de compilación
```bash
pio run -v  # Verbose mode
```

## Desarrollo

### Agregar nueva librería

Edita `platformio.ini`:
```ini
lib_deps =
    nueva/libreria@^1.0.0
```

### Buscar librerías
```bash
pio lib search "nombre"
```

### Actualizar plataforma
```bash
pio platform update atmelavr
```
