# Firmware - RoboCup Soccer

Firmware para los componentes físicos del sistema RoboCup Soccer.

## Estructura

```
firmware/
├── msr/          # Robot MSR (control de motores, comunicación RF)
└── tablero/      # Tablero de marcador/cronómetro
```

## Hardware

- **Placa**: Arduino Nano (ATmega328P)
- **Comunicación**: NRF24L01 (RF 2.4GHz)
- **MSR**: Puente H MX1508, solenoide, motores DC
- **Tablero**: Displays 7 segmentos con shift registers

## Comandos PlatformIO

### Compilar

```bash
# MSR
cd firmware/msr
pio run

# Tablero
cd firmware/tablero
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
