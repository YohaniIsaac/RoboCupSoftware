# Build Flags - IDs de Robots

## Uso de Build Flags para Robots

Cada robot tiene un ID único (1-4) que se define al compilar usando **build flags**.

### Cómo funciona

El `platformio.ini` de MSR tiene 4 environments:

```ini
[env:robot1]  → ROBOT_ID=1 → Dirección RF: "00002"
[env:robot2]  → ROBOT_ID=2 → Dirección RF: "00003"
[env:robot3]  → ROBOT_ID=3 → Dirección RF: "00004"
[env:robot4]  → ROBOT_ID=4 → Dirección RF: "00005"

Tablero      →              → Dirección RF: "00001" (no usar para robots)
```

Cada robot escucha en una **dirección RF única**, por lo que puedes controlarlos individualmente.

---

## Compilar y Flashear Robots

### Robot 1
```bash
cd firmware/msr
pio run -e robot1 -t upload --upload-port /dev/ttyUSB0
```

### Robot 2
```bash
cd firmware/msr
pio run -e robot2 -t upload --upload-port /dev/ttyUSB0
```

### Robot 3
```bash
cd firmware/msr
pio run -e robot3 -t upload --upload-port /dev/ttyUSB0
```

### Robot 4
```bash
cd firmware/msr
pio run -e robot4 -t upload --upload-port /dev/ttyUSB0
```

---

## Verificar el ID del Robot

Cuando enciendas un robot, el buzzer emitirá beeps según su ID:
- **Robot 1**: 1 beep
- **Robot 2**: 2 beeps
- **Robot 3**: 3 beeps
- **Robot 4**: 4 beeps

Esto te ayuda a identificar qué ID tiene cada robot físico.

---

## Proceso Completo de Flasheo

### 1. Flashear cada robot con su ID

```bash
cd firmware/msr

# Conectar Robot 1
pio run -e robot1 -t upload --upload-port /dev/ttyUSB0

# Marcar físicamente el robot (etiqueta: "Robot 1")

# Conectar Robot 2
pio run -e robot2 -t upload --upload-port /dev/ttyUSB0

# Marcar físicamente el robot (etiqueta: "Robot 2")

# Y así sucesivamente...
```

### 2. Flashear el transmisor

```bash
cd firmware/transmisor
pio run -t upload --upload-port /dev/ttyUSB0
```

### 3. Flashear el tablero (opcional)

```bash
cd firmware/tablero
pio run -t upload --upload-port /dev/ttyUSB0
```

---

## Cómo Funciona Internamente

### En `config.h`:

```cpp
#ifndef ROBOT_ID
#define ROBOT_ID 1  // Default si no se especifica
#endif

// Nota: "00001" está reservado para el Tablero
#if ROBOT_ID == 1
#define RF_ADDRESS "00002"
#elif ROBOT_ID == 2
#define RF_ADDRESS "00003"
#elif ROBOT_ID == 3
#define RF_ADDRESS "00004"
#elif ROBOT_ID == 4
#define RF_ADDRESS "00005"
#endif
```

### En `platformio.ini`:

```ini
[env:robot1]
build_flags = -DROBOT_ID=1

[env:robot2]
build_flags = -DROBOT_ID=2
```

El flag `-DROBOT_ID=X` define la macro `ROBOT_ID` durante la compilación.

---

## Transmisor

El transmisor **cambia automáticamente la dirección RF** según el robot que estés controlando:

```cpp
// Cuando envías "R1F" (Robot 1 Forward)
radio.openWritingPipe(addressRobot1);  // "00002"
radio.write(&comando, sizeof(comando));

// Cuando envías "R2F" (Robot 2 Forward)
radio.openWritingPipe(addressRobot2);  // "00003"
radio.write(&comando, sizeof(comando));

// Cuando envías "T2" (Tablero: Gol equipo 1)
radio.openWritingPipe(addressTablero);  // "00001"
radio.write(&data, sizeof(data));
```

---

## Control Individual

Ahora puedes controlar cada robot de forma independiente:

```bash
python test_manual.py

>>> 1f    # Solo Robot 1 se mueve adelante
>>> 2r    # Solo Robot 2 gira derecha
>>> 3p    # Solo Robot 3 patea
>>> 4d    # Solo Robot 4 activa rodillo
```

---

## Tips

### Marcar los robots físicamente

Usa etiquetas o marcadores para identificar cada robot:
- 🟢 Robot 1
- 🔵 Robot 2
- 🟡 Robot 3
- 🔴 Robot 4

### Crear alias para flasheo rápido

En tu `.bashrc` o `.zshrc`:

```bash
alias flash-r1='cd ~/git/RoboCupSoftware/firmware/msr && pio run -e robot1 -t upload --upload-port /dev/ttyUSB0'
alias flash-r2='cd ~/git/RoboCupSoftware/firmware/msr && pio run -e robot2 -t upload --upload-port /dev/ttyUSB0'
alias flash-r3='cd ~/git/RoboCupSoftware/firmware/msr && pio run -e robot3 -t upload --upload-port /dev/ttyUSB0'
alias flash-r4='cd ~/git/RoboCupSoftware/firmware/msr && pio run -e robot4 -t upload --upload-port /dev/ttyUSB0'
```

Luego simplemente:
```bash
flash-r1  # Flashea robot 1
```

---

## Troubleshooting

### No sé qué ID tiene mi robot

1. Enciéndelo
2. Cuenta los beeps del buzzer
3. Ese es su ID

### Necesito cambiar el ID de un robot

Simplemente vuelve a flashearlo con otro environment:

```bash
# Si era robot2 y quieres hacerlo robot3
pio run -e robot3 -t upload --upload-port /dev/ttyUSB0
```

### Todos los robots responden al mismo comando

Verifica que:
1. Flasheaste cada robot con un environment diferente (`robot1`, `robot2`, etc.)
2. El transmisor está actualizado con el código que cambia direcciones
3. Los módulos NRF24L01 están bien conectados
