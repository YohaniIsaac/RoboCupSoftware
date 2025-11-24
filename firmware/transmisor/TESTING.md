# Guía de Testing - Transmisor RF

## Pruebas Manuales desde Consola

### Método 1: Script Python Interactivo (Recomendado)

El script `test_manual.py` te permite controlar robots de forma fácil:

```bash
cd firmware/transmisor
python test_manual.py
```

**Comandos disponibles:**

```
Robots (formato: [ID][acción]):
  1f  - Robot 1 adelante
  1b  - Robot 1 atrás
  1l  - Robot 1 izquierda
  1r  - Robot 1 derecha
  1p  - Robot 1 patear
  1d  - Robot 1 activar rodillo
  1s  - Robot 1 detener rodillo

  2f, 3f, 4f  - Otros robots (2, 3, 4)

Tablero:
  t1  - Pausar/reanudar
  t2  - Gol equipo 1
  t3  - Gol equipo 2
  t4  - Reset goles
  t5  - Reset tiempo

Otros:
  help  - Ver ayuda
  exit  - Salir
```

**Ejemplo de sesión:**
```
>>> 1f        # Robot 1 adelante
✓ OK: Robot 1 <- F

>>> 1p        # Robot 1 patear
✓ OK: Robot 1 <- P

>>> t2        # Gol equipo 1
✓ OK: Tablero <- G2

>>> 2r        # Robot 2 derecha
✓ OK: Robot 2 <- R
```

---

### Método 2: Monitor Serial de PlatformIO

Envía comandos directamente por el monitor serial:

```bash
cd firmware/transmisor
pio device monitor --port /dev/ttyUSB0
```

Luego escribe los comandos en formato del protocolo:
```
R1F    # Robot 1 adelante
R1P    # Robot 1 patear
R2L    # Robot 2 izquierda
T2     # Gol equipo 1
```

---

### Método 3: Echo directo (Linux/Mac)

Envía comandos directamente al puerto serial:

```bash
# Robot 1 adelante
echo "R1F" > /dev/ttyUSB0

# Robot 1 patear
echo "R1P" > /dev/ttyUSB0

# Gol equipo 1
echo "T2" > /dev/ttyUSB0
```

**Nota:** Puede que necesites configurar el puerto primero:
```bash
stty -F /dev/ttyUSB0 115200
```

---

### Método 4: Screen/Minicom

```bash
# Con screen
screen /dev/ttyUSB0 115200

# Con minicom
minicom -D /dev/ttyUSB0 -b 115200
```

Luego escribe los comandos directamente.

---

## Workflow de Pruebas

### Paso 1: Preparación

1. **Flashear el transmisor:**
```bash
cd firmware/transmisor
pio run -t upload --upload-port /dev/ttyUSB0
```

2. **Flashear al menos un robot:**
```bash
cd firmware/msr
pio run -t upload --upload-port /dev/ttyUSB0
```

3. **Encender el robot** (asegúrate de que tenga batería)

### Paso 2: Verificar Conexión

```bash
# Terminal 1: Monitor del transmisor
cd firmware/transmisor
pio device monitor
```

Deberías ver:
```
Transmisor RF inicializado
Protocolo:
  Robots: R[1-4][F/B/L/R/P/D/S/Q]
  Tablero: T[1-5]
Esperando comandos...
```

### Paso 3: Enviar Comandos de Prueba

```bash
# Terminal 2: Script de test
cd firmware/transmisor
python test_manual.py
```

Prueba en este orden:

1. **Test básico de movimiento:**
```
>>> 1f
```
El robot debería moverse hacia adelante por 100ms y detenerse.

2. **Test de giro:**
```
>>> 1l    # Izquierda
>>> 1r    # Derecha
```

3. **Test de pateo:**
```
>>> 1p
```
El solenoide debería activarse brevemente.

4. **Test de rodillo:**
```
>>> 1d    # Activar
>>> 1s    # Detener
```

### Paso 4: Calibración de Movimientos

Si los movimientos no son correctos, ajusta las velocidades en `firmware/msr/include/config.h`:

```cpp
// Velocidades de motores (0-255 para PWM)
#define VELOCIDAD_ADELANTE_IZQ 47
#define VELOCIDAD_ADELANTE_DER 45
#define VELOCIDAD_ATRAS_IZQ 47
#define VELOCIDAD_ATRAS_DER 45
#define VELOCIDAD_GIRO_LENTO 30
#define VELOCIDAD_GIRO_RAPIDO 40
```

Después de modificar, vuelve a compilar y flashear:
```bash
cd firmware/msr
pio run -t upload
```

---

## Troubleshooting

### El transmisor no responde

**Verificar puerto:**
```bash
ls /dev/tty{USB,ACM}*
```

**Verificar permisos:**
```bash
sudo chmod 666 /dev/ttyUSB0
# O agregar a grupo:
sudo usermod -aG uucp $USER  # Arch/Manjaro
sudo usermod -aG dialout $USER  # Debian/Ubuntu
```

**Verificar conexión:**
```bash
pio device list
```

### El robot no se mueve

1. **Verificar alimentación:** ¿Tiene batería?
2. **Verificar NRF24L01:** ¿Está bien conectado?
3. **Verificar LED del Arduino:** ¿Está encendido?
4. **Probar con comando simple:**
```bash
echo "R1F" > /dev/ttyUSB0
```

5. **Ver respuesta del transmisor:**
```bash
pio device monitor
```

### El robot se mueve pero de forma extraña

**Ajustar velocidades en config.h:**
- Si gira cuando debería ir recto → ajustar `VELOCIDAD_ADELANTE_IZQ/DER`
- Si va muy lento → aumentar velocidades
- Si va muy rápido → disminuir velocidades

**Verificar conexión de motores:**
- Motor1 (izquierdo): A0, A1
- Motor2 (derecho): A2, A3

### El solenoide no funciona

**Verificar en config.h:**
```cpp
#define TIEMPO_PATEO_MS 50  // Aumentar si es muy corto
```

**Verificar pin:**
```cpp
#define SOLENOIDE_PIN A4
```

---

## Scripts de Test Automatizados

### Test de Secuencia Completa

Crea un archivo `test_sequence.sh`:

```bash
#!/bin/bash

PORT=/dev/ttyUSB0

echo "=== Test de Robot 1 ==="
echo "Adelante..."
echo "R1F" > $PORT
sleep 0.5

echo "Atrás..."
echo "R1B" > $PORT
sleep 0.5

echo "Izquierda..."
echo "R1L" > $PORT
sleep 0.5

echo "Derecha..."
echo "R1R" > $PORT
sleep 0.5

echo "Patear..."
echo "R1P" > $PORT
sleep 0.5

echo "=== Test completado ==="
```

Ejecutar:
```bash
chmod +x test_sequence.sh
./test_sequence.sh
```

---

## Monitoreo Avanzado

### Ver todos los comandos enviados

```bash
# Terminal 1: Monitor
pio device monitor

# Terminal 2: Test manual
python test_manual.py

# Verás en tiempo real todos los comandos y respuestas
```

### Logging de comandos

```bash
pio device monitor | tee log_comandos.txt
```

---

## Checklist de Verificación

- [ ] Transmisor flasheado correctamente
- [ ] Transmisor conectado por USB a PC
- [ ] Robot flasheado con firmware MSR
- [ ] Robot encendido con batería
- [ ] NRF24L01 bien conectado en ambos (transmisor y robot)
- [ ] Permisos de puerto serial configurados
- [ ] Monitor serial muestra "Transmisor RF inicializado"
- [ ] Comando de prueba `R1F` responde con "OK"
- [ ] Robot se mueve al enviar comando

---

## Próximos Pasos

Una vez que los robots respondan correctamente a los comandos manuales:

1. ✅ Calibrar velocidades de motores
2. ✅ Probar todos los comandos (F, B, L, R, P, D, S)
3. ✅ Verificar alcance RF (hasta qué distancia funcionan)
4. ✅ Probar con múltiples robots simultáneamente
5. ✅ Integrar con el código Python de IA/visión

**Ejemplo de integración:**
```python
# En tu código de IA
from robot_soccer.controllers.rf_transmitter import RFTransmitter, RobotCommand

transmitter = RFTransmitter('/dev/ttyUSB0')
transmitter.connect()

# Cuando la IA decide que el robot debe moverse
transmitter.send_robot_command(1, RobotCommand.FORWARD)
```
