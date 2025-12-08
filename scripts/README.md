# Scripts de Calibración

Scripts para calibrar el sistema de percepción de Robot Soccer.

## 📋 Scripts Disponibles

### 🎨 calibrate_ball_color.py
Calibra el rango de color HSV para detectar la pelota.

```bash
python scripts/calibrate_ball_color.py
```

**Características:**
- Trackbars (deslizantes) para ajustar H, S, V en tiempo real
- 3 ventanas: Original, Mask (binaria), Result (pelota aislada)
- Indicador de calidad (muy restrictivo/bueno/muy permisivo)
- Guarda directamente en `config.py`

**Controles:**
- Ajustar trackbars → Ver resultado en vivo
- **R** → Resetear a valores por defecto
- **ENTER** → Guardar en config.py
- **ESC** → Salir sin guardar

---

### 🔲 calibrate_perspective.py
Calibra la transformación de perspectiva (birds-eye view).

```bash
python scripts/calibrate_perspective.py
```

**Características:**
- Click en 4 esquinas de la cancha
- Preview en tiempo real de la transformación
- Recorta y corrige distorsión de perspectiva

**Controles:**
- **Click izquierdo** → Seleccionar esquinas (4 puntos)
- **R** → Resetear puntos
- **ENTER** → Guardar en config.py
- **ESC** → Salir sin guardar

---

### 🏷️ generate_aruco_markers.py
Genera marcadores ArUco listos para imprimir.

```bash
python scripts/generate_aruco_markers.py --ids 0 1 2 3
```

**Características:**
- Genera marcadores 5x5 y 6x6
- Hojas de impresión 2x2 (4 marcadores)
- Tamaño: 5.5 cm x 5.5 cm
- Salida: `markers_output/`

**Opciones:**
- `--ids 0 1 2 3` → IDs a generar
- `--output-dir DIR` → Directorio de salida

---

### ⚙️ calibrate_robot_motors.py
Calibra los motores individuales de cada robot para compensar diferencias físicas.

```bash
python scripts/calibrate_robot_motors.py --robot-id 0
```

**Características:**
- Ajuste en tiempo real de factores de calibración
- Prueba de movimiento con visualización de cámara
- Dos niveles de ajuste: grueso (0.05) y fino (0.01)
- Guarda en `src/robot_soccer/config/robot_calibration.json`

**Controles:**
- **Q/A** → max_speed_left (±0.05)
- **W/S** → max_speed_right (±0.05)
- **E/D** → bias_correction (±0.01)
- **Mayúsculas** → Ajuste fino (x0.2)
- **Flechas** → Mover robot (probar calibración)
- **ESPACIO** → Detener robot
- **R** → Reset a neutro
- **ENTER** → Guardar
- **ESC** → Salir sin guardar

**Parámetros:**
- `max_speed_left/right`: Factor de velocidad máxima (0.0-1.0)
- `bias_correction`: Corrección cuando va recto (-0.3 a 0.3)

**Opciones:**
- `--robot-id N` → ID del robot a calibrar
- `--camera-id N` → ID de cámara (default: 2)
- `--serial-port PORT` → Puerto Arduino (default: /dev/ttyUSB0)

---

## 🔄 Flujo de Calibración Recomendado

### Setup Inicial (una vez)

1. **Generar marcadores** (si no los tienes):
   ```bash
   python scripts/generate_aruco_markers.py --ids 0 1 2 3
   ```

2. **Calibrar perspectiva** (recorte de cancha):
   ```bash
   python scripts/calibrate_perspective.py
   ```

3. **Calibrar color de pelota** (rango HSV):
   ```bash
   python scripts/calibrate_ball_color.py
   ```

### Calibración de Robots (por cada robot)

4. **Calibrar motores de cada robot**:
   ```bash
   # Robot 0
   python scripts/calibrate_robot_motors.py --robot-id 0

   # Robot 1
   python scripts/calibrate_robot_motors.py --robot-id 1

   # ... repetir para cada robot
   ```

5. **Probar detección**:
   ```bash
   python examples/test_perception.py
   ```

---

## 💡 Notas

- Scripts de percepción guardan en `src/robot_soccer/config.py`
- Calibración de motores guarda en `src/robot_soccer/config/robot_calibration.json`
- Requieren DroidCam corriendo: `cd algoritmos_basicos/aruco_tag && ./start_droidcam.sh`
- Usa cámara ID 2 por defecto (DroidCam)
- Para robots reales, necesitas Arduino conectado en `/dev/ttyUSB0`

## 🔧 Workflow: Software → Firmware

**Estrategia recomendada para calibración de motores:**

1. **Fase de desarrollo** (software):
   - Usa `calibrate_robot_motors.py` para encontrar valores óptimos
   - Ajusta rápidamente sin reprogramar Arduino
   - Los valores se guardan en JSON

2. **Fase de producción** (firmware):
   - Una vez encontrados los valores óptimos en JSON
   - Transfiere esos valores al firmware de cada robot
   - Resetea valores de JSON a neutros (1.0, 1.0, 0.0)
   - La calibración ahora vive EN el robot (independiente del PC)
