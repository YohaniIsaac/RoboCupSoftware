# Scripts de Calibración

Scripts para calibrar el sistema de percepción de Robot Soccer.

## 📋 Scripts Disponibles

### 🎨 calibrate_ball_color.py
Calibra la detección de pelota en dos fases.

**FASE 1 - Color HSV:**
```bash
python scripts/calibrate_ball_color.py
```
Ajusta H/S/V hasta que solo la pelota sea blanca en 'Mask'. Presiona **N** para siguiente fase.

**FASE 2 - Parámetros de Detección:**
```bash
python scripts/calibrate_ball_color.py --skip-phase1
```
Ajusta morfología y HoughCircles hasta detectar exactamente 1 círculo. Presiona **ENTER** para guardar.

**Controles:**
- **R** → Reset | **N** → Siguiente fase | **B** → Fase anterior
- **ENTER** → Guardar | **ESC** → Salir sin guardar

---

### 🔲 calibrate_perspective.py
Calibra la transformación de perspectiva (birds-eye view).

```bash
python scripts/calibrate_perspective.py
```
Click en 4 esquinas de la cancha. **ENTER** para guardar.

---

### 🏷️ generate_aruco_markers.py
Genera marcadores ArUco para imprimir.

```bash
python scripts/generate_aruco_markers.py --ids 0 1 2 3
```
Salida en `markers_output/` (5.5cm x 5.5cm, hojas 2x2).

---

### ⚙️ Calibración de Motores (2 Pasos)

#### Paso 1: calibrate_robot_pwm_range.py
Determina el rango PWM útil [min, max] de cada robot.

```bash
python scripts/calibrate_robot_pwm_range.py --robot-id 0
```

**Objetivo:** Encontrar PWM_min (mínimo para moverse) y PWM_max (máximo donde la cámara detecta).

**Controles:**
- **ESPACIO/BACKSPACE** → Mover adelante/atrás
- **↑/↓** o **W/S** → Ajustar PWM de prueba
- **N/M** → Ajustar PWM_min | **,/.** → Ajustar PWM_max
- **G** → Guardar rango en JSON

#### Paso 2: calibrate_robot_motors_multipoint.py
Calibración bidireccional de 10 puntos (5 adelante + 5 atrás).

```bash
python scripts/calibrate_robot_motors_multipoint.py --robot-id 0
```

**Características:**
- Puntos personalizados basados en el rango PWM de cada robot
- Calibración bidireccional (adelante/atrás independientes)
- Dead-zone individual por motor

**Controles:**
- **PgUp/PgDn** → Navegar entre puntos
- **Q/A**, **W/S** → Ajustar velocidad máxima izq/der (grueso)
- **E/D** → Ajustar corrección de bias (grueso)
- **1-6** → Ajustes finos
- **Z/X**, **C/V** → Ajustar dead-zone
- **Flechas** → Probar movimiento | **ENTER** → Guardar calibración completa
- Guarda en `src/robot_soccer/config/robot_calibration_multipoint.json`

#### Paso 3: calibrate_pid_controllers.py
Calibra parámetros PID del controlador de movimiento del robot.

```bash
python scripts/calibrate_pid_controllers.py --robot-id 0
```

**Arquitectura de 3 procesos optimizada:**
- **Percepción ultra-rápida:** 28-40 FPS (detección ArUco + envío de frames)
- **Control PID puro:** ~100 Hz (solo PID + comandos RF, sin UI)
- **Visualización:** 28-40 FPS (frames reales + panel de información)

**Objetivo:** Ajustar 6 parámetros PID para que el robot alcance waypoints con precisión:
- **PID Posición (KP/KI/KD):** Controla qué tan bien sigue trayectorias lineales
- **PID Angular (KP/KI/KD):** Controla rotación y corrección de orientación

**Controles:**
- **Click en frame** → Establecer waypoint objetivo
- **ESPACIO** → Toggle movimiento (activar/pausar)
- **1/q** → Ajustar PID Posición KP (±0.001)
- **2/w** → Ajustar PID Posición KI (±0.0001)
- **3/e** → Ajustar PID Posición KD (±0.01)
- **4/r** → Ajustar PID Ángulo KP (±0.01)
- **5/t** → Ajustar PID Ángulo KI (±0.001)
- **6/y** → Ajustar PID Ángulo KD (±0.01)
- **g** → Guardar parámetros a `config.py`
- **z** → Reset PID a valores por defecto
- **ESC** → Salir

**Metodología de ajuste:**
1. Establecer waypoint cercano con click
2. Presionar ESPACIO para que el robot se mueva
3. Observar comportamiento:
   - **Oscilaciones:** Reducir KP
   - **Error permanente:** Aumentar KI
   - **Overshoot:** Aumentar KD
4. Repetir con waypoints a diferentes distancias y ángulos
5. Presionar **g** para guardar cuando esté satisfecho

---

## 🔄 Flujo de Calibración Recomendado

**Setup Inicial:**
1. Generar marcadores ArUco (si no los tienes)
2. Calibrar perspectiva (recorte de cancha)
3. Calibrar pelota:
   - Ejecutar FASE 1 (Color HSV) → presionar **N**
   - Ajustar FASE 2 (Detección) → presionar **ENTER** para guardar

**Calibración de Motores y Control (para cada robot `--robot-id 0, 1, 2, 3`):**
4. **Paso 1:** `calibrate_robot_pwm_range.py`
   - Determinar rango PWM útil [min, max]
   - Presionar **G** para guardar → actualiza `robot_calibration_multipoint.json`

5. **Paso 2:** `calibrate_robot_motors_multipoint.py`
   - Calibrar 10 puntos (5 adelante + 5 atrás)
   - Ajustar max_left, max_right, bias, dead-zone
   - Presionar **ENTER** para guardar → actualiza `robot_calibration_multipoint.json`

6. **Paso 3:** `calibrate_pid_controllers.py`
   - Calibrar parámetros PID (6 parámetros: posición y angular)
   - Click para establecer waypoints, ajustar con teclas 1-6
   - Presionar **g** para guardar → actualiza `config.py`

7. Probar con `python examples/test_perception.py`

---

## 💡 Notas

- Requiere DroidCam: `cd algoritmos_basicos/aruco_tag && ./start_droidcam.sh`
- **Percepción** guarda en `src/robot_soccer/config.py` (color pelota, perspectiva)
- **Motores** guardan en `src/robot_soccer/config/robot_calibration_multipoint.json` (PWM range, calibración 10 puntos)
- **PID** guarda en `src/robot_soccer/config.py` (6 parámetros PID_POSITION_* y PID_ANGLE_*)
- Valores de motores pueden transferirse al firmware para producción
