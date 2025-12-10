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

### ⚙️ calibrate_robot_motors.py
Calibra motores de cada robot para compensar diferencias físicas.

```bash
python scripts/calibrate_robot_motors.py --robot-id 0
```

**Controles:**
- **Q/A** → max_speed_left | **W/S** → max_speed_right | **E/D** → bias_correction
- **Flechas** → Probar movimiento | **ENTER** → Guardar
- Guarda en `src/robot_soccer/config/robot_calibration.json`

---

## 🔄 Flujo de Calibración Recomendado

**Setup Inicial:**
1. Generar marcadores ArUco (si no los tienes)
2. Calibrar perspectiva (recorte de cancha)
3. Calibrar pelota:
   - Ejecutar FASE 1 (Color HSV) → presionar **N**
   - Ajustar FASE 2 (Detección) → presionar **ENTER** para guardar
4. Calibrar motores de cada robot (`--robot-id 0, 1, 2, 3`)
5. Probar con `python examples/test_perception.py`

---

## 💡 Notas

- Requiere DroidCam: `cd algoritmos_basicos/aruco_tag && ./start_droidcam.sh`
- Percepción guarda en `src/robot_soccer/config.py`
- Motores guardan en `src/robot_soccer/config/robot_calibration.json`
- Valores de motores pueden transferirse al firmware para producción
