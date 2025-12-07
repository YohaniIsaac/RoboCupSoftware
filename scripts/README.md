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

## 🔄 Flujo de Calibración Recomendado

1. **Generar marcadores** (si no los tienes):
   ```bash
   python scripts/generate_aruco_markers.py
   ```

2. **Calibrar perspectiva** (recorte de cancha):
   ```bash
   python scripts/calibrate_perspective.py
   ```

3. **Calibrar color de pelota** (rango HSV):
   ```bash
   python scripts/calibrate_ball_color.py
   ```

4. **Probar detección**:
   ```bash
   python examples/test_perception.py
   ```

---

## 💡 Notas

- Todos los scripts guardan configuración en `src/robot_soccer/config.py`
- Requieren DroidCam corriendo: `cd algoritmos_basicos/aruco_tag && ./start_droidcam.sh`
- Usa cámara ID 2 por defecto (DroidCam)
