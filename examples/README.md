# Examples - Ejemplos Simples

Scripts simples para probar módulos individuales del sistema de percepción.

## 📋 Scripts Disponibles

### 🤖 test_robot_detection_rate.py
Mide la tasa de detección de robots (marcadores ArUco).

```bash
python examples/test_robot_detection_rate.py --frames 100
```

**Muestra:**
- Cuántas veces detecta cada robot (por ID)
- Porcentaje de detección
- Evaluación de calidad (Excelente/Buena/Regular/Mala)

---

### ⚽ test_ball_detection_rate.py
Mide la tasa de detección de pelota (por color naranja).

```bash
python examples/test_ball_detection_rate.py --frames 100
```

**Muestra:**
- Cuántas veces detecta la pelota exitosamente
- Porcentaje de detección
- Evaluación de calidad

---

### 🎯 test_perception.py
Test integrado completo: robots + pelota simultáneamente.

```bash
python examples/test_perception.py
```

**Muestra:**
- Detección de jugadores y pelota en tiempo real
- Posición, orientación, IDs
- Contador de goles
- Presiona 'q' para salir

---

## 🚀 Uso Rápido

1. **Iniciar DroidCam:**
   ```bash
   cd algoritmos_basicos/aruco_tag
   ./start_droidcam.sh
   ```

2. **Ejecutar ejemplo:**
   ```bash
   cd ../..  # Volver a raíz
   python examples/test_robot_detection_rate.py
   ```

---

## 💡 Notas

- Todos los scripts usan cámara ID 2 (DroidCam) por defecto
- Usa `--camera-id N` para cambiar la cámara
- Usa `--frames N` para cambiar cantidad de frames a analizar
- Presiona 'q' en la ventana para salir antes de tiempo
