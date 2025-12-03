# Scripts de DroidCam y Control de Celular

Scripts para gestionar DroidCam y controlar el celular desde la PC.

## 📋 Scripts Disponibles

### 🚀 start_droidcam.sh
Inicia DroidCam y configura la cámara automáticamente.
- Despierta el celular
- Abre DroidCam
- **Abre scrcpy para que cierres anuncios manualmente** (presiona 'q' cuando termines)
- Configura port forwarding
- Inicia droidcam-cli

```bash
./start_droidcam.sh
```

**Flujo interactivo:**
1. Se abre DroidCam en el celular (aquí aparecen los anuncios)
2. Se abre ventana scrcpy en tu PC automáticamente
3. Tú cierras los anuncios con el mouse en la ventana de scrcpy
4. Presionas **ENTER en la consola** → cierra scrcpy y continúa automáticamente

### 🛑 stop_droidcam.sh
Detiene DroidCam y ahorra batería del celular.
- Detiene droidcam-cli
- Cierra DroidCam en el celular
- Apaga la pantalla del celular

```bash
./stop_droidcam.sh
```

### 🧪 setup_and_test_droidcam.sh
Prueba completa de DroidCam con visualización.
- Configura todo desde cero
- Muestra video de prueba
- Útil para verificar que todo funciona

```bash
./setup_and_test_droidcam.sh
```

### 🎮 control_phone.sh
Control remoto del celular desde PC.

```bash
# Ver pantalla y controlar con mouse
./control_phone.sh mirror

# Cerrar anuncios manualmente
./control_phone.sh close-ad

# Hacer tap en coordenadas
./control_phone.sh tap X Y

# Tomar screenshot
./control_phone.sh screen

# Ver app actual
./control_phone.sh current

# Abrir DroidCam
./control_phone.sh droidcam
```

### 🏷️ generar_marcadores_test.py
Genera marcadores ArUco de prueba.

```bash
python generar_marcadores_test.py
```

---

## 🔧 Flujo de Trabajo Típico

### Iniciar sesión:
```bash
cd AlgortimosBasicos/ArucoTag
./start_droidcam.sh
```

### Usar la cámara en tu código:
```python
import cv2
cap = cv2.VideoCapture(2)  # DroidCam en /dev/video2
```

### Terminar sesión (ahorrar batería):
```bash
./stop_droidcam.sh
```

---

## 📝 Notas Importantes

- **start_droidcam.sh abre scrcpy automáticamente** para que cierres anuncios con el mouse
- **Requiere "Depuración USB (Configuración de seguridad)"** activada en el celular
- **Requiere scrcpy instalado**: `sudo pacman -S scrcpy`
- **control_phone.sh mirror** usa scrcpy para ver y controlar el celular
- **stop_droidcam.sh** apaga la pantalla del celular para maximizar duración de batería

---

## 🔧 Calibración del Sistema (usar scripts del proyecto principal)

Para calibrar transformación de perspectiva, dimensiones de robots, etc:

### 1. Calibrar perspectiva:
```bash
cd ../../  # Volver a raíz del proyecto
python scripts/calibrate_perspective.py
```

### 2. Probar módulo de cámara:
```bash
python -m robot_soccer --video-only --camera
```

### 3. Probar detección de robots:
```bash
python -m robot_soccer --perception --camera
```

### 4. Ajustar dimensiones de robots en config.py:
```python
ROBOT_DETECTION_HALF_WIDTH = 35
ROBOT_DETECTION_HALF_HEIGHT = 22
ROBOT_ORIENTATION_LINE_LENGTH = 25
```

---

## 🏷️ Generar Marcadores ArUco

Usa el script del proyecto principal:
```bash
cd ../../  # Volver a raíz
python scripts/generate_aruco_markers.py --ids 0 1 2 3
```

Esto genera marcadores listos para imprimir a 5.5 cm.
