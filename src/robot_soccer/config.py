"""Archivo de configuración centralizado para el proyecto de fútbol de robots.

Contiene constantes y variables compartidas entre diferentes módulos.
"""

from pathlib import Path


BALL_CAPTURE_DISTANCE = 25

# ==========================================
# Configuraciones de rutas
# ==========================================

# Obtener la ruta absoluta al directorio donde está config.py (raíz del proyecto)
ROOT_DIR = Path(__file__).parent.parent  # sube 2 niveles

# Si arucoMarkers está un nivel más arriba de la raíz del proyecto
ARUCO_MARKERS_DIR = ROOT_DIR.parent / "arucoMarkers"

# ==========================================
# Dimensiones y configuración del campo
# ==========================================
ANCHO_CAMPO = 1500  # Ancho del área de juego en píxeles
ALTO_CAMPO = 900    # Alto del área de juego en píxeles
MARGEN_CANCHA = 50  # Margen alrededor del campo

# Dimensiones completas incluyendo márgenes
ANCHO_TOTAL = ANCHO_CAMPO + MARGEN_CANCHA * 2
ALTO_TOTAL = ALTO_CAMPO + MARGEN_CANCHA * 2

ZONA_POR_EQUIPO = 0.4
# Zonas del campo (en porcentaje del ancho)
ZONA_IZQUIERDA = ZONA_POR_EQUIPO
ZONA_DERECHA = 1.0 - ZONA_POR_EQUIPO

# Dimensiones de la portería
LARGO_ARCO = 200

# ==========================================
# Configuración de los robots
# ==========================================
# Dimensiones físicas de los robots
ROBOT_RADIO = 30      # Radio del robot
ROBOT_ANCHO = 52      # Ancho del robot
ROBOT_ALTO = 70       # Alto del robot

# Parámetros de velocidad y movimiento
MAX_VELOCIDAD = 5.0   # Velocidad máxima del robot
FACTOR_ACELERACION = 0.2  # Factor de aceleración
FACTOR_DESACELERACION = 0.5  # Factor de desaceleración

# ==========================================
# Configuración de la pelota
# ==========================================
PELOTA_RADIO = 30
PELOTA_MASA = 2.7

# ==========================================
# Configuración RRT* Smart
# ==========================================
RRT_STEP_LEN = 50          # Longitud del paso
RRT_GOAL_SAMPLE_RATE = 0.5  # Tasa de muestreo del objetivo
RRT_SEARCH_RADIUS = 5       # Radio de búsqueda
RRT_ITER_MAX = 10000        # Número máximo de iteraciones

# ==========================================
# Configuración de la simulación
# ==========================================
FPS = 40  # Frames por segundo

# ==========================================
# Configuración de ROI (Region of Interest) para cámara
# ==========================================
# Transformación de perspectiva para la cancha
# Define los 4 puntos (esquinas) de la cancha en la imagen de la cámara
# Orden: [top-left, top-right, bottom-right, bottom-left]
# Formato: (x, y) en píxeles

CAMERA_PERSPECTIVE_ENABLED = True  # Habilitar/deshabilitar transformación de perspectiva

# Puntos de origen (esquinas de la cancha en la imagen de la cámara)
# Ajusta estos valores usando el script scripts/calibrate_perspective.py
CAMERA_PERSPECTIVE_SRC_POINTS = [
    (8, 34),      # Top-left (esquina superior izquierda)
    (616, 9),     # Top-right (esquina superior derecha)
    (630, 365),    # Bottom-right (esquina inferior derecha)
    (18, 393)       # Bottom-left (esquina inferior izquierda)
]

# Dimensiones de la imagen de salida (rectángulo destino)
CAMERA_PERSPECTIVE_WIDTH = 640   # Ancho de la imagen transformada
CAMERA_PERSPECTIVE_HEIGHT = 480  # Alto de la imagen transformada

# ==========================================
# Configuración de detección de robots (ArUco)
# ==========================================
# Dimensiones del rectángulo que representa al robot en la imagen
# Estos valores definen el tamaño del bounding box alrededor del marcador ArUco
ROBOT_DETECTION_HALF_WIDTH = 35   # Mitad del ancho del robot en píxeles (simulacion 52 total = 104) largo robot
ROBOT_DETECTION_HALF_HEIGHT = 22  # Mitad del alto del robot en píxeles (simulacion 70 total = 140) ancho robot

# Longitud de la línea que indica la orientación del robot
ROBOT_ORIENTATION_LINE_LENGTH = 25  # Longitud en píxeles de la línea verde de orientación

# ==========================================
# Colores (en formato BGR para OpenCV y RGB para Pygame)
# ==========================================
COLOR_NEGRO = (0, 0, 0)
COLOR_BLANCO = (255, 255, 255)
COLOR_VERDE = (0, 255, 0)

# Colores para OpenCV (BGR)
COLOR_ROJO_CV = (0, 0, 255)
COLOR_AZUL_CV = (255, 0, 0)
COLOR_CIAN_CV = (255, 255, 0)
COLOR_MAGENTA_CV = (255, 0, 255)
COLOR_AMARILLO_CV = (0, 255, 255)
COLOR_NARANJO_CV = (0, 98, 244)

# Colores para Pygame (RGB)
COLOR_ROJO_PG = (255, 0, 0)
COLOR_AZUL_PG = (0, 0, 255)
COLOR_CIAN_PG = (0, 255, 255)
COLOR_MAGENTA_PG = (255, 0, 255)
COLOR_AMARILLO_PG = (255, 255, 0)
COLOR_NARANJO_PG = (244, 98, 0)
COLOR_CESPED_PG = (40, 128, 40)

# ==========================================
# Configuración de lógica difusa
# ==========================================
# Parámetros para cambio de roles
MIN_TIME_BETWEEN_CHANGES = 5.0  # Tiempo mínimo en segundos entre cambios de rol
UMBRAL_CAMBIO_ROLES = 10.0      # Umbral de distancia para cambiar roles
MIN_TIME_IN_ZONE = 3.0          # Tiempo mínimo en segundos para cambiar de zona

# ==========================================
# Identificadores de equipo
# ==========================================
EQUIPO_ROJO = 'red'
EQUIPO_AZUL = 'blue'
LADO_IZQUIERDO = 'LEFT'
LADO_DERECHO = 'RIGHT'

# ==========================================
# Identificadores de jugador
# ==========================================

ROL_ATACANTE = 1
ROL_DEFENSIVO = 2

# ==========================================
# Estados de la máquina de estados
# ==========================================
# Estados de la pelota
ESTADO_PELOTA_LIBRE = "LIBRE"
ESTADO_PELOTA_POSESION = "POSESION"

# Proximidad de la pelota
PROXIMIDAD_ALIADA = "ALIADA"
PROXIMIDAD_RIVAL = "RIVAL"

# Zonas del campo
ZONA_DEFENSIVA = "DEFENSIVA"
ZONA_NEUTRAL = "NEUTRAL"
ZONA_OFENSIVA = "OFENSIVA"
ZONA_FUERA = "FUERA"

# Estados de los robots
ESTADO_CAPTURAR = "INTENTA CAPTURAR LA PELOTA"
ESTADO_DEFENSIVO = "SE POSICIONA DE FORMA DEFENSIVA"
ESTADO_PREPARAR_PASE = "SE PREPARA PARA UN PASE"
ESTADO_INTERCEPTAR = "INTERCEPTA QUE EL RIVAL TOME LA PELOTA"
ESTADO_BLOQUEAR = "BLOQUEA POSIBLES TIROS"
ESTADO_PRESIONAR = "PRESIONA A LOS RIVALES"
ESTADO_AVANZAR = "LLEVA LA PELOTA A LA MITAD RIVAL"
ESTADO_ADELANTAR = "SE ADELANTA A LA MITAD RIVAL"
ESTADO_LANZAR = "SE PREPARA PARA LANZAR AL ARCO"
ESTADO_APOYAR = "BUSCA UNA POSICIÓN PARA APOYAR"
ESTADO_RETROCEDER = "RETROCEDE A DEFENDER ARCO ALIADO"

# ==========================================
# Rangos de colores HSV para detección
# ==========================================
# Rango para color naranja (pelota)
RANGO_COLOR_NARANJO = ((15, 114, 141), (30, 255, 255))  # Rango HSV para pelota naranja

# Rangos para colores de equipos
RANGO_COLOR_ROJO = ((0, 100, 20), (8, 255, 255), (175, 100, 20), (179, 255, 255))
RANGO_COLOR_AZUL = ((110, 150, 150), (130, 255, 255), None, None)
RANGO_COLOR_MAGENTA = ((145, 150, 150), (165, 255, 255), None, None)
RANGO_COLOR_CIAN = ((85, 150, 150), (95, 255, 255), None, None)

# Parámetros de detección de pelota (morfología y HoughCircles)
BALL_DETECTION_KERNEL_SIZE = 3  # Tamaño del kernel morfológico
BALL_DETECTION_MORPH_ITERATIONS = 3  # Iteraciones de apertura/cierre
BALL_DETECTION_HOUGH_PARAM1 = 40  # Umbral Canny
BALL_DETECTION_HOUGH_PARAM2 = 6  # Umbral acumulador
BALL_DETECTION_MIN_RADIUS = 2  # Radio mínimo (px)
BALL_DETECTION_MAX_RADIUS = 9  # Radio máximo (px)

# =============================================================================
# Parámetros de Control de Movimiento del Robot
# =============================================================================

# --- REFERENCIAS DE MOTOR DC ---
# Los motores DC tienen un "dead zone" donde PWM muy bajo no genera suficiente
# torque para vencer la fricción estática.
MOTOR_DEAD_ZONE_PWM = 30         # PWM < 30: Motor probablemente NO se mueve (dead zone)
MOTOR_MIN_MOVEMENT_PWM = 50      # PWM ≥ 50: Movimiento confiable del motor
MOTOR_MAX_PWM = 255              # PWM máximo absoluto del motor

# --- Velocidades de Rotación (en PWM: 0-255) ---
# IMPORTANTE: Con corrección angular y calibración asimétrica (L=1.0, R=0.79),
# las velocidades efectivas pueden reducirse ~20-40%. Por eso los mínimos deben
# ser suficientemente altos para superar el dead zone del motor (~30 PWM).
#
# AJUSTADAS para latencia del sistema (~65ms: 40ms captura + 15ms proceso + 10ms RF)
# A 25 FPS real (no 60 FPS), el robot se mueve significativamente entre frames
ROBOT_MIN_ROTATION_SPEED = 18  # Velocidad mínima cuando LEJOS del objetivo (PWM)
ROBOT_MAX_ROTATION_SPEED = 23  # Velocidad máxima de rotación (PWM)
ROBOT_ROTATION_ARRIVAL_ANGLE_DEG = 25.0  # Ángulo donde empieza rampa de desaceleración (grados)
ROBOT_ROTATION_NEAR_MIN = 18  # Velocidad mínima EN LA RAMPA (PWM)
                                  # Debe ser <= ROBOT_MIN_ROTATION_SPEED y >= MOTOR_MIN_MOVEMENT_PWM

# --- Velocidades de Movimiento Lineal (en PWM: 0-255) ---
# Con corrección angular (MAX_ANGULAR_CORRECTION_PWM) y calibración (R×0.79),
# el motor más lento puede perder ~35% de velocidad. Por eso MIN debe ser ≥90
# para asegurar que incluso después de corrección y calibración se supere el dead zone.
#
# Ejemplo: Speed=90 PWM → Con corrección: L=80, R=100
#          → Con calibración: L=80, R=79 → AMBOS > 50 PWM ✓
ROBOT_MIN_LINEAR_SPEED = 10  # Velocidad mínima cuando LEJOS (PWM)
ROBOT_MAX_LINEAR_SPEED = 21  # Velocidad máxima de movimiento (PWM)
ROBOT_LINEAR_ARRIVAL_DISTANCE = 51  # Distancia donde empieza rampa de desaceleración (píxeles)
ROBOT_LINEAR_NEAR_MIN = 65  # Velocidad mínima EN LA RAMPA (PWM)
                                  # Debe ser <= ROBOT_MIN_LINEAR_SPEED y >= MOTOR_MIN_MOVEMENT_PWM

# --- Umbral de Inicio de Movimiento Lineal ---
# Ángulo máximo donde el robot puede comenzar a moverse linealmente mientras corrige
# FASE 1: Si error angular > umbral → Gira en lugar (solo rotación, sin avance)
# FASE 2: Si error angular ≤ umbral → Se mueve linealmente mientras corrige ángulo
ROBOT_LINEAR_START_ANGLE_THRESHOLD_DEG = 30.0  # Grados (30° = conservador, 45° = agresivo)

# --- Corrección Angular (en PWM) ---
# Límite máximo de diferencia PWM entre motores para corrección angular durante movimiento lineal
# Antes: 0.04 (4% de velocidad normalizada) → Ahora: 10 PWM directo
MAX_ANGULAR_CORRECTION_PWM = 10  # Máximo ±10 PWM de diferencia L/R para corrección angular

# --- Thresholds de Precisión ---
ROBOT_POSITION_THRESHOLD = 16  # Distancia para considerar waypoint alcanzado (píxeles)
ROBOT_ANGLE_THRESHOLD_DEG = 7  # Error angular aceptable (grados)
