"""Archivo de configuración centralizado para el proyecto de fútbol de robots.

Contiene constantes y variables compartidas entre diferentes módulos.
"""

from dataclasses import dataclass
from pathlib import Path

import cv2


@dataclass
class FieldGeometry:
    """Geometría calibrada del campo de juego.

    Encapsula dimensiones del campo, posiciones de arcos y métodos
    de conversión para que la lógica del juego sea independiente
    de la resolución (simulación vs cámara).
    """

    width: int
    height: int
    goal_left_x: int
    goal_left_top_y: int
    goal_left_bottom_y: int
    goal_right_x: int
    goal_right_top_y: int
    goal_right_bottom_y: int
    margin: int = 30

    @property
    def goal_left_center(self):
        """Centro del arco izquierdo (x, y)."""
        mid_y = (self.goal_left_top_y + self.goal_left_bottom_y) // 2
        return (self.goal_left_x, mid_y)

    @property
    def goal_right_center(self):
        """Centro del arco derecho (x, y)."""
        mid_y = (self.goal_right_top_y + self.goal_right_bottom_y) // 2
        return (self.goal_right_x, mid_y)

    @property
    def goal_left_size(self):
        """Tamaño vertical del arco izquierdo en px."""
        return self.goal_left_bottom_y - self.goal_left_top_y

    @property
    def goal_right_size(self):
        """Tamaño vertical del arco derecho en px."""
        return self.goal_right_bottom_y - self.goal_right_top_y

    @property
    def center(self):
        """Centro del campo (x, y)."""
        return (self.width // 2, self.height // 2)

    def clamp(self, pos):
        """Restringe una posición dentro de los límites del campo con margen."""
        x = max(self.margin, min(self.width - self.margin, int(pos[0])))
        y = max(self.margin, min(self.height - self.margin, int(pos[1])))
        return (x, y)

    def ratio_to_px(self, ratio):
        """Convierte un ratio (proporción del ancho) a píxeles."""
        return int(ratio * self.width)

    def zone_x(self, percent):
        """Convierte un porcentaje del ancho a coordenada x."""
        return int(percent * self.width)

    def zone_y(self, percent):
        """Convierte un porcentaje del alto a coordenada y."""
        return int(percent * self.height)

BALL_CAPTURE_DISTANCE = 25

# ==========================================
# Configuraciones de rutas
# ==========================================

# Obtener la ruta absoluta al directorio donde está config.py (raíz del proyecto)
ROOT_DIR = Path(__file__).parent.parent  # sube 2 niveles

# Si arucoMarkers está un nivel más arriba de la raíz del proyecto
ARUCO_MARKERS_DIR = ROOT_DIR.parent / "arucoMarkers"

# ==========================================
# Configuración de marcadores ArUco
# ==========================================
# Para cambiar el diccionario, solo modifica ARUCO_DICTIONARY_CAMERA.
# Opciones comunes:
#   cv2.aruco.DICT_4X4_50         - 4x4, 50 IDs, detección rápida
#   cv2.aruco.DICT_5X5_50         - 5x5, 50 IDs, más robusto
#   cv2.aruco.DICT_APRILTAG_16H5  - AprilTag 4x4, Hamming ≥5, excelente anti-confusión
#   cv2.aruco.DICT_APRILTAG_25H9  - AprilTag 5x5, Hamming ≥9, máxima robustez
ARUCO_DICTIONARY_CAMERA = cv2.aruco.DICT_APRILTAG_16H5  # Diccionario para cámara real
ARUCO_DICTIONARY_SIM = cv2.aruco.DICT_7X7_1000           # Diccionario para simulación

# IDs de marcadores asignados a cada robot
ARUCO_ROBOT_IDS = [0, 1, 2, 3]          # Todos los robots
ARUCO_TEAM_RED_IDS = [0, 1]             # Equipo rojo: robot 0 y 1
ARUCO_TEAM_BLUE_IDS = [2, 3]            # Equipo azul: robot 2 y 3

# --- Parámetros de detección ArUco ---
# Estos se usan en TODOS los detectores (pipeline principal, calibración, tests).
# Optimizados para detección en movimiento con cámara de baja resolución.
ARUCO_ADAPTIVE_THRESH_WIN_SIZE_MIN = 5
ARUCO_ADAPTIVE_THRESH_WIN_SIZE_MAX = 51
ARUCO_ADAPTIVE_THRESH_WIN_SIZE_STEP = 10
ARUCO_MIN_MARKER_PERIMETER_RATE = 0.01
ARUCO_MAX_MARKER_PERIMETER_RATE = 6.0
ARUCO_CORNER_REFINEMENT_METHOD = cv2.aruco.CORNER_REFINE_NONE  # NONE = más rápido
ARUCO_ERROR_CORRECTION_RATE = 0.8        # 80% tolerancia a errores (crítico para motion blur)
ARUCO_PERSPECTIVE_REMOVE_PX_PER_CELL = 6
ARUCO_PERSPECTIVE_REMOVE_IGNORED_MARGIN = 0.10
ARUCO_MIN_DISTANCE_TO_BORDER = 1
ARUCO_MARKER_BORDER_BITS = 1
ARUCO_MIN_OTSU_STD_DEV = 2.0
ARUCO_POLYGONAL_APPROX_ACCURACY_RATE = 0.08

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
# Instancias de geometría del campo
# ==========================================
FIELD_SIM = FieldGeometry(
    width=ANCHO_CAMPO,
    height=ALTO_CAMPO,
    goal_left_x=0,
    goal_left_top_y=(ALTO_CAMPO - LARGO_ARCO) // 2,
    goal_left_bottom_y=(ALTO_CAMPO + LARGO_ARCO) // 2,
    goal_right_x=ANCHO_CAMPO,
    goal_right_top_y=(ALTO_CAMPO - LARGO_ARCO) // 2,
    goal_right_bottom_y=(ALTO_CAMPO + LARGO_ARCO) // 2,
    margin=30,
)

# FIELD_CAM se define más abajo, después de CAMERA_PERSPECTIVE_WIDTH/HEIGHT

# ==========================================
# Ratios de umbrales de comportamiento
# ==========================================
# Proporción del ancho del campo — independiente de resolución
BT_SHOT_DISTANCE_RATIO = 0.27
BT_PASS_MIN_RATIO = 0.067
BT_PASS_MAX_RATIO = 0.40
BT_BLOCK_RATIO = 0.033
BT_CAPTURE_RANGE_RATIO = 0.04
BT_APPROACH_RATIO = 0.017
BT_CAPTURE_ACTIVATE_RATIO = 0.033
BT_CAPTURE_CONFIRM_RATIO = 0.023
BT_INTERCEPT_RATIO = 0.033
BT_SUPPORT_DISTANCE_RATIO = 0.133
BT_DEFENDER_WAIT_RATIO = 0.20
BT_DRIBBLE_SPACING_RATIO = 0.08
BT_DRIBBLE_GOAL_RATIO = 0.133
BT_DEFENSIVE_ARRIVAL_RATIO = 0.02
BT_ATTACKER_PRIORITY_MARGIN_RATIO = 0.133

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

CAMERA_PERSPECTIVE_ENABLED = True  # Habilitar/deshabilitar transformacion de perspectiva

# Puntos de origen (esquinas de la cancha en la imagen de la cámara)
# Ajusta estos valores usando el script scripts/calibrate_perspective.py
CAMERA_PERSPECTIVE_SRC_POINTS = [
    (32, 36),      # Top-left (esquina superior izquierda)
    (631, 45),     # Top-right (esquina superior derecha)
    (634, 404),    # Bottom-right (esquina inferior derecha)
    (18, 394)       # Bottom-left (esquina inferior izquierda)
]

# Dimensiones de la imagen de salida (rectángulo destino)
CAMERA_PERSPECTIVE_WIDTH = 640   # Ancho de la imagen transformada
CAMERA_PERSPECTIVE_HEIGHT = 480  # Alto de la imagen transformada

# Geometría del campo para cámara (valores default, actualizados por calibración de arcos)
FIELD_CAM = FieldGeometry(
    width=CAMERA_PERSPECTIVE_WIDTH,
    height=CAMERA_PERSPECTIVE_HEIGHT,
    goal_left_x=27,
    goal_left_top_y=196,
    goal_left_bottom_y=295,
    goal_right_x=616,
    goal_right_top_y=193,
    goal_right_bottom_y=294,
    margin=15,
)

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
RANGO_COLOR_NARANJO = ((8, 116, 159), (30, 200, 255))  # Rango HSV para pelota naranja

# Rangos para colores de equipos
RANGO_COLOR_ROJO = ((0, 100, 20), (8, 255, 255), (175, 100, 20), (179, 255, 255))
RANGO_COLOR_AZUL = ((110, 150, 150), (130, 255, 255), None, None)
RANGO_COLOR_MAGENTA = ((145, 150, 150), (165, 255, 255), None, None)
RANGO_COLOR_CIAN = ((85, 150, 150), (95, 255, 255), None, None)

# Parámetros de detección de pelota (morfología y HoughCircles)
BALL_DETECTION_KERNEL_SIZE = 3  # Tamaño del kernel morfológico
BALL_DETECTION_MORPH_ITERATIONS = 4  # Iteraciones de apertura/cierre
BALL_DETECTION_HOUGH_PARAM1 = 7  # Umbral Canny
BALL_DETECTION_HOUGH_PARAM2 = 7  # Umbral acumulador
BALL_DETECTION_MIN_RADIUS = 5  # Radio mínimo (px)
BALL_DETECTION_MAX_RADIUS = 10  # Radio máximo (px)

# =============================================================================
# Parámetros de Control de Movimiento del Robot
# =============================================================================

# --- REFERENCIAS DE MOTOR DC ---
# Los motores DC tienen un "dead zone" donde PWM muy bajo no genera suficiente
# torque para vencer la fricción estática.
MOTOR_DEAD_ZONE_PWM = 30         # PWM < 30: Motor probablemente NO se mueve (dead zone)
MOTOR_MIN_MOVEMENT_PWM = 50      # PWM ≥ 50: Movimiento confiable del motor
MOTOR_MAX_PWM = 127              # PWM máximo absoluto del motor (límite firmware int8_t)

# --- Velocidades de Rotación (en PWM: 0-127) ---
# IMPORTANTE: Todos los valores deben superar MOTOR_MIN_MOVEMENT_PWM (50) para
# garantizar movimiento real del motor. Con calibración asimétrica (L=1.0, R=0.79),
# las velocidades efectivas pueden reducirse ~20-40%.
#
# Perfil: LEJOS → velocidad constante MIN, RAMPA → desacelera de MIN a NEAR_MIN
ROBOT_MIN_ROTATION_SPEED = 50  # Velocidad cuando LEJOS del objetivo (PWM, >= MOTOR_MIN_MOVEMENT_PWM)
ROBOT_MAX_ROTATION_SPEED = 65  # Velocidad máxima de rotación (PWM)
ROBOT_ROTATION_ARRIVAL_ANGLE_DEG = 25.0  # Ángulo donde empieza rampa de desaceleración (grados)
ROBOT_ROTATION_NEAR_MIN = 45  # Velocidad mínima EN LA RAMPA (PWM, debe ser <= MIN y >= dead zone)

# --- Velocidades de Movimiento Lineal (en PWM: 0-127) ---
# Todos los valores deben superar MOTOR_MIN_MOVEMENT_PWM (50) para garantizar
# movimiento real. NEAR_MIN debe ser <= MIN para que la rampa desacelere.
#
# Perfil: LEJOS → velocidad constante MIN, RAMPA → desacelera de MIN a NEAR_MIN
ROBOT_MIN_LINEAR_SPEED = 55  # Velocidad cuando LEJOS (PWM, >= MOTOR_MIN_MOVEMENT_PWM)
ROBOT_MAX_LINEAR_SPEED = 80  # Velocidad máxima de movimiento (PWM)
ROBOT_LINEAR_ARRIVAL_DISTANCE = 51  # Distancia donde empieza rampa de desaceleración (píxeles)
ROBOT_LINEAR_NEAR_MIN = 50  # Velocidad mínima EN LA RAMPA (PWM, debe ser <= MIN y >= dead zone)

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
ROBOT_POSITION_THRESHOLD = 5  # Distancia para considerar waypoint alcanzado (píxeles)
ROBOT_ANGLE_THRESHOLD_DEG = 1  # Error angular aceptable (grados)

# --- Captura de pelota con dribbler ---
# Distancia (ArUco center → ball center) a la que se activa el dribbler y
# se emite el target de overshoot. Debe superar ROBOT_POSITION_THRESHOLD.
# Calibrar con scripts/calibrate_behavior_thresholds.py (teclas U/J)
CAPTURE_ACTIVATE_DISTANCE_PX = 47  # px — activar dribbler + iniciar creep

# Píxeles MÁS ALLÁ del centro de la pelota donde apunta el creep.
# Tras move_to_ball, el robot queda a ~dist_real px de la pelota (por inercia).
# Para que el robot se mueva durante capture se requiere:
#   dist_real + OVERSHOOT > ROBOT_POSITION_THRESHOLD
#   → OVERSHOOT > THRESHOLD - dist_real  (ej: 32 - 23 = 9px mínimo)
# El robot es detenido físicamente por la pelota; CAPTURE_CONFIRM_DISTANCE_PX
# confirma que el dribbler la tocó.
# Calibrar con scripts/calibrate_behavior_thresholds.py (teclas I/K)
CAPTURE_OVERSHOOT_PX = 27  # px — empuje suave para asegurar contacto dribbler-pelota

# Distancia a la que se confirma la captura (_has_ball = True).
# El robot físicamente captura cuando el dribbler toca la pelota, lo que ocurre
# cuando dist(ArUco_center, ball_center) ≈ robot_radius - ball_radius ≈ 23-27px.
# Este valor debe ser mayor que la distancia real de parada (~23px observado).
# Calibrar con scripts/calibrate_behavior_thresholds.py (teclas O/L)
CAPTURE_CONFIRM_DISTANCE_PX = 17  # px — confirmar pelota en dribbler

# Velocidad PWM para el acercamiento lento hacia la pelota (sin dribbler).
# Se envía directamente a los motores sin PID.
# Debe superar MOTOR_DEAD_ZONE_PWM (30) para garantizar movimiento.
# A mayor valor: más rápido pero más riesgo de empujar la pelota.
# Calibrar con scripts/calibrate_behavior_thresholds.py (teclas N/M)
CAPTURE_CREEP_SPEED_PWM = 18  # PWM — velocidad máxima lineal durante PID de avance al contacto
                              # Se aplica como max_linear_pwm_override: capea v pero permite
                              # corrección angular completa. Calibrar con teclas N/M.

# Tiempo de espera (segundos) tras confirmar contacto con la pelota.
# Permite que la pelota se acomode contra el robot antes de disparar.
# Si la pelota escapa durante este tiempo, el ciclo se reinicia.
CONTACT_SETTLE_TIME_S = 0.35  # s — espera de asentamiento antes del disparo

# Factores de escape para detectar que la pelota se alejó demasiado.
# Durante avance: dist > BEHIND_BALL_APPROACH_PX * ADVANCE_ESCAPE_FACTOR → abortar
# Durante asentamiento: dist > CAPTURE_CONFIRM_DISTANCE_PX * SETTLE_ESCAPE_FACTOR → abortar
ADVANCE_ESCAPE_FACTOR = 1.5  # factor — margen de escape durante avance al contacto
SETTLE_ESCAPE_FACTOR = 2.0   # factor — margen de escape durante asentamiento

# Factor multiplicador de PWM cuando el robot tiene posesión de la pelota.
# El dribbler genera fricción que frena la rotación/movimiento. Este factor
# compensa esa resistencia amplificando los PWM enviados a los motores.
# Valor 1.0 = sin cambio. Valor 1.3 = 30% más potencia. Solo aplica post-captura.
# Calibrar con scripts/calibrate_behavior_thresholds.py (teclas T/Y)
DRIBBLE_PWM_FACTOR = 1.0  # factor — compensación de fricción del dribbler

# Potencia del motor dribbler en PWM (0-255) durante la fase de CAPTURA.
# El firmware usa SoftPWM: 255 = máxima fuerza de agarre.
# Se activa cuando el BT inicia capture_ball (fase 2: creep forward).
# Calibrar con scripts/calibrate_behavior_thresholds.py (teclas 1/2)
DRIBBLER_CAPTURE_POWER = 0  # PWM para atrapar pelota

# Potencia del motor dribbler en PWM (0-255) mientras MANTIENE la pelota.
# Potencia reducida para disminuir consumo de corriente y proteger
# el regulador de tensión durante rotación con pelota.
# Se usa en keepalive y durante orient_to_goal / shoot_to_goal.
# Calibrar con scripts/calibrate_behavior_thresholds.py (teclas 3/4)
DRIBBLER_HOLD_POWER = 0  # PWM reducido para sostener pelota

# Dribbler intermitente: duración del pulso ON (ms) mientras mantiene pelota.
# El motor se activa durante ON ms, luego se apaga durante OFF ms, y repite.
# Reduce calor en el motor al trabarse, protegiendo el regulador de tensión.
# Si DRIBBLER_PULSE_OFF_MS = 0, funciona de forma continua (sin intermitencia).
# Calibrar con scripts/calibrate_behavior_thresholds.py (teclas 5/6 y 7/8)
DRIBBLER_PULSE_ON_MS = 90  # ms — duración del pulso encendido
DRIBBLER_PULSE_OFF_MS = 0  # ms — duración del pulso apagado (0=continuo)

# --- Posicionamiento detrás de la pelota (ataque sin dribbler) ---
# El atacante se posiciona en la línea pelota-arco ANTES de hacer contacto,
# eliminando la necesidad de rotar con la pelota o activar el dribbler.
# BEHIND_BALL_APPROACH_PX debe ser > CAPTURE_ACTIVATE_DISTANCE_PX para no
# rozar la pelota durante el posicionamiento.
BEHIND_BALL_APPROACH_PX = 52  # px — distancia robot-pelota al posicionarse detrás
BEHIND_BALL_LATERAL_OFFSET_PX = 75 # px — desvío lateral para rodear la pelota
BEHIND_BALL_ALIGN_TOLERANCE_DEG = 15.0  # ° — tolerancia angular aceptable al posicionar
PUSH_BURST_PWM = 70                # PWM — pulso anti-stiction al iniciar avance al contacto

# --- Detección de robot atascado (stuck / anti-stall) ---
# Si el robot no avanza más de STUCK_MOVEMENT_THRESHOLD_PX píxeles dentro de una
# ventana de STUCK_DETECTION_WINDOW_S segundos, se suma STUCK_BOOST_INCREMENT PWM
# adicional por cada ventana consecutiva (máx STUCK_BOOST_MAX). Al moverse, decae.
STUCK_MOVEMENT_THRESHOLD_PX = 3   # px — desplazamiento mínimo para "no estar atascado"
STUCK_DETECTION_WINDOW_S = 1.2    # s  — ventana de tiempo (~30 frames @ 25 FPS)
STUCK_BOOST_INCREMENT = 1         # PWM — boost adicional por ventana sin movimiento
STUCK_BOOST_MAX = 8               # PWM — boost máximo acumulado (hard cap)
STUCK_BOOST_DECAY = 5             # PWM — reducción por ventana con movimiento

# =============================================================================
# Parámetros de Control PID
# =============================================================================

# --- PID de Posición (Control Lineal) ---
# Controla qué tan bien el robot sigue una trayectoria hacia un waypoint
# - Kp (Proporcional): Respuesta inmediata al error de posición
# - Ki (Integral): Corrige error acumulado (evita offset permanente)
# - Kd (Derivativo): Reduce overshoot y oscilaciones
#
# PID basado en TIEMPO (dt): independiente de la frecuencia del control loop.
# Las unidades son:  Kp=por pixel, Ki=por pixel·segundo, Kd=por pixel/segundo
# Escala: pid_output × max_smooth_speed (80 PWM) = velocidad final
PID_POSITION_KP = 5.505199999999991
PID_POSITION_KI = 0.1499999999999998
PID_POSITION_KD = 0.0836

# --- PID Angular (Control de Orientación) ---
# Controla qué tan bien el robot mantiene/corrige su orientación
# - kp_angle: CRÍTICO - Valor alto causa oscilaciones durante movimiento lineal
# - ki_angle: Corrige desviaciones acumuladas (bias del robot)
# - kd_angle: Damping para reducir oscilaciones (muy importante)
#
# PID basado en TIEMPO (dt): independiente de la frecuencia del control loop.
# Las unidades son:  Kp=por rad, Ki=por rad·segundo, Kd=por rad/segundo
# Escala: pid_output × 255 = velocidad de rotación
# NOTA: kp_angle se comparte con corrección angular durante movimiento lineal
PID_ANGLE_KP = 0.3522000000008896
PID_ANGLE_KI = 0.02
PID_ANGLE_KD = 0.05
