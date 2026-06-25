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
BT_DEFENDER_STAND_DIST_RATIO = 0.20  # distancia del sweeper desde el arco propio (fracción del ancho del campo)
DEFENDER_WAIT_MAX_S = 5.0  # s — timeout máximo de espera del defensor

# --- Cesión del defensor al atacante aliado (prioridad del atacante) ---
# Cuando el atacante aliado se acerca demasiado al defensor, el defensor le cede el
# paso retrocediendo en línea recta (sin girar), evitando el choque entre aliados.
# El atacante tiene prioridad de movimiento. Histéresis START/CLEAR para no oscilar.
# Distancias centro-a-centro (radio físico del robot ~30 px → contacto ~60 px).
DEFENDER_YIELD_START_PX   = 70  # px — atacante más cerca que esto → empezar a retroceder
DEFENDER_YIELD_CLEAR_PX   = 95  # px — atacante más lejos que esto → dejar de ceder
DEFENDER_YIELD_REVERSE_PWM = 30  # PWM — velocidad de retroceso recto (ambas ruedas, negativo)
BT_DRIBBLE_SPACING_RATIO = 0.08
BT_DRIBBLE_GOAL_RATIO = 0.133
BT_DEFENSIVE_ARRIVAL_RATIO = 0.05   # 32px — umbral de llegada al punto defensivo
BT_ATTACKER_PRIORITY_MARGIN_RATIO = 0.133

# Arbitraje dinámico de roles
BT_ROLE_SWITCH_HYSTERESIS  = 1.5   # El rival debe ser ×1.5 más cercano para robar el rol
BT_ROLE_COMMITMENT_RATIO   = 0.23  # ratio → ~147px: si atacante está a <N px no se cambia rol
BT_ROLE_SWITCH_COOLDOWN_S  = 3.0   # s — tiempo mínimo entre cambios de rol

# Cesión inter-equipo en 2v2: el atacante deja de ir a la pelota y se posiciona
# como interceptor cuando el rival tiene ventaja clara para llegar primero.
# Score por jugador = dist_to_ball + K_ANGLE_PX_PER_DEG * |heading_error_deg|.
BT_INTERCEPT_ENTER_RATIO    = 1.20  # rival_score * 1.20 < my_score → entrar a ceder
BT_INTERCEPT_EXIT_RATIO     = 1.10  # my_score * 1.10 < rival_score → salir de ceder
BT_INTERCEPT_K_ANGLE_PX_DEG = 1.5   # peso del error angular en el score (px/grado)
BT_INTERCEPT_DEPTH_RATIO    = 0.40  # fracción del camino mi_arco → pelota

# ==========================================
# Reglas de partido
# ==========================================
BALL_OUT_MARGIN_PX = 30   # pelota fuera si ≤30px de cualquier borde (excluye zona de arcos)

# Posiciones de reset tras gol (px en FIELD_CAM 640×480)
# Estilo SSL simplificado: robots en su propia mitad, lejos del centro.
# RRT* con evasión de obstáculos se usa automáticamente al navegar a estas posiciones.
RESET_POS = {
    0: (200, 150),   # R0 rojo — zona media-izquierda, arriba
    1: (120, 330),   # R1 rojo — zona defensiva izquierda, abajo
    2: (440, 150),   # R2 azul — zona media-derecha, arriba
    3: (520, 330),   # R3 azul — zona defensiva derecha, abajo
}

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
PATH_PLANNING_OBSTACLE_CLEARANCE = 60    # Margen de seguridad (px) añadido a cada obstáculo
                                         # (subido de 45→60: rutas más separadas de los robots)
PATH_PLANNING_ROBOT_OBSTACLE_RADIUS = 45 # Radio (px) con que se modela cada robot como obstáculo
PATH_PLANNING_BALL_OBSTACLE_RADIUS  = 10 # Radio (px) con que se modela la pelota como obstáculo
                                         # Zona exclusión = radio(10) + clearance + margin(6).
                                         # Con clearance completo(60) = 76px > BEHIND_BALL(60) → el
                                         # target detrás de la pelota sería inalcanzable; por eso el
                                         # posicionamiento usa PATH_PLANNING_BALL_POSITIONING_CLEARANCE.

# --- Inflación dependiente del contexto (zona de disputa de la pelota) ---
# Cerca de la pelota convergen varios robots y el contacto es OBLIGATORIO; el
# clearance global infla cada robot a ~3x su tamaño real (45+60=105px) y vuelve
# imposible maniobrar ahi. Cuando el goal cae dentro de PATH_PLANNING_CONTEST_RADIUS_PX
# de la pelota se usa un clearance reducido, permitiendo aproximarse a disputar
# sin que el planner proyecte el goal hacia atras. El contacto final lo cierra
# advance_to_contact en modo directo (sin planner).
PATH_PLANNING_CONTEST_RADIUS_PX = 120 # px — goal a <=N de la pelota => intencion de disputa.
                                      # Cubre el staging behind-ball: hasta BEHIND_BALL_APPROACH_PX(60)
                                      # + BALL_INTERCEPT_MAX_PX(60) = 120px de la pelota cuando la
                                      # predicción de intercepción adelanta el punto. Con 90 el punto
                                      # predicho quedaba con clearance completo (60) y un compañero
                                      # estático (defensor) lo inflaba hasta bloquearlo → deadlock.
PATH_PLANNING_CONTEST_CLEARANCE = 20  # px — clearance reducido en la zona de disputa

# Clearance dedicado al POSICIONAMIENTO behind-ball (move_robot_to con avoid_ball=True).
# El contest clearance (20) deja la zona de exclusión de la pelota en 10+20+6=36px,
# menor que el radio del cuerpo del robot (~30px) → lo roza/empuja al rodearla. Con 37
# la zona = 10+37+6 = 53px: el centro del robot se mantiene a ≥53px de la pelota (apenas el
# aire justo para no rozar). BEHIND_BALL_APPROACH_PX(64) > 53 sigue alcanzable con holgura
# sana (~11px, evita el parpadeo proyectado/freeze; C5 es backstop). Solo aplica al
# posicionamiento; el contacto final lo cierra advance_to_contact en modo directo.
PATH_PLANNING_BALL_POSITIONING_CLEARANCE = 37  # px — clearance al rodear la pelota posicionándose
RRT_WAYPOINT_ARRIVAL_PX  = 20    # px — umbral de llegada a waypoints intermedios
RRT_REPLAN_POSITION_PX   = 80    # px — trigger replan si robot se aleja >N px del punto enviado
RRT_REPLAN_COOLDOWN_S    = 0.5   # s  — tiempo mínimo entre replans por posición/obstáculo
                                  # (bajado de 2.0→0.5: con cooldown 2s y robots a ~80px/s,
                                  #  los obstáculos se movían ~160px antes del replan → colisión)
RRT_OBSTACLE_MOVE_PX     = 25    # px — trigger replan si un obstáculo se mueve >N px
                                  # (bajado de 40→25: detectar movimiento antes de que el path quede inválido)

# --- Goals/starts degenerados (dentro de un obstáculo inflado) ---
# Si el goal solicitado cae dentro del radio inflado de un obstáculo (otro robot
# parado encima), RRT* jamás puede conectar y el control caía a PID directo sin
# evasión. El planner proyecta el goal al borde del obstáculo con este margen y
# el robot espera ahí, replanificando periódicamente hasta que se despeje.
RRT_GOAL_PROJECTION_MARGIN_PX = 6   # px — margen al proyectar goal/start fuera del obstáculo inflado
RRT_HOLD_REPLAN_PERIOD_S      = 2.0 # s  — replan periódico mientras el goal real siga bloqueado
RRT_WAITING_GOAL_CLEAR_TIMEOUT_S = 3.0 # s — si el goal sigue bloqueado tras N s de espera, el
                                       # bloqueador no es transitorio (p.ej. un compañero estático):
                                       # se abandona la espera y se libera la acción para que el BT
                                       # reevalúe, en vez de sostener (0,0) indefinidamente.

# --- Umbral de llegada efectivo según proximidad de obstáculos ---
# El umbral nominal de llegada (arrival_threshold por comando) se contrae
# linealmente cuando hay otro robot cerca del target, para evitar que dos
# robots con thresholds laxos se solapen en círculos de aceptación y
# colisionen. Ver _effective_threshold en robot_command_manager.py.
OBSTACLE_PROXIMITY_NEAR_PX  = 60   # px — obstáculo más cercano <= este → umbral mínimo
OBSTACLE_PROXIMITY_FAR_PX   = 120  # px — obstáculo más cercano >= este → umbral nominal
OBSTACLE_TIGHT_THRESHOLD_PX = 8    # px — umbral mínimo cuando hay obstáculo cercano

# Umbrales nominales de llegada por contexto. El contexto determina la
# precisión deseada al llegar; el umbral efectivo se contrae automáticamente
# si hay obstáculos cerca del target (ver _effective_threshold).
BEHIND_BALL_ARRIVAL_PX     = 15  # px — el BT chequea alineación angular después
DEFENSIVE_POS_ARRIVAL_PX   = 40  # px — punto defensivo es ilustrativo (circunferencia)
INTERCEPT_HOLD_ARRIVAL_PX  = 25  # px — interceptor en línea arco-pelota, tolerancia mediana

# Cooldown mínimo entre cambios de modo en should_yield_to_rival. Sin esto,
# my_score y opp_score oscilan dentro de la banda muerta de los ratios
# ENTER/EXIT y el robot alterna ENTRADA/SALIDA cada tick. Con cooldown,
# el modo se mantiene al menos N segundos antes de poder cambiar.
# Equivale al modo "Match Once" de la solución estándar SSL (ZJUNlict).
BT_INTERCEPT_MIN_HOLD_S = 0.8

# Anti-deadlock de cesión mutua: si la pelota lleva este tiempo sin que ningún
# jugador (de cualquier equipo) esté a < CAPTURE_ACTIVATE_DISTANCE_PX, el
# atacante deja de ceder y va por la pelota. Rompe el caso en que ambos
# atacantes se ceden mutuamente (cada uno cree que el rival llega primero) y
# nadie disputa la pelota.
BT_INTERCEPT_DEADLOCK_TIMEOUT_S = 2.0

RESET_MOVE_FACTOR = 0.80         # fracción de pwm_max para movimiento de reset (flooreado en pwm_min)
RESET_ANGLE       = {'red': 0.0, 'blue': 180.0}  # ángulo canónico de orientación (dirección de ataque)

# --- Saque (kickoff) estilo SSL tras un gol ---
# El equipo que RECIBIÓ el gol toma el saque (ventaja), igual que en fútbol:
# uno de sus robots se ubica detrás de la pelota (centro) listo para atacar; el
# equipo que anotó se repliega a su mitad (RESET_POS, ya fuera del círculo central).
KICKOFF_STAGING_OFFSET_PX  = 70  # px — distancia del sacador detrás de la pelota (centro del campo)
KICKOFF_BALL_CENTER_TOL_PX = 60  # px — la pelota se considera "repuesta al centro" si está a <= N del centro

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
    (55, 24),      # Top-left (esquina superior izquierda)
    (613, 27),     # Top-right (esquina superior derecha)
    (620, 351),    # Bottom-right (esquina inferior derecha)
    (56, 355)       # Bottom-left (esquina inferior izquierda)
]

# Dimensiones de la imagen de salida (rectángulo destino)
CAMERA_PERSPECTIVE_WIDTH = 640   # Ancho de la imagen transformada
CAMERA_PERSPECTIVE_HEIGHT = 480  # Alto de la imagen transformada

# Geometría del campo para cámara (valores default, actualizados por calibración de arcos)
FIELD_CAM = FieldGeometry(
    width=CAMERA_PERSPECTIVE_WIDTH,
    height=CAMERA_PERSPECTIVE_HEIGHT,
    goal_left_x=5,
    goal_left_top_y=187,
    goal_left_bottom_y=296,
    goal_right_x=632,
    goal_right_top_y=196,
    goal_right_bottom_y=296,
    # Margen >= radio del cuerpo del robot (~30px): clamp mantiene los targets
    # (behind_pos, defensivo, etc.) lo bastante adentro para que el cuerpo no
    # penetre la pared. Con 15px el centro quedaba a 15px del muro -> colisión.
    margin=35,
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
RANGO_COLOR_NARANJO = ((10, 142, 199), (41, 231, 255))  # Rango HSV para pelota naranja

# Rangos para colores de equipos
RANGO_COLOR_ROJO = ((0, 100, 20), (8, 255, 255), (175, 100, 20), (179, 255, 255))
RANGO_COLOR_AZUL = ((110, 150, 150), (130, 255, 255), None, None)
RANGO_COLOR_MAGENTA = ((145, 150, 150), (165, 255, 255), None, None)
RANGO_COLOR_CIAN = ((85, 150, 150), (95, 255, 255), None, None)

# Parámetros de detección de pelota (morfología y HoughCircles)
BALL_DETECTION_KERNEL_SIZE = 3  # Tamaño del kernel morfológico
BALL_DETECTION_MORPH_ITERATIONS = 3  # Iteraciones de apertura/cierre
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
ROBOT_MIN_ROTATION_SPEED = 50  # [SIN USO] rotate_to_angle pasa el rango del JSON; default inalcanzable de _apply_rotation_profile
ROBOT_MAX_ROTATION_SPEED = 65  # [SIN USO] idem ROBOT_MIN_ROTATION_SPEED (default inalcanzable)
ROBOT_ROTATION_ARRIVAL_ANGLE_DEG = 20.0  # Frontera lejos/cerca: inicio de rampa y conmutación de gain scheduling (grados)
ROBOT_ROTATION_NEAR_MIN = 45  # [SIN USO] self.rotation_near_min no tiene ningún lector

# --- Velocidades de Movimiento Lineal (en PWM: 0-127) ---
# Todos los valores deben superar MOTOR_MIN_MOVEMENT_PWM (50) para garantizar
# movimiento real. NEAR_MIN debe ser <= MIN para que la rampa desacelere.
#
# Perfil: LEJOS → velocidad constante MIN, RAMPA → desacelera de MIN a NEAR_MIN
ROBOT_MIN_LINEAR_SPEED = 55  # Velocidad cuando LEJOS (PWM, >= MOTOR_MIN_MOVEMENT_PWM)
ROBOT_MAX_LINEAR_SPEED = 80  # Velocidad máxima de movimiento (PWM)
ROBOT_LINEAR_ARRIVAL_DISTANCE = 51  # Distancia donde empieza rampa de desaceleración (píxeles)
ROBOT_LINEAR_NEAR_MIN = 50  # [SIN USO] self.linear_near_min no tiene ningún lector

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
CAPTURE_CONFIRM_DISTANCE_PX = 35  # px — confirmar pelota en dribbler

# --- Geometría del punto de golpeo (kick_point) ---
# El gate de "pelota lista para disparo" no se mide centro_robot ↔ centro_pelota
# (eso ignora la dirección del robot), sino kick_point ↔ centro_pelota, donde
# kick_point = robot_pos + KICK_POINT_OFFSET_PX * heading_unit. Esto exige que
# la pelota esté delante del robot, no a un costado.
KICK_POINT_OFFSET_PX    = 30  # px — distancia centro robot → punto donde el solenoide impacta
                              #      (calibrar físicamente: medir desde marker ArUco hasta
                              #      el punto de impacto del solenoide cuando el robot está
                              #      en posición ideal de kick).
KICK_POINT_TOLERANCE_PX = 10  # px — tolerancia LATERAL: desvío máx. de la pelota respecto al eje
                              #      de la nariz para confirmar contacto (no un radio simétrico; el
                              #      avance longitudinal lo gobierna CONTACT_REACH_MARGIN_PX). 10px ≈
                              #      ~19.5° de error de heading a la distancia de impacto: cubre un
                              #      staging de 15° con margen y queda bajo el aborto de 25°.
CONTACT_REACH_MARGIN_PX = 4   # px — margen longitudinal: el solenoide se considera que ALCANZÓ la
                              #      pelota cuando L (avance de la pelota sobre la nariz) <=
                              #      KICK_POINT_OFFSET_PX + este margen (=34). Evita el disparo "corto"
                              #      (p.ej. L=42, kick_point 12px antes de la pelota) y deja que el
                              #      overshoot empuje hasta que la pelota frene al robot (~L=30).
KICK_POINT_ANGLE_OFFSET_DEG = 0.0  # ° — offset angular entre el eje del marker ArUco y el eje
                                    #      real del solenoide (desalineación mecánica). Calibrar
                                    #      con el robot en múltiples orientaciones en el centro
                                    #      del campo: ajustar con </>  en calibrate_behavior_thresholds
                                    #      hasta que la cruz del kick_point coincida con la pelota.
                                    #      Positivo = sentido horario.

# Overshoot del creep de advance_to_contact: el target se emite MÁS ALLÁ del
# kick_point (hacia la pelota) por esta cantidad. Sin overshoot el target ES el
# kick_point exacto y STOP PREDICTIVO frena al robot ~ROBOT_POSITION_THRESHOLD
# antes, congelando el heading desalineado → kick_err > tolerancia → nunca hay
# contacto. Con overshoot el robot sigue avanzando y corrigiendo heading hasta
# que kick_err detecta el contacto. Análogo a CAPTURE_OVERSHOOT_PX del dribbler.
# Debe superar ROBOT_POSITION_THRESHOLD; no tan grande que empuje la pelota antes
# de detectar contacto (el creep es lento, pwm=CAPTURE_CREEP_SPEED_PWM).
CONTACT_APPROACH_OVERSHOOT_PX = 10  # px — overshoot del target del creep pasado el kick_point

# --- Corrección de paralaje por altura del marker ---
# El marker ArUco está elevado sobre el campo. Una cámara perspectiva desplaza
# objetos elevados hacia afuera del centro de la imagen respecto a su posición
# real en el suelo. La corrección se aplica en player_tracking.py al calcular
# el centro del marker.
CAMERA_HEIGHT_ABOVE_FIELD_CM = 128.0  # cm — altura física de la cámara (referencia; solvePnP ~124 cm)
MARKER_HEIGHT_ABOVE_FIELD_CM = 8.5    # cm — altura física del marcador (referencia geométrica)
# Corrección de paralaje desplegada (calibrada al tanteo con scripts/calibrate_parallax.py).
# Modelo radial: c' = c - (c - centro) * factor. El centro NO coincide con el de la
# imagen (320, 240): es el nadir efectivo de la cámara. El factor geométrico sería
# h/H ≈ 0.066; el valor que minimiza el error de posición en todo el campo es 0.032.
PARALLAX_FACTOR   = 0.032
PARALLAX_CENTER_X = 333
PARALLAX_CENTER_Y = 280
FIELD_PHYSICAL_WIDTH_CM      = 150.0  # cm — ancho real del área de juego (lado largo)
FIELD_PHYSICAL_HEIGHT_CM     = 88.0   # cm — alto real del área de juego (lado corto)
# Nota: estas dimensiones corrigen la anisotropía del warpPerspective en el cómputo
# del heading (atan2 en pixels da heading erróneo cuando px/cm_x != px/cm_y).

# --- Detección de kick exitoso vs. fallido ---
# Tras kick_immediately, se compara la posición de la pelota antes/después
# para distinguir kick mecánicamente exitoso (pelota voló) de fallo
# (pelota apenas se movió). El behavior tree usa esto para decidir si
# retroceder (kick desalineado) o re-avanzar (fallo mecánico).
KICK_FAIL_DETECT_WINDOW_S = 0.5  # s — ventana tras el kick para evaluar éxito
KICK_SUCCESS_MIN_PX       = 25   # px — desplazamiento mínimo de la bola que indica kick exitoso

# Velocidad PWM para el acercamiento lento hacia la pelota (sin dribbler).
# Se envía directamente a los motores sin PID.
# Debe superar MOTOR_DEAD_ZONE_PWM (30) para garantizar movimiento.
# A mayor valor: más rápido pero más riesgo de empujar la pelota.
# Calibrar con scripts/calibrate_behavior_thresholds.py (teclas N/M)
CAPTURE_CREEP_SPEED_PWM = 20  # PWM — velocidad máxima lineal durante PID de avance al contacto
                              # Se aplica como max_linear_pwm_override: capea v pero permite
                              # corrección angular completa. Calibrar con teclas N/M.
                              # Intencionalmente por debajo de pwm_min del JSON: en creep mode
                              # el piso es 0 (no pwm_min), lo que permite avance ultra-lento.

# Distancia perpendicular (px) bajo la cual se considera que otro robot bloquea
# el corredor del creep robot→overshoot. Si la proyección de un robot sobre el
# segmento cae estrictamente dentro de él Y la distancia perpendicular es menor
# a este valor, advancing_to_contact retorna FAILURE y el atacante reposiciona
# por flanco lateral via PosicionarDetrásPelota.
CORRIDOR_CLEARANCE_PX = 40

# --- Filtros temporales contra detección ruidosa de la pelota ---
# La detección puede caer al 7-30% en condiciones de luz adversas. Estos
# filtros evitan que cada frame con pelota stale o flicker dispare un cambio
# de comportamiento.

# Edad máxima (s) de la última detección de pelota para que se considere
# fresca. Usada en sub-fase 3 (pre-partido): si la pelota no fue vista
# recientemente, los robots se orientan al ángulo canónico del equipo en
# lugar de a una posición stale.
BALL_FRESHNESS_TIMEOUT_S = 0.5
# Tiempo (s) desde la última detección hasta parada completa del robot.
# A 20 fps: 0.5 s = 10 frames consecutivos sin detectar → stop.
ROBOT_DETECTION_LOST_RAMPDOWN_S = 0.5

# Frames consecutivos requeridos para activar/desactivar ball_out_active.
# Antes era hardcoded a 3 solo para activación; la desactivación era
# instantánea, lo que causaba flicker entre PELOTA FUERA y EN JUEGO. Ahora
# es simétrico: 10 frames (~500ms a 20fps) en ambas direcciones — antes 5
# permitía oscilación cuando la detección de pelota era irregular (~50%).
BALL_OUT_DEBOUNCE_FRAMES = 10

# Diferencia angular (grados) bajo la cual una orden de rotate_robot_to
# repetida se ignora si ya hay una rotación en curso. Antes hardcoded a 5°,
# pero el target angle saltaba 4-8° por frame por jitter de detección de
# pelota, generando cadenas de "Robot N: Ordenado giro a X grados".
ROTATE_RECOMMAND_MIN_DEG = 8.0

# Tiempo de espera (segundos) tras confirmar contacto con la pelota.
# Permite que la pelota se acomode contra el robot antes de disparar.
# Si la pelota escapa durante este tiempo, el ciclo se reinicia.
CONTACT_SETTLE_TIME_S  = 1.2  # s — espera de asentamiento antes del disparo (verifica posesión
                              #      estable: la pelota debe seguir centrada al final, no solo presente)
POST_KICK_COOLDOWN_S   = 0.8  # s — cooldown tras patear antes de poder detectar contacto
                               #     inmediato de nuevo. Fuerza reposicionamiento físico.

# Tiempo máximo que el atacante, ya en contacto y alineado, espera a que un rival
# despeje la línea de tiro antes de disparar igual (tiro disputado). Acota la espera
# de kick_when_clear para que nunca cuelgue; el tope duro de posesión es POSSESSION_MAX_TIME_S.
SHOT_WAIT_MAX_S = 2.0  # s — espera máx. por línea de tiro libre antes de disparar igual

# Tope duro de posesión: tiempo máximo que el atacante puede estar 'en posesión'
# (continuamente cerca de la pelota: dist < CIRCLE_BALL_ACTIVATE_DISTANCE_PX) antes de
# disparar a la fuerza y retirarse. Evita que el robot orbite/forcejee indefinidamente
# junto a la pelota (oscilación observada de ~11s). La SSL no impone un límite de TIEMPO
# de posesión en cancha (su regla análoga es de distancia: driblar <=1m); 4s es una
# adaptación al campo chico, donde la posesión sin dribbler es frágil.
POSSESSION_MAX_TIME_S = 4.0          # s  — tope de posesión antes del disparo forzado
POSSESSION_RELEASE_DISTANCE_PX = 100 # px — la pelota a >N px resetea el cronómetro (posesión perdida)

# Factores de escape para detectar que la pelota se alejó demasiado.
# Durante avance: dist > BEHIND_BALL_APPROACH_PX * ADVANCE_ESCAPE_FACTOR → abortar
# Durante asentamiento: dist > CAPTURE_CONFIRM_DISTANCE_PX * SETTLE_ESCAPE_FACTOR → abortar
ADVANCE_ESCAPE_FACTOR  = 1.5   # factor — margen de escape durante avance al contacto
SETTLE_ESCAPE_FACTOR   = 2.0   # factor — margen de escape durante asentamiento
ADVANCE_MAX_TIME_S     = 7.0   # s  — timeout en acercamiento sin lograr contacto (margen extra para distancias largas a creep)
ADVANCE_BALL_DRIFT_DEG = 50.0  # °  — deriva máx. del ángulo a pelota desde inicio del avance
# Anti-arrastre: si el robot ya está pegado a la pelota (dist <= CAPTURE_ACTIVATE)
# pero su nariz NO apunta a la pelota dentro de este umbral, abortar el avance ANTES
# de pivotar junto a ella (el pivote la empuja y deriva). Re-stagea por circle_ball
# limpio. Geométricamente el contacto (kick_err < tol) es imposible si el error de
# heading a la pelota supera ~atan(tol/offset)=atan(12/30)≈22°; 25 da un pequeño margen.
ADVANCE_CONTACT_ALIGN_DEG = 25.0  # ° — error máx. heading→pelota pegado a la pelota antes de re-stage

# Parámetros del yield cuando el rival tiene la pelota.
# RIVAL_HOLD_YIELD_S: cuánto tiempo el atacante cedente mantiene posición de press
#   antes de reintentar captura. Evita el busy-loop pero no bloquea indefinidamente.
# RIVAL_PRESS_MARGIN_PX: margen extra sobre CAPTURE_ACTIVATE_DISTANCE para el press,
#   manteniendo al robot justo fuera de la zona del rival.
BEHIND_BALL_RECALC_MIN_S  = 0.30  # s  — intervalo mínimo entre recalculaciones por pelota movida
                                   #      limita inundación del planner tras un kick (~3Hz máx)
RIVAL_HOLD_YIELD_S    = 1.5   # s  — tiempo cediendo posesión antes de reintentar captura
RIVAL_PRESS_MARGIN_PX = 15    # px — margen sobre CAPTURE_ACTIVATE_DISTANCE en posición press

# Parámetros de intercepción de pelota en movimiento.
# Cuando la pelota se desplaza rápido (tras un kick), los robots calculan el punto
# de llegada en vez de perseguir la posición actual → anticipan en lugar de perseguir.
# MIN_SPEED: umbral para ignorar ruido estático de cámara (~3-5 px/tick).
# LOOKAHEAD: ticks hacia adelante a predecir (tick ≈ 0.1s; 3 ticks = ~0.3s).
# MAX_PX: límite de desplazamiento de predicción para evitar overshoots agresivos.
BALL_INTERCEPT_MIN_SPEED_PX_PER_TICK = 8    # px/tick — velocidad mínima para activar predicción
BALL_INTERCEPT_LOOKAHEAD_TICKS       = 3    # ticks   — adelanto de predicción (~0.3s)
BALL_INTERCEPT_MAX_PX                = 60   # px      — desplazamiento máximo de predicción

# Factor multiplicador de PWM cuando el robot tiene posesión de la pelota.
# El dribbler genera fricción que frena la rotación/movimiento. Este factor
# compensa esa resistencia amplificando los PWM enviados a los motores.
# Valor 1.0 = sin cambio. Valor 1.3 = 30% más potencia. Solo aplica post-captura.
# Calibrar con scripts/calibrate_behavior_thresholds.py (teclas T/Y)
DRIBBLE_PWM_FACTOR = 1.0  # factor — compensación de fricción del dribbler

# Cap blando (Python) del PWM del dribbler. set_dribbler recorta cualquier valor a este
# techo ANTES de enviarlo por RF. Ajustable sin reflashear.
# ⚠️ NO usar el dribbler cerca del PWM máximo (255): el motor DC (N20, BJT como switch)
# NO tiene sensor de corriente; a PWM alto y en stall (rodillo trabado) la corriente de
# stall sostenida QUEMA el componente (ya ocurrió). Mantener el PWM bajo (rango de los
# motores de tracción, ~20-47) y pulsado. Subir solo de a poco vigilando temperatura.
DRIBBLER_MAX_PWM = 70  # PWM — techo blando; el valor de trabajo va MUY por debajo de 255

# Potencia del motor dribbler en PWM (0-255) durante la fase de CAPTURA (agarrar la pelota
# en el avance recto al contacto). En la escala SoftPWM de los motores de tracción
# (firmware 30-47, calibración 19-32): el rodillo tiene poca carga, gira bien a ~50.
# Valor de arranque conservador (lejos de 255 por seguridad); calibrar en banco.
DRIBBLER_CAPTURE_POWER = 50  # PWM para atrapar pelota (0-255 directo)

# Potencia del motor dribbler en PWM (0-255) mientras MANTIENE la pelota (keepalive).
# Reducida para bajar consumo. NO se usa rotando con la pelota (dribbler off al rotar).
DRIBBLER_HOLD_POWER = 30  # PWM reducido para sostener pelota (0-255 directo)

# Dribbler intermitente: duración del pulso ON (ms) mientras mantiene pelota.
# El motor se activa durante ON ms, luego se apaga durante OFF ms, y repite.
# Reduce calor en el motor al trabarse (pulsado macro): la corriente media baja en
# proporción ON/(ON+OFF), dejando enfriar entre pulsos. ON=80ms cubre el keepalive
# (<100ms del watchdog firmware). OFF>0 es clave para no cocinar el motor en stall.
DRIBBLER_PULSE_ON_MS = 80   # ms — duración del pulso encendido (= keepalive)
DRIBBLER_PULSE_OFF_MS = 20  # ms — duración del pulso apagado (>0 limita corriente; corto = agarre casi continuo)

# Distancia robot-pelota a la que se ENCIENDE el dribbler durante el avance al contacto.
# Solo gira cuando está lo bastante cerca para capturar (no desde behind_pos, lejos): da
# tiempo a que el rodillo tome vueltas justo antes del contacto. Debe ser < BEHIND_BALL_APPROACH_PX
# y >= CAPTURE_ACTIVATE_DISTANCE_PX. Subir = enciende antes (más margen); bajar = menos corriente.
DRIBBLER_ENGAGE_DISTANCE_PX = 50  # px — enciende el dribbler bajo esta distancia a la pelota

# --- Posicionamiento detrás de la pelota (ataque sin dribbler) ---
# El atacante se posiciona en la línea pelota-arco ANTES de hacer contacto,
# eliminando la necesidad de rotar con la pelota o activar el dribbler.
# BEHIND_BALL_APPROACH_PX debe ser > la zona-obstáculo de la pelota
# (PATH_PLANNING_BALL_OBSTACLE_RADIUS+POSITIONING_CLEARANCE+margin = 53px) con holgura,
# para que el staging sea alcanzable por el planner sin proyectar el goal (freeze).
BEHIND_BALL_APPROACH_PX = 64  # px — distancia robot-pelota al posicionarse detrás
BEHIND_BALL_LATERAL_OFFSET_PX = 75 # px — desvío lateral para rodear la pelota
BEHIND_BALL_ALIGN_TOLERANCE_DEG = 15.0  # ° — tolerancia angular aceptable al posicionar
# Techo de velocidad lineal al rodear la pelota de cerca (fase 'circle', única que se
# acerca a <CIRCLE_BALL_ACTIVATE_DISTANCE_PX por diseño). Entre el creep (30) y la rampa
# lejana (50-80): si el arco llegara a rozar la pelota, el contacto es suave (la nudge,
# no la lanza). Defensa en profundidad sobre PATH_PLANNING_BALL_POSITIONING_CLEARANCE.
BEHIND_BALL_NEAR_CEILING_PWM = 40  # PWM — techo lineal en el arco de aproximación a la pelota

# --- Skill: aproximación geométrica a la pelota (arco) ---
# Cuando el robot está cerca de la pelota Y en el lado equivocado, describe un
# arco a radio fijo alrededor de la pelota en vez de delegar al RRT* (que
# enrutaría indirectamente porque la pelota es obstáculo). Sin esto, el robot
# entra en loops de replanificación al tocar físicamente la pelota.
# Activar si dist(robot, pelota) < este umbral. Calibrado a partir de la
# distancia observada (~43 px) cuando R3 quedó atascado en el log 21:43 + 10 px.
CIRCLE_BALL_ACTIVATE_DISTANCE_PX = 55
# Radio del arco. Igual a BEHIND_BALL_APPROACH_PX para que el final del arco
# coincida exactamente con behind_pos (transición sin discontinuidades al exit).
CIRCLE_BALL_RADIUS_PX = 64
# Avance angular por waypoint del arco. La cuerda entre waypoints consecutivos
# (2·R·sin(step/2) = 37 px a R=72) DEBE superar el guard de de-duplicación de
# move_robot_to (20 px) y el umbral de llegada (BEHIND_BALL_ARRIVAL_PX=15): con
# 15° la cuerda era 15.7 px < ambos, el robot llegaba a un waypoint y el siguiente
# quedaba dentro del guard → cada comando se ignoraba → freeze al alinear. 30°
# garantiza progreso real por salto. El PID interpola entre waypoints (sigue suave).
CIRCLE_BALL_STEP_ANGLE_DEG = 30.0
# Tolerancia para salir del estado 'circle' y cambiar a 'direct'. Más laxo que
# BEHIND_BALL_ALIGN_TOLERANCE_DEG (15°) porque después viene el approach final.
CIRCLE_BALL_EXIT_TOLERANCE_DEG = 30.0

PUSH_BURST_PWM = 70                # PWM — pulso anti-stiction al iniciar avance al contacto

# --- Detección de robot atascado (stuck / anti-stall) ---
# Si el robot no avanza más de STUCK_MOVEMENT_THRESHOLD_PX píxeles dentro de una
# ventana de STUCK_DETECTION_WINDOW_S segundos, se suma STUCK_BOOST_INCREMENT PWM
# adicional por cada ventana consecutiva (máx STUCK_BOOST_MAX). Al moverse, decae.
STUCK_MOVEMENT_THRESHOLD_PX = 5  # px — desplazamiento mínimo para "no estar atascado"
STUCK_DETECTION_WINDOW_S = 0.8  # s  — ventana de tiempo (~30 frames @ 25 FPS)
STUCK_BOOST_INCREMENT = 3  # PWM — boost adicional por ventana sin movimiento
STUCK_BOOST_MAX = 12  # PWM — boost máximo acumulado (hard cap)
STUCK_BOOST_DECAY = 5             # PWM — reducción por ventana con movimiento
STUCK_AUTO_KICK = True  # Si True, dispara kick al llegar a STUCK_BOOST_MAX

# --- Regulador de velocidad del creep de captura por cámara (advance_to_contact) ---
# El cap fijo/por-distancia hacía que el robot llegara a la pelota demasiado rápido y
# la empujara. Ahora la velocidad base del creep se regula con un LAZO CERRADO sobre el
# desplazamiento REAL medido por la cámara (px por ventana): si se mueve menos del
# objetivo (no avanza / no es detectable) sube el base; si se mueve más (rápido,
# empujaría) lo baja; dentro de banda muerta lo mantiene. Así encuentra solo el mínimo
# detectable por robot/batería/piso — lo más lento que la cámara aún ve como movimiento.
# Lo aplica el controlador (DifferentialDriveController) en modo creep; el base regulado
# se observa en el campo `cv=` del [STATUS]. Calibrar en hardware.
CREEP_REGULATOR_ENABLED = True  # False → cap estático CAPTURE_CREEP_SPEED_PWM (comportamiento previo)
CREEP_BASE_MIN_PWM   = 16   # PWM — base más lenta y piso de rueda para el desaturado inferior
CREEP_BASE_MAX_PWM   = 30   # PWM — ceiling del base (techo del regulador y gate del creep)
CREEP_TARGET_DISP_PX = 5    # px — desplazamiento objetivo por ventana (> piso de ruido ArUco ~3-5)
CREEP_DISP_DEADBAND_PX = 2  # px — banda muerta alrededor del objetivo para no oscilar
# Ventana de medición + paso del ajuste gobiernan qué tan rápido sube el PWM base.
# Antes (0.4 s, +2): ~5 PWM/s — al vencer la fricción estática el robot daba un
# tirón a PWM alto y empujaba la pelota. Ahora (0.6 s, +1): ~1.7 PWM/s, rampa 3x
# más suave; rompe la fricción más cerca del umbral real (tirón menor) y la ventana
# más larga da una señal de desplazamiento más limpia frente al jitter de ArUco
# (menos subidas espurias justo al lado de la pelota).
CREEP_REG_WINDOW_S   = 0.6  # s  — ventana de medición de desplazamiento (~12 frames @ 20 FPS)
CREEP_BASE_STEP_PWM  = 1    # PWM — ajuste del base por ventana

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

# Gain scheduling de rotación: ganancia P agresiva usada SOLO en rotate_to_angle
# cuando |error| > ROBOT_ROTATION_ARRIVAL_ANGLE_DEG (zona lejos). Satura el perfil
# al techo del robot (pwm_max del JSON) para girar rápido lejos del objetivo. Cerca
# (|error| <= ROBOT_ROTATION_ARRIVAL_ANGLE_DEG) se usa el PID_ANGLE_KP calibrado
# (asentamiento suave, sin sobreimpulso). NO afecta la corrección angular del
# movimiento lineal (esa sigue con PID_ANGLE_KP). Calibrable en banco.
PID_ANGLE_KP_ROTATION_FAR = 2.0
