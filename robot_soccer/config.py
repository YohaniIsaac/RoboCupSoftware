import os
from pathlib import Path

"""
Archivo de configuración centralizado para el proyecto de fútbol de robots
Contiene constantes y variables compartidas entre diferentes módulos
"""
# ==========================================
# Configuraciones de rutas
# ==========================================

# Obtener la ruta absoluta al directorio donde está config.py (raíz del proyecto)
ROOT_DIR = Path(os.path.dirname(os.path.abspath(__file__)))

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

# Zonas del campo (en porcentaje del ancho)
ZONA_IZQUIERDA = 0.3  # 30% del ancho desde la izquierda
ZONA_DERECHA = 0.7    # 70% del ancho desde la izquierda

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
RANGO_COLOR_NARANJO = ((10, 100, 20), (30, 255, 255))

# Rangos para colores de equipos
RANGO_COLOR_ROJO = ((0, 100, 20), (8, 255, 255), (175, 100, 20), (179, 255, 255))
RANGO_COLOR_AZUL = ((110, 150, 150), (130, 255, 255), None, None)
RANGO_COLOR_MAGENTA = ((145, 150, 150), (165, 255, 255), None, None)
RANGO_COLOR_CIAN = ((85, 150, 150), (95, 255, 255), None, None)
