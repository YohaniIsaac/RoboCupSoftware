"""Módulo para la detección y seguimiento de la pelota mediante el color de ésta.

Este módulo proporciona funcionalidades para detectar a la pelota en el campo
utilizando el color distintivo de ésta. Incluye procesamiento de imagen para
identificar la posición y delimitar la circunferencia de la pelota.
"""
import cv2 as cv
from robot_soccer.config import (
    COLOR_NEGRO,
    COLOR_ROJO_CV,
    COLOR_AZUL_CV,
    BALL_DETECTION_KERNEL_SIZE,
    BALL_DETECTION_MORPH_ITERATIONS,
    BALL_DETECTION_HOUGH_PARAM1,
    BALL_DETECTION_HOUGH_PARAM2,
    BALL_DETECTION_MIN_RADIUS,
    BALL_DETECTION_MAX_RADIUS,
)
#########################
# BUSQUEDA DE LA PELOTA #
#########################


class Ball:
    """Clase para el seguimiento y detección de la pelota con búsqueda inteligente.

    Esta clase implementa un sistema de tracking adaptativo que optimiza el rendimiento:
    - SEARCHING: Busca en toda la imagen cuando no tiene lock de la pelota
    - TRACKING: Busca solo en ROI pequeño una vez que tiene tracking estable

    El sistema cambia automáticamente entre modos según detecciones exitosas/fallidas.

    Attributes:
        color (tuple): Rangos de colores HSV para la detección de la pelota.
        x (int): Coordenada X del centro de la pelota.
        y (int): Coordenada Y del centro de la pelota.
        vecindad (int): Radio de vecindad para el seguimiento optimizado en ROI (default: 40px).
        goles_rojo (int): Contador de goles del equipo rojo.
        goles_azul (int): Contador de goles del equipo azul.
        pelota_fuera (bool): Estado que indica si la pelota está fuera del área de gol.
        roi_hsv (numpy.ndarray): Región de interés en espacio de color HSV.
        roi_img (numpy.ndarray): Región de interés en espacio de color RGB.
        tracking_mode (bool): False=SEARCHING (imagen completa), True=TRACKING (ROI).
        consecutive_detections (int): Contador de detecciones exitosas consecutivas.
        consecutive_failures (int): Contador de fallos consecutivos en modo TRACKING.
        frames_to_lock (int): Frames necesarios para cambiar a modo TRACKING (default: 10).
        max_failures (int): Máximo de fallos antes de volver a SEARCHING (default: 5).
    """

    def __init__(self, color, centro):
        """Inicializa una nueva instancia de la clase Ball.

        Args:
            color (tuple): Tupla con los rangos de colores HSV para detectar la pelota.
                          Formato: ((h_min, s_min, v_min), (h_max, s_max, v_max)).
            centro (tuple): Tupla con las coordenadas iniciales del centro de la pelota.
                           Formato: (x, y).

        Note:
            - La vecindad se establece en 40 píxeles por defecto para el tracking optimizado
            - Comienza en modo SEARCHING (búsqueda en toda la imagen)
            - Requiere 10 detecciones consecutivas para cambiar a modo TRACKING
        """
        self.color = color
        self.x, self.y = centro
        self.vecindad = 40

        self.goles_rojo = 0
        self.goles_azul = 0
        self.pelota_fuera = True

        self.roi_hsv = None
        self.roi_img = None

        # Sistema de tracking inteligente
        self.tracking_mode = False  # False = SEARCHING (buscar en toda imagen), True = TRACKING (ROI)
        self.consecutive_detections = 0  # Contador de detecciones consecutivas
        self.frames_to_lock = 10  # Frames necesarios para cambiar a modo TRACKING
        self.max_failures = 5  # Máximo de fallos consecutivos antes de volver a SEARCHING
        self.consecutive_failures = 0  # Contador de fallos consecutivos

    def seguimiento(self, hsv, img, frame):
        """Realiza el seguimiento de la pelota con búsqueda inteligente.

        Implementa dos modos de operación:
        - SEARCHING: Busca la pelota en toda la imagen (modo inicial y al perder tracking)
        - TRACKING: Busca solo en ROI pequeño alrededor de última posición (más rápido)

        El cambio entre modos se hace automáticamente según detecciones/fallos consecutivos.

        Args:
            hsv (numpy.ndarray): Imagen en espacio de color HSV.
            img (numpy.ndarray): Imagen en espacio de color RGB para procesamiento (¡NO BGR!).
            frame (numpy.ndarray): Frame original en BGR donde se dibujarán los elementos visuales.

        Returns:
            tuple: Tupla con las coordenadas actualizadas de la pelota (x, y).

        Note:
            - Modo SEARCHING: Requiere 10 detecciones consecutivas para cambiar a TRACKING
            - Modo TRACKING: 5 fallos consecutivos vuelven a modo SEARCHING
        """
        # Decidir si buscar en toda la imagen o en ROI
        if self.tracking_mode:
            # MODO TRACKING: Buscar solo en ROI alrededor de la última posición
            y_min = max(0, self.y - self.vecindad)
            y_max = min(hsv.shape[0], self.y + self.vecindad)
            x_min = max(0, self.x - self.vecindad)
            x_max = min(hsv.shape[1], self.x + self.vecindad)

            self.roi_hsv = hsv[y_min:y_max, x_min:x_max]
            self.roi_img = img[y_min:y_max, x_min:x_max]

            if len(self.roi_hsv) > 0 and self.roi_hsv.shape[0] > 0 and self.roi_hsv.shape[1] > 0:
                # Detectar en el ROI
                x_roi, y_roi, r_nuevo = self.detectar_circulos_color(self.roi_hsv, self.color, self.roi_img)

                if x_roi is not None and y_roi is not None:
                    # Detección exitosa en ROI
                    self.x = x_min + x_roi
                    self.y = y_min + y_roi
                    self.consecutive_failures = 0
                    cv.circle(frame, (self.x, self.y), 1, COLOR_NEGRO, -1)
                    if r_nuevo is not None:
                        cv.circle(frame, (self.x, self.y), r_nuevo, COLOR_NEGRO, 1)
                    self.goles(frame)
                else:
                    # Fallo en detección
                    self.consecutive_failures += 1
                    if self.consecutive_failures >= self.max_failures:
                        # Demasiados fallos, volver a modo SEARCHING
                        self.tracking_mode = False
                        self.consecutive_detections = 0
                        self.consecutive_failures = 0
        else:
            # MODO SEARCHING: Buscar en toda la imagen
            x_full, y_full, r_nuevo = self.detectar_circulos_color(hsv, self.color, img)

            if x_full is not None and y_full is not None:
                # Detección exitosa en imagen completa
                self.x = x_full
                self.y = y_full
                self.consecutive_detections += 1
                self.consecutive_failures = 0

                cv.circle(frame, (self.x, self.y), 1, COLOR_NEGRO, -1)
                if r_nuevo is not None:
                    cv.circle(frame, (self.x, self.y), r_nuevo, COLOR_NEGRO, 1)
                self.goles(frame)

                # Si hemos detectado consistentemente, cambiar a modo TRACKING
                if self.consecutive_detections >= self.frames_to_lock:
                    self.tracking_mode = True
            else:
                # Fallo en búsqueda completa
                self.consecutive_detections = 0

        return self.x, self.y

    def goles(self, frame):
        """Detecta y contabiliza los goles de ambos equipos.

        Verifica si la pelota ha entrado en alguna de las porterías basándose en
        su posición actual. Incrementa el contador correspondiente y actualiza
        el estado de la pelota. También dibuja el marcador en el frame.

        Args:
            frame (numpy.ndarray): Frame donde se dibujará el marcador de goles.

        Note:
            - Gol azul: pelota en x >= 1260 y 225 < y < 325
            - Gol rojo: pelota en x <= 22 y 225 < y < 325
            - El flag pelota_fuera evita contar múltiples goles por la misma jugada.
        """
        # distancia_derecha = ((self.x - x__der_arco)**2 + (self.y - y_der_arco)**2)**0.5
        # Contador de goles para el equipo azul
        if self.x + 5 >= 1260 and 225 < self.y < 325 and self.pelota_fuera:
            self.goles_azul += 1
            self.pelota_fuera = False

        # Contador de goles para el equipo rojo
        elif self.x - 5 <= 22 and 225 < self.y < 325 and self.pelota_fuera:
            self.goles_rojo += 1
            self.pelota_fuera = False

        font = cv.FONT_HERSHEY_SIMPLEX
        posicion_texto_rojo = (50, 50)
        posicion_texto_azul = (frame.shape[1] - 200, 50)
        goles_texto_rojo = f"Goles Rojos: {self.goles_rojo}"
        goles_texto_azul = f"Goles Azules: {self.goles_azul}"

        cv.putText(frame, goles_texto_rojo, posicion_texto_rojo, font, 0.7, COLOR_ROJO_CV, 2, cv.LINE_AA)
        cv.putText(frame, goles_texto_azul, posicion_texto_azul, font, 0.7, COLOR_AZUL_CV, 2, cv.LINE_AA)

    @classmethod
    def detectar_circulos_color(cls, img_hsv, colores, img_origi):
        """Detecta círculos de un color específico en la imagen usando transformada de Hough.

        Aplica filtrado por color, operaciones morfológicas para limpiar la máscara,
        y la transformada de Hough para detectar círculos que correspondan a la pelota.

        Args:
            img_hsv (numpy.ndarray): Imagen en espacio de color HSV para filtrado.
            colores (tuple): Tupla con los rangos de colores HSV.
                            Formato: ((h_min, s_min, v_min), (h_max, s_max, v_max)).
            img_origi (numpy.ndarray): Imagen original en RGB para aplicar la máscara.

        Returns:
            tuple: Tupla con las coordenadas del centro y radio del círculo detectado.
                  - Si se detecta: (x, y, radio) como enteros
                  - Si NO se detecta: (None, None, None)

        Note:
            Utiliza parámetros calibrados desde config.py:
            - Morfología: kernel size, iteraciones de apertura/cierre
            - HoughCircles: param1, param2, min/max radius
        """
        color_bajo, color_alto = colores

        # Crear una máscara utilizando los rangos de color especificados
        mascara = cv.inRange(img_hsv, color_bajo, color_alto)

        # Aplicar operaciones morfológicas para limpiar la máscara (reduce ruido y redondea forma)
        if BALL_DETECTION_KERNEL_SIZE > 0 and BALL_DETECTION_MORPH_ITERATIONS > 0:
            kernel = cv.getStructuringElement(
                cv.MORPH_ELLIPSE,
                (BALL_DETECTION_KERNEL_SIZE, BALL_DETECTION_KERNEL_SIZE)
            )
            for _ in range(BALL_DETECTION_MORPH_ITERATIONS):
                mascara = cv.morphologyEx(mascara, cv.MORPH_CLOSE, kernel)
                mascara = cv.morphologyEx(mascara, cv.MORPH_OPEN, kernel)

        # Aplicar la máscara a la imagen original
        imagen_filtrada = cv.bitwise_and(img_origi, img_origi, mask=mascara)

        # Convertir la imagen filtrada a escala de grises (img_origi es RGB)
        imagen_gris = cv.cvtColor(imagen_filtrada, cv.COLOR_RGB2GRAY)

        # Aplicar un filtro de suavizado para reducir el ruido
        imagen_suavizada = cv.GaussianBlur(imagen_gris, (5, 5), 0)

        # Aplicar la transformada de Hough para detectar círculos con parámetros calibrados
        circulos = cv.HoughCircles(
            imagen_suavizada,
            cv.HOUGH_GRADIENT,
            1,
            minDist=20,
            param1=BALL_DETECTION_HOUGH_PARAM1,
            param2=BALL_DETECTION_HOUGH_PARAM2,
            minRadius=BALL_DETECTION_MIN_RADIUS,
            maxRadius=BALL_DETECTION_MAX_RADIUS
        )

        # Si se detectaron círculos, devolver el primero
        if circulos is not None and len(circulos) > 0:
            x, y, r = circulos[0][0][0], circulos[0][0][1], circulos[0][0][2]
            return int(x), int(y), int(r)

        # Si no se detectó ningún círculo, retornar None
        return None, None, None
