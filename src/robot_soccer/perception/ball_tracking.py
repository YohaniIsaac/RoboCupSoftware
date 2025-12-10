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
    """Clase para el seguimiento y detección de la pelota en el campo de juego.

    Esta clase maneja la detección, seguimiento y análisis de la pelota utilizando
    técnicas de procesamiento de imágenes con OpenCV. Incluye funcionalidades para
    detectar goles y mantener el estado de la pelota durante el juego.

    Attributes:
        color (tuple): Rangos de colores HSV para la detección de la pelota.
        x (int): Coordenada X del centro de la pelota.
        y (int): Coordenada Y del centro de la pelota.
        vecindad (int): Radio de vecindad para el seguimiento optimizado.
        goles_rojo (int): Contador de goles del equipo rojo.
        goles_azul (int): Contador de goles del equipo azul.
        pelota_fuera (bool): Estado que indica si la pelota está fuera del área de gol.
        roi_hsv (numpy.ndarray): Región de interés en espacio de color HSV.
        roi_img (numpy.ndarray): Región de interés en espacio de color RGB.
    """

    def __init__(self, color, centro):
        """Inicializa una nueva instancia de la clase Ball.

        Args:
            color (tuple): Tupla con los rangos de colores HSV para detectar la pelota.
                          Formato: ((h_min, s_min, v_min), (h_max, s_max, v_max)).
            centro (tuple): Tupla con las coordenadas iniciales del centro de la pelota.
                           Formato: (x, y).

        Note:
            La vecindad se establece en 40 píxeles por defecto para optimizar
            el seguimiento en frames posteriores.
        """
        self.color = color
        self.x, self.y = centro
        self.vecindad = 40

        self.goles_rojo = 0
        self.goles_azul = 0
        self.pelota_fuera = True

        self.roi_hsv = None
        self.roi_img = None

    def seguimiento(self, hsv, img, frame):
        """Realiza el seguimiento de la pelota en el frame actual.

        Recorta la imagen HSV y RGB en la vecindad de la última posición conocida
        de la pelota para optimizar la detección. Actualiza la posición de la pelota
        y dibuja elementos visuales en el frame.

        Args:
            hsv (numpy.ndarray): Imagen en espacio de color HSV.
            img (numpy.ndarray): Imagen en espacio de color RGB para procesamiento.
            frame (numpy.ndarray): Frame original donde se dibujarán los elementos visuales.

        Returns:
            tuple: Tupla con las coordenadas actualizadas de la pelota (x, y).

        Note:
            Este método utiliza la región de interés (ROI) para mejorar el rendimiento
            del seguimiento, limitando la búsqueda a un área específica alrededor de
            la última posición conocida.
        """
        # Recorta la imagen HSV y RGB
        self.roi_hsv = hsv[self.y - self.vecindad:self.y + self.vecindad, self.x - self.vecindad:self.x + self.vecindad]

        self.roi_img = img[self.y - self.vecindad:self.y + self.vecindad, self.x - self.vecindad:self.x + self.vecindad]

        if len(self.roi_hsv) > 0:
            # Detecta los circulos dentro del recorte y su centro
            x_nuevo, y_nuevo, r_nuevo = self.detectar_circulos_color(self.roi_hsv, self.color, self.roi_img)
            cv.circle(self.roi_hsv, (x_nuevo, y_nuevo), r_nuevo, COLOR_NEGRO, 1)
            # Reescribe el centro y actualiza este en el objeto
            self.x, self.y = self.x + x_nuevo - self.vecindad, self.y + y_nuevo - self.vecindad
            # Dibuja un círculo en el centro de la pelota
            cv.circle(frame, (self.x, self.y), 1, COLOR_NEGRO, -1)
            cv.circle(self.roi_hsv, (self.x, self.y), r_nuevo, COLOR_NEGRO, 1)
            self.goles(frame)

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
                  Formato: (x, y, radio). Retorna (None, None, None) si no se detecta.

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

        # Convertir la imagen filtrada a escala de grises
        imagen_gris = cv.cvtColor(imagen_filtrada, cv.COLOR_BGR2GRAY)

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

        # Si se detectaron círculos, agregarlos a la lista de circulos_detectados
        x, y, r = None, None, None
        if circulos is not None:
            x, y, r = circulos[0][0][0], circulos[0][0][1], circulos[0][0][2]
        return int(x), int(y), int(r)
