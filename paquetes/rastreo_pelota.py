import cv2 as cv
from config import *


#########################
# BUSQUEDA DE LA PELOTA #
#########################


class Ball:

    def __init__(self, color, centro):
        """
        Valores inciales para  la clase.

        Args:
        color      -- (array) Rangos de colores.
        centro      -- (array) Coordenadas del centro del jugador.
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
        """
        Recorta la imagen original y hsv, para poder tener sólo la vecindad donde es posible que se mueva la pelota
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
        """
        Detecta los goles de cada uno de los equipos ****
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
        """
        Detecta los circulos de colores

        Args:
        img_hsv  -- (matriz) Matriz de la imagen en HSV
        colores     -- (array) Rangos de los colores.
        imagen      -- (matriz) Matriz de la imagen en RGB

        Return:
        circulos_detetados  -- (array)  Contiene un vector por cada color detectado
                                        este contiene: el rago de colores, centro, radio
        """
        color_bajo, color_alto = colores

        # Crear una máscara utilizando los rangos de color especificados
        mascara = cv.inRange(img_hsv, color_bajo, color_alto)

        # Aplicar la máscara a la imagen original
        imagen_filtrada = cv.bitwise_and(img_origi, img_origi, mask=mascara)

        # Convertir la imagen filtrada a escala de grises
        imagen_gris = cv.cvtColor(imagen_filtrada, cv.COLOR_BGR2GRAY)

        # Aplicar un filtro de suavizado para reducir el ruido
        imagen_suavizada = cv.GaussianBlur(imagen_gris, (5, 5), 0)

        # Aplicar la transformada de Hough para detectar círculos
        circulos = cv.HoughCircles(imagen_suavizada, cv.HOUGH_GRADIENT, 1, minDist=20,
                                   param1=15, param2=15,
                                   minRadius=10, maxRadius=50)

        # Si se detectaron círculos, agregarlos a la lista de circulos_detectados
        x, y, r = None, None, None
        if circulos is not None:
            x, y, r = circulos[0][0][0], circulos[0][0][1], circulos[0][0][2]
        return int(x), int(y), int(r)
