import cv2
import os
import time
import numpy as np
import math
import datetime



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

    def seguimiento(self, hsv, img, frame):
        """
        Recorta la imagen original y hsv, para poder tener sólo la vecindad
        donde es posible que se mueva el jugador
        """
        # Recorta la imagen HSV y RGB

        print("verificar:", self.x, self.y, vecindad)

        self.roi_hsv = hsv[ self.y - vecindad:self.y + vecindad ,
                            self.x - vecindad:self.x + vecindad]
                            
        self.roi_img = img[ self.y - vecindad:self.y + vecindad , 
                            self.x - vecindad:self.x + vecindad]

        if len(self.roi_hsv) > 0: 
            # Detecta los circulos dentro del recorte y su centro 
            self.x_nuevo, self.y_nuevo, self.r_nuevo = self.detectar_circulos_color(self.roi_hsv, self.color, self.roi_img)
            # Reescribe el centro y actualiza este en el objeto
            self.x, self.y = self.x + self.x_nuevo - vecindad , self.y + self.y_nuevo - vecindad
            # Dibuja un circulo en el centro del jugador
            cv2.circle(frame, (self.x, self.y), 1, (255, 255, 255), -1)
        return


    @classmethod 
    def detectar_circulos_color(cls, imagen_hsv, colores, imagen_original):
        """
        Detecta los circulos de colores

        Args:
        imagen_hsv  -- (matriz) Matriz de la imagen en HSV 
        colores     -- (array) Rangos de los colores.
        imagen      -- (matriz) Matriz de la imagen en RGB

        Return:
        circulos_detetados  -- (array)  Contiene un vector por cada color detectado
                                        este contiene: el rago de colores, centro, radio
        """
        circulos_detectados = []

        color_bajo, color_alto = colores

        # Crear una máscara utilizando los rangos de color especificados
        mascara = cv2.inRange(imagen_hsv, color_bajo, color_alto)

        # Aplicar la máscara a la imagen original
        imagen_filtrada = cv2.bitwise_and(imagen_original, imagen_original, mask=mascara)

        # Convertir la imagen filtrada a escala de grises
        imagen_gris = cv2.cvtColor(imagen_filtrada, cv2.COLOR_BGR2GRAY)

        # Aplicar un filtro de suavizado para reducir el ruido
        imagen_suavizada = cv2.GaussianBlur(imagen_gris, (5,5),0)

        # Aplicar la transformada de Hough para detectar círculos
        circulos = cv2.HoughCircles(imagen_suavizada, cv2.HOUGH_GRADIENT, 1, minDist=20,
                                    param1=15, param2=15,
                                    minRadius= 5, maxRadius= 50)

        # Si se detectaron círculos, agregarlos a la lista de circulos_detectados
        print(circulos[0][0][2])
        if circulos is not None:
            x , y , r = circulos[0][0][0], circulos[0][0][1], circulos[0][0][2]
        return int(x), int(y), int(r)

def main():
    #radio de la vecindad de busqueda
    vecindad = 20
    # Color          
    naranjo= ((10, 100, 20), (30, 255, 255))  # Rango de color para el naranjo

    # capture video
    ruta = os.path.join(os.path.dirname(__file__), '../videos/video_futbol.mp4')
    cap = cv2.VideoCapture(ruta)
    
    first_frame = True

    while cap.read()[0] == True:
        ret, frame = cap.read()
        img = np.copy(frame)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if first_frame:
            x,y,r = Ball.detectar_circulos_color(hsv, naranjo, img)
            pelota = Ball(naranjo, (x,y))

            first_frame = False
            
        else:
            pelota.seguimiento(hsv, img, frame)
            cv2.imshow("jugador 1", pelota.roi_img)

        if ret == False:
            break


        # Mostrar la imagen con los círculos detectados
        cv2.imshow("img", img)
        cv2.imshow("frame", frame)
        cv2.waitKey(0)
        #time.sleep(2)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
