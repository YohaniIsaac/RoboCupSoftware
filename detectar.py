import cv2
import os
import time
import numpy as np
import math
import datetime

class Jugador:

    def __init__(self, equipo, colorID, centro):
        """
        Valores inciales para  la clase.

        Args:
        equipo      -- (array) Rangos de colores del equipo.
        colorId     -- (array) Rangos de colores del tag (un color por jugador).
        centro      -- (array) Coordenadas del centro del jugador.
        """
        self.equipo = equipo
        self.x, self.y = centro
        self.colorID = colorID
        self.vecindad = 70
        
    def dibujar(self, frame):   
        """
        Dibuja una línea entre los tag del jugador, con variables
        dentro de la misma clase.
        """
        # Reajusta los centros de los para poder dibujarlos en el 
        # frame principal
        self.x_tag1, self.y_tag1 = self.tags[0][1]
        self.x_tag1, self.y_tag1 = self.x + self.x_tag1 - self.vecindad , self.y + self.y_tag1 - self.vecindad
        self.x_tag2, self.y_tag2 = self.tags[1][1]
        self.x_tag2, self.y_tag2 = self.x + self.x_tag2 - self.vecindad , self.y + self.y_tag2 - self.vecindad
        cv2.line(frame, (self.x_tag1, self.y_tag1), (self.x_tag2, self.y_tag2), (255,255,255),2)

    def seguimiento_players(self, hsv, img, frame):
        """
        Recorta la imagen original y hsv, para poder tener sólo la self.vecindad
        donde es posible que se mueva el jugador
        """
        # Recorta la imagen HSV y RGB
        self.roi_hsv = hsv[ self.y - self.vecindad : self.y + self.vecindad,
                            self.x - self.vecindad : self.x + self.vecindad]

        self.roi_img = img[ self.y - self.vecindad : self.y + self.vecindad,
                            self.x - self.vecindad : self.x + self.vecindad]
        if len(self.roi_img) > 0:
            # Detecta los circulos dentro del recorte y su centro 
            self.team = self.detectar_circulos_color(self.roi_hsv, self.equipo, self.roi_img)
            self.tags = self.detectar_circulos_color(self.roi_hsv, self.colorID, self.roi_img)
            self.centros = self.detectar_centro(self.team, self.tags)
            if len(self.centros) > 0:
                self.dibujar(frame)
                # Reescribe el centro y actualiza este en el objeto
                self.x_nuevo, self.y_nuevo = self.centros[0][5]
                self.x = self.x + self.x_nuevo - self.vecindad
                self.y = self.y + self.y_nuevo - self.vecindad

                # Dibuja un circulo en el centro del jugador
                cv2.circle(frame, (self.x, self.y), 4, (255, 0, 255), -1)
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

        color_bajo, color_alto, color_bajo2, color_alto2 = colores

        # Crear una máscara utilizando los rangos de color especificados
        mascara = cv2.inRange(imagen_hsv, color_bajo, color_alto)
        if color_alto2 and color_bajo2 is not None:
            mascara1 = mascara
            mascara2 = cv2.inRange(imagen_hsv, color_bajo2, color_alto2)
            mascara = cv2.add(mascara1, mascara2)

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
        if circulos is not None:
            circulos = np.round(circulos[0, :]).astype(int)
            for (x, y, r) in circulos:
                #           (rango de colores , centro, radio)
                circulos_detectados.append([colores, (x,y), r])
        return circulos_detectados

    @classmethod
    def detectar_centro(cls, all_equipos, all_identificadores):
        """
        Detecta el centro del jugador en base al circulo de color del equipo, busca los 
        identificadores cercanos e identifica de que color son, creando un vector por cada
        jugador encontrado, con su

        Args:
        all_equipos         -- (array)  Un array de vectores, donde cada hay un vector 
                                        por cada circulo detectado de los colores de su
                                        equipo, este contiene:
                                        rango de colores, centro, radio
        all_identificadores -- (array)  Array de vectores, por cada identificador detectado
                                        por cada circulo del identificador contiene:
                                        rando de colores, centro, radio 

        Return:
        Jugadores   -- (array)  Contiene un vector por cada jugador encontrado
                                Cada vector contiene:
                                rango de color del equipo, rango de color del identificador,
                                centro del circulo por quipo, centro del ciruclo del primer 
                                identificador, centro del circulo del segundo identificador,
                                centro del jugador.
        """
        Jugadores = []

        for team in all_equipos:
            x_aux, y_aux = None,None
            for tag in all_identificadores:
                x_val , y_val = tag[1]
                x_cen , y_cen =team[1]

                d = math.sqrt((x_val - x_cen)**2 + (y_val - y_cen)**2) 
                if d <= 40:
                    if x_aux is None and y_aux is None:
                        x_aux, y_aux = x_val , y_val
                    else:
                        x_centro = (x_cen + x_aux + x_val) / 3
                        y_centro = (y_cen + y_aux + y_val) / 3
                        centro = (int(x_centro), int(y_centro))
                        # color equipo, color identificador, centro circulo team, centro ID1, centro ID2, centro del jugador 
                        Jugadores.append([team[0], tag[0], (x_cen , y_cen),(x_aux, y_aux), (x_val , y_val), centro])

        return Jugadores


def main():
    # Colores
    rojo = ((0, 100, 20), (8, 255, 255), (175, 100, 20), (179, 255, 255))    # Rango de color para el rojo
    azul = ((110, 150, 150), (130, 255, 255), None, None)  # Rango de color para el azul
    magenta = ((145, 150, 150), (165, 255, 255), None, None)  # Rango de color para el magenta
    cian = ((85, 150, 150), (95, 255, 255), None, None)  # Rango de color para el cian           

    # capture video
    ruta = os.path.join(os.path.dirname(__file__), '../videos/video_futbol.mp4')
    cap = cv2.VideoCapture(ruta)
    
    first_frame = True

    while cap.read()[0] == True:
        ret, frame = cap.read()
        img = np.copy(frame)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if first_frame:
            circulos_rojos      = Jugador.detectar_circulos_color(hsv, rojo, img) 
            circulos_azul       = Jugador.detectar_circulos_color(hsv, azul, img) 
            circulos_cian       = Jugador.detectar_circulos_color(hsv, cian, img) 
            circulos_magenta    = Jugador.detectar_circulos_color(hsv, magenta, img) 

            all_equipos = circulos_rojos + circulos_azul
            all_identificadores = circulos_magenta + circulos_cian

            Jugadores = Jugador.detectar_centro(all_equipos,all_identificadores)
            

            equipo = Jugadores[0][0]
            colorID = Jugadores[0][1]
            centro = Jugadores[0][5]
            
            player1 = Jugador(equipo, colorID, centro)

            # players = []
            # for jugador in Jugadores:
            #     equipo = jugador[0]
            #     colorID = jugador[1]
            #     centro2 = jugador[3]
            #     centro3 = jugador[4]
            #     centro = jugador[5]

            #     player = Jugador(equipo, colorID, centro)
            #     players.append(player)

            first_frame = False
            
        else:
            player1.seguimiento_players(hsv, img, frame)
            cv2.imshow("jugador 1", player1.roi_img)
            # for player in players:
            #     player.seguimiento_players(hsv,img,frame)
            #     print("verficacion 3:", player.x, player.y)
            #     cv2.imshow("jugador 1", player.roi_img)
            #     cv2.waitKey(1)

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