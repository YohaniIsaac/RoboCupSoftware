import cv2
import os
import time
import numpy as np
import math

class Objeto:
    def __init__(self, equipo, colorID, centro, frame):
        self.equipo = globals()[equipo]
        self.x, self.y = centro
        if colorID is not None:
            self.colorID = globals()[colorID]

    def seguimiento_players(self, hsv, imagen):
        self.hsv = hsv
        self.imagen = imagen
        self.roi = hsv[self.y-50:self.y+50 , self.x-50:self.x+50]
        self.team = detectar_circulos_color(self.hsv, self.equipo, self.imagen, None)
        self.tags = detectar_circulos_color(self.hsv, self.colorID, self.imagen, None)
        self.centros = detectar_centro(self.team, self.tags)
        print(self.centros)
        if len(self.centros) > 0:
            self.x , self.y = self.centros[0]["centro"]
            cv2.circle(frame, (self.x, self.y) ,  2, (255,255,255),-1)
            cv2.imshow("asda", self.roi)
        

    def seguimiento_ball(self,hsv,imagen):
        self.hsv = hsv
        self.imagen = imagen
        self.roi = hsv[self.y-20:self.y+20 , self.x-20:self.x+20]
        self.ball = detectar_circulos_color(self.hsv, self.equipo, self.imagen, None)
        self.x, self.y = self.ball[0]["centro"]

        cv2.circle(frame, (self.x, self.y) ,  1, (0,0,0),-1)
        cv2.imshow("sdas", self.roi)


rojo = ((0, 100, 20), (8, 255, 255), (175, 100, 20), (179, 255, 255))    # Rango de color para el rojo
azul = ((110, 150, 150), (130, 255, 255), None, None)  # Rango de color para el azul
magenta = ((145, 150, 150), (165, 255, 255), None, None)  # Rango de color para el magenta
cian = ((85, 150, 150), (95, 255, 255), None, None)  # Rango de color para el cian           
naranjo= ((10, 100, 20), (30, 255, 255), None, None)  # Rango de color para el naranjo


def detectar_circulos_color(imagen_hsv, colores, imagen, color):
    circulos_detectados = []

    color_bajo, color_alto, color_bajo2, color_alto2 = colores
    # Crear una máscara utilizando los rangos de color especificados
    mascara = cv2.inRange(imagen_hsv, color_bajo, color_alto)
    if color_alto2 and color_bajo2 is not None:
        mascara1 = mascara
        mascara2 = cv2.inRange(imagen_hsv, color_bajo2, color_alto2)
        mascara = cv2.add(mascara1, mascara2)

    # Aplicar la máscara a la imagen original
    imagen_filtrada = cv2.bitwise_and(imagen, imagen, mask=mascara)

    # Convertir la imagen filtrada a escala de grises
    imagen_gris = cv2.cvtColor(imagen_filtrada, cv2.COLOR_BGR2GRAY)

    # Aplicar un filtro de suavizado para reducir el ruido
    imagen_suavizada = cv2.GaussianBlur(imagen_gris, (5,5),0)
    
    # Aplicar la detección de bordes
    # bordes = cv2.Canny(imagen_suavizada, 10, 200)

    # Aplicar la transformada de Hough para detectar círculos
    circulos = cv2.HoughCircles(imagen_suavizada, cv2.HOUGH_GRADIENT, 1, minDist=20,
                                param1=15, param2=15,
                                minRadius= 5, maxRadius= 50)

    # Si se detectaron círculos, agregarlos a la lista de circulos_detectados
    if circulos is not None:
        circulos = np.round(circulos[0, :]).astype(int)
        for (x, y, r) in circulos:
            circulos_detectados.append({"color": color, "centro": (x, y), "radio": r})
    return circulos_detectados

def detectar_centro(equipo, identificador):
    Jugadores = []

    for team in equipo:
        x_aux, y_aux = None,None
        for tag in identificador:
            x_val , y_val = tag["centro"]
            x_cen , y_cen = team["centro"]

            d = math.sqrt((x_val - x_cen)**2 + (y_val - y_cen)**2) 

            if d <= 30:
                if x_aux is None and y_aux is None:
                    x_aux, y_aux = x_val , y_val
                else:
                    x_centro = (x_cen + x_aux + x_val) / 3
                    y_centro = (y_cen + y_aux + y_val) / 3
                    centro = (int(x_centro), int(y_centro))
                    Jugadores.append({'equipo': team["color"], 'colorID': tag["color"], \
                        'centro1':(x_cen , y_cen), 'centro2':(x_aux, y_aux), 'centro3': (x_val , y_val), 'centro':centro})
    return Jugadores

if __name__ == "__main__":
    # capture video
    ruta = os.path.join(os.path.dirname(__file__), 'video_futbol.avi')
    cap = cv2.VideoCapture(ruta)
    
    first_frame = True

    while cap.read()[0] == True:
        ret, frame = cap.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        print(first_frame)

        if first_frame:
            circulos_rojos      = detectar_circulos_color(hsv, rojo, frame, "rojo") 
            circulos_azul       = detectar_circulos_color(hsv, azul, frame, "azul") 
            circulos_cian       = detectar_circulos_color(hsv, cian, frame, "cian") 
            circulos_magenta    = detectar_circulos_color(hsv, magenta, frame, "magenta") 

            circulos_naranjo    = detectar_circulos_color(hsv, naranjo, frame, "naranjo")

            equipo = circulos_rojos + circulos_azul
            identificador = circulos_magenta + circulos_cian

            Jugadores = detectar_centro(equipo,identificador)

            
            player_1 = Objeto(Jugadores[0]['equipo'], Jugadores[0]['colorID'], Jugadores[0]['centro'], frame)
            player_2 = Objeto(Jugadores[1]['equipo'], Jugadores[1]['colorID'], Jugadores[1]['centro'], frame)
            player_3 = Objeto(Jugadores[2]['equipo'], Jugadores[2]['colorID'], Jugadores[2]['centro'], frame)
            player_4 = Objeto(Jugadores[3]['equipo'], Jugadores[3]['colorID'], Jugadores[3]['centro'], frame)

            ball = Objeto(circulos_naranjo[0]['color'], None, circulos_naranjo[0]['centro'], frame)
            first_frame = False
        else:
            ball.seguimiento_ball(hsv, frame)
            player_1.seguimiento_players(hsv, frame)
            player_2.seguimiento_players(hsv, frame)
            player_3.seguimiento_players(hsv, frame)
            player_4.seguimiento_players(hsv, frame)
            



        if ret == False:
            break


        # Mostrar la imagen con los círculos detectados
        cv2.imshow("original", frame)
        #time.sleep(2)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

    cv2.destroyAllWindows()