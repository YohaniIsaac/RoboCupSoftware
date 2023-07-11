import cv2 as cv
import numpy as np
import random
import os

desaceleracion_roce = 0.01

class Jugador:
    def __init__(self, x, y, equipo, etiqueta, angulo,dx,dy,theta, radio):
        self.x          = x
        self.y          = y
        self.equipo     = equipo
        self.etiqueta   = etiqueta
        self.angulo     = angulo
        self.dx         = dx
        self.dy         = dy
        self.theta      = theta
        self.radio      = radio

    def mover(self):
        self.x      += self.dx
        self.y      += self.dy
        self.angulo += self.theta

    def colision(self, obj):
        pos_x = int(self.x - obj.x)
        pos_y = int(self.y - obj.y)
        rad = int(self.radio + obj.radio)
        if abs(pos_x) < rad and abs(pos_y) < rad: 
        # La velocidad de la pelota debe ser de 2 mínimamente
            if pos_x < 0:
                obj.dx = obj.dx*-1
            elif pos_y < 0:
                obj.dy = obj.dy*-1
            elif pos_x > 0:
                obj.dx = obj.dx*-1
            elif pos_y > 0: 
                obj.dy = obj.dy*-1

    def colision_borde(self):
        if (self.x - self.radio) < 0:
            self.dx = self.dx*-1
        elif (self.x + self.radio) > ancho:
            self.dx = self.dx*-1
        if (self.y - self.radio) < 0:
            self.dy = self.dy*-1
        elif (self.y + self.radio) > alto:
            self.dy = self.dy*-1 

    def desaceleracion(self):
        if abs(self.dx) > 0 or abs(self.dy) > 0:
            if self.dx > 0:
                self.dx -= desaceleracion_roce
                self.dx = max(self.dx,0)
            elif self.dx < 0:
                self.dx += desaceleracion_roce
                self.dx = min(self.dx, 0)

            if self.dy > 0:
                self.dy -= desaceleracion_roce
                self.dy = max(self.dy,0)
            elif self.dy < 0:
                self.dy += desaceleracion_roce
                self.dy = min(self.dy, 0)

    def teclas(self):

        #self.dx, self.dy, self.theta = 0, 0, 0
        if cv.waitKey(1) == ord("w"):
            self.dy -= 0.5
            self.dy = min(self.dy, -2)
        if cv.waitKey(1) == ord("s"):
            self.dy += 0.5
            self.dy = max(self.dy, 2)
        if cv.waitKey(1) == ord("a"):
            self.dx -= 0.5
            self.dx = min(self.dx, -2)
        if cv.waitKey(1) == ord("d"):
            self.dx += 0.5
            self.dx = max(self.dx,2)
        if cv.waitKey(1) == ord("p"):
            self.dx = 0
            self.dy = 0



def circulo(img, x, y, equipo, tag, angle):
    cv.circle(img, (x,y), 30, (0,0,0), -1)
    M = cv.getRotationMatrix2D((x, y), angle, 1)
    tf1 = np.dot(M, np.array([[x], [y+15], [1]]))
    tf2 = np.dot(M, np.array([[x+15], [y-10], [1]]))
    tf3 = np.dot(M, np.array([[x-15], [y-10], [1]]))
    cv.circle(img, (int(tf1[0]), int(tf1[1])), 10, equipo,-1)
    cv.circle(img, (int(tf2[0]), int(tf2[1])), 10, tag,-1)
    cv.circle(img, (int(tf3[0]), int(tf3[1])), 10, tag,-1)



# Parámetros de movimiento
t_inicial = 0.0
t_final = 1.0
pasos = 25

# Colores
rojo    = (0,0,255)
azul    = (255,0,0)
cian    = (255,255,0)
magenta = (255,0,255)

# Calcular el cambio en la posición vertical en cada paso
delta_t = (t_final - t_inicial) / pasos
t = t_inicial
dy = 0.0  # Cambio inicial en la posición vertical

# Configuración del video
ancho = 1280
alto = 650
fps = 40
duración = 10  # Duración en segundos

# Crear el video
fourcc = cv.VideoWriter_fourcc(*"XVID")
video_salida = cv.VideoWriter("../videos/video_futbol.avi", fourcc, fps, (ancho, alto))

# Generar el fondo de la cancha de fútbol
fondo = np.zeros((alto, ancho, 3), dtype=np.uint8)
fondo[:] = (40, 128, 40)  # Color verde para el césped

# Factores para escalar cancha
ratio_x = ancho / 640  
ratio_y = alto / 480  

# Dibujar las líneas de la cancha
cv.rectangle(fondo,(int(5 * ratio_x), int(5 * ratio_y)),(int(635 * ratio_x), int(470 * ratio_y)),(255, 255, 255), 2)
cv.circle(fondo,(int(ancho/2), int(alto/2)), int(73 * ratio_x), (255, 255, 255), 2)
cv.line(fondo,(int(320 * ratio_x), int(5 * ratio_y)),(int(320 * ratio_x), int(470 * ratio_y)),(255, 255, 255), 2)

# Crear instancias de la clase Jugador
player_1 = Jugador(200,             int(alto/2),    rojo, cian,     0,      0,  0,  0, 30)
player_2 = Jugador(int(ancho-200),  int(alto/2),    rojo, magenta,  45,     -1,  -1,  -1.1, 30)
player_3 = Jugador(int(ancho/2),    250,            azul, cian,     180,    -1,  1,  1.26, 30)
player_4 = Jugador(int(ancho/2),    int(alto-250),  azul, magenta,  270,    1,  -1,  -1.29, 30)

# Pelota
pelota   = Jugador(int(ancho/2),    int(alto/2),  azul, None,  270,    -2,  -2,  -1.29, 10)

# Bucle principal para generar el video
frames = duración * fps
while True:

    if cv.waitKey(1) == 27: # 27 ASCII para ecs
        break

    # Dibujar los jugadores en el fotograma actual
    fotograma = fondo.copy()

    circulo(fotograma, int(player_1.x), int(player_1.y) , player_1.equipo , player_1.etiqueta, player_1.angulo)
    #circulo(fotograma, int(player_2.x), int(player_2.y) , player_2.equipo , player_2.etiqueta, player_2.angulo)
    #circulo(fotograma, int(player_3.x), int(player_3.y) , player_3.equipo , player_3.etiqueta, player_3.angulo)
    #circulo(fotograma, int(player_4.x), int(player_4.y) , player_4.equipo , player_4.etiqueta, player_4.angulo)

    cv.circle(fotograma, (int(pelota.x), int(pelota.y)), 10, (6,100,255), -1 )

    # Actualizar la posición de los jugadores
    
    #player_2.mover()
    #player_3.mover()
    #player_4.mover()

    # Mover el jugador según las teclas presionadas
    player_1.teclas()
    player_1.mover()
    player_1.desaceleracion()

    pelota.desaceleracion()
    pelota.mover()


    # cambiar el sentido en caso de colisión con el borde del campo
    pelota.colision_borde()

    player_1.colision_borde()
    #player_2.colision_borde()
    #player_3.colision_borde()
    #player_4.colision_borde()

    # Cambiar dirección en caso de colisión con algún otro objeto 
    player_1.colision(pelota)
    #player_2.colision(pelota)
    #player_3.colision(pelota)
    #player_4.colision(pelota)


    # Agregar el fotograma al video de salida
    video_salida.write(fotograma)
    cv.imshow("video de futbolito", fotograma)
    


# Cerrar el video de salida
video_salida.release()

# Leer el video generado
video = cv.VideoCapture("../videos/video_futbol.avi")

# Reproducir el video fotograma por fotograma
# while True:
#     ret, fotograma = video.read()
#     if not ret:
#         break

#     cv.imshow("Video de fútbol", fotograma)

#     if cv.waitKey(25) & 0xFF == ord("q"):
#         break

# # Liberar recursos y cerrar la ventana de visualización
video.release()
cv.destroyAllWindows()