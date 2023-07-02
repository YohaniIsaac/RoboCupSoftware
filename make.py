import cv2 as cv
import numpy as np
import random
import os

class Jugador:
    def __init__(self, x, y, equipo, etiqueta, angulo,dx,dy,theta):
        self.x          = x
        self.y          = y
        self.equipo     = equipo
        self.etiqueta   = etiqueta
        self.angulo     = angulo
        self.dx         = dx
        self.dy         = dy
        self.theta      = theta
    def mover(self):
        self.x      += self.dx
        self.y      += self.dy
        self.angulo += self.theta


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
duración = 2  # Duración en segundos

# Crear el video
fourcc = cv.VideoWriter_fourcc(*"XVID")
video_salida = cv.VideoWriter("video_futbol.avi", fourcc, fps, (ancho, alto))

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
player_1 = Jugador(200,             int(alto/2),    rojo, cian,     0,      1.3,  -1.4,  1.5)
player_2 = Jugador(int(ancho-200),  int(alto/2),    rojo, magenta,  45,     -1,  -1,  -1.1)
player_3 = Jugador(int(ancho/2),    250,            azul, cian,     180,    -1,  1,  1.26)
player_4 = Jugador(int(ancho/2),    int(alto-250),  azul, magenta,  270,    1,  -1,  -1.29)

# Pelota
pelota   = Jugador(int(ancho/2),    int(alto/2),  azul, None,  270,    4,  -4,  -1.29)

# Bucle principal para generar el video
frames = duración * fps
for i in range(frames):

    # Dibujar los jugadores en el fotograma actual
    fotograma = fondo.copy()

    circulo(fotograma, int(player_1.x), int(player_1.y) , player_1.equipo , player_1.etiqueta, player_1.angulo)
    circulo(fotograma, int(player_2.x), int(player_2.y) , player_2.equipo , player_2.etiqueta, player_2.angulo)
    circulo(fotograma, int(player_3.x), int(player_3.y) , player_3.equipo , player_3.etiqueta, player_3.angulo)
    circulo(fotograma, int(player_4.x), int(player_4.y) , player_4.equipo , player_4.etiqueta, player_4.angulo)

    cv.circle(fotograma, (int(pelota.x), int(pelota.y)), 10, (6,100,255), -1 )
    # Actualizar la posición de los jugadores
    player_1.mover()
    player_2.mover()
    player_3.mover()
    player_4.mover()

    pelota.mover()
    # Agregar el fotograma al video de salida
    video_salida.write(fotograma)

# Cerrar el video de salida
video_salida.release()

# Leer el video generado
video = cv.VideoCapture("video_futbol.avi")

# Reproducir el video fotograma por fotograma
while True:
    ret, fotograma = video.read()
    if not ret:
        break

    cv.imshow("Video de fútbol", fotograma)
    if cv.waitKey(25) & 0xFF == ord("q"):
        break

# Liberar recursos y cerrar la ventana de visualización
video.release()
cv.destroyAllWindows()