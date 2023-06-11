import cv2 as cv
import numpy as np
import random
import os

def movimiento_vertical(t):
    a = 0.5  # Coeficiente cuadrático
    b = -2.0  # Coeficiente lineal
    c = 1.0  # Término independiente

    return a * t**2 + b * t + c

# Parámetros de movimiento
t_inicial = 0.0
t_final = 1.0
pasos = 25

# Calcular el cambio en la posición vertical en cada paso
delta_t = (t_final - t_inicial) / pasos
t = t_inicial
dy = 0.0  # Cambio inicial en la posición vertical

########################################################################

# Configuración del video
ancho = 1280
alto = 650
fps = 40
duración = 2  # Duración en segundos

# Leer imagen
img = cv.imread(os.path.dirname(__file__) + '\circulo_sin_fondo.png')
# Escalar
#img = cv.resize(img, (10, 10))
alto_img, ancho_img, _ = img.shape

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

# Diccionario de los jugadores
jugadores = [
    {"x": 200,              "y": int(alto/2),   "equipo" : (0,0,255), "id1": (255,255,0), "id2": (255,0,255)},  # Jugador rojo
    {"x": int(ancho-200),   "y": int(alto/2),   "equipo" : (0,0,255), "id1": (255,0,255), "id2": (255,255,0)},  # Jugador rojo
    {"x": int(ancho/2),     "y": 250,           "equipo" : (255,0,0), "id1": (255,255,0), "id2": (255,0,255)},  # Jugador azul
    {"x": int(ancho/2),     "y": int(alto-250), "equipo" : (255,0,0), "id1": (255,0,255), "id2": (255,255,0)}   # Jugador azul
]
# Pelota
pelota = {"x": int(ancho/2), "y": int(alto/2), "color": (24,194,243)}

# Bucle principal para generar el video
frames = duración * fps
for i in range(frames):

    # Dibujar los jugadores en el fotograma actual
    fotograma = fondo.copy()

    for jugador in jugadores:
        # Pelota con contorno
        cv.circle(fotograma, (pelota["x"],pelota["y"]), 30, pelota["color"], -1)
        cv.circle(fotograma, (pelota["x"],pelota["y"]), 32, (0), 1)

        # Jugador en negro
        cv.circle(fotograma, (jugador["x"], jugador["y"]), 30, (0), -1)

        # Código de color para jugadores
        cv.circle(fotograma, (jugador["x"], jugador["y"]-15), 10, jugador["equipo"], -1)
        cv.circle(fotograma, (jugador["x"]-15, jugador["y"]+10), 10, jugador["id1"], -1)
        cv.circle(fotograma, (jugador["x"]+15, jugador["y"]+10), 10, jugador["id2"], -1)


    # Actualizar la posición de los jugadores
    for jugador in jugadores:
        dx = 1
        dy = 1
        t += delta_t
        jugador["x"] += dx
        jugador["y"] += dy

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