import cv2
import numpy as np
import random

# Configuración del video
ancho = 800
alto = 600
fps = 30
duracion = 5  # Duración en segundos

# Crear el video
fourcc = cv2.VideoWriter_fourcc(*"XVID")
video_salida = cv2.VideoWriter("video.mov", fourcc, fps, (ancho, alto))

# Generar el fondo de la cancha de fútbol
fondo = np.zeros((alto, ancho, 3), dtype=np.uint8)
fondo[:] = (40, 128, 40)  # Color verde para el césped

# Factores para escalar cancha
ratio_x = ancho / 640  
ratio_y = alto / 480  

# Dibujar las líneas de la cancha
cv2.rectangle(fondo,(int(5 * ratio_x), int(5 * ratio_y)),(int(635 * ratio_x), int(470 * ratio_y)),(255, 255, 255), 2)
cv2.circle(fondo,(int(ancho/2), int(alto/2)), int(73 * ratio_x), (255, 255, 255), 2)
cv2.line(fondo,(int(320 * ratio_x), int(5 * ratio_y)),(int(320 * ratio_x), int(470 * ratio_y)),(255, 255, 255), 2)

# Clase para representar un círculo
class Circulo:
    def __init__(self, x, y, radio, color):
        self.x = x
        self.y = y
        self.radio = radio
        self.color = color
        self.velocidad_x = random.uniform(-1, 1)  # Velocidad en el eje x
        self.velocidad_y = random.uniform(-1, 1)  # Velocidad en el eje y
        self.circulos_secundarios = self.generar_circulos_secundarios()

    def generar_circulos_secundarios(self):
        circulos = []
        for _ in range(3):
            radio_secundario = 15
            x_secundario = random.uniform(-self.radio + radio_secundario, self.radio - radio_secundario)
            y_secundario = random.uniform(-self.radio + radio_secundario, self.radio - radio_secundario)
            color_secundario = random.choice([(255, 0, 0), (0, 0, 255)])  # Rojo o azul
            circulos.append((x_secundario, y_secundario, radio_secundario, color_secundario))
        return circulos

    def actualizar(self, circulos):
        # Actualizar posición del círculo
        self.x += self.velocidad_x
        self.y += self.velocidad_y

        # Comprobar colisiones con los bordes de la ventana
        if self.x < self.radio or self.x > ancho - self.radio:
            self.velocidad_x *= -1
        if self.y < self.radio or self.y > alto - self.radio:
            self.velocidad_y *= -1

        # Comprobar colisiones con otros círculos
        for otro_circulo in circulos:
            if otro_circulo != self:
                distancia = np.sqrt((self.x - otro_circulo.x) ** 2 + (self.y - otro_circulo.y) ** 2)
                if distancia <= self.radio + otro_circulo.radio:
                    # Cambiar dirección de ambos círculos
                    self.velocidad_x *= -1
                    self.velocidad_y *= -1
                    otro_circulo.velocidad_x *= -1
                    otro_circulo.velocidad_y *= -1

    def dibujar(self, fotograma):
        # Dibujar círculo principal
        cv2.circle(fotograma, (int(self.x), int(self.y)), self.radio, self.color, -1)

        # Dibujar círculos secundarios
        for x_sec, y_sec, r_sec, color_sec in self.circulos_secundarios:
            x = int(self.x + x_sec)
            y = int(self.y + y_sec)
            cv2.circle(fotograma, (x, y), r_sec, color_sec, -1)

# Crear los círculos
circulo1 = Circulo(100, 100, 50, (0, 0, 0))
circulo2 = Circulo(200, 200, 50, (0, 0, 0))
circulo3 = Circulo(300, 300, 50, (0, 0, 0))
circulo4 = Circulo(400, 400, 50, (0, 0, 0))

# Bucle principal para generar el video
frames = duracion * fps
for i in range(frames):
    # Dibujar los círculos en el fotograma actual
    fotograma = fondo.copy()

    circulo1.actualizar([circulo2, circulo3, circulo4])
    circulo2.actualizar([circulo1, circulo3, circulo4])
    circulo3.actualizar([circulo1, circulo2, circulo4])
    circulo4.actualizar([circulo1, circulo2, circulo3])

    circulo1.dibujar(fotograma)
    circulo2.dibujar(fotograma)
    circulo3.dibujar(fotograma)
    circulo4.dibujar(fotograma)

    # Agregar el fotograma al video de salida
    video_salida.write(fotograma)    # Dibujar los círculos en el fotograma actual

# Cerrar el video de salida
video_salida.release()

# Cerrar el video de salida
video_salida.release()
# Leer el video generado
video = cv2.VideoCapture("video.mov")

# Reproducir el video fotograma por fotograma
while True:
    ret, fotograma = video.read()
    if not ret:
        break

    cv2.imshow("Video de fútbol", fotograma)
    if cv2.waitKey(25) & 0xFF == ord("q"):
        break
# Liberar recursos y cerrar la ventana de visualización
video.release()
cv2.destroyAllWindows()