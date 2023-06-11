import cv2
import numpy as np

# Dimensiones de la imagen
ancho = 30
alto = 30

# Crear una imagen con fondo transparente
imagen = np.zeros((alto, ancho, 4), dtype=np.uint8)

# Coordenadas del centro del círculo
centro_x = ancho // 2
centro_y = alto // 2

# Radio del círculo
radio = 10

# Color del círculo
color = (0, 255, 0, 255)  # Rojo (R, G, B, A)

# Dibujar el círculo en la imagen
cv2.circle(imagen, (centro_x, centro_y), radio, color, -1)

# Guardar la imagen en formato PNG sin fondo
cv2.imwrite("circulo_sin_fondo.png", imagen)