import cv2
import numpy as np

# Cargar la imagen
path = "Tag_1"
image = cv2.imread(path + ".png")  # Reemplaza con la ruta de tu imagen
image_height, image_width = image.shape[:2]

# Tamaño del marco (margen en píxeles)
margin = 100  # Cambia este valor según el tamaño del marco que desees

# Crear un fondo blanco con dimensiones ajustadas
background_width = image_width + 2 * margin
background_height = image_height + 2 * margin
background = np.ones((background_height, background_width, 3), dtype=np.uint8) * 255

# Calcular la posición para centrar la imagen en el fondo
x = margin
y = margin

# Superponer la imagen sobre el fondo
background[y:y+image_height, x:x+image_width] = image

# Guardar la imagen resultante con un nuevo nombre
cv2.imwrite(path + "v2.png", background)