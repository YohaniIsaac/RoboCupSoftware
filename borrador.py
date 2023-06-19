import cv2
import numpy as np

def mover_figura():
    # Dimensiones de la imagen y posición inicial de la figura
    width, height = 800, 600
    x, y = 400, 300

    # Crear una imagen en blanco como lienzo
    imagen = np.zeros((height, width, 3), dtype=np.uint8)

    # Definir los parámetros de la figura
    radio_grande = 100
    radio_pequeno = 20
    color_grande = (0, 0, 255)  # Rojo
    color_pequeno = (0, 255, 0)  # Verde

    # Dibujar la figura inicial en el lienzo
    cv2.circle(imagen, (x, y), radio_grande, color_grande, -1)
    cv2.circle(imagen, (x, y), radio_pequeno, color_pequeno, -1)
    cv2.circle(imagen, (x - 50, y), radio_pequeno, color_pequeno, -1)
    cv2.circle(imagen, (x + 50, y), radio_pequeno, color_pequeno, -1)

    # Crear una ventana para mostrar la imagen
    cv2.namedWindow("Figura")

    # Bucle principal
    while True:
        # Mostrar la imagen actualizada en la ventana
        cv2.imshow("Figura", imagen)

        # Esperar por una tecla
        key = cv2.waitKey(1) & 0xFF

        # Salir del bucle si se presiona la tecla "q"
        if key == ord("q"):
            break

        # Mover la figura
        if key == ord("w"):
            y -= 10
        elif key == ord("s"):
            y += 10
        elif key == ord("a"):
            x -= 10
        elif key == ord("d"):
            x += 10

        # Actualizar la posición de la figura en la imagen
        imagen = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.circle(imagen, (x, y), radio_grande, color_grande, -1)
        cv2.circle(imagen, (x, y), radio_pequeno, color_pequeno, -1)
        cv2.circle(imagen, (x - 50, y), radio_pequeno, color_pequeno, -1)
        cv2.circle(imagen, (x + 50, y), radio_pequeno, color_pequeno, -1)

    # Cerrar la ventana
    cv2.destroyAllWindows()

# Ejecutar la función para mover la figura
mover_figura()