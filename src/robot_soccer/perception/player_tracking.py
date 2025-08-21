"""Módulo para la detección y seguimiento de jugadores mediante marcadores ArUco.

Este módulo proporciona funcionalidades para detectar robots jugadores en el campo
utilizando marcadores ArUco. Incluye procesamiento de imagen para identificar
la posición, orientación y delimitar las esquinas de cada robot detectado.
"""
import math
import numpy as np
import cv2 as cv
from robot_soccer.config import COLOR_VERDE, COLOR_AZUL_CV

##############################
# BUSQUEDA DE LLOS JUGADORES #
##############################


def deteccion_jugadores_aruco_tag(frame):
    """Detecta jugadores mediante marcadores ArUco y calcula su posición y orientación.

    Esta función procesa una imagen para identificar marcadores ArUco que representan
    robots jugadores. Para cada marcador detectado, calcula su posición central,
    ángulo de orientación y las esquinas de un rectángulo que representa al robot.
    También dibuja visualizaciones sobre la imagen original.

    Args:
        frame (numpy.ndarray): Imagen BGR de entrada donde se buscarán los marcadores.
            Debe ser una matriz numpy con forma (height, width, 3).

    Returns:
        tuple: Una tupla conteniendo:
            - frame (numpy.ndarray): Imagen procesada con visualizaciones dibujadas
              incluyendo líneas verdes para los contornos de los robots, círculos
              azules en las esquinas y líneas verdes indicando la orientación.
            - datos (list): Lista de diccionarios, cada uno representando un jugador
              detectado con las siguientes claves:
              - 'id' (int): Identificador del marcador ArUco
              - 'x' (int): Coordenada X del centro del robot
              - 'y' (int): Coordenada Y del centro del robot
              - 'angulo' (float): Ángulo de orientación en grados
              - 'esquinas' (list): Lista de tuplas (x, y) con las 4 esquinas
                rotadas del rectángulo que representa al robot

    Note:
        - Utiliza el diccionario ArUco DICT_7X7_1000 para la detección
        - El rectángulo del robot tiene dimensiones fijas de 104x140 píxeles
        - Las esquinas se calculan rotadas según la orientación del marcador
        - Si no se detectan marcadores, retorna la imagen original y una lista vacía
    """
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_7X7_1000)

    parameters = cv.aruco.DetectorParameters()
    detector = cv.aruco.ArucoDetector(aruco_dict, parameters)

    corners, ids, _ = detector.detectMarkers(gray)
    datos = []

    if ids is not None:
        for corner, aruco_id in zip(corners, ids):
            # corners[i] tiene la forma [4, 1, 2], con 4 esquinas, 1 array por esquina y 2 coordenadas (x, y)
            corner_points = corner.reshape(
                4, 2
            )  # Aplanar la matriz para obtener las esquinas

            # Calcular el centro (promedio de las coordenadas de las esquinas)
            center_x = int(np.mean(corner_points[:, 0]))
            center_y = int(np.mean(corner_points[:, 1]))

            # Calcular el ángulo de rotación
            # Usaremos las primeras dos esquinas para calcular el ángulo
            # Se asume que las esquinas están ordenadas de manera consistente
            vector_1 = corner_points[1] - corner_points[0]
            angle = np.arctan2(vector_1[1], vector_1[0])
            angle_deg = np.degrees(angle)

            identificador = aruco_id[0]

            # para el rectángulo que representa al robot
            des_x = 52
            des_y = 70
            esquinas = [
                (center_x - des_x, center_y + des_y),  # Esquina superior izquierda
                (center_x + des_x, center_y + des_y),  # Esquina superior derecha
                (center_x + des_x, center_y - des_y),  # Esquina inferior izquierda
                (center_x - des_x, center_y - des_y),  # Esquina inferior derecha
            ]
            list_puntos_rotados = []

            for punto in esquinas:
                x_desplazado, y_desplazado = punto[0] - center_x, punto[1] - center_y

                # Aplicar la matriz de rotacion
                x_rotado = x_desplazado * math.cos(angle) - y_desplazado * math.sin(
                    angle
                )
                y_rotado = x_desplazado * math.sin(angle) + y_desplazado * math.cos(
                    angle
                )

                list_puntos_rotados.append(
                    (int(center_x + x_rotado), int(center_y + y_rotado))
                )
            datos.append(
                {
                    "id": identificador,
                    "x": center_x,
                    "y": center_y,
                    "angulo": angle_deg,
                    "esquinas": list_puntos_rotados,
                }
            )

            for i in range(4):
                cv.line(
                    frame,
                    list_puntos_rotados[i],
                    list_puntos_rotados[(i + 1) % 4],
                    COLOR_VERDE,
                    2,
                )
                cv.circle(frame, list_puntos_rotados[i], 5, COLOR_AZUL_CV, -1)
            # Dibujar el centro y la orientación en la imagen
            cv.circle(frame, (int(center_x), int(center_y)), 5, COLOR_VERDE, -1)
            end_point = (
                int(center_x + 50 * np.cos(angle)),
                int(center_y + 50 * np.sin(angle)),
            )
            cv.line(frame, (int(center_x), int(center_y)), end_point, COLOR_VERDE, 2)
    return frame, datos
