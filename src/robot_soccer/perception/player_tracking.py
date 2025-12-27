"""Módulo para la detección y seguimiento de jugadores mediante marcadores ArUco.

Este módulo proporciona funcionalidades para detectar robots jugadores en el campo
utilizando marcadores ArUco. Incluye procesamiento de imagen para identificar
la posición, orientación y delimitar las esquinas de cada robot detectado.
"""
import logging
import math
import numpy as np
import cv2 as cv
from robot_soccer.config import (COLOR_VERDE, COLOR_AZUL_CV,
                                  ROBOT_DETECTION_HALF_WIDTH, ROBOT_DETECTION_HALF_HEIGHT,
                                  ROBOT_ORIENTATION_LINE_LENGTH)

log = logging.getLogger(__name__)

##############################
# BUSQUEDA DE LLOS JUGADORES #
##############################


def deteccion_jugadores_aruco_tag(frame, use_camera=False, allowed_ids=None):
    """Detecta jugadores mediante marcadores ArUco y calcula su posición y orientación.

    Esta función procesa una imagen para identificar marcadores ArUco que representan
    robots jugadores. Para cada marcador detectado, calcula su posición central,
    ángulo de orientación y las esquinas de un rectángulo que representa al robot.
    También dibuja visualizaciones sobre la imagen original.

    Args:
        frame (numpy.ndarray): Imagen BGR de entrada donde se buscarán los marcadores.
            Debe ser una matriz numpy con forma (height, width, 3).
        use_camera (bool): Si True usa diccionario DICT_6X6_1000 (cámara física),
            si False usa DICT_7X7_1000 (simulación). Default: False.
        allowed_ids (list or set, optional): Lista/set de IDs válidos de marcadores.
            Si se especifica, solo se detectarán marcadores con estos IDs.
            Default: None (detecta todos los IDs).

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
        - Utiliza el diccionario ArUco DICT_6X6_1000 (cámara) o DICT_7X7_1000 (simulación)
        - El rectángulo del robot tiene dimensiones fijas de 104x140 píxeles
        - Las esquinas se calculan rotadas según la orientación del marcador
        - Si no se detectan marcadores, retorna la imagen original y una lista vacía
    """
    # Convertir a escala de grises
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # ===== PRE-PROCESAMIENTO PARA MEJORAR DETECCIÓN EN MOVIMIENTO =====
    if use_camera:  # Solo aplicar en cámara real (puede causar problemas en simulación)
        # 1. Sharpening para contrarrestar motion blur
        kernel_sharpen = np.array([[-1, -1, -1],
                                   [-1,  9, -1],
                                   [-1, -1, -1]])
        gray = cv.filter2D(gray, -1, kernel_sharpen)

        # 2. CLAHE (Contrast Limited Adaptive Histogram Equalization)
        # Mejora contraste local sin amplificar ruido
        clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray)

        # 3. Denoise ligero (bilateral filter preserva bordes)
        gray = cv.bilateralFilter(gray, 5, 50, 50)

        log.debug("Pre-procesamiento aplicado: sharpening + CLAHE + denoise")

    # Seleccionar diccionario según el modo
    if use_camera:
        aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_1000)
        log.debug("Usando diccionario DICT_5X5_1000 (cámara física)")
    else:
        aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_7X7_1000)
        log.debug("Usando diccionario DICT_7X7_1000 (simulación)")

    parameters = cv.aruco.DetectorParameters()

    # ===== PARÁMETROS OPTIMIZADOS PARA DETECCIÓN EN MOVIMIENTO =====
    # 1. Adaptive threshold: Ventanas más grandes para manejar blur
    parameters.adaptiveThreshWinSizeMin = 5  # Aumentado de 3
    parameters.adaptiveThreshWinSizeMax = 51  # Aumentado de 23 (más tolerante)
    parameters.adaptiveThreshWinSizeStep = 10

    # 2. Perímetro del marcador: Más tolerante
    parameters.minMarkerPerimeterRate = 0.01  # Reducido de 0.03 (permite marcadores más pequeños)
    parameters.maxMarkerPerimeterRate = 6.0   # Aumentado de 4.0 (permite marcadores más grandes)

    # 3. Detección de esquinas: Más agresiva
    parameters.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX  # Refinamiento subpixel
    parameters.cornerRefinementWinSize = 3  # Ventana pequeña para refinamiento
    parameters.cornerRefinementMaxIterations = 50  # Más iteraciones
    parameters.cornerRefinementMinAccuracy = 0.01  # Más tolerante

    # 4. Umbral de bits erróneos: Más permisivo (crítico para motion blur)
    parameters.errorCorrectionRate = 0.8  # Permite hasta 80% de corrección de errores

    # 5. Perspectiva: Más tolerante a deformaciones
    parameters.perspectiveRemovePixelPerCell = 6  # Más píxeles por celda
    parameters.perspectiveRemoveIgnoredMarginPerCell = 0.10  # Ignorar 10% del margen

    # 6. Detección de marcadores: Más agresiva
    parameters.minDistanceToBorder = 1  # Reducido para detectar cerca de bordes
    parameters.markerBorderBits = 1  # Marcadores 5x5 tienen 1 bit de borde

    # 7. Umbral de detección: Más permisivo
    parameters.minOtsuStdDev = 2.0  # Reducido para aceptar más candidatos
    parameters.polygonalApproxAccuracyRate = 0.08  # Más tolerante en aproximación

    log.debug("Parámetros ArUco optimizados para detección en movimiento")

    detector = cv.aruco.ArucoDetector(aruco_dict, parameters)

    corners, ids, _ = detector.detectMarkers(gray)

    # Convertir allowed_ids a set para búsqueda rápida
    if allowed_ids is not None:
        allowed_ids_set = set(allowed_ids) if not isinstance(allowed_ids, set) else allowed_ids
    else:
        allowed_ids_set = None

    # Log de depuración
    if ids is not None:
        detected_ids = ids.flatten()
        log.debug("Detectados %d marcadores: IDs = %s", len(ids), detected_ids)
    else:
        log.debug("No se detectaron marcadores")

    datos = []

    if ids is not None:
        for corner, aruco_id in zip(corners, ids):
            identificador = aruco_id[0]

            # ===== FILTRO DE IDs PERMITIDOS =====
            # Solo procesar marcadores con IDs válidos
            if allowed_ids_set is not None and identificador not in allowed_ids_set:
                continue  # Saltar este marcador silenciosamente

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
            des_x = ROBOT_DETECTION_HALF_WIDTH
            des_y = ROBOT_DETECTION_HALF_HEIGHT
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
                int(center_x + ROBOT_ORIENTATION_LINE_LENGTH * np.cos(angle)),
                int(center_y + ROBOT_ORIENTATION_LINE_LENGTH * np.sin(angle)),
            )
            cv.line(frame, (int(center_x), int(center_y)), end_point, COLOR_VERDE, 2)
    return frame, datos
