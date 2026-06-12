"""Módulo para la detección y seguimiento de jugadores mediante marcadores ArUco.

Este módulo proporciona funcionalidades para detectar robots jugadores en el campo
utilizando marcadores ArUco. Incluye procesamiento de imagen para identificar
la posición, orientación y delimitar las esquinas de cada robot detectado.

Uso principal:
    # Crear detector una vez
    detector = create_aruco_detector(use_camera=True)

    # Reutilizar en cada frame
    frame, datos = deteccion_jugadores_aruco_tag(frame, detector, allowed_ids={0,1,2,3})
"""
import logging
import math
import numpy as np
import cv2 as cv
from robot_soccer.config import (
    COLOR_VERDE, COLOR_AZUL_CV,
    ROBOT_DETECTION_HALF_WIDTH, ROBOT_DETECTION_HALF_HEIGHT,
    ROBOT_ORIENTATION_LINE_LENGTH,
    # ArUco config
    ARUCO_DICTIONARY_CAMERA, ARUCO_DICTIONARY_SIM,
    ARUCO_ADAPTIVE_THRESH_WIN_SIZE_MIN, ARUCO_ADAPTIVE_THRESH_WIN_SIZE_MAX,
    ARUCO_ADAPTIVE_THRESH_WIN_SIZE_STEP,
    ARUCO_MIN_MARKER_PERIMETER_RATE, ARUCO_MAX_MARKER_PERIMETER_RATE,
    ARUCO_CORNER_REFINEMENT_METHOD, ARUCO_ERROR_CORRECTION_RATE,
    ARUCO_PERSPECTIVE_REMOVE_PX_PER_CELL, ARUCO_PERSPECTIVE_REMOVE_IGNORED_MARGIN,
    ARUCO_MIN_DISTANCE_TO_BORDER, ARUCO_MARKER_BORDER_BITS,
    ARUCO_MIN_OTSU_STD_DEV, ARUCO_POLYGONAL_APPROX_ACCURACY_RATE,
    # Corrección de paralaje y escala métrica
    CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT,
    PARALLAX_FACTOR, PARALLAX_CENTER_X, PARALLAX_CENTER_Y,
    FIELD_PHYSICAL_WIDTH_CM, FIELD_PHYSICAL_HEIGHT_CM,
)

log = logging.getLogger(__name__)

##############################
# BUSQUEDA DE LOS JUGADORES #
##############################


def create_aruco_detector(use_camera=False):
    """Crea un detector ArUco reutilizable con parámetros de config.py.

    Debe llamarse UNA sola vez al inicio del programa. El detector devuelto
    se reutiliza entre frames para evitar recrearlo (~0.5ms de ahorro/frame).

    Args:
        use_camera (bool): Si True usa el diccionario configurado para cámara real
            (ARUCO_DICTIONARY_CAMERA en config.py), si False usa ARUCO_DICTIONARY_SIM
            para simulación. Default: False.

    Returns:
        cv2.aruco.ArucoDetector: Detector listo para reutilizar con detectMarkers().
    """
    if use_camera:
        aruco_dict = cv.aruco.getPredefinedDictionary(ARUCO_DICTIONARY_CAMERA)
        log.info("Detector ArUco creado con diccionario de cámara (const=%d)", ARUCO_DICTIONARY_CAMERA)
    else:
        aruco_dict = cv.aruco.getPredefinedDictionary(ARUCO_DICTIONARY_SIM)
        log.info("Detector ArUco creado con diccionario de simulación (const=%d)", ARUCO_DICTIONARY_SIM)

    params = cv.aruco.DetectorParameters()

    # Parámetros centralizados desde config.py
    params.adaptiveThreshWinSizeMin = ARUCO_ADAPTIVE_THRESH_WIN_SIZE_MIN
    params.adaptiveThreshWinSizeMax = ARUCO_ADAPTIVE_THRESH_WIN_SIZE_MAX
    params.adaptiveThreshWinSizeStep = ARUCO_ADAPTIVE_THRESH_WIN_SIZE_STEP
    params.minMarkerPerimeterRate = ARUCO_MIN_MARKER_PERIMETER_RATE
    params.maxMarkerPerimeterRate = ARUCO_MAX_MARKER_PERIMETER_RATE
    params.cornerRefinementMethod = ARUCO_CORNER_REFINEMENT_METHOD
    params.errorCorrectionRate = ARUCO_ERROR_CORRECTION_RATE
    params.perspectiveRemovePixelPerCell = ARUCO_PERSPECTIVE_REMOVE_PX_PER_CELL
    params.perspectiveRemoveIgnoredMarginPerCell = ARUCO_PERSPECTIVE_REMOVE_IGNORED_MARGIN
    params.minDistanceToBorder = ARUCO_MIN_DISTANCE_TO_BORDER
    params.markerBorderBits = ARUCO_MARKER_BORDER_BITS
    params.minOtsuStdDev = ARUCO_MIN_OTSU_STD_DEV
    params.polygonalApproxAccuracyRate = ARUCO_POLYGONAL_APPROX_ACCURACY_RATE

    return cv.aruco.ArucoDetector(aruco_dict, params)


def deteccion_jugadores_aruco_tag(frame, detector, allowed_ids=None, draw=True):
    """Detecta jugadores mediante marcadores ArUco y calcula su posición y orientación.

    Esta función procesa una imagen para identificar marcadores ArUco que representan
    robots jugadores. Para cada marcador detectado, calcula su posición central,
    ángulo de orientación y las esquinas de un rectángulo que representa al robot.

    Args:
        frame (numpy.ndarray): Imagen BGR de entrada donde se buscarán los marcadores.
        detector (cv2.aruco.ArucoDetector): Detector pre-creado con create_aruco_detector().
        allowed_ids (list, set or None): IDs válidos de marcadores a detectar.
            Si None, detecta todos los IDs. Default: None.
        draw (bool): Si True, dibuja visualizaciones sobre el frame. Default: True.

    Returns:
        tuple: (frame, datos) donde datos es lista de dicts con claves:
            - 'id' (int): Identificador del marcador ArUco
            - 'x' (int): Coordenada X del centro del robot
            - 'y' (int): Coordenada Y del centro del robot
            - 'angulo' (float): Ángulo de orientación en grados
            - 'esquinas' (list): Lista de tuplas (x, y) con las 4 esquinas rotadas
    """
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    corners, ids, _ = detector.detectMarkers(gray)

    # Convertir allowed_ids a set para búsqueda rápida
    if allowed_ids is not None:
        allowed_ids_set = set(allowed_ids) if not isinstance(allowed_ids, set) else allowed_ids
    else:
        allowed_ids_set = None

    if ids is not None:
        log.debug("Detectados %d marcadores: IDs = %s", len(ids), ids.flatten())
    else:
        log.debug("No se detectaron marcadores")

    datos = []

    if ids is not None:
        for corner, aruco_id in zip(corners, ids):
            identificador = aruco_id[0]

            # Filtro de IDs permitidos
            if allowed_ids_set is not None and identificador not in allowed_ids_set:
                continue

            corner_points = corner.reshape(4, 2)

            # Centro del marcador (en float para la corrección de paralaje)
            center_x = float(np.mean(corner_points[:, 0]))
            center_y = float(np.mean(corner_points[:, 1]))

            # Corrección de paralaje: el marker está elevado sobre el campo y la
            # cámara desplaza los objetos elevados hacia afuera del nadir. Se
            # corrige con un modelo radial cuyo factor y centro están calibrados
            # empíricamente en config.py (PARALLAX_*); el centro no es el de la imagen.
            center_x = center_x - (center_x - PARALLAX_CENTER_X) * PARALLAX_FACTOR
            center_y = center_y - (center_y - PARALLAX_CENTER_Y) * PARALLAX_FACTOR
            center_x = int(center_x)
            center_y = int(center_y)

            # Ángulo de orientación (vector esquina 0 → esquina 1).
            # Se normaliza a espacio métrico antes del atan2 para eliminar la
            # anisotropía del warpPerspective: sin esta corrección, atan2 en
            # píxeles produce hasta ~7° de error cuando el campo físico no tiene
            # la misma proporción que la imagen warped (150x88cm vs 640x480px).
            vector_1 = corner_points[1] - corner_points[0]
            dx_metric = vector_1[0] / CAMERA_PERSPECTIVE_WIDTH  * FIELD_PHYSICAL_WIDTH_CM
            dy_metric = vector_1[1] / CAMERA_PERSPECTIVE_HEIGHT * FIELD_PHYSICAL_HEIGHT_CM
            angle = np.arctan2(dy_metric, dx_metric)
            angle_deg = np.degrees(angle)

            # Rectángulo que representa al robot (rotado según orientación)
            des_x = ROBOT_DETECTION_HALF_WIDTH
            des_y = ROBOT_DETECTION_HALF_HEIGHT
            esquinas = [
                (center_x - des_x, center_y + des_y),
                (center_x + des_x, center_y + des_y),
                (center_x + des_x, center_y - des_y),
                (center_x - des_x, center_y - des_y),
            ]

            cos_a, sin_a = math.cos(angle), math.sin(angle)
            list_puntos_rotados = []
            for punto in esquinas:
                x_d = punto[0] - center_x
                y_d = punto[1] - center_y
                list_puntos_rotados.append((
                    int(center_x + x_d * cos_a - y_d * sin_a),
                    int(center_y + x_d * sin_a + y_d * cos_a),
                ))

            datos.append({
                "id": identificador,
                "x": center_x,
                "y": center_y,
                "angulo": angle_deg,
                "esquinas": list_puntos_rotados,
            })

            if draw:
                for i in range(4):
                    cv.line(frame, list_puntos_rotados[i],
                            list_puntos_rotados[(i + 1) % 4], COLOR_VERDE, 2)
                    cv.circle(frame, list_puntos_rotados[i], 5, COLOR_AZUL_CV, -1)
                cv.circle(frame, (center_x, center_y), 5, COLOR_VERDE, -1)
                end_point = (
                    int(center_x + ROBOT_ORIENTATION_LINE_LENGTH * np.cos(angle)),
                    int(center_y + ROBOT_ORIENTATION_LINE_LENGTH * np.sin(angle)),
                )
                cv.line(frame, (center_x, center_y), end_point, COLOR_VERDE, 2)

    return frame, datos
