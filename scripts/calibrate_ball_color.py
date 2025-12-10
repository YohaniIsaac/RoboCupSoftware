#!/usr/bin/env python3
"""Script para calibrar el rango de color HSV y parámetros de detección de la pelota.

Este script te permite ajustar interactivamente:
  FASE 1: Rangos de color HSV para filtrar la pelota
  FASE 2: Parámetros de detección (morfología y círculos)

Uso:
    python scripts/calibrate_ball_color.py [--camera-id 2]

Controles FASE 1 (Color HSV):
    - Trackbars: Ajustar rangos H, S, V (min y max)
    - R: Resetear a valores por defecto
    - N: Siguiente fase (calibración de detección)
    - ESC: Salir sin guardar

Controles FASE 2 (Detección):
    - Trackbars: Ajustar morfología y parámetros de círculos
    - R: Resetear a valores por defecto
    - ENTER: Guardar configuración en config.py
    - B: Volver a fase 1 (color)
    - ESC: Salir sin guardar
"""
import sys
import argparse
from pathlib import Path

import cv2
import numpy as np

# Agregar src al path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

# pylint: disable=wrong-import-position
from robot_soccer.config import (
    RANGO_COLOR_NARANJO,
    BALL_DETECTION_KERNEL_SIZE,
    BALL_DETECTION_MORPH_ITERATIONS,
    BALL_DETECTION_HOUGH_PARAM1,
    BALL_DETECTION_HOUGH_PARAM2,
    BALL_DETECTION_MIN_RADIUS,
    BALL_DETECTION_MAX_RADIUS,
    CAMERA_PERSPECTIVE_ENABLED,
    CAMERA_PERSPECTIVE_SRC_POINTS,
    CAMERA_PERSPECTIVE_WIDTH,
    CAMERA_PERSPECTIVE_HEIGHT,
)


# Nombres de ventanas (constantes)
WINDOW_ORIGINAL = "1. Original - Camera Feed"
WINDOW_MASK = "2. Mask - Deteccion Binaria"
WINDOW_RESULT = "3. Result - Pelota Detectada"
WINDOW_CONTROLS = "Controles HSV"

# Variables globales para trackbars - FASE 1 (Color HSV)
# pylint: disable=invalid-name,global-statement
h_min = 0
h_max = 179
s_min = 0
s_max = 255
v_min = 0
v_max = 255

# Variables globales para trackbars - FASE 2 (Detección)
kernel_size = 5  # Tamaño del kernel morfológico (debe ser impar)
morph_iterations = 1  # Iteraciones de apertura/cierre
hough_param1 = 50  # Umbral superior para detección de bordes Canny
hough_param2 = 30  # Umbral de acumulador para detección de círculos
min_radius = 5  # Radio mínimo del círculo
max_radius = 100  # Radio máximo del círculo


def get_perspective_matrix():
    """Calcula la matriz de transformación de perspectiva.

    Returns:
        numpy.ndarray or None: Matriz de perspectiva si está habilitada, None si no.
    """
    if not CAMERA_PERSPECTIVE_ENABLED:
        return None

    # Puntos de origen (trapecio en la imagen de la cámara)
    src_points = np.float32(CAMERA_PERSPECTIVE_SRC_POINTS)

    # Puntos de destino (rectángulo perfecto)
    dst_points = np.float32([
        [0, 0],                                              # Top-left
        [CAMERA_PERSPECTIVE_WIDTH - 1, 0],                   # Top-right
        [CAMERA_PERSPECTIVE_WIDTH - 1, CAMERA_PERSPECTIVE_HEIGHT - 1],  # Bottom-right
        [0, CAMERA_PERSPECTIVE_HEIGHT - 1]                   # Bottom-left
    ])

    # Calcular matriz de transformación
    return cv2.getPerspectiveTransform(src_points, dst_points)


def apply_perspective_transform(frame, perspective_matrix):
    """Aplica transformación de perspectiva al frame si está habilitada.

    Args:
        frame: Frame original de la cámara.
        perspective_matrix: Matriz de transformación (None si está deshabilitada).

    Returns:
        numpy.ndarray: Frame transformado o frame original si la perspectiva está deshabilitada.
    """
    if CAMERA_PERSPECTIVE_ENABLED and perspective_matrix is not None:
        return cv2.warpPerspective(
            frame,
            perspective_matrix,
            (CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT)
        )
    return frame


def nothing(_):
    """Callback vacío para trackbars."""


def create_color_preview(h_min_val, h_max_val, s_min_val, s_max_val, v_min_val, v_max_val):
    """Crea una imagen de preview del rango de color HSV actual."""
    # Imagen de controles (más alta para acomodar la barra de color)
    img = np.zeros((300, 500, 3), dtype=np.uint8)

    # Instrucciones
    cv2.putText(img, "FASE 1: Ajusta rangos HSV", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.putText(img, "Objetivo: Solo pelota en blanco", (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
    cv2.putText(img, "R = Reset | N = Siguiente Fase", (10, 85),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(img, "ESC = Salir sin guardar", (10, 110),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Título de la barra de color
    cv2.putText(img, "Rango de Color Detectado:", (10, 140),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # Crear barra de color HSV (gradiente del rango)
    bar_height = 50
    bar_width = 480
    bar_x = 10
    bar_y = 150

    # Crear imagen HSV con el rango seleccionado
    color_bar_hsv = np.zeros((bar_height, bar_width, 3), dtype=np.uint8)

    # Llenar con gradiente de H (de h_min a h_max)
    h_range = h_max_val - h_min_val if h_max_val > h_min_val else 1
    for i in range(bar_width):
        # Calcular H en el rango
        h_val = h_min_val + int((i / bar_width) * h_range)
        # Usar valores medios-altos de S y V para mostrar el color claramente
        s_val = (s_min_val + s_max_val) // 2
        v_val = max(v_max_val, 200)  # Al menos 200 para que se vea bien

        color_bar_hsv[:, i] = [h_val, s_val, v_val]

    # Convertir a BGR para mostrar
    color_bar_bgr = cv2.cvtColor(color_bar_hsv, cv2.COLOR_HSV2BGR)

    # Insertar la barra en la imagen
    img[bar_y:bar_y+bar_height, bar_x:bar_x+bar_width] = color_bar_bgr

    # Dibujar borde alrededor de la barra
    cv2.rectangle(img, (bar_x, bar_y), (bar_x+bar_width, bar_y+bar_height),
                  (255, 255, 255), 2)

    # Mostrar valores HSV
    cv2.putText(img, f"H: {h_min_val}-{h_max_val}  S: {s_min_val}-{s_max_val}  V: {v_min_val}-{v_max_val}",
                (10, 215), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    # Leyenda
    cv2.putText(img, "H = Matiz (color)", (10, 245),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 200, 255), 1)
    cv2.putText(img, "S = Saturacion (intensidad)", (10, 265),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 200, 255), 1)
    cv2.putText(img, "V = Valor (brillo)", (10, 285),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 200, 255), 1)

    return img


def load_hsv_from_config():
    """Carga los valores HSV del config sin actualizar trackbars."""
    global h_min, h_max, s_min, s_max, v_min, v_max

    # Obtener valores de config
    (h_min_cfg, s_min_cfg, v_min_cfg), (h_max_cfg, s_max_cfg, v_max_cfg) = RANGO_COLOR_NARANJO

    h_min, s_min, v_min = h_min_cfg, s_min_cfg, v_min_cfg
    h_max, s_max, v_max = h_max_cfg, s_max_cfg, v_max_cfg


def reset_to_default():
    """Resetea los valores a los del config actual y actualiza trackbars."""
    load_hsv_from_config()

    # Actualizar trackbars (solo si existen)
    try:
        cv2.setTrackbarPos("H Min", WINDOW_CONTROLS, h_min)
        cv2.setTrackbarPos("H Max", WINDOW_CONTROLS, h_max)
        cv2.setTrackbarPos("S Min", WINDOW_CONTROLS, s_min)
        cv2.setTrackbarPos("S Max", WINDOW_CONTROLS, s_max)
        cv2.setTrackbarPos("V Min", WINDOW_CONTROLS, v_min)
        cv2.setTrackbarPos("V Max", WINDOW_CONTROLS, v_max)
        print("✅ Valores reseteados a los de config.py")
    except cv2.error:
        # Trackbars no existen, solo cargamos valores
        print("✅ Valores HSV cargados de config.py")


def calibrate_detection_params(camera_id=2):
    """FASE 2: Calibra los parámetros de detección de círculos y morfología."""
    global kernel_size, morph_iterations, hough_param1, hough_param2, min_radius, max_radius

    print("\n" + "=" * 70)
    print("FASE 2: CALIBRACIÓN DE DETECCIÓN DE PELOTA")
    print("=" * 70)
    print("\nControles:")
    print("  Trackbars    - Ajustar parámetros de detección")
    print("  R            - Resetear a valores por defecto")
    print("  B            - Volver a calibración de color (Fase 1)")
    print("  ENTER        - Guardar configuración completa")
    print("  ESC          - Salir sin guardar")
    print("\nVentanas:")
    print("  1. Original     - Feed de cámara")
    print("  2. Mask+Morph   - Máscara con filtros morfológicos")
    print("  3. Detection    - Círculos detectados")
    print("=" * 70)

    # Ventanas para fase 2
    WINDOW_DETECTION = "3. Detection - Circulos Detectados"
    WINDOW_CONTROLS_DETECT = "Controles Deteccion"

    # Abrir cámara
    cap = cv2.VideoCapture(camera_id)

    if not cap.isOpened():
        print(f"\n❌ Error: No se pudo abrir la cámara {camera_id}")
        return False

    # Calcular matriz de perspectiva
    perspective_matrix = get_perspective_matrix()
    if CAMERA_PERSPECTIVE_ENABLED:
        print(f"\n⚠️  Transformación de perspectiva HABILITADA "
              f"({CAMERA_PERSPECTIVE_WIDTH}x{CAMERA_PERSPECTIVE_HEIGHT})")
    print("=" * 70)

    # Crear ventanas
    cv2.namedWindow(WINDOW_ORIGINAL)
    cv2.namedWindow(WINDOW_MASK)
    cv2.namedWindow(WINDOW_DETECTION)
    cv2.namedWindow(WINDOW_CONTROLS_DETECT)

    # Cargar valores guardados de config.py
    kernel_size = BALL_DETECTION_KERNEL_SIZE
    morph_iterations = BALL_DETECTION_MORPH_ITERATIONS
    hough_param1 = BALL_DETECTION_HOUGH_PARAM1
    hough_param2 = BALL_DETECTION_HOUGH_PARAM2
    min_radius = BALL_DETECTION_MIN_RADIUS
    max_radius = BALL_DETECTION_MAX_RADIUS

    # Calcular posición inicial del trackbar de kernel (debe ser impar: 1,3,5,7,9,11)
    # kernel_idx se mapea: 0→1, 1→3, 2→5, 3→7, 4→9, 5→11
    kernel_idx = (kernel_size - 1) // 2  # Convertir 1,3,5,7,9,11 de vuelta a 0-5

    # Crear trackbars con valores cargados desde config
    cv2.createTrackbar("Kernel (*2+1)", WINDOW_CONTROLS_DETECT, kernel_idx, 5, nothing)  # 0-5 → 1,3,5,7,9,11
    cv2.createTrackbar("Morph Iters", WINDOW_CONTROLS_DETECT, morph_iterations, 5, nothing)
    cv2.createTrackbar("Hough Param1", WINDOW_CONTROLS_DETECT, hough_param1, 200, nothing)
    cv2.createTrackbar("Hough Param2", WINDOW_CONTROLS_DETECT, hough_param2, 100, nothing)
    cv2.createTrackbar("Min Radius", WINDOW_CONTROLS_DETECT, min_radius, 50, nothing)
    cv2.createTrackbar("Max Radius", WINDOW_CONTROLS_DETECT, max_radius, 200, nothing)

    back_to_phase1 = False

    while True:
        # Leer frame
        ret, frame = cap.read()
        if not ret:
            print("❌ Error leyendo frame")
            break

        # Aplicar transformación de perspectiva (igual que en camera_feed.py)
        frame = apply_perspective_transform(frame, perspective_matrix)

        # Obtener valores de trackbars
        kernel_idx = cv2.getTrackbarPos("Kernel (*2+1)", WINDOW_CONTROLS_DETECT)
        kernel_size = kernel_idx * 2 + 1  # Convertir 0-5 a 1,3,5,7,9,11
        morph_iterations = cv2.getTrackbarPos("Morph Iters", WINDOW_CONTROLS_DETECT)
        hough_param1 = cv2.getTrackbarPos("Hough Param1", WINDOW_CONTROLS_DETECT)
        hough_param2 = cv2.getTrackbarPos("Hough Param2", WINDOW_CONTROLS_DETECT)
        min_radius = cv2.getTrackbarPos("Min Radius", WINDOW_CONTROLS_DETECT)
        max_radius = cv2.getTrackbarPos("Max Radius", WINDOW_CONTROLS_DETECT)

        # Convertir a HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Aplicar máscara de color (usando valores de FASE 1)
        lower_color = np.array([h_min, s_min, v_min])
        upper_color = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Aplicar operaciones morfológicas
        if kernel_size > 0 and morph_iterations > 0:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
            for _ in range(morph_iterations):
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Aplicar máscara y convertir a gris
        imagen_filtrada = cv2.bitwise_and(frame, frame, mask=mask)
        imagen_gris = cv2.cvtColor(imagen_filtrada, cv2.COLOR_BGR2GRAY)
        imagen_suavizada = cv2.GaussianBlur(imagen_gris, (5, 5), 0)

        # Detectar círculos con HoughCircles
        circulos = None
        if hough_param1 > 0 and hough_param2 > 0 and max_radius > min_radius:
            circulos = cv2.HoughCircles(
                imagen_suavizada,
                cv2.HOUGH_GRADIENT,
                1,
                minDist=20,
                param1=hough_param1,
                param2=hough_param2,
                minRadius=min_radius,
                maxRadius=max_radius
            )

        # Preparar frames para visualización
        info_frame = frame.copy()
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        detection_frame = frame.copy()

        # Dibujar círculos detectados
        num_circles = 0
        if circulos is not None:
            circulos = np.uint16(np.around(circulos))
            for i in circulos[0, :]:
                # Dibujar círculo exterior
                cv2.circle(detection_frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
                # Dibujar centro
                cv2.circle(detection_frame, (i[0], i[1]), 2, (0, 0, 255), 3)
                num_circles += 1

        # Información en frames
        cv2.putText(info_frame, "Original - Pelota en escena", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.putText(mask_bgr, "Mask + Morfologia", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(mask_bgr, f"Kernel: {kernel_size}x{kernel_size}, Iters: {morph_iterations}",
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        status_color = (0, 255, 0) if num_circles == 1 else (0, 0, 255)
        cv2.putText(detection_frame, f"Circulos Detectados: {num_circles}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        cv2.putText(detection_frame, f"P1:{hough_param1} P2:{hough_param2} R:{min_radius}-{max_radius}",
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Crear panel de controles
        controls_img = np.zeros((350, 500, 3), dtype=np.uint8)
        cv2.putText(controls_img, "FASE 2: Parametros de Deteccion", (10, 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        y_offset = 60
        cv2.putText(controls_img, f"Kernel: {kernel_size}x{kernel_size} (morfologia)", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        y_offset += 25
        cv2.putText(controls_img, f"Iteraciones: {morph_iterations}", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        y_offset += 35
        cv2.putText(controls_img, "Parametros HoughCircles:", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 255), 2)
        y_offset += 25
        cv2.putText(controls_img, f"  Param1 (Canny): {hough_param1}", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        y_offset += 25
        cv2.putText(controls_img, f"  Param2 (Acumulador): {hough_param2}", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        y_offset += 25
        cv2.putText(controls_img, f"  Radio Min: {min_radius} px", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        y_offset += 25
        cv2.putText(controls_img, f"  Radio Max: {max_radius} px", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        y_offset += 40
        result_color = (0, 255, 0) if num_circles == 1 else (0, 165, 255) if num_circles == 0 else (0, 0, 255)
        result_text = "Perfecto!" if num_circles == 1 else "No detecta" if num_circles == 0 else "Multiples!"
        cv2.putText(controls_img, f"Estado: {result_text} ({num_circles} circulos)", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, result_color, 2)

        y_offset += 35
        cv2.putText(controls_img, "Objetivo: Detectar exactamente 1 circulo", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        y_offset += 35
        cv2.putText(controls_img, "R = Reset | B = Volver Fase 1", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y_offset += 25
        cv2.putText(controls_img, "ENTER = Guardar | ESC = Salir", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Mostrar ventanas
        cv2.imshow(WINDOW_ORIGINAL, info_frame)
        cv2.imshow(WINDOW_MASK, mask_bgr)
        cv2.imshow(WINDOW_DETECTION, detection_frame)
        cv2.imshow(WINDOW_CONTROLS_DETECT, controls_img)

        # Capturar tecla
        key = cv2.waitKey(1) & 0xFF

        # Resetear
        if key == ord('r') or key == ord('R'):
            cv2.setTrackbarPos("Kernel (*2+1)", WINDOW_CONTROLS_DETECT, 2)
            cv2.setTrackbarPos("Morph Iters", WINDOW_CONTROLS_DETECT, 1)
            cv2.setTrackbarPos("Hough Param1", WINDOW_CONTROLS_DETECT, 50)
            cv2.setTrackbarPos("Hough Param2", WINDOW_CONTROLS_DETECT, 30)
            cv2.setTrackbarPos("Min Radius", WINDOW_CONTROLS_DETECT, 5)
            cv2.setTrackbarPos("Max Radius", WINDOW_CONTROLS_DETECT, 100)
            print("✅ Parámetros reseteados a valores por defecto")

        # Volver a fase 1
        elif key == ord('b') or key == ord('B'):
            print("\n🔙 Volviendo a calibración de color (Fase 1)...")
            back_to_phase1 = True
            break

        # Guardar
        elif key == 13:  # ENTER
            print("\n" + "=" * 70)
            print("GUARDANDO CONFIGURACIÓN COMPLETA")
            print("=" * 70)
            save_complete_config()
            break

        # Salir sin guardar
        elif key == 27:  # ESC
            print("\n❌ Calibración cancelada - no se guardaron cambios")
            break

    cap.release()
    cv2.destroyAllWindows()

    return back_to_phase1


def calibrate_ball_color(camera_id=2):
    """FASE 1: Calibra el rango de color HSV para detectar la pelota."""
    global h_min, h_max, s_min, s_max, v_min, v_max

    print("=" * 70)
    print("FASE 1: CALIBRACIÓN DE COLOR HSV - DETECCIÓN DE PELOTA")
    print("=" * 70)
    print("\nControles:")
    print("  Trackbars    - Ajustar rangos HSV")
    print("  R            - Resetear a valores por defecto")
    print("  N            - Siguiente fase (calibrar detección)")
    print("  ESC          - Salir sin guardar")
    print("\nVentanas:")
    print("  1. Original  - Feed de cámara")
    print("  2. Mask      - Detección binaria (blanco = pelota)")
    print("  3. Result    - Resultado con máscara aplicada")
    print("=" * 70)

    # Abrir cámara
    cap = cv2.VideoCapture(camera_id)

    if not cap.isOpened():
        print(f"\n❌ Error: No se pudo abrir la cámara {camera_id}")
        print("Asegúrate de que droidcam-cli esté corriendo:")
        print("  cd algoritmos_basicos/aruco_tag && ./start_droidcam.sh")
        return False

    print("\n✅ Cámara abierta")
    print("\nAjusta los trackbars hasta que solo la pelota sea blanca en 'Mask'")

    # Calcular matriz de perspectiva
    perspective_matrix = get_perspective_matrix()
    if CAMERA_PERSPECTIVE_ENABLED:
        print(f"\n⚠️  Transformación de perspectiva HABILITADA "
              f"({CAMERA_PERSPECTIVE_WIDTH}x{CAMERA_PERSPECTIVE_HEIGHT})")
        print("    La calibración se hará sobre la imagen transformada (igual que en detección real)")
    else:
        print("\n⚠️  Transformación de perspectiva DESHABILITADA")
    print("=" * 70)

    # Crear ventanas
    cv2.namedWindow(WINDOW_ORIGINAL)
    cv2.namedWindow(WINDOW_MASK)
    cv2.namedWindow(WINDOW_RESULT)
    cv2.namedWindow(WINDOW_CONTROLS)

    # Inicializar con valores de config
    reset_to_default()

    # Crear trackbars en ventana de controles
    cv2.createTrackbar("H Min", WINDOW_CONTROLS, h_min, 179, nothing)
    cv2.createTrackbar("H Max", WINDOW_CONTROLS, h_max, 179, nothing)
    cv2.createTrackbar("S Min", WINDOW_CONTROLS, s_min, 255, nothing)
    cv2.createTrackbar("S Max", WINDOW_CONTROLS, s_max, 255, nothing)
    cv2.createTrackbar("V Min", WINDOW_CONTROLS, v_min, 255, nothing)
    cv2.createTrackbar("V Max", WINDOW_CONTROLS, v_max, 255, nothing)

    while True:
        # Leer frame
        ret, frame = cap.read()
        if not ret:
            print("❌ Error leyendo frame")
            break

        # Aplicar transformación de perspectiva (igual que en camera_feed.py)
        frame = apply_perspective_transform(frame, perspective_matrix)

        # Obtener valores de trackbars
        h_min = cv2.getTrackbarPos("H Min", WINDOW_CONTROLS)
        h_max = cv2.getTrackbarPos("H Max", WINDOW_CONTROLS)
        s_min = cv2.getTrackbarPos("S Min", WINDOW_CONTROLS)
        s_max = cv2.getTrackbarPos("S Max", WINDOW_CONTROLS)
        v_min = cv2.getTrackbarPos("V Min", WINDOW_CONTROLS)
        v_max = cv2.getTrackbarPos("V Max", WINDOW_CONTROLS)

        # Convertir a HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Crear máscara con rango actual
        lower_color = np.array([h_min, s_min, v_min])
        upper_color = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Aplicar máscara
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Añadir información en frame original
        info_frame = frame.copy()
        cv2.putText(info_frame, "Original - Busca la pelota", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(info_frame, f"HSV: H[{h_min}-{h_max}] S[{s_min}-{s_max}] V[{v_min}-{v_max}]",
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Añadir información en máscara (convertir a BGR para mostrar texto en color)
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.putText(mask_bgr, "Mask - Solo pelota debe ser blanca", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Contar píxeles blancos (pelota detectada)
        white_pixels = cv2.countNonZero(mask)
        total_pixels = mask.shape[0] * mask.shape[1]
        percentage = (white_pixels / total_pixels) * 100

        # Indicador de calidad
        if percentage < 0.5:
            quality_text = "Muy restrictivo"
            quality_color = (0, 0, 255)  # Rojo
        elif percentage < 5:
            quality_text = "Bueno"
            quality_color = (0, 255, 0)  # Verde
        else:
            quality_text = "Muy permisivo"
            quality_color = (0, 165, 255)  # Naranja

        cv2.putText(mask_bgr, f"Cobertura: {percentage:.2f}% - {quality_text}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, quality_color, 2)

        # Añadir información en resultado
        cv2.putText(result, "Result - Pelota aislada", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # Crear imagen de controles dinámica con visualización del color
        controls_img = create_color_preview(h_min, h_max, s_min, s_max, v_min, v_max)

        # Mostrar ventanas
        cv2.imshow(WINDOW_ORIGINAL, info_frame)
        cv2.imshow(WINDOW_MASK, mask_bgr)
        cv2.imshow(WINDOW_RESULT, result)
        cv2.imshow(WINDOW_CONTROLS, controls_img)

        # Capturar tecla
        key = cv2.waitKey(1) & 0xFF

        # Resetear
        if key == ord('r') or key == ord('R'):
            reset_to_default()

        # Siguiente fase (detección)
        elif key == ord('n') or key == ord('N'):
            print("\n➡️  Pasando a calibración de detección (Fase 2)...")
            cap.release()
            cv2.destroyAllWindows()
            return True  # Continuar a fase 2

        # Salir sin guardar
        elif key == 27:  # ESC
            print("\n❌ Calibración cancelada - no se guardaron cambios")
            break

    cap.release()
    cv2.destroyAllWindows()
    return False  # No continuar a fase 2


def save_complete_config():
    """Guarda la configuración completa (HSV + parámetros de detección) en config.py."""
    config_path = Path(__file__).parent.parent / "src" / "robot_soccer" / "config.py"

    # Leer archivo
    with open(config_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    # Modificar líneas necesarias
    new_lines = []
    skip_until_next_section = False
    detection_params_found = False

    for line in lines:
        # Actualizar RANGO_COLOR_NARANJO
        if line.strip().startswith("RANGO_COLOR_NARANJO ="):
            new_line = (f"RANGO_COLOR_NARANJO = (({h_min}, {s_min}, {v_min}), "
                       f"({h_max}, {s_max}, {v_max}))"
                       "  # Rango HSV para pelota naranja\n")
            new_lines.append(new_line)
            continue

        # Actualizar o crear sección de parámetros de detección
        if line.strip().startswith("# Parámetros de detección de pelota"):
            detection_params_found = True
            new_lines.append(line)
            # Agregar parámetros actualizados
            new_lines.append(f"BALL_DETECTION_KERNEL_SIZE = {kernel_size}  "
                           f"# Tamaño del kernel morfológico\n")
            new_lines.append(f"BALL_DETECTION_MORPH_ITERATIONS = {morph_iterations}  "
                           f"# Iteraciones de apertura/cierre\n")
            new_lines.append(f"BALL_DETECTION_HOUGH_PARAM1 = {hough_param1}  # Umbral Canny\n")
            new_lines.append(f"BALL_DETECTION_HOUGH_PARAM2 = {hough_param2}  # Umbral acumulador\n")
            new_lines.append(f"BALL_DETECTION_MIN_RADIUS = {min_radius}  # Radio mínimo (px)\n")
            new_lines.append(f"BALL_DETECTION_MAX_RADIUS = {max_radius}  # Radio máximo (px)\n")
            skip_until_next_section = True
            continue

        # Saltar las viejas líneas de parámetros de detección
        if skip_until_next_section:
            if line.strip().startswith("BALL_DETECTION_"):
                continue  # Saltar líneas viejas
            if line.strip() and not line.strip().startswith("#"):
                # Nueva sección encontrada, dejar de saltar
                skip_until_next_section = False
                new_lines.append(line)
            elif line.strip().startswith("#"):
                # Comentario, podría ser nueva sección
                skip_until_next_section = False
                new_lines.append(line)
            else:
                # Línea vacía
                new_lines.append(line)
        else:
            new_lines.append(line)

    # Si no existían los parámetros de detección, agregarlos al final
    if not detection_params_found:
        new_lines.append("\n# Parámetros de detección de pelota (morfología y HoughCircles)\n")
        new_lines.append(f"BALL_DETECTION_KERNEL_SIZE = {kernel_size}  # Tamaño del kernel morfológico\n")
        new_lines.append(f"BALL_DETECTION_MORPH_ITERATIONS = {morph_iterations}  # Iteraciones de apertura/cierre\n")
        new_lines.append(f"BALL_DETECTION_HOUGH_PARAM1 = {hough_param1}  # Umbral Canny\n")
        new_lines.append(f"BALL_DETECTION_HOUGH_PARAM2 = {hough_param2}  # Umbral acumulador\n")
        new_lines.append(f"BALL_DETECTION_MIN_RADIUS = {min_radius}  # Radio mínimo (px)\n")
        new_lines.append(f"BALL_DETECTION_MAX_RADIUS = {max_radius}  # Radio máximo (px)\n")

    # Escribir archivo
    with open(config_path, 'w', encoding='utf-8') as f:
        f.writelines(new_lines)

    print("\n✅ Configuración HSV actualizada:")
    print(f"   H (Hue):        {h_min} - {h_max}")
    print(f"   S (Saturation): {s_min} - {s_max}")
    print(f"   V (Value):      {v_min} - {v_max}")

    print("\n✅ Parámetros de detección guardados:")
    print(f"   Kernel Size:       {kernel_size}x{kernel_size}")
    print(f"   Morph Iterations:  {morph_iterations}")
    print(f"   Hough Param1:      {hough_param1}")
    print(f"   Hough Param2:      {hough_param2}")
    print(f"   Min Radius:        {min_radius} px")
    print(f"   Max Radius:        {max_radius} px")

    print(f"\n✅ Guardado en: {config_path}")
    print("\nPuedes probar la detección con:")
    print("  python examples/test_ball_detection_rate.py --frames 50")
    print("  python examples/test_perception.py")


def main():
    """Función principal - maneja el flujo entre Fase 1 y Fase 2."""
    parser = argparse.ArgumentParser(
        description="Calibrar color HSV y parámetros de detección de la pelota"
    )
    parser.add_argument(
        '--camera-id',
        type=int,
        default=2,
        help='ID de la cámara (default: 2 para DroidCam)'
    )
    parser.add_argument(
        '--skip-phase1',
        action='store_true',
        help='Saltar Fase 1 (color) e ir directo a Fase 2 (detección)'
    )

    args = parser.parse_args()

    # Determinar fase inicial
    if args.skip_phase1:
        print("\n⏭️  Saltando Fase 1, iniciando en Fase 2 (Detección)...")
        # Cargar valores HSV del config (sin actualizar trackbars)
        load_hsv_from_config()
        calibrate_detection_params(args.camera_id)
    else:
        # Loop entre fases
        while True:
            # Fase 1: Calibración de color HSV
            go_to_phase2 = calibrate_ball_color(args.camera_id)

            if not go_to_phase2:
                # Usuario presionó ESC o salió
                break

            # Fase 2: Calibración de detección
            back_to_phase1 = calibrate_detection_params(args.camera_id)

            if not back_to_phase1:
                # Usuario presionó ENTER (guardar) o ESC (salir)
                break
            # Si back_to_phase1 es True, el loop continúa y vuelve a Fase 1


if __name__ == "__main__":
    main()
