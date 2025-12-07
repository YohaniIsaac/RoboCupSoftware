#!/usr/bin/env python3
"""Script para calibrar el rango de color HSV para detectar la pelota.

Este script te permite ajustar interactivamente los rangos de color HSV
usando trackbars (deslizantes) para optimizar la detección de la pelota.

Uso:
    python scripts/calibrate_ball_color.py [--camera-id 2]

Controles:
    - Trackbars: Ajustar rangos H, S, V (min y max)
    - R: Resetear a valores por defecto (naranja)
    - ENTER: Guardar configuración en config.py
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
from robot_soccer.config import RANGO_COLOR_NARANJO


# Nombres de ventanas (constantes)
WINDOW_ORIGINAL = "1. Original - Camera Feed"
WINDOW_MASK = "2. Mask - Deteccion Binaria"
WINDOW_RESULT = "3. Result - Pelota Detectada"
WINDOW_CONTROLS = "Controles HSV"

# Variables globales para trackbars
# pylint: disable=invalid-name,global-statement
h_min = 0
h_max = 179
s_min = 0
s_max = 255
v_min = 0
v_max = 255


def nothing(_):
    """Callback vacío para trackbars."""


def create_color_preview(h_min_val, h_max_val, s_min_val, s_max_val, v_min_val, v_max_val):
    """Crea una imagen de preview del rango de color HSV actual."""
    # Imagen de controles (más alta para acomodar la barra de color)
    img = np.zeros((300, 500, 3), dtype=np.uint8)

    # Instrucciones
    cv2.putText(img, "Ajusta los rangos HSV", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(img, "Objetivo: Solo pelota en blanco", (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
    cv2.putText(img, "R = Reset | ENTER = Guardar | ESC = Salir", (10, 85),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Título de la barra de color
    cv2.putText(img, "Rango de Color Detectado:", (10, 120),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # Crear barra de color HSV (gradiente del rango)
    bar_height = 60
    bar_width = 480
    bar_x = 10
    bar_y = 130

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
                (10, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    # Leyenda
    cv2.putText(img, "H = Matiz (color)", (10, 250),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 200, 255), 1)
    cv2.putText(img, "S = Saturacion (intensidad)", (10, 270),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 200, 255), 1)
    cv2.putText(img, "V = Valor (brillo)", (10, 290),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 200, 255), 1)

    return img


def reset_to_default():
    """Resetea los valores a los del config actual."""
    global h_min, h_max, s_min, s_max, v_min, v_max

    # Obtener valores de config
    (h_min_cfg, s_min_cfg, v_min_cfg), (h_max_cfg, s_max_cfg, v_max_cfg) = RANGO_COLOR_NARANJO

    h_min, s_min, v_min = h_min_cfg, s_min_cfg, v_min_cfg
    h_max, s_max, v_max = h_max_cfg, s_max_cfg, v_max_cfg

    # Actualizar trackbars
    cv2.setTrackbarPos("H Min", WINDOW_CONTROLS, h_min)
    cv2.setTrackbarPos("H Max", WINDOW_CONTROLS, h_max)
    cv2.setTrackbarPos("S Min", WINDOW_CONTROLS, s_min)
    cv2.setTrackbarPos("S Max", WINDOW_CONTROLS, s_max)
    cv2.setTrackbarPos("V Min", WINDOW_CONTROLS, v_min)
    cv2.setTrackbarPos("V Max", WINDOW_CONTROLS, v_max)

    print("✅ Valores reseteados a los de config.py")


def calibrate_ball_color(camera_id=2):
    """Calibra el rango de color HSV para detectar la pelota."""
    global h_min, h_max, s_min, s_max, v_min, v_max

    print("=" * 70)
    print("CALIBRACIÓN DE COLOR - DETECCIÓN DE PELOTA")
    print("=" * 70)
    print("\nControles:")
    print("  Trackbars    - Ajustar rangos HSV")
    print("  R            - Resetear a valores por defecto")
    print("  ENTER        - Guardar configuración en config.py")
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
        return

    print("\n✅ Cámara abierta")
    print("\nAjusta los trackbars hasta que solo la pelota sea blanca en 'Mask'")
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

        # Guardar
        elif key == 13:  # ENTER
            print("\n" + "=" * 70)
            print("GUARDANDO CONFIGURACIÓN")
            print("=" * 70)
            save_color_config(h_min, h_max, s_min, s_max, v_min, v_max)
            break

        # Salir sin guardar
        elif key == 27:  # ESC
            print("\n❌ Calibración cancelada - no se guardaron cambios")
            break

    cap.release()
    cv2.destroyAllWindows()


def save_color_config(h_min_val, h_max_val, s_min_val, s_max_val, v_min_val, v_max_val):
    """Guarda la configuración del rango de color en config.py."""
    config_path = Path(__file__).parent.parent / "src" / "robot_soccer" / "config.py"

    # Leer archivo
    with open(config_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    # Modificar línea de RANGO_COLOR_NARANJO
    new_lines = []
    for line in lines:
        if line.strip().startswith("RANGO_COLOR_NARANJO ="):
            # Reemplazar con nuevos valores
            new_line = (f"RANGO_COLOR_NARANJO = (({h_min_val}, {s_min_val}, {v_min_val}), "
                       f"({h_max_val}, {s_max_val}, {v_max_val}))"
                       "  # Rango HSV para pelota naranja\n")
            new_lines.append(new_line)
            print("\n✅ Configuración actualizada:")
            print(f"   H (Hue):        {h_min_val} - {h_max_val}")
            print(f"   S (Saturation): {s_min_val} - {s_max_val}")
            print(f"   V (Value):      {v_min_val} - {v_max_val}")
        else:
            new_lines.append(line)

    # Escribir archivo
    with open(config_path, 'w', encoding='utf-8') as f:
        f.writelines(new_lines)

    print(f"\n✅ Guardado en: {config_path}")
    print("\nPuedes probar la detección con:")
    print("  python examples/test_ball_detection_rate.py --frames 50")
    print("  python examples/test_perception.py")


def main():
    """Función principal."""
    parser = argparse.ArgumentParser(
        description="Calibrar rango de color HSV para detectar la pelota"
    )
    parser.add_argument(
        '--camera-id',
        type=int,
        default=2,
        help='ID de la cámara (default: 2 para DroidCam)'
    )

    args = parser.parse_args()
    calibrate_ball_color(args.camera_id)


if __name__ == "__main__":
    main()
