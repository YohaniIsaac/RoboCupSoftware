#!/usr/bin/env python3
"""Script para calibrar la transformación de perspectiva de la cámara.

Este script permite seleccionar las 4 esquinas de la cancha en la imagen
de la cámara para aplicar una transformación de perspectiva y obtener
una vista rectificada (birds-eye view).

Uso:
    python scripts/calibrate_perspective.py

Controles:
    - Click en 4 puntos para definir las esquinas de la cancha
    - Orden: Top-Left, Top-Right, Bottom-Right, Bottom-Left
    - R: Resetear puntos
    - ENTER: Guardar configuración
    - ESC: Salir sin guardar
"""
import cv2
import sys
import numpy as np
from pathlib import Path

# Agregar src al path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from robot_soccer.config import (CAMERA_PERSPECTIVE_SRC_POINTS,
                                  CAMERA_PERSPECTIVE_WIDTH,
                                  CAMERA_PERSPECTIVE_HEIGHT)

# Variables globales para el callback del mouse
points = []
window_name = 'Calibracion_Perspectiva'  # Sin espacios ni caracteres especiales


def mouse_callback(event, x, y, flags, param):
    """Callback para capturar clicks del mouse."""
    global points

    if event == cv2.EVENT_LBUTTONDOWN:
        if len(points) < 4:
            points.append((x, y))
            print(f"Punto {len(points)}: ({x}, {y})")

            if len(points) == 4:
                print("\n✅ 4 puntos seleccionados")
                print("   Presiona ENTER para guardar o R para resetear")


def draw_points(frame, pts):
    """Dibuja los puntos y líneas en el frame."""
    frame_copy = frame.copy()
    labels = ['TL (Top-Left)', 'TR (Top-Right)', 'BR (Bottom-Right)', 'BL (Bottom-Left)']
    colors = [(0, 255, 0), (255, 255, 0), (0, 165, 255), (255, 0, 255)]

    # Dibujar líneas entre puntos
    if len(pts) > 1:
        for i in range(len(pts)):
            if len(pts) == 4:
                pt1 = pts[i]
                pt2 = pts[(i + 1) % 4]
                cv2.line(frame_copy, pt1, pt2, (0, 255, 0), 2)
            elif i < len(pts) - 1:
                cv2.line(frame_copy, pts[i], pts[i + 1], (0, 255, 0), 2)

    # Dibujar puntos y labels
    for i, point in enumerate(pts):
        color = colors[i] if i < len(colors) else (0, 255, 0)
        label = labels[i] if i < len(labels) else f"P{i+1}"

        cv2.circle(frame_copy, point, 8, color, -1)
        cv2.circle(frame_copy, point, 10, (255, 255, 255), 2)
        cv2.putText(frame_copy, f"{i+1}: {label}",
                   (point[0] + 15, point[1] - 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    # Instrucciones
    if len(pts) < 4:
        next_label = labels[len(pts)] if len(pts) < len(labels) else "punto"
        cv2.putText(frame_copy, f"Click en esquina {len(pts) + 1}: {next_label}",
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    else:
        cv2.putText(frame_copy, "4 puntos OK - ENTER=guardar R=resetear ESC=salir",
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return frame_copy


def show_transformed_preview(original, src_pts, dst_width, dst_height):
    """Muestra un preview de la transformación de perspectiva."""
    if len(src_pts) != 4:
        return None

    src_points = np.float32(src_pts)
    dst_points = np.float32([
        [0, 0],
        [dst_width - 1, 0],
        [dst_width - 1, dst_height - 1],
        [0, dst_height - 1]
    ])

    matrix = cv2.getPerspectiveTransform(src_points, dst_points)
    warped = cv2.warpPerspective(original, matrix, (dst_width, dst_height))

    return warped


def calibrate_perspective(camera_id=2):
    """Calibra la transformación de perspectiva interactivamente."""
    global points, window_name

    print("=" * 70)
    print("CALIBRACIÓN DE TRANSFORMACIÓN DE PERSPECTIVA")
    print("=" * 70)
    print("\nInstrucciones:")
    print("  1. Haz click en las 4 ESQUINAS de la cancha en este orden:")
    print("     - Top-Left (esquina superior izquierda)")
    print("     - Top-Right (esquina superior derecha)")
    print("     - Bottom-Right (esquina inferior derecha)")
    print("     - Bottom-Left (esquina inferior izquierda)")
    print("\n  2. Después de seleccionar los 4 puntos:")
    print("     ENTER - Guardar configuración")
    print("     R     - Resetear puntos")
    print("     ESC   - Salir sin guardar")
    print("=" * 70)

    # Abrir cámara
    cap = cv2.VideoCapture(camera_id)

    if not cap.isOpened():
        print(f"\n❌ Error: No se pudo abrir la cámara {camera_id}")
        print("Asegúrate de que droidcam-cli esté corriendo:")
        print("  cd AlgortimosBasicos/ArucoTag && ./start_droidcam.sh")
        return

    # Obtener dimensiones
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f"\n✅ Cámara abierta: {width}x{height}")
    print("\nEsperando clicks en las esquinas de la cancha...")
    print("(Asegúrate de que toda la cancha sea visible)")

    # Inicializar con puntos existentes si los hay
    if len(CAMERA_PERSPECTIVE_SRC_POINTS) == 4:
        print(f"\nPuntos actuales: {CAMERA_PERSPECTIVE_SRC_POINTS}")
        print("Puedes hacer click para redefinirlos o presionar ENTER para mantenerlos")
        points = list(CAMERA_PERSPECTIVE_SRC_POINTS)

    # Capturar primer frame estable
    print("\n📸 Capturando frame...")
    for _ in range(10):
        cap.read()

    ret, current_frame = cap.read()
    if not ret:
        print("❌ Error leyendo frame")
        cap.release()
        return

    print("✅ Frame capturado - comenzando calibración...")
    print("\nHaz click en las 4 esquinas de la cancha")

    # Crear ventana y configurar callback
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_callback)

    preview_window = 'Preview_Transformada'
    warped_preview = None

    while True:
        # Dibujar puntos actuales
        display_frame = draw_points(current_frame, points)
        cv2.imshow(window_name, display_frame)

        # Actualizar preview si hay 4 puntos
        if len(points) == 4:
            warped_preview = show_transformed_preview(
                current_frame,
                points,
                CAMERA_PERSPECTIVE_WIDTH,
                CAMERA_PERSPECTIVE_HEIGHT
            )
            if warped_preview is not None:
                cv2.imshow(preview_window, warped_preview)

        key = cv2.waitKey(50) & 0xFF

        # Resetear
        if key == ord('r') or key == ord('R'):
            points = []
            cv2.destroyWindow(preview_window)
            print("\n🔄 Puntos reseteados - selecciona 4 nuevas esquinas")

        # Guardar
        elif key == 13:  # ENTER
            if len(points) == 4:
                print("\n" + "=" * 70)
                print("GUARDANDO CONFIGURACIÓN")
                print("=" * 70)
                save_perspective_config(points, CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT)
                break
            else:
                print(f"\n⚠️  Necesitas seleccionar 4 puntos (tienes {len(points)})")

        # Salir sin guardar
        elif key == 27:  # ESC
            print("\n❌ Calibración cancelada - no se guardaron cambios")
            break

    cap.release()
    cv2.destroyAllWindows()


def save_perspective_config(pts, dst_width, dst_height):
    """Guarda la configuración de perspectiva en config.py."""
    config_path = Path(__file__).parent.parent / "src" / "robot_soccer" / "config.py"

    # Leer archivo
    with open(config_path, 'r') as f:
        lines = f.readlines()

    # Modificar líneas
    new_lines = []
    skip_until_bracket = False

    for i, line in enumerate(lines):
        if line.startswith("CAMERA_PERSPECTIVE_ENABLED ="):
            new_lines.append("CAMERA_PERSPECTIVE_ENABLED = True  # Habilitar/deshabilitar transformación de perspectiva\n")
        elif line.startswith("CAMERA_PERSPECTIVE_SRC_POINTS ="):
            # Escribir los puntos en formato multilínea
            new_lines.append("CAMERA_PERSPECTIVE_SRC_POINTS = [\n")
            new_lines.append(f"    {pts[0]},      # Top-left (esquina superior izquierda)\n")
            new_lines.append(f"    {pts[1]},     # Top-right (esquina superior derecha)\n")
            new_lines.append(f"    {pts[2]},    # Bottom-right (esquina inferior derecha)\n")
            new_lines.append(f"    {pts[3]}       # Bottom-left (esquina inferior izquierda)\n")
            new_lines.append("]\n")
            skip_until_bracket = True
        elif skip_until_bracket:
            if line.strip() == "]":
                skip_until_bracket = False
            continue  # Skip lines until we find the closing bracket
        elif line.startswith("CAMERA_PERSPECTIVE_WIDTH ="):
            new_lines.append(f"CAMERA_PERSPECTIVE_WIDTH = {dst_width}   # Ancho de la imagen transformada\n")
        elif line.startswith("CAMERA_PERSPECTIVE_HEIGHT ="):
            new_lines.append(f"CAMERA_PERSPECTIVE_HEIGHT = {dst_height}  # Alto de la imagen transformada\n")
        else:
            new_lines.append(line)

    # Escribir archivo
    with open(config_path, 'w') as f:
        f.writelines(new_lines)

    print(f"\n✅ Configuración guardada en: {config_path}")
    print(f"   Puntos: {pts}")
    print(f"   Tamaño destino: {dst_width}x{dst_height}")
    print("\n📋 Los puntos representan las esquinas de la cancha en este orden:")
    print("   1. Top-Left (superior izquierda)")
    print("   2. Top-Right (superior derecha)")
    print("   3. Bottom-Right (inferior derecha)")
    print("   4. Bottom-Left (inferior izquierda)")
    print("\nPuedes usar esta configuración ejecutando:")
    print("   python -m robot_soccer --perception --camera")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Calibrar transformación de perspectiva")
    parser.add_argument('--camera-id', type=int, default=2,
                       help='ID de la cámara (default: 2 para DroidCam)')
    args = parser.parse_args()

    calibrate_perspective(args.camera_id)
