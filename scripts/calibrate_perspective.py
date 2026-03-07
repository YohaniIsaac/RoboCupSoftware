#!/usr/bin/env python3
"""Script para calibrar la transformación de perspectiva de la cámara.

Este script permite seleccionar las 4 esquinas de la cancha en la imagen
de la cámara para aplicar una transformación de perspectiva y obtener
una vista rectificada (birds-eye view).

Paso 2 (opcional): Calibrar posiciones de arcos en la imagen transformada.

Uso:
    python scripts/calibrate_perspective.py

Controles:
    Paso 1 (esquinas):
    - Click en 4 puntos para definir las esquinas de la cancha
    - Orden: Top-Left, Top-Right, Bottom-Right, Bottom-Left
    - R: Resetear puntos
    - ENTER: Confirmar y pasar a paso 2
    - ESC: Salir sin guardar

    Paso 2 (arcos):
    - Click en 4 puntos para definir los postes de los arcos
    - Orden: Arco izq top, Arco izq bottom, Arco der top, Arco der bottom
    - R: Resetear puntos de arcos
    - ENTER: Guardar todo
    - S: Saltar calibración de arcos (guardar solo perspectiva)
    - ESC: Salir sin guardar
"""
import sys
from pathlib import Path

import cv2
import numpy as np

# Agregar src al path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from robot_soccer.config import (  # pylint: disable=wrong-import-position
    CAMERA_PERSPECTIVE_SRC_POINTS,
    CAMERA_PERSPECTIVE_WIDTH,
    CAMERA_PERSPECTIVE_HEIGHT
)

# Constantes
WINDOW_NAME = 'Calibracion_Perspectiva'  # Sin espacios ni caracteres especiales


def create_mouse_callback(points_list, max_points=4):
    """Crea un callback del mouse que usa una lista mutable."""
    def mouse_callback(event, x, y, flags, param):  # pylint: disable=unused-argument
        """Callback para capturar clicks del mouse."""
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(points_list) < max_points:
                points_list.append((x, y))
                print(f"Punto {len(points_list)}: ({x}, {y})")

                if len(points_list) == max_points:
                    print(f"\n{max_points} puntos seleccionados")
                    print("   Presiona ENTER para confirmar o R para resetear")
    return mouse_callback


def draw_points(frame, pts):
    """Dibuja los puntos y líneas en el frame."""
    frame_copy = frame.copy()
    labels = ['TL (Top-Left)', 'TR (Top-Right)', 'BR (Bottom-Right)', 'BL (Bottom-Left)']
    colors = [(0, 255, 0), (255, 255, 0), (0, 165, 255), (255, 0, 255)]

    # Dibujar líneas entre puntos
    if len(pts) > 1:
        for i, _ in enumerate(pts):
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


def draw_goal_points(frame, pts):
    """Dibuja los puntos de los arcos en la imagen transformada."""
    frame_copy = frame.copy()
    labels = ['Arco Izq TOP', 'Arco Izq BOTTOM', 'Arco Der TOP', 'Arco Der BOTTOM']
    colors = [(0, 0, 255), (0, 0, 200), (255, 0, 0), (200, 0, 0)]

    # Dibujar líneas de los arcos
    if len(pts) >= 2:
        cv2.line(frame_copy, pts[0], pts[1], (0, 0, 255), 2)  # Arco izquierdo
    if len(pts) >= 4:
        cv2.line(frame_copy, pts[2], pts[3], (255, 0, 0), 2)  # Arco derecho

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
        cv2.putText(frame_copy, f"PASO 2: Click en {next_label} ({len(pts)+1}/4)",
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame_copy, "S=saltar arcos  R=resetear  ESC=cancelar",
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    else:
        cv2.putText(frame_copy, "Arcos OK - ENTER=guardar R=resetear ESC=cancelar",
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


def calibrate_goals(warped_frame, dst_width, dst_height):
    """Paso 2: Calibrar posiciones de arcos en la imagen transformada.

    Args:
        warped_frame: Imagen transformada (birds-eye view)
        dst_width: Ancho de la imagen transformada
        dst_height: Alto de la imagen transformada

    Returns:
        list or None: Lista de 4 puntos [(x,y),...] de postes,
                      o None si se saltó la calibración
    """
    print("\n" + "=" * 70)
    print("PASO 2: CALIBRACIÓN DE POSICIONES DE ARCOS")
    print("=" * 70)
    print("\nEn la imagen transformada, haz click en los 4 postes:")
    print("  1. Arco IZQUIERDO - poste SUPERIOR (top)")
    print("  2. Arco IZQUIERDO - poste INFERIOR (bottom)")
    print("  3. Arco DERECHO - poste SUPERIOR (top)")
    print("  4. Arco DERECHO - poste INFERIOR (bottom)")
    print("\n  S     - Saltar (usar valores por defecto)")
    print("  R     - Resetear puntos")
    print("  ENTER - Confirmar y guardar")
    print("  ESC   - Cancelar todo")
    print("=" * 70)

    goal_window = 'Calibracion_Arcos'
    cv2.namedWindow(goal_window)

    goal_points = []
    callback = create_mouse_callback(goal_points, max_points=4)
    cv2.setMouseCallback(goal_window, callback)

    while True:
        display = draw_goal_points(warped_frame, goal_points)
        cv2.imshow(goal_window, display)

        key = cv2.waitKey(50) & 0xFF

        # Resetear
        if key == ord('r') or key == ord('R'):
            goal_points.clear()
            print("\nPuntos de arcos reseteados")

        # Saltar calibración de arcos
        if key == ord('s') or key == ord('S'):
            print("\nSaltando calibración de arcos (se usarán valores por defecto)")
            cv2.destroyWindow(goal_window)
            return None

        # Confirmar
        if key == 13:  # ENTER
            if len(goal_points) == 4:
                cv2.destroyWindow(goal_window)
                return goal_points
            print(f"\nNecesitas 4 puntos (tienes {len(goal_points)})")

        # Cancelar
        if key == 27:  # ESC
            cv2.destroyWindow(goal_window)
            return "cancel"

    return None


def calibrate_perspective(camera_id=2):
    """Calibra la transformación de perspectiva interactivamente."""
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
    print("     ENTER - Confirmar y pasar a calibración de arcos")
    print("     R     - Resetear puntos")
    print("     ESC   - Salir sin guardar")
    print("=" * 70)

    # Abrir cámara
    cap = cv2.VideoCapture(camera_id)

    if not cap.isOpened():
        print(f"\nError: No se pudo abrir la camara {camera_id}")
        print("Asegurate de que droidcam-cli este corriendo:")
        print("  cd AlgortimosBasicos/ArucoTag && ./start_droidcam.sh")
        return

    # Obtener dimensiones
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f"\nCamara abierta: {width}x{height}")
    print("\nEsperando clicks en las esquinas de la cancha...")
    print("(Asegurate de que toda la cancha sea visible)")

    # Inicializar puntos
    points = []
    if len(CAMERA_PERSPECTIVE_SRC_POINTS) == 4:
        print(f"\nPuntos actuales: {CAMERA_PERSPECTIVE_SRC_POINTS}")
        print("Puedes hacer click para redefinirlos o presionar ENTER para mantenerlos")
        points = list(CAMERA_PERSPECTIVE_SRC_POINTS)

    # Capturar primer frame estable
    print("\nCapturando frame...")
    for _ in range(10):
        cap.read()

    ret, current_frame = cap.read()
    if not ret:
        print("Error leyendo frame")
        cap.release()
        return

    print("Frame capturado - comenzando calibracion...")
    print("\nHaz click en las 4 esquinas de la cancha")

    # Crear ventana y configurar callback
    cv2.namedWindow(WINDOW_NAME)
    callback = create_mouse_callback(points)
    cv2.setMouseCallback(WINDOW_NAME, callback)

    preview_window = 'Preview_Transformada'
    warped_preview = None

    while True:
        # Dibujar puntos actuales
        display_frame = draw_points(current_frame, points)
        cv2.imshow(WINDOW_NAME, display_frame)

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
            points.clear()
            cv2.destroyWindow(preview_window)
            print("\nPuntos reseteados - selecciona 4 nuevas esquinas")
            print(f"   Puntos actuales: {len(points)} (esperando clicks...)")

        # Confirmar paso 1 y pasar a paso 2
        if key == 13:  # ENTER
            if len(points) == 4:
                print("\nPaso 1 completado - esquinas confirmadas")

                # Cerrar ventanas del paso 1
                cv2.destroyWindow(WINDOW_NAME)
                cv2.destroyWindow(preview_window)

                # Generar imagen transformada para paso 2
                warped_frame = show_transformed_preview(
                    current_frame, points,
                    CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT
                )

                # Paso 2: Calibrar arcos
                goal_result = calibrate_goals(
                    warped_frame,
                    CAMERA_PERSPECTIVE_WIDTH,
                    CAMERA_PERSPECTIVE_HEIGHT
                )

                if goal_result == "cancel":
                    print("\nCalibracion cancelada - no se guardaron cambios")
                    break

                # Guardar configuración
                print("\n" + "=" * 70)
                print("GUARDANDO CONFIGURACION")
                print("=" * 70)
                save_perspective_config(
                    points, CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT,
                    goal_points=goal_result
                )
                break
            print(f"\nNecesitas seleccionar 4 puntos (tienes {len(points)})")

        # Salir sin guardar
        if key == 27:  # ESC
            print("\nCalibracion cancelada - no se guardaron cambios")
            break

    cap.release()
    cv2.destroyAllWindows()


def save_perspective_config(pts, dst_width, dst_height, goal_points=None):
    """Guarda la configuración de perspectiva y arcos en config.py.

    Args:
        pts: Lista de 4 puntos de esquinas
        dst_width: Ancho destino
        dst_height: Alto destino
        goal_points: Lista de 4 puntos de postes [izq_top, izq_bot, der_top, der_bot]
                     o None para usar valores por defecto
    """
    config_path = Path(__file__).parent.parent / "src" / "robot_soccer" / "config.py"

    # Leer archivo
    with open(config_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    # Preparar valores de FIELD_CAM
    if goal_points is not None:
        goal_left_x = goal_points[0][0]  # X del arco izquierdo (ambos postes tienen ~mismo X)
        goal_left_top_y = goal_points[0][1]
        goal_left_bottom_y = goal_points[1][1]
        goal_right_x = goal_points[2][0]  # X del arco derecho
        goal_right_top_y = goal_points[2][1]
        goal_right_bottom_y = goal_points[3][1]
    else:
        # Valores por defecto
        goal_left_x = 0
        goal_left_top_y = (dst_height - 100) // 2
        goal_left_bottom_y = (dst_height + 100) // 2
        goal_right_x = dst_width
        goal_right_top_y = (dst_height - 100) // 2
        goal_right_bottom_y = (dst_height + 100) // 2

    # Modificar líneas
    new_lines = []
    skip_until_bracket = False
    skip_field_cam = False

    for line in lines:
        if line.startswith("CAMERA_PERSPECTIVE_ENABLED ="):
            new_lines.append(
                "CAMERA_PERSPECTIVE_ENABLED = True  "
                "# Habilitar/deshabilitar transformacion de perspectiva\n"
            )
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
        elif line.startswith("FIELD_CAM = FieldGeometry("):
            # Reemplazar FIELD_CAM completo
            new_lines.append("FIELD_CAM = FieldGeometry(\n")
            new_lines.append(f"    width=CAMERA_PERSPECTIVE_WIDTH,\n")
            new_lines.append(f"    height=CAMERA_PERSPECTIVE_HEIGHT,\n")
            new_lines.append(f"    goal_left_x={goal_left_x},\n")
            new_lines.append(f"    goal_left_top_y={goal_left_top_y},\n")
            new_lines.append(f"    goal_left_bottom_y={goal_left_bottom_y},\n")
            new_lines.append(f"    goal_right_x={goal_right_x},\n")
            new_lines.append(f"    goal_right_top_y={goal_right_top_y},\n")
            new_lines.append(f"    goal_right_bottom_y={goal_right_bottom_y},\n")
            new_lines.append(f"    margin=15,\n")
            new_lines.append(")\n")
            skip_field_cam = True
        elif skip_field_cam:
            if line.strip() == ")":
                skip_field_cam = False
            continue
        else:
            new_lines.append(line)

    # Escribir archivo
    with open(config_path, 'w', encoding='utf-8') as f:
        f.writelines(new_lines)

    print(f"\nConfiguracion guardada en: {config_path}")
    print(f"   Puntos perspectiva: {pts}")
    print(f"   Tamano destino: {dst_width}x{dst_height}")
    if goal_points is not None:
        print(f"   Arco izquierdo: x={goal_left_x}, y=[{goal_left_top_y}, {goal_left_bottom_y}]")
        print(f"   Arco derecho: x={goal_right_x}, y=[{goal_right_top_y}, {goal_right_bottom_y}]")
    else:
        print("   Arcos: valores por defecto (no calibrados)")
    print("\nPuedes usar esta configuracion ejecutando:")
    print("   python -m robot_soccer --perception --camera")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Calibrar transformacion de perspectiva")
    parser.add_argument('--camera-id', type=int, default=2,
                       help='ID de la camara (default: 2 para DroidCam)')
    args = parser.parse_args()

    calibrate_perspective(args.camera_id)
