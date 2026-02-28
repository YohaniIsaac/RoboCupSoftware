#!/usr/bin/env python3
"""Identificador visual de marcadores ArUco para orientar y asignar a robots.

Muestra en tiempo real cada marcador detectado con:
- ID del marcador y equipo asignado (rojo/azul)
- Flecha de orientación (hacia dónde "mira" el marcador)
- Esquina 0 marcada con un círculo (referencia de orientación)
- Color del equipo en el borde del marcador

La esquina 0 (círculo) indica el origen del marcador. La flecha muestra
la dirección "frente" del robot. Usa esto para pegar los marcadores
en los robots con la orientación correcta.

Uso:
    python basic_test/test_marker_identification.py
    python basic_test/test_marker_identification.py --camera-id 0
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

# Agregar src al path
ROOT_DIR = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))

from robot_soccer.config import (
    ARUCO_ROBOT_IDS,
    ARUCO_TEAM_RED_IDS,
    ARUCO_TEAM_BLUE_IDS,
)
from robot_soccer.perception.player_tracking import create_aruco_detector

# Colores BGR
RED = (0, 0, 230)
BLUE = (230, 100, 0)
GREEN = (0, 200, 0)
WHITE = (255, 255, 255)
YELLOW = (0, 230, 255)
GRAY = (150, 150, 150)


def get_team_info(marker_id):
    """Retorna (nombre_equipo, color_bgr) según el ID del marcador."""
    if marker_id in ARUCO_TEAM_RED_IDS:
        return "ROJO", RED
    if marker_id in ARUCO_TEAM_BLUE_IDS:
        return "AZUL", BLUE
    return "???", GRAY


def draw_marker_info(frame, corners, marker_id):
    """Dibuja información de identificación y orientación sobre el marcador."""
    pts = corners.reshape(4, 2).astype(int)
    center = pts.mean(axis=0).astype(int)
    cx, cy = center

    team_name, team_color = get_team_info(marker_id)

    # Borde del marcador con color de equipo
    cv2.polylines(frame, [pts], True, team_color, 3)

    # Esquina 0: círculo grande (referencia de orientación)
    cv2.circle(frame, tuple(pts[0]), 8, YELLOW, -1)
    cv2.putText(frame, "C0", (pts[0][0] + 10, pts[0][1] - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, YELLOW, 1, cv2.LINE_AA)

    # Esquinas 1, 2, 3: círculos pequeños
    for i in range(1, 4):
        cv2.circle(frame, tuple(pts[i]), 4, WHITE, -1)

    # Flecha de orientación (esquina 0 → esquina 1 = "frente")
    vec_front = pts[1].astype(float) - pts[0].astype(float)
    vec_front_norm = vec_front / (np.linalg.norm(vec_front) + 1e-6)
    arrow_len = 60
    arrow_end = (int(cx + vec_front_norm[0] * arrow_len),
                 int(cy + vec_front_norm[1] * arrow_len))
    cv2.arrowedLine(frame, (cx, cy), arrow_end, GREEN, 3, tipLength=0.3)

    # Ángulo en grados
    angle_deg = np.degrees(np.arctan2(vec_front[1], vec_front[0]))

    # Etiqueta con ID, equipo y ángulo
    label = f"ID {marker_id} - {team_name}"
    angle_label = f"{angle_deg:.1f} grados"

    # Fondo semitransparente para el texto
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
    text_x = cx - tw // 2
    text_y = cy - 40
    cv2.rectangle(frame, (text_x - 4, text_y - th - 4),
                  (text_x + tw + 4, text_y + 4), (0, 0, 0), -1)
    cv2.putText(frame, label, (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, team_color, 2, cv2.LINE_AA)

    # Ángulo debajo
    (tw2, th2), _ = cv2.getTextSize(angle_label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    text_x2 = cx - tw2 // 2
    text_y2 = cy + 50
    cv2.rectangle(frame, (text_x2 - 3, text_y2 - th2 - 3),
                  (text_x2 + tw2 + 3, text_y2 + 3), (0, 0, 0), -1)
    cv2.putText(frame, angle_label, (text_x2, text_y2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 1, cv2.LINE_AA)


def draw_legend(frame):
    """Dibuja leyenda en la parte inferior del frame."""
    h, w = frame.shape[:2]
    y_base = h - 80

    # Fondo
    cv2.rectangle(frame, (0, y_base - 10), (w, h), (20, 20, 20), -1)

    # Asignación de equipos
    cv2.putText(frame, f"Equipo ROJO: IDs {ARUCO_TEAM_RED_IDS}",
                (10, y_base + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, RED, 1, cv2.LINE_AA)
    cv2.putText(frame, f"Equipo AZUL: IDs {ARUCO_TEAM_BLUE_IDS}",
                (10, y_base + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, BLUE, 1, cv2.LINE_AA)

    # Instrucciones
    cv2.putText(frame, "Circulo amarillo = Esquina 0 | Flecha verde = Frente del robot | 'q' = salir",
                (10, y_base + 65), cv2.FONT_HERSHEY_SIMPLEX, 0.4, GRAY, 1, cv2.LINE_AA)


def main(camera_id):
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"Error: no se pudo abrir la camara {camera_id}")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    detector = create_aruco_detector(use_camera=True)
    allowed_ids = set(ARUCO_ROBOT_IDS)

    print(f"Camara {camera_id} abierta. Mostrando marcadores AprilTag_16h5.")
    print(f"  Equipo ROJO: IDs {ARUCO_TEAM_RED_IDS}")
    print(f"  Equipo AZUL: IDs {ARUCO_TEAM_BLUE_IDS}")
    print("Presiona 'q' para salir.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            for corner, mid in zip(corners, ids):
                if mid[0] in allowed_ids:
                    draw_marker_info(frame, corner, mid[0])

        # Contador de detectados
        detected = 0 if ids is None else sum(1 for mid in ids if mid[0] in allowed_ids)
        cv2.putText(frame, f"Detectados: {detected}/{len(ARUCO_ROBOT_IDS)}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, WHITE, 2, cv2.LINE_AA)

        draw_legend(frame)
        cv2.imshow("Identificador de Marcadores", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Identificador visual de marcadores ArUco")
    parser.add_argument('--camera-id', type=int, default=2, help='ID de camara (default: 2)')
    args = parser.parse_args()
    main(args.camera_id)
