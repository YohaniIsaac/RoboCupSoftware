#!/usr/bin/env python3
"""Ajuste interactivo (al tanteo) de la corrección de paralaje del marcador.

El marcador ArUco está elevado ~8,5 cm sobre el plano del campo, así que su
proyección se desplaza hacia afuera respecto del centro real del robot. La
función de producción corrige esto con un modelo radial desde el centro de la
imagen (en deteccion_jugadores_aruco_tag):

    corregido = centro - (centro - C) * pf,   con  pf = h/H  y  C = (320, 240)

Ese modelo es exacto solo si la cámara mira perpendicular al campo y está
centrada sobre él. Con inclinación o descentrada, la corrección queda bien en
el centro (donde el desplazamiento es ~0) y se desvía hacia los bordes.

Este script REPLICA el pipeline real (undistort -> warp -> ArUco) y aplica la
misma corrección con tres parámetros ajustables por trackbar:

    pf x1000 : factor de paralaje x1000 (default ~ 1000*h/H)
    cx       : centro x del modelo radial (default 320)
    cy       : centro y del modelo radial (default 240)

Coloca el robot en distintos puntos, sobre todo en los bordes, y mueve los
sliders hasta que el punto verde (corregido) y el recuadro queden sobre el
centro real del robot. No modifica código fuente ni config: solo imprime los
valores ajustados para que decidas cómo aplicarlos.

Uso:
    cd ~/git/RoboCupSoftware
    python scripts/calibrate_parallax.py
    python scripts/calibrate_parallax.py --camera-id 2

Teclas: P = imprimir valores | R = reset | ESC = salir.
"""
import sys
import math
import argparse
import logging
from pathlib import Path

ROOT_DIR = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))

import cv2
import numpy as np

from robot_soccer.config import (  # pylint: disable=wrong-import-position
    CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH,
    CAMERA_PERSPECTIVE_ENABLED, CAMERA_PERSPECTIVE_SRC_POINTS,
    PARALLAX_FACTOR, PARALLAX_CENTER_X, PARALLAX_CENTER_Y,
    ROBOT_DETECTION_HALF_WIDTH, ROBOT_DETECTION_HALF_HEIGHT,
    FIELD_PHYSICAL_WIDTH_CM, FIELD_PHYSICAL_HEIGHT_CM,
)
from robot_soccer.perception.player_tracking import create_aruco_detector  # noqa: E402
from robot_soccer.utils.camera_undistort import load_intrinsics, undistort_frame  # noqa: E402
from robot_soccer.utils.camera_utils import get_camera_index  # noqa: E402

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-8s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

WINDOW = "Ajuste de paralaje"
ROBOT_IDS = {0, 1, 2, 3}
DEFAULT_PF_X1000 = int(round(1000 * PARALLAX_FACTOR))   # arranca desde lo desplegado
DEFAULT_CX = PARALLAX_CENTER_X
DEFAULT_CY = PARALLAX_CENTER_Y

# Escala por eje (mm/px) del marco rectificado, para reportar el desplazamiento.
_MM_PER_PX_X = FIELD_PHYSICAL_WIDTH_CM * 10.0 / CAMERA_PERSPECTIVE_WIDTH
_MM_PER_PX_Y = FIELD_PHYSICAL_HEIGHT_CM * 10.0 / CAMERA_PERSPECTIVE_HEIGHT


def correct_parallax(rx, ry, cx, cy, pf):
    """Modelo radial de paralaje, idéntico al de producción pero con C y pf libres."""
    return rx - (rx - cx) * pf, ry - (ry - cy) * pf


def build_perspective_matrix():
    """Matriz de perspectiva idéntica a la del pipeline de captura, o None."""
    if not CAMERA_PERSPECTIVE_ENABLED:
        return None
    src = np.float32(CAMERA_PERSPECTIVE_SRC_POINTS)
    dst = np.float32([
        [0, 0],
        [CAMERA_PERSPECTIVE_WIDTH - 1, 0],
        [CAMERA_PERSPECTIVE_WIDTH - 1, CAMERA_PERSPECTIVE_HEIGHT - 1],
        [0, CAMERA_PERSPECTIVE_HEIGHT - 1],
    ])
    return cv2.getPerspectiveTransform(src, dst)


def marker_angle(pts):
    """Ángulo del marcador (esquina 0 -> 1) con normalización métrica, como la fuente."""
    v = pts[1] - pts[0]
    dx = v[0] / CAMERA_PERSPECTIVE_WIDTH * FIELD_PHYSICAL_WIDTH_CM
    dy = v[1] / CAMERA_PERSPECTIVE_HEIGHT * FIELD_PHYSICAL_HEIGHT_CM
    return math.atan2(dy, dx)


def draw_box(view, cx, cy, ang, color):
    """Recuadro rotado del robot, igual que deteccion_jugadores_aruco_tag."""
    dx, dy = ROBOT_DETECTION_HALF_WIDTH, ROBOT_DETECTION_HALF_HEIGHT
    base = [(-dx, dy), (dx, dy), (dx, -dy), (-dx, -dy)]
    ca, sa = math.cos(ang), math.sin(ang)
    rot = [(int(cx + x * ca - y * sa), int(cy + x * sa + y * ca)) for x, y in base]
    for i in range(4):
        cv2.line(view, rot[i], rot[(i + 1) % 4], color, 2)


def _noop(_):
    pass


def main():
    parser = argparse.ArgumentParser(
        description="Ajuste interactivo (al tanteo) de la corrección de paralaje."
    )
    parser.add_argument("--camera-id", type=int, default=None,
                        help="ID de cámara (auto-detecta DroidCam si se omite).")
    args = parser.parse_args()

    camera_id = args.camera_id
    if camera_id is None:
        camera_id = get_camera_index(prefer_droidcam=True, fallback_index=0)
        log.info("Cámara auto-detectada: /dev/video%d", camera_id)
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        log.error("No se pudo abrir la cámara %d", camera_id)
        return

    K, D = load_intrinsics()
    matrix = build_perspective_matrix()
    detector = create_aruco_detector(use_camera=True)

    cv2.namedWindow(WINDOW)
    cv2.createTrackbar("pf x1000", WINDOW, DEFAULT_PF_X1000, 200, _noop)
    cv2.createTrackbar("cx", WINDOW, DEFAULT_CX, CAMERA_PERSPECTIVE_WIDTH, _noop)
    cv2.createTrackbar("cy", WINDOW, DEFAULT_CY, CAMERA_PERSPECTIVE_HEIGHT, _noop)

    print("=" * 64)
    print("  AJUSTE DE PARALAJE (al tanteo)")
    print(f"  Default: pf={DEFAULT_PF_X1000 / 1000:.3f}  cx={DEFAULT_CX}  cy={DEFAULT_CY}")
    print("  Mueve el robot a los bordes y ajusta los sliders hasta que el punto")
    print("  verde (corregido) quede sobre el centro real del robot.")
    print("  P=imprimir valores | R=reset | ESC=salir")
    print("=" * 64)

    while True:
        ret, raw = cap.read()
        if not ret:
            continue
        raw = undistort_frame(raw, K, D)
        frame = cv2.warpPerspective(
            raw, matrix, (CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT)
        ) if matrix is not None else raw

        pf = cv2.getTrackbarPos("pf x1000", WINDOW) / 1000.0
        cx = cv2.getTrackbarPos("cx", WINDOW)
        cy = cv2.getTrackbarPos("cy", WINDOW)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        view = frame.copy()

        cv2.drawMarker(view, (cx, cy), (255, 255, 0), cv2.MARKER_CROSS, 18, 1)
        cv2.putText(view, "centro paralaje", (cx + 8, cy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

        if ids is not None:
            for corner, aruco_id in zip(corners, ids):
                rid = int(aruco_id[0])
                if rid not in ROBOT_IDS:
                    continue
                pts = corner.reshape(4, 2)
                rx = float(np.mean(pts[:, 0]))
                ry = float(np.mean(pts[:, 1]))
                ox, oy = correct_parallax(rx, ry, cx, cy, pf)
                ang = marker_angle(pts)

                draw_box(view, int(ox), int(oy), ang, (0, 255, 0))
                cv2.circle(view, (int(rx), int(ry)), 4, (0, 0, 255), -1)   # crudo (rojo)
                cv2.circle(view, (int(ox), int(oy)), 5, (0, 255, 0), -1)   # corregido (verde)
                cv2.line(view, (int(rx), int(ry)), (int(ox), int(oy)), (0, 255, 0), 1)

                shift_mm = math.hypot((ox - rx) * _MM_PER_PX_X, (oy - ry) * _MM_PER_PX_Y)
                cv2.putText(view, f"id{rid}  dif {shift_mm:.0f}mm", (int(ox) + 8, int(oy) + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.putText(view, f"pf={pf:.3f}  cx={cx}  cy={cy}", (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(view, "rojo=crudo  verde=corregido    P=imprimir  R=reset  ESC=salir",
                    (10, CAMERA_PERSPECTIVE_HEIGHT - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        cv2.imshow(WINDOW, view)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        if key in (ord("p"), ord("P")):
            print(f"  Calibrado -> PARALLAX_FACTOR = {pf:.4f}  "
                  f"PARALLAX_CENTER_X = {cx}  PARALLAX_CENTER_Y = {cy}")
        if key in (ord("r"), ord("R")):
            cv2.setTrackbarPos("pf x1000", WINDOW, DEFAULT_PF_X1000)
            cv2.setTrackbarPos("cx", WINDOW, DEFAULT_CX)
            cv2.setTrackbarPos("cy", WINDOW, DEFAULT_CY)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
