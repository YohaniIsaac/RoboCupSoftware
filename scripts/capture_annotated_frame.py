#!/usr/bin/env python3
"""Captura una imagen anotada del campo con perspectiva corregida.

Sobre cada robot detectado se dibuja el rectángulo del modelo geométrico
alineado con la orientación del marcador ArUco. Sobre la pelota se
superpone el círculo detectado por la transformada de Hough. De forma
opcional se anota junto a cada rectángulo el identificador del marcador.

Este script consume el paquete `robot_soccer` como biblioteca y no
modifica su código fuente.

Uso típico (modo interactivo):
    python scripts/capture_annotated_frame.py --camera-id 2
    # SPACE = guardar frame anotado | ESC = salir

Modo de una sola captura sin ventana:
    python scripts/capture_annotated_frame.py --oneshot

Por defecto los PNG se guardan en `captures_output/` (en la raíz del
repositorio, ignorado por git mediante `*.png`).
"""
import argparse
import logging
from pathlib import Path

import cv2 as cv
import numpy as np

from robot_soccer.config import (
    CAMERA_PERSPECTIVE_ENABLED,
    CAMERA_PERSPECTIVE_SRC_POINTS,
    CAMERA_PERSPECTIVE_WIDTH,
    CAMERA_PERSPECTIVE_HEIGHT,
)
from robot_soccer.perception.player_tracking import (
    create_aruco_detector,
    deteccion_jugadores_aruco_tag,
)
from robot_soccer.perception.ball_tracking import Ball
from robot_soccer.utils.camera_undistort import load_intrinsics, undistort_frame


BALL_HSV_RANGE = ((10, 100, 20), (30, 255, 255))
COLOR_CIRCULO_PELOTA = (0, 165, 255)   # BGR — naranja
COLOR_ETIQUETA_ID    = (0, 255, 255)   # BGR — amarillo

DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parent.parent / "captures_output"
DEFAULT_OUTPUT_NAME = "captura_anotada.png"

# PNG sin pérdida con compresión moderada. Mantiene los bordes nítidos de
# las anotaciones (líneas ArUco, círculo de Hough, texto) intactos al
# incluirse como figura en LaTeX. Nivel 3 = balance tamaño/velocidad;
# 0 = sin compresión, 9 = máxima compresión (todos lossless).
PNG_WRITE_PARAMS = [cv.IMWRITE_PNG_COMPRESSION, 3]


def build_perspective_matrix():
    if not CAMERA_PERSPECTIVE_ENABLED:
        return None
    src = np.float32(CAMERA_PERSPECTIVE_SRC_POINTS)
    dst = np.float32([
        [0, 0],
        [CAMERA_PERSPECTIVE_WIDTH - 1, 0],
        [CAMERA_PERSPECTIVE_WIDTH - 1, CAMERA_PERSPECTIVE_HEIGHT - 1],
        [0, CAMERA_PERSPECTIVE_HEIGHT - 1],
    ])
    return cv.getPerspectiveTransform(src, dst)


def capture_warped_frame(cap, K, D, persp):
    ok, raw = cap.read()
    if not ok:
        return None
    undist = undistort_frame(raw, K, D)
    if persp is None:
        return undist
    return cv.warpPerspective(
        undist, persp,
        (CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT),
    )


def annotate(frame_bgr, detector, draw_ids=True):
    """Devuelve (frame_anotado, datos_jugadores, info_pelota).

    `frame_bgr` se modifica in-place al pasar por la detección ArUco.
    La detección de la pelota se hace sobre una copia limpia para que
    las anotaciones de ArUco no contaminen el filtrado HSV.
    """
    limpio = frame_bgr.copy()

    anotado, datos = deteccion_jugadores_aruco_tag(frame_bgr, detector, draw=True)

    hsv = cv.cvtColor(limpio, cv.COLOR_BGR2HSV)
    x, y, r = Ball.detectar_circulos_color(hsv, BALL_HSV_RANGE, limpio)
    info_pelota = None
    if x is not None:
        cv.circle(anotado, (x, y), r, COLOR_CIRCULO_PELOTA, 2)
        cv.circle(anotado, (x, y), 2, COLOR_CIRCULO_PELOTA, -1)
        info_pelota = (x, y, r)

    if draw_ids:
        for d in datos:
            etiqueta = f"ID {d['id']}"
            posicion = (d["x"] + 14, d["y"] - 14)
            cv.putText(anotado, etiqueta, posicion,
                       cv.FONT_HERSHEY_SIMPLEX, 0.55,
                       COLOR_ETIQUETA_ID, 2, cv.LINE_AA)

    return anotado, datos, info_pelota


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--camera-id", type=int, default=2,
                        help="ID de la cámara (default: 2)")
    parser.add_argument("--out", type=Path, default=None,
                        help=f"Ruta del PNG de salida "
                             f"(default: {DEFAULT_OUTPUT_DIR.name}/{DEFAULT_OUTPUT_NAME})")
    parser.add_argument("--use-sim-dict", action="store_true",
                        help="Usar diccionario ArUco de simulación en vez del de cámara")
    parser.add_argument("--no-ids", action="store_true",
                        help="No dibujar la etiqueta de ID junto al rectángulo")
    parser.add_argument("--oneshot", action="store_true",
                        help="Capturar un único frame y salir (sin ventana)")
    parser.add_argument("--warmup", type=int, default=10,
                        help="Frames descartados antes de empezar (default: 10)")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(levelname)-8s - %(message)s")

    out_path = args.out if args.out is not None else DEFAULT_OUTPUT_DIR / DEFAULT_OUTPUT_NAME
    if out_path.suffix.lower() != ".png":
        original = out_path
        out_path = out_path.with_suffix(".png")
        logging.warning("Forzado a PNG (lossless) para preservar bordes: %s → %s",
                        original.name, out_path.name)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    cap = cv.VideoCapture(args.camera_id)
    if not cap.isOpened():
        logging.error("No se pudo abrir la cámara %d", args.camera_id)
        return 1

    K, D = load_intrinsics()
    persp = build_perspective_matrix()
    detector = create_aruco_detector(use_camera=not args.use_sim_dict)

    for _ in range(args.warmup):
        cap.read()

    if args.oneshot:
        frame = capture_warped_frame(cap, K, D, persp)
        cap.release()
        if frame is None:
            logging.error("No se pudo leer un frame de la cámara")
            return 1
        anotado, datos, pelota = annotate(frame, detector,
                                          draw_ids=not args.no_ids)
        cv.imwrite(str(out_path), anotado, PNG_WRITE_PARAMS)
        logging.info("Guardado: %s | robots=%d | pelota=%s",
                     out_path, len(datos), pelota)
        return 0

    logging.info("SPACE = guardar  |  ESC = salir")
    try:
        while True:
            frame = capture_warped_frame(cap, K, D, persp)
            if frame is None:
                logging.warning("Frame inválido, reintentando…")
                continue
            anotado, datos, pelota = annotate(frame, detector,
                                              draw_ids=not args.no_ids)
            cv.imshow("captura anotada", anotado)
            k = cv.waitKey(1) & 0xFF
            if k == 27:        # ESC
                break
            if k == 32:        # SPACE
                cv.imwrite(str(out_path), anotado, PNG_WRITE_PARAMS)
                logging.info("Guardado: %s | robots=%d | pelota=%s",
                             out_path, len(datos), pelota)
    finally:
        cap.release()
        cv.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
