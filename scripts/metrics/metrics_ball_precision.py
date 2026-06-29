#!/usr/bin/env python3
"""Medición de precisión de detección de pelota (OBJ 1, con cámara).

Flujo manual de un solo proceso, sin robot, sin RF, sin control:
cámara -> perspectiva -> detección HSV de pelota -> superposición de marcas
de referencia en los cuatro postes -> captura por tecla.

Procedimiento (una sola ejecución cubre las 20 posiciones):
  1. Posicionar la pelota en una posición interior del campo.
  2. Medir con cinta la distancia desde un poste de referencia (marcado en
     pantalla) hasta la pelota.
  3. Pulsar C: el sistema acumula 100 lecturas de la posición detectada.
  4. Al completarse, introducir en la consola el poste y la distancia medida.
  5. Repetir para cada posición. Pulsar ESC para guardar y salir.

Ejecutar con:
    cd ~/git/RoboCupSoftware
    python scripts/metrics/metrics_ball_precision.py

Produce LOG/ball_precision_<timestamp>.json con, por posición:
  - mean_px, std_x_px, std_y_px, jitter_px (repetibilidad con pelota estática)
  - reference_post, measured_mm, detected_mm, error_mm (precisión métrica)
"""

import sys
import math
import time
import argparse
import logging
import statistics
from pathlib import Path

ROOT_DIR = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))
sys.path.insert(0, str(ROOT_DIR / "scripts"))

import cv2
import numpy as np

from robot_soccer.config import (
    CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH,
    CAMERA_PERSPECTIVE_ENABLED, CAMERA_PERSPECTIVE_SRC_POINTS,
    RANGO_COLOR_NARANJO, FIELD_CAM,
)
from robot_soccer.utils.camera_utils import get_camera_index

from metrics.metrics_capture import save_metrics
from metrics.session_recorder import SessionRecorder

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-8s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

# Dimensiones reales del campo (mm) que el marco de perspectiva representa.
FIELD_REAL_WIDTH_MM = 1500
FIELD_REAL_HEIGHT_MM = 900

N_FRAMES_PER_POINT = 100      # lecturas detectadas a acumular por posición
MAX_FRAMES_PER_CAPTURE = 600  # tope de frames por captura (evita bloqueo si no detecta)

# Postes interiores de los arcos, tomados de FIELD_CAM (marco 640x480).
# Etiqueta -> (x_px, y_px). El usuario mide desde uno de ellos hasta la pelota.
REFERENCE_POSTS = {
    "LI-sup": (FIELD_CAM.goal_left_x, FIELD_CAM.goal_left_top_y),
    "LI-inf": (FIELD_CAM.goal_left_x, FIELD_CAM.goal_left_bottom_y),
    "RD-sup": (FIELD_CAM.goal_right_x, FIELD_CAM.goal_right_top_y),
    "RD-inf": (FIELD_CAM.goal_right_x, FIELD_CAM.goal_right_bottom_y),
}


def dist_mm(p_px, q_px):
    """Distancia en mm entre dos puntos del marco, con escala por eje."""
    dx = (p_px[0] - q_px[0]) * FIELD_REAL_WIDTH_MM / CAMERA_PERSPECTIVE_WIDTH
    dy = (p_px[1] - q_px[1]) * FIELD_REAL_HEIGHT_MM / CAMERA_PERSPECTIVE_HEIGHT
    return math.hypot(dx, dy)


def detect_ball(frame):
    """Detecta la pelota por segmentación HSV. Devuelve (x, y) o None.

    Misma lógica que el proceso de percepción de test_chase_ball.py.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(RANGO_COLOR_NARANJO[0]),
                       np.array(RANGO_COLOR_NARANJO[1]))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) <= 10:
        return None
    (bx, by), radius = cv2.minEnclosingCircle(largest)
    if 2 <= radius <= 30:
        return (int(bx), int(by))
    return None


def summarize_capture(samples, test_id):
    """Calcula estadísticos de una captura y solicita el ground truth por consola."""
    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    cx, cy = statistics.mean(xs), statistics.mean(ys)
    std_x = statistics.stdev(xs) if len(xs) > 1 else 0.0
    std_y = statistics.stdev(ys) if len(ys) > 1 else 0.0
    jitter = statistics.mean(math.hypot(x - cx, y - cy) for x, y in samples)

    # Ground truth por consola (la ventana queda en pausa durante la entrada).
    print(f"\n--- Posición {test_id} capturada ({len(samples)} lecturas) ---")
    print(f"    Centro detectado: ({cx:.1f}, {cy:.1f}) px | jitter {jitter:.2f} px")
    print(f"    Postes de referencia disponibles: {', '.join(REFERENCE_POSTS)}")
    post = input("    Poste medido (Enter = LI-inf): ").strip() or "LI-inf"
    if post not in REFERENCE_POSTS:
        log.warning("Poste '%s' desconocido; se guarda sin error métrico.", post)
        post = None
    measured_mm = None
    error_mm = None
    detected_mm = None
    if post is not None:
        raw = input("    Distancia medida con cinta (cm): ").strip()
        try:
            measured_mm = float(raw.replace(",", ".")) * 10.0
            detected_mm = dist_mm((cx, cy), REFERENCE_POSTS[post])
            error_mm = abs(measured_mm - detected_mm)
            print(f"    Detectada {detected_mm:.1f} mm | medida {measured_mm:.1f} mm "
                  f"| error {error_mm:.1f} mm")
        except ValueError:
            log.warning("Valor no numérico; se guarda sin distancia medida.")

    return {
        "test_id": test_id,
        "n_samples": len(samples),
        "mean_px": [round(cx, 2), round(cy, 2)],
        "std_x_px": round(std_x, 3),
        "std_y_px": round(std_y, 3),
        "jitter_px": round(jitter, 3),
        "reference_post": post,
        "measured_mm": round(measured_mm, 1) if measured_mm is not None else None,
        "detected_mm": round(detected_mm, 1) if detected_mm is not None else None,
        "error_mm": round(error_mm, 1) if error_mm is not None else None,
        "samples_px": [[x, y] for x, y in samples],
    }


def draw_overlay(frame, ball_pos, capturing, capture_count, n_frames,
                 results, target_points):
    """Dibuja postes de referencia, pelota, estado de captura y controles."""
    # Postes de referencia
    for label, (px, py) in REFERENCE_POSTS.items():
        cv2.drawMarker(frame, (px, py), (255, 0, 255), cv2.MARKER_CROSS, 14, 2)
        cv2.putText(frame, label, (px + 6, py - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)

    # Pelota detectada
    if ball_pos:
        bx, by = ball_pos
        cv2.circle(frame, (bx, by), 12, (0, 165, 255), 2, cv2.LINE_AA)
        cv2.circle(frame, (bx, by), 2, (0, 165, 255), -1, cv2.LINE_AA)

    # Estado
    done = len(results)
    cv2.putText(frame, f"Posiciones: {done}/{target_points}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    if capturing:
        cv2.putText(frame, f"CAPTURANDO {capture_count}/{n_frames}", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    else:
        b_text = f"Pelota: ({ball_pos[0]},{ball_pos[1]})" if ball_pos else "Pelota: --"
        cv2.putText(frame, b_text, (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 200, 255) if ball_pos else (0, 0, 200), 1)

    cv2.putText(frame, "C=Capturar  ESC=Guardar y salir",
                (10, CAMERA_PERSPECTIVE_HEIGHT - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)


def main():
    parser = argparse.ArgumentParser(
        description="Medición de precisión de detección de pelota (OBJ 1)."
    )
    parser.add_argument("--camera-id", type=int, default=None,
                        help="ID de cámara (auto-detecta DroidCam si se omite).")
    parser.add_argument("--points", type=int, default=20,
                        help="Número de posiciones objetivo (default 20).")
    parser.add_argument("--frames", type=int, default=N_FRAMES_PER_POINT,
                        help=f"Lecturas por posición (default {N_FRAMES_PER_POINT}).")
    args = parser.parse_args()

    camera_id = args.camera_id
    if camera_id is None:
        camera_id = get_camera_index(prefer_droidcam=True, fallback_index=0)
        log.info("Cámara auto-detectada: /dev/video%d", camera_id)

    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        log.error("No se pudo abrir la cámara %d", camera_id)
        return

    perspective_matrix = None
    if CAMERA_PERSPECTIVE_ENABLED:
        src = np.float32(CAMERA_PERSPECTIVE_SRC_POINTS)
        dst = np.float32([
            [0, 0],
            [CAMERA_PERSPECTIVE_WIDTH - 1, 0],
            [CAMERA_PERSPECTIVE_WIDTH - 1, CAMERA_PERSPECTIVE_HEIGHT - 1],
            [0, CAMERA_PERSPECTIVE_HEIGHT - 1],
        ])
        perspective_matrix = cv2.getPerspectiveTransform(src, dst)

    window = "Precision de deteccion de pelota"
    cv2.namedWindow(window)

    results = []
    capturing = False
    samples = []
    frames_seen = 0
    recorder = SessionRecorder("ball_precision")
    video_path = None

    print("=" * 64)
    print("  MEDICIÓN DE PRECISIÓN DE DETECCIÓN DE PELOTA")
    print(f"  {args.points} posiciones objetivo | {args.frames} lecturas por posición")
    print("  C = capturar posición | ESC = guardar y salir")
    print("=" * 64)

    try:
        while True:
            ret, raw = cap.read()
            if not ret:
                continue
            if perspective_matrix is not None:
                frame = cv2.warpPerspective(
                    raw, perspective_matrix,
                    (CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT))
            else:
                frame = raw

            ball_pos = detect_ball(frame)

            if capturing:
                frames_seen += 1
                if ball_pos:
                    samples.append(ball_pos)
                if len(samples) >= args.frames or frames_seen >= MAX_FRAMES_PER_CAPTURE:
                    capturing = False
                    if len(samples) >= 2:
                        results.append(summarize_capture(samples, len(results) + 1))
                    else:
                        log.warning("Captura sin detecciones suficientes; descartada.")
                    samples = []
                    frames_seen = 0

            view = frame.copy()
            draw_overlay(view, ball_pos, capturing, len(samples),
                         args.frames, results, args.points)
            cv2.imshow(window, view)
            recorder.write(view)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                break
            elif key in (ord("c"), ord("C")) and not capturing:
                if ball_pos is None:
                    log.warning("No hay pelota detectada; acerca o ajusta antes de capturar.")
                else:
                    capturing = True
                    samples = []
                    frames_seen = 0
                    log.info("Capturando posición %d ...", len(results) + 1)

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
        video_path = recorder.close()

    if not results:
        print("\nNo se capturó ninguna posición; no se guarda archivo.")
        return

    errors = [r["error_mm"] for r in results if r["error_mm"] is not None]
    jitters = [r["jitter_px"] for r in results]
    summary = {
        "n_points": len(results),
        "frames_per_point": args.frames,
        "field_real_mm": {"width": FIELD_REAL_WIDTH_MM, "height": FIELD_REAL_HEIGHT_MM},
        "camera_id": camera_id,
        "video": str(video_path) if video_path else None,
        "error_mm_mean": round(statistics.mean(errors), 2) if errors else None,
        "error_mm_std": round(statistics.stdev(errors), 2) if len(errors) > 1 else 0.0,
        "error_mm_max": round(max(errors), 2) if errors else None,
        "jitter_px_mean": round(statistics.mean(jitters), 3) if jitters else None,
        "points": results,
    }
    try:
        out = save_metrics("ball_precision", summary)
        print(f"\n{'=' * 64}")
        print(f"  Métricas guardadas en: {out}")
        print(f"  Posiciones: {len(results)} | "
              f"error medio: {summary['error_mm_mean']} mm | "
              f"jitter medio: {summary['jitter_px_mean']} px")
        print("=" * 64)
    except Exception as e:
        log.warning("No se pudieron guardar métricas: %s", e)


if __name__ == "__main__":
    main()
