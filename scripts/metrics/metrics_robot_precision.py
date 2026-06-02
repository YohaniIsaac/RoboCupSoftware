#!/usr/bin/env python3
"""Medición de precisión de detección de robots ArUco (OBJ 1, con cámara).

Replica el pipeline de percepción de producción: cámara -> corrección de
perspectiva -> detección ArUco oficial (con corrección de paralaje y ángulo
métrico de la función deteccion_jugadores_aruco_tag). Un solo proceso, sin
robot motorizado, sin RF, sin control.

Tres modos seleccionables con --mode:

  position     Precisión de posición. Se coloca un robot en una posición
               interior, se mide con cinta la distancia desde un poste de
               referencia (marcado en pantalla) hasta el centro del robot, se
               pulsa C para acumular 100 lecturas y se introduce el poste y la
               distancia por consola. El error es la diferencia entre la
               distancia medida y la distancia detectada (mm).

  orientation  Precisión angular. Se coloca un robot sobre una plantilla con
               marcas angulares, se rota a un ángulo conocido, se pulsa C para
               acumular 50 lecturas y se introduce el ángulo real por consola.
               El error es la diferencia angular mínima (grados).

  multi        Detección simultánea. Se cuentan automáticamente cuántos de los
               cuatro robots se detectan en cada fotograma durante 1000
               fotogramas. No requiere medición física.

Ejecutar con:
    cd ~/git/RoboCupSoftware
    python scripts/metrics/metrics_robot_precision.py --mode position
    python scripts/metrics/metrics_robot_precision.py --mode orientation
    python scripts/metrics/metrics_robot_precision.py --mode multi

Produce LOG/robot_precision_<mode>_<timestamp>.json.
"""

import sys
import math
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
    FIELD_CAM, FIELD_PHYSICAL_WIDTH_CM, FIELD_PHYSICAL_HEIGHT_CM,
)
from robot_soccer.perception.player_tracking import (
    create_aruco_detector, deteccion_jugadores_aruco_tag,
)
from robot_soccer.utils.camera_utils import get_camera_index

from metrics.metrics_capture import save_metrics

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-8s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

# Dimensiones reales del campo (mm) que el marco de perspectiva representa.
FIELD_REAL_WIDTH_MM = int(FIELD_PHYSICAL_WIDTH_CM * 10)
FIELD_REAL_HEIGHT_MM = int(FIELD_PHYSICAL_HEIGHT_CM * 10)

ROBOT_IDS = {0, 1, 2, 3}        # marcadores de los cuatro robots del partido
N_FRAMES_POSITION = 100         # lecturas por posición (precisión de posición)
N_FRAMES_ORIENTATION = 50       # lecturas por orientación (precisión angular)
N_FRAMES_MULTI = 1000           # fotogramas de conteo (detección simultánea)
MAX_FRAMES_PER_CAPTURE = 600    # tope por captura (evita bloqueo si no detecta)

# Postes interiores de los arcos, tomados de FIELD_CAM (marco 640x480).
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


def angular_diff(a_deg, b_deg):
    """Diferencia angular mínima en grados, en el rango [-180, 180]."""
    return (a_deg - b_deg + 180.0) % 360.0 - 180.0


def circular_mean_deg(angles_deg):
    """Media circular de una lista de ángulos en grados."""
    s = sum(math.sin(math.radians(a)) for a in angles_deg)
    c = sum(math.cos(math.radians(a)) for a in angles_deg)
    return math.degrees(math.atan2(s, c))


def open_camera(camera_id):
    """Abre la cámara y devuelve el objeto VideoCapture, o None si falla."""
    if camera_id is None:
        camera_id = get_camera_index(prefer_droidcam=True, fallback_index=0)
        log.info("Cámara auto-detectada: /dev/video%d", camera_id)
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        log.error("No se pudo abrir la cámara %d", camera_id)
        return None, camera_id
    return cap, camera_id


def build_perspective_matrix():
    """Matriz de perspectiva idéntica a la del proceso de captura, o None."""
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


def acquire(cap, matrix, detector):
    """Captura un fotograma, aplica perspectiva y detecta robots.

    Returns:
        tuple: (frame_warped, datos) donde datos es la lista de dicts que
        devuelve deteccion_jugadores_aruco_tag, o (None, []) si no hay frame.
    """
    ret, raw = cap.read()
    if not ret:
        return None, []
    frame = cv2.warpPerspective(raw, matrix,
                                (CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT)) \
        if matrix is not None else raw
    frame, datos = deteccion_jugadores_aruco_tag(
        frame, detector, allowed_ids=ROBOT_IDS, draw=True)
    return frame, datos


def robot_by_id(datos, target_id):
    """Devuelve el dict del robot con id == target_id, o None."""
    for d in datos:
        if d["id"] == target_id:
            return d
    return None


# ---------------------------------------------------------------------------
# Modo position: precisión de posición (cinta desde poste de referencia)
# ---------------------------------------------------------------------------

def summarize_position(samples, test_id):
    """Estadísticos de una captura de posición y ground truth por consola."""
    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    cx, cy = statistics.mean(xs), statistics.mean(ys)
    std_x = statistics.stdev(xs) if len(xs) > 1 else 0.0
    std_y = statistics.stdev(ys) if len(ys) > 1 else 0.0
    jitter = statistics.mean(math.hypot(x - cx, y - cy) for x, y in samples)

    print(f"\n--- Posición {test_id} capturada ({len(samples)} lecturas) ---")
    print(f"    Centro detectado: ({cx:.1f}, {cy:.1f}) px | jitter {jitter:.2f} px")
    print(f"    Postes de referencia: {', '.join(REFERENCE_POSTS)}")
    post = input("    Poste medido (Enter = LI-inf): ").strip() or "LI-inf"
    if post not in REFERENCE_POSTS:
        log.warning("Poste '%s' desconocido; se guarda sin error métrico.", post)
        post = None
    measured_mm = detected_mm = error_mm = None
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
    }


def run_position(cap, matrix, detector, camera_id, target_id, n_frames):
    """Bucle de captura para precisión de posición."""
    window = "Precision de posicion de robot"
    cv2.namedWindow(window)
    results, capturing, samples, frames_seen = [], False, [], 0

    print("=" * 64)
    print("  PRECISIÓN DE POSICIÓN DE ROBOT (ArUco)")
    print(f"  Robot objetivo: id {target_id} | {n_frames} lecturas por posición")
    print("  C = capturar posición | ESC = guardar y salir")
    print("=" * 64)

    while True:
        frame, datos = acquire(cap, matrix, detector)
        if frame is None:
            continue
        target = robot_by_id(datos, target_id)
        pos = (target["x"], target["y"]) if target else None

        if capturing:
            frames_seen += 1
            if pos:
                samples.append(pos)
            if len(samples) >= n_frames or frames_seen >= MAX_FRAMES_PER_CAPTURE:
                capturing = False
                if len(samples) >= 2:
                    results.append(summarize_position(samples, len(results) + 1))
                else:
                    log.warning("Captura sin detecciones suficientes; descartada.")
                samples, frames_seen = [], 0

        view = frame.copy()
        for label, (px, py) in REFERENCE_POSTS.items():
            cv2.drawMarker(view, (px, py), (255, 0, 255), cv2.MARKER_CROSS, 14, 2)
            cv2.putText(view, label, (px + 6, py - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
        _hud(view, f"Posiciones: {len(results)}", capturing, len(samples), n_frames,
             f"Robot {target_id}: ({pos[0]},{pos[1]})" if pos else f"Robot {target_id}: --",
             bool(pos))
        cv2.imshow(window, view)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        if key in (ord("c"), ord("C")) and not capturing:
            if pos is None:
                log.warning("Robot %d no detectado; ajusta antes de capturar.", target_id)
            else:
                capturing, samples, frames_seen = True, [], 0
                log.info("Capturando posición %d ...", len(results) + 1)

    if not results:
        return None
    errors = [r["error_mm"] for r in results if r["error_mm"] is not None]
    jitters = [r["jitter_px"] for r in results]
    return {
        "mode": "position",
        "target_id": target_id,
        "n_points": len(results),
        "frames_per_point": n_frames,
        "field_real_mm": {"width": FIELD_REAL_WIDTH_MM, "height": FIELD_REAL_HEIGHT_MM},
        "camera_id": camera_id,
        "error_mm_mean": round(statistics.mean(errors), 2) if errors else None,
        "error_mm_std": round(statistics.stdev(errors), 2) if len(errors) > 1 else 0.0,
        "error_mm_max": round(max(errors), 2) if errors else None,
        "jitter_px_mean": round(statistics.mean(jitters), 3) if jitters else None,
        "points": results,
    }


# ---------------------------------------------------------------------------
# Modo orientation: precisión angular (plantilla física)
# ---------------------------------------------------------------------------

def summarize_orientation(samples, test_id):
    """Estadísticos de una captura de ángulo y ground truth por consola."""
    mean_ang = circular_mean_deg(samples)
    devs = [angular_diff(a, mean_ang) for a in samples]
    std_ang = statistics.stdev(devs) if len(devs) > 1 else 0.0

    print(f"\n--- Orientación {test_id} capturada ({len(samples)} lecturas) ---")
    print(f"    Ángulo detectado: {mean_ang:.2f}° | dispersión {std_ang:.2f}°")
    raw = input("    Ángulo real de la plantilla (grados): ").strip()
    measured_deg = error_deg = None
    try:
        measured_deg = float(raw.replace(",", "."))
        error_deg = abs(angular_diff(mean_ang, measured_deg))
        print(f"    Detectado {mean_ang:.1f}° | real {measured_deg:.1f}° "
              f"| error {error_deg:.1f}°")
    except ValueError:
        log.warning("Valor no numérico; se guarda sin ángulo real.")

    return {
        "test_id": test_id,
        "n_samples": len(samples),
        "mean_deg": round(mean_ang, 2),
        "std_deg": round(std_ang, 3),
        "measured_deg": round(measured_deg, 1) if measured_deg is not None else None,
        "error_deg": round(error_deg, 2) if error_deg is not None else None,
    }


def run_orientation(cap, matrix, detector, camera_id, target_id, n_frames):
    """Bucle de captura para precisión angular."""
    window = "Precision angular de robot"
    cv2.namedWindow(window)
    results, capturing, samples, frames_seen = [], False, [], 0

    print("=" * 64)
    print("  PRECISIÓN ANGULAR DE ROBOT (ArUco)")
    print(f"  Robot objetivo: id {target_id} | {n_frames} lecturas por orientación")
    print("  C = capturar orientación | ESC = guardar y salir")
    print("=" * 64)

    while True:
        frame, datos = acquire(cap, matrix, detector)
        if frame is None:
            continue
        target = robot_by_id(datos, target_id)
        ang = target["angulo"] if target else None

        if capturing:
            frames_seen += 1
            if ang is not None:
                samples.append(ang)
            if len(samples) >= n_frames or frames_seen >= MAX_FRAMES_PER_CAPTURE:
                capturing = False
                if len(samples) >= 2:
                    results.append(summarize_orientation(samples, len(results) + 1))
                else:
                    log.warning("Captura sin detecciones suficientes; descartada.")
                samples, frames_seen = [], 0

        view = frame.copy()
        if target is not None:
            cx, cy = target["x"], target["y"]
            # Eje de referencia 0°: +x (derecha de la imagen, hacia el arco
            # derecho). El ángulo crece en sentido horario en pantalla, porque
            # el eje y de la imagen apunta hacia abajo. La flecha verde de la
            # función oficial muestra la orientación actual del marcador.
            cv2.arrowedLine(view, (cx, cy), (cx + 70, cy), (0, 255, 255), 2, tipLength=0.25)
            cv2.putText(view, "0 deg", (cx + 74, cy + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            cv2.putText(view, f"{ang:+.1f} deg", (cx - 30, cy - 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        _hud(view, f"Orientaciones: {len(results)}", capturing, len(samples), n_frames,
             f"Robot {target_id}: {ang:+.1f} deg" if ang is not None else f"Robot {target_id}: --",
             ang is not None)
        cv2.imshow(window, view)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        if key in (ord("c"), ord("C")) and not capturing:
            if ang is None:
                log.warning("Robot %d no detectado; ajusta antes de capturar.", target_id)
            else:
                capturing, samples, frames_seen = True, [], 0
                log.info("Capturando orientación %d ...", len(results) + 1)

    if not results:
        return None
    errors = [r["error_deg"] for r in results if r["error_deg"] is not None]
    jitters = [r["std_deg"] for r in results]
    return {
        "mode": "orientation",
        "target_id": target_id,
        "n_points": len(results),
        "frames_per_point": n_frames,
        "camera_id": camera_id,
        "error_deg_mean": round(statistics.mean(errors), 2) if errors else None,
        "error_deg_std": round(statistics.stdev(errors), 2) if len(errors) > 1 else 0.0,
        "error_deg_max": round(max(errors), 2) if errors else None,
        "jitter_deg_mean": round(statistics.mean(jitters), 3) if jitters else None,
        "points": results,
    }


# ---------------------------------------------------------------------------
# Modo multi: detección simultánea (conteo automático)
# ---------------------------------------------------------------------------

def run_multi(cap, matrix, detector, camera_id, n_frames):
    """Conteo automático de robots detectados por fotograma."""
    window = "Deteccion simultanea multi-robot"
    cv2.namedWindow(window)
    counts = {0: 0, 1: 0, 2: 0, 3: 0, 4: 0}
    per_id = {rid: 0 for rid in sorted(ROBOT_IDS)}
    total = 0

    print("=" * 64)
    print("  DETECCIÓN SIMULTÁNEA MULTI-ROBOT")
    print(f"  Objetivo: {n_frames} fotogramas | ESC = detener y guardar")
    print("=" * 64)

    while total < n_frames:
        frame, datos = acquire(cap, matrix, detector)
        if frame is None:
            continue
        ids_seen = {d["id"] for d in datos if d["id"] in ROBOT_IDS}
        k = min(len(ids_seen), 4)
        counts[k] += 1
        for rid in ids_seen:
            per_id[rid] += 1
        total += 1

        view = frame.copy()
        cv2.putText(view, f"Frame {total}/{n_frames}  detectados: {k}/4", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(view, "ESC = detener y guardar",
                    (10, CAMERA_PERSPECTIVE_HEIGHT - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        cv2.imshow(window, view)
        if (cv2.waitKey(1) & 0xFF) == 27:
            break

    if total == 0:
        return None
    pct = {k: round(100.0 * counts[k] / total, 1) for k in counts}
    return {
        "mode": "multi",
        "camera_id": camera_id,
        "total_frames": total,
        "counts": counts,
        "percentages": pct,
        "per_id_detection_rate_pct": {
            str(rid): round(100.0 * per_id[rid] / total, 1) for rid in per_id
        },
    }


# ---------------------------------------------------------------------------
# HUD compartido y dispatcher
# ---------------------------------------------------------------------------

def _hud(frame, counter_text, capturing, capture_count, n_frames, target_text, has_target):
    """Dibuja contador, estado de captura y controles sobre el frame."""
    cv2.putText(frame, counter_text, (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    if capturing:
        cv2.putText(frame, f"CAPTURANDO {capture_count}/{n_frames}", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    else:
        cv2.putText(frame, target_text, (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 200, 255) if has_target else (0, 0, 200), 1)
    cv2.putText(frame, "C=Capturar  ESC=Guardar y salir",
                (10, CAMERA_PERSPECTIVE_HEIGHT - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)


def main():
    parser = argparse.ArgumentParser(
        description="Medición de precisión de detección de robots ArUco (OBJ 1)."
    )
    parser.add_argument("--mode", choices=["position", "orientation", "multi"],
                        required=True, help="Métrica a medir.")
    parser.add_argument("--camera-id", type=int, default=None,
                        help="ID de cámara (auto-detecta DroidCam si se omite).")
    parser.add_argument("--target-id", type=int, default=0,
                        help="ID del robot a medir en position/orientation (default 0).")
    parser.add_argument("--frames", type=int, default=None,
                        help="Lecturas por captura o fotogramas de conteo (según modo).")
    args = parser.parse_args()

    cap, camera_id = open_camera(args.camera_id)
    if cap is None:
        return
    matrix = build_perspective_matrix()
    detector = create_aruco_detector(use_camera=True)

    try:
        if args.mode == "position":
            n = args.frames or N_FRAMES_POSITION
            summary = run_position(cap, matrix, detector, camera_id, args.target_id, n)
        elif args.mode == "orientation":
            n = args.frames or N_FRAMES_ORIENTATION
            summary = run_orientation(cap, matrix, detector, camera_id, args.target_id, n)
        else:
            n = args.frames or N_FRAMES_MULTI
            summary = run_multi(cap, matrix, detector, camera_id, n)
    finally:
        cap.release()
        cv2.destroyAllWindows()

    if summary is None:
        print("\nNo se capturó ningún dato; no se guarda archivo.")
        return
    try:
        out = save_metrics(f"robot_precision_{args.mode}", summary)
        print(f"\n{'=' * 64}")
        print(f"  Métricas guardadas en: {out}")
        print("=" * 64)
    except Exception as e:
        log.warning("No se pudieron guardar métricas: %s", e)


if __name__ == "__main__":
    main()
