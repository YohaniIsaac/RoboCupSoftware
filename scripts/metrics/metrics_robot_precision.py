#!/usr/bin/env python3
"""Medición de precisión de detección de robots ArUco (OBJ 1, con cámara).

Replica el pipeline de percepción de producción: cámara -> corrección de
lente (undistort) -> corrección de perspectiva -> detección ArUco oficial
(con corrección de paralaje y ángulo métrico de la función
deteccion_jugadores_aruco_tag). Un solo proceso, sin robot motorizado, sin
RF, sin control.

Tres modos seleccionables con --mode:

  position     Precisión de pose (posición 2D + ángulo). Con G se fija el
               objetivo: poste de referencia, offset dx,dy en cm desde ese
               poste (+dx hacia el interior, +dy hacia arriba) y ángulo real
               del robot. El punto y las rectas dibujados verifican el dato
               tecleado (signo y magnitud). Con C se acumulan 100 lecturas y
               se calculan el error de posición 2D (mm, por eje y total) y el
               error angular (grados) contra ese ground truth.

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
from robot_soccer.utils.camera_undistort import load_intrinsics, undistort_frame

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

# Intrínsecos de cámara (K, D). Se cargan en main() y se aplican con undistort
# ANTES del warp, igual que el pipeline de producción (camera_feed.py). Quedan
# en None si no existe camera_intrinsics.json (undistort = no-op).
_K, _D = None, None


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


def acquire(cap, matrix, detector, allowed_ids=ROBOT_IDS):
    """Captura un fotograma, aplica perspectiva y detecta robots.

    Args:
        allowed_ids: IDs que se detectan y dibujan. En los modos de un solo
            objetivo (position, orientation) se restringe a {target_id} para
            que en pantalla solo aparezca el recuadro del robot bajo medición;
            el modo multi usa los cuatro.

    Returns:
        tuple: (frame_warped, datos) donde datos es la lista de dicts que
        devuelve deteccion_jugadores_aruco_tag, o (None, []) si no hay frame.
    """
    ret, raw = cap.read()
    if not ret:
        return None, []
    raw = undistort_frame(raw, _K, _D)  # corrige lente antes del warp (como producción)
    frame = cv2.warpPerspective(raw, matrix,
                                (CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT)) \
        if matrix is not None else raw
    frame, datos = deteccion_jugadores_aruco_tag(
        frame, detector, allowed_ids=allowed_ids, draw=True)
    return frame, datos


def robot_by_id(datos, target_id):
    """Devuelve el dict del robot con id == target_id, o None."""
    for d in datos:
        if d["id"] == target_id:
            return d
    return None


# ---------------------------------------------------------------------------
# Modo position: precisión de pose (posición 2D + ángulo vs ground truth)
# ---------------------------------------------------------------------------

def _parse_offset(raw):
    """Parsea 'dx,dy' (cm) a (float, float), o None si es inválido."""
    try:
        dx, dy = raw.replace(" ", "").split(",")
        return float(dx), float(dy)
    except (ValueError, IndexError):
        return None


def ground_truth_px(post, dx_cm, dy_cm):
    """Punto de referencia en píxeles desde un poste y un offset en cm.

    Convención: +dx hacia el interior del campo (signo automático según el
    poste sea izquierdo o derecho), +dy hacia arriba (el eje y de la imagen
    apunta hacia abajo, por eso se resta). Usa la misma escala por eje que
    dist_mm(), de modo que el error es consistente con el resto del script.
    """
    post_x, post_y = REFERENCE_POSTS[post]
    sign_x = 1.0 if post_x < CAMERA_PERSPECTIVE_WIDTH / 2.0 else -1.0
    dpx_x = dx_cm * 10.0 * CAMERA_PERSPECTIVE_WIDTH / FIELD_REAL_WIDTH_MM
    dpx_y = dy_cm * 10.0 * CAMERA_PERSPECTIVE_HEIGHT / FIELD_REAL_HEIGHT_MM
    return int(round(post_x + sign_x * dpx_x)), int(round(post_y - dpx_y))


def prompt_pose_target():
    """Pide poste, offset (cm) y ángulo real por consola. Devuelve dict o None.

    Se invoca con la tecla G, antes de capturar, para fijar el ground truth de
    la pose. El punto y las rectas que se dibujan después solo sirven para
    verificar que el dato tecleado (signo, magnitud) corresponde a la marca
    física; el robot no debe moverse para calzarlos.
    """
    print(f"\n    Postes de referencia: {', '.join(REFERENCE_POSTS)}")
    post = input("    Poste de referencia (Enter = LI-inf): ").strip() or "LI-inf"
    if post not in REFERENCE_POSTS:
        log.warning("Poste '%s' desconocido; punto descartado.", post)
        return None
    off = _parse_offset(input(
        "    Offset dx,dy desde el poste en cm (+dx hacia interior, +dy arriba): ").strip())
    if off is None:
        log.warning("Offset inválido (usa 'dx,dy', p. ej. 30,10); punto descartado.")
        return None
    try:
        angle_gt = float(input(
            "    Ángulo real del robot en grados (0 = hacia arco derecho, + horario): ")
            .strip().replace(",", "."))
    except ValueError:
        log.warning("Ángulo inválido; punto descartado.")
        return None
    gx, gy = ground_truth_px(post, off[0], off[1])
    print(f"    Objetivo: {post} + ({off[0]:+.1f},{off[1]:+.1f}) cm -> ({gx},{gy}) px "
          f"| ángulo {angle_gt:+.1f}°")
    return {"post": post, "dx_cm": off[0], "dy_cm": off[1],
            "gt_px": (gx, gy), "angle_gt": angle_gt}


def summarize_pose(pending, pos_samples, ang_samples, test_id):
    """Estadísticos de una captura: error 2D de posición y error angular."""
    xs = [s[0] for s in pos_samples]
    ys = [s[1] for s in pos_samples]
    cx, cy = statistics.mean(xs), statistics.mean(ys)
    jitter = statistics.mean(math.hypot(x - cx, y - cy) for x, y in pos_samples)

    gx, gy = pending["gt_px"]
    err_x_mm = (cx - gx) * FIELD_REAL_WIDTH_MM / CAMERA_PERSPECTIVE_WIDTH
    err_y_mm = (cy - gy) * FIELD_REAL_HEIGHT_MM / CAMERA_PERSPECTIVE_HEIGHT
    err_mm = math.hypot(err_x_mm, err_y_mm)

    mean_ang = circular_mean_deg(ang_samples)
    devs = [angular_diff(a, mean_ang) for a in ang_samples]
    ang_std = statistics.stdev(devs) if len(devs) > 1 else 0.0
    ang_err = abs(angular_diff(mean_ang, pending["angle_gt"]))

    print(f"\n--- Pose {test_id} capturada (pos {len(pos_samples)} / ang {len(ang_samples)}) ---")
    print(f"    Posición detectada ({cx:.1f},{cy:.1f}) px | objetivo ({gx},{gy}) px")
    print(f"    Error posición: x {err_x_mm:+.1f} mm, y {err_y_mm:+.1f} mm, "
          f"total {err_mm:.1f} mm | jitter {jitter:.2f} px")
    print(f"    Ángulo detectado {mean_ang:+.1f}° | objetivo {pending['angle_gt']:+.1f}° "
          f"| error {ang_err:.1f}° | dispersión {ang_std:.2f}°")
    if ang_err > 90.0:
        print("    (!) Error angular grande: ¿mediste el ángulo al revés? "
              "Revisa la recta 0° y el sentido (+ horario) en pantalla.")

    return {
        "test_id": test_id,
        "n_samples_pos": len(pos_samples),
        "n_samples_ang": len(ang_samples),
        "post": pending["post"],
        "offset_cm": [round(pending["dx_cm"], 1), round(pending["dy_cm"], 1)],
        "gt_px": [gx, gy],
        "mean_px": [round(cx, 2), round(cy, 2)],
        "jitter_px": round(jitter, 3),
        "error_x_mm": round(err_x_mm, 1),
        "error_y_mm": round(err_y_mm, 1),
        "error_mm": round(err_mm, 1),
        "angle_gt_deg": round(pending["angle_gt"], 1),
        "angle_mean_deg": round(mean_ang, 2),
        "angle_std_deg": round(ang_std, 3),
        "angle_error_deg": round(ang_err, 2),
    }


def run_position(cap, matrix, detector, camera_id, target_id, n_frames):
    """Bucle de captura de pose: posición 2D y ángulo, error vs ground truth."""
    window = "Precision de pose de robot"
    cv2.namedWindow(window)
    results, pending = [], None
    capturing, pos_samples, ang_samples, frames_seen = False, [], [], 0

    print("=" * 64)
    print("  PRECISIÓN DE POSE DE ROBOT (posición 2D + ángulo, ArUco)")
    print(f"  Robot objetivo: id {target_id} | {n_frames} lecturas por punto")
    print("  G = definir punto objetivo (poste, offset dx,dy cm, ángulo)")
    print("  C = capturar | ESC = guardar y salir")
    print("  Coloca el robot en la marca FÍSICA y teclea su pose real; no lo")
    print("  muevas para calzar el punto ni las rectas (eso anularía el error).")
    print("=" * 64)

    while True:
        frame, datos = acquire(cap, matrix, detector, allowed_ids={target_id})
        if frame is None:
            continue
        target = robot_by_id(datos, target_id)
        pos = (target["x"], target["y"]) if target else None
        ang = target["angulo"] if target else None

        if capturing:
            frames_seen += 1
            if pos is not None:
                pos_samples.append(pos)
            if ang is not None:
                ang_samples.append(ang)
            if len(pos_samples) >= n_frames or frames_seen >= MAX_FRAMES_PER_CAPTURE:
                capturing = False
                if len(pos_samples) >= 2 and len(ang_samples) >= 2:
                    results.append(summarize_pose(pending, pos_samples, ang_samples,
                                                  len(results) + 1))
                    pending = None
                else:
                    log.warning("Captura sin detecciones suficientes; descartada.")
                pos_samples, ang_samples, frames_seen = [], [], 0

        view = frame.copy()
        for label, (px, py) in REFERENCE_POSTS.items():
            cv2.drawMarker(view, (px, py), (255, 0, 255), cv2.MARKER_CROSS, 14, 2)
            cv2.putText(view, label, (px + 6, py - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
        if pos is not None:
            _draw_angle_live(view, pos[0], pos[1], ang)
        if pending is not None:
            _draw_pose_target(view, pending, pos, ang)
        _hud_pose(view, len(results), pending, capturing, len(pos_samples),
                  n_frames, pos, ang, target_id)
        cv2.imshow(window, view)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        if key in (ord("g"), ord("G")) and not capturing:
            new_target = prompt_pose_target()
            if new_target is not None:
                pending = new_target
        elif key in (ord("c"), ord("C")) and not capturing:
            if pending is None:
                log.warning("Define primero un punto objetivo con G.")
            elif pos is None:
                log.warning("Robot %d no detectado; ajusta antes de capturar.", target_id)
            else:
                capturing, pos_samples, ang_samples, frames_seen = True, [], [], 0
                log.info("Capturando pose %d ...", len(results) + 1)

    if not results:
        return None
    errors = [r["error_mm"] for r in results]
    ang_errors = [r["angle_error_deg"] for r in results]
    jitters = [r["jitter_px"] for r in results]
    return {
        "mode": "position",
        "target_id": target_id,
        "n_points": len(results),
        "frames_per_point": n_frames,
        "field_real_mm": {"width": FIELD_REAL_WIDTH_MM, "height": FIELD_REAL_HEIGHT_MM},
        "camera_id": camera_id,
        "error_mm_mean": round(statistics.mean(errors), 2),
        "error_mm_std": round(statistics.stdev(errors), 2) if len(errors) > 1 else 0.0,
        "error_mm_max": round(max(errors), 2),
        "error_x_mm_mean": round(statistics.mean([r["error_x_mm"] for r in results]), 2),
        "error_y_mm_mean": round(statistics.mean([r["error_y_mm"] for r in results]), 2),
        "jitter_px_mean": round(statistics.mean(jitters), 3),
        "angle_error_deg_mean": round(statistics.mean(ang_errors), 2),
        "angle_error_deg_std": round(statistics.stdev(ang_errors), 2) if len(ang_errors) > 1 else 0.0,
        "angle_error_deg_max": round(max(ang_errors), 2),
        "angle_jitter_deg_mean": round(statistics.mean([r["angle_std_deg"] for r in results]), 3),
        "points": results,
    }


def _draw_angle_live(view, cx, cy, ang):
    """Dibuja en vivo la recta de 0° (con su sentido positivo) y la del robot.

    Sirve para leer el ángulo del robot ANTES de teclearlo: 0° apunta a +x
    (hacia el arco derecho) y el sentido positivo es horario en pantalla, lo
    que evita medir el ángulo al revés. La recta naranja es la orientación
    detectada; es una guía visual, no para copiar el valor.
    """
    L = 60
    cv2.arrowedLine(view, (cx, cy), (cx + L, cy), (0, 255, 255), 1, tipLength=0.2)
    cv2.putText(view, "0 deg", (cx + L + 3, cy + 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    cv2.ellipse(view, (cx, cy), (26, 26), 0, 0, 45, (0, 255, 255), 1)
    cv2.putText(view, "+", (cx + 30, cy + 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    if ang is not None:
        ad = math.radians(ang)
        hx, hy = int(cx + L * math.cos(ad)), int(cy + L * math.sin(ad))
        cv2.arrowedLine(view, (cx, cy), (hx, hy), (0, 128, 255), 2, tipLength=0.2)
        cv2.putText(view, "robot", (hx + 3, hy + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 128, 255), 1)


def _draw_pose_target(view, pending, pos, ang):
    """Dibuja el punto objetivo, el vector desde el poste y la recta del ángulo
    tecleado (cian). Las rectas de 0° y del robot las dibuja _draw_angle_live.

    El punto y el vector verifican el offset tecleado (signo y magnitud); la
    recta cian es el ángulo objetivo tecleado, para compararlo con la recta
    naranja del robot y detectar de inmediato una medida al revés.
    """
    gx, gy = pending["gt_px"]
    post_xy = REFERENCE_POSTS[pending["post"]]
    cv2.arrowedLine(view, post_xy, (gx, gy), (255, 255, 0), 1, tipLength=0.12)
    cv2.drawMarker(view, (gx, gy), (255, 255, 0), cv2.MARKER_TILTED_CROSS, 16, 2)
    cv2.circle(view, (gx, gy), 9, (255, 255, 0), 1)
    cv2.putText(view, f"GT {pending['dx_cm']:+.0f},{pending['dy_cm']:+.0f}cm",
                (gx + 10, gy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

    ox, oy = pos if pos is not None else (gx, gy)
    L = 60
    at = math.radians(pending["angle_gt"])
    tx, ty = int(ox + L * math.cos(at)), int(oy + L * math.sin(at))
    cv2.arrowedLine(view, (ox, oy), (tx, ty), (255, 255, 0), 1, tipLength=0.2)
    cv2.putText(view, f"obj {pending['angle_gt']:+.0f}", (tx + 3, ty + 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)


def _hud_pose(view, n_points, pending, capturing, n_pos, n_frames, pos, ang, target_id):
    """HUD del modo pose: contador, objetivo pendiente, lectura y controles."""
    cv2.putText(view, f"Poses: {n_points}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    if capturing:
        cv2.putText(view, f"CAPTURANDO {n_pos}/{n_frames}", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    elif pending is not None:
        cv2.putText(view,
                    f"Objetivo {pending['post']} {pending['dx_cm']:+.0f},"
                    f"{pending['dy_cm']:+.0f}cm  th={pending['angle_gt']:+.0f}",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
    else:
        cv2.putText(view, "Sin objetivo (G para definir)", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 200), 1)
    det = (f"Robot {target_id}: ({pos[0]},{pos[1]})  {ang:+.1f} deg"
           if pos is not None and ang is not None else f"Robot {target_id}: --")
    cv2.putText(view, det, (10, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0, 200, 255) if pos is not None else (0, 0, 200), 1)
    cv2.putText(view, "G=Punto  C=Capturar  ESC=Guardar y salir",
                (10, CAMERA_PERSPECTIVE_HEIGHT - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)


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
        frame, datos = acquire(cap, matrix, detector, allowed_ids={target_id})
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
            a = math.radians(ang)
            L = 70  # misma longitud para ambas rectas, para compararlas a ojo
            # Recta de 0°: +x (derecha de la imagen, hacia el arco derecho).
            # El ángulo crece en sentido horario en pantalla porque el eje y de
            # la imagen apunta hacia abajo (+90° apunta hacia abajo, -90° hacia
            # arriba, ±180° hacia el arco izquierdo).
            cv2.arrowedLine(view, (cx, cy), (cx + L, cy), (0, 255, 255), 2, tipLength=0.2)
            cv2.putText(view, "0 deg", (cx + L + 4, cy + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            # Recta de orientación detectada del robot, misma longitud que la de
            # 0°. No es para copiar el valor: sirve para ubicar el plano de 0° y
            # el sentido de giro al alinear el robot a un ángulo conocido.
            hx, hy = int(cx + L * math.cos(a)), int(cy + L * math.sin(a))
            cv2.arrowedLine(view, (cx, cy), (hx, hy), (0, 128, 255), 2, tipLength=0.2)
            cv2.putText(view, "robot", (hx + 4, hy + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 128, 255), 1)
            cv2.putText(view, f"{ang:+.1f} deg  (+ horario)", (cx - 40, cy - 16),
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
    global _K, _D
    _K, _D = load_intrinsics()  # undistort antes del warp, igual que producción

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
