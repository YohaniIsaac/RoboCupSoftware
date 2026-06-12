#!/usr/bin/env python3
"""Mide la latencia observada por cámara de los actuadores de un robot.

Para cada comando se registra t1 (envío vía RFController) y se detecta el
primer fotograma en que el robot o la pelota supera el criterio de cambio
respecto al estado inicial (t2). La latencia observada es t2 - t1.

Criterios por comando
---------------------
motores_avanzar  : centroide del ArUco se desplaza ≥ --delta px
motores_giro     : orientación del ArUco cambia ≥ --delta-deg grados
kicker           : la pelota DESAPARECE del ROI inicial durante ≥ 2 frames
                   consecutivos (no se requiere detectarla en otra posición)

Región de interés (ROI)
-----------------------
Se procesa solo un recorte del frame centrado en la posición inicial del
robot o la pelota. Para motores: radio = 100 px (el robot cabe aunque avance
un poco). Para kicker: radio = 40 px (ajustado para que la pelota salga del
ROI casi de inmediato tras el disparo).

PWM y movimiento esperado
-------------------------
MOTOR_PWM = 70 (velocidad de juego calibrada ≈ 11 cm/s sobre el campo).
Con delta = 8 px (≈ 18 mm), el robot recorre ~2-3 cm totales antes de frenar.
Para motores_giro el robot solo rota ~5° sobre su eje; sin desplazamiento lineal.

Uso:
    python scripts/integration/test_actuator_latency.py --robot-id 0
    python scripts/integration/test_actuator_latency.py --robot-id 0 \\
        --commands motores_avanzar motores_giro   # solo motores, sin pelota
    python scripts/integration/test_actuator_latency.py --robot-id 0 \\
        --commands kicker                          # solo kicker, con pelota

    Los datos de fases anteriores se acumulan automáticamente en el JSON.

Produce LOG/actuator_latency_r<N>_<timestamp>.json (N = ID en prosa, 1-4).
"""

import sys
import time
import logging
import argparse
import math
import statistics
import json
import glob
from pathlib import Path

ROOT_DIR = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))
sys.path.insert(0, str(ROOT_DIR / "scripts"))

import cv2
import numpy as np

from metrics.metrics_capture import save_metrics

from robot_soccer.config import (
    CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH,
    CAMERA_PERSPECTIVE_ENABLED, CAMERA_PERSPECTIVE_SRC_POINTS,
    RANGO_COLOR_NARANJO,
    KICK_POINT_OFFSET_PX, KICK_POINT_ANGLE_OFFSET_DEG,
)
from robot_soccer.communication.rf_controller import RFController
from robot_soccer.perception.player_tracking import create_aruco_detector
from robot_soccer.utils.camera_utils import get_camera_index

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s %(levelname)-8s %(message)s',
    datefmt='%H:%M:%S'
)
log = logging.getLogger(__name__)

MOTOR_PWM = 70          # velocidad de juego calibrada
KICK_POWER = 0.5        # el firmware usa tiempo fijo; la potencia es nominal
INIT_FRAMES = 10        # frames para establecer estado inicial
MAX_WAIT_S = 3.0        # tiempo máximo esperando cambio tras el comando

ROBOT_ROI_HALF = 100    # px: radio del ROI para detección del robot
BALL_ROI_HALF = 40      # px: radio del ROI para detección de la pelota
                        #     (apretado para que salga rápido tras el kick)
KICKER_ABSENT_FRAMES = 2  # frames consecutivos sin pelota → kick confirmado

COMMANDS = ["motores_avanzar", "motores_giro", "kicker"]


# ---------------------------------------------------------------------------
# Perspectiva y frame
# ---------------------------------------------------------------------------

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
    return cv2.getPerspectiveTransform(src, dst)


def grab_frame(cap, persp):
    ret, raw = cap.read()
    if not ret:
        return None
    if persp is not None:
        return cv2.warpPerspective(
            raw, persp, (CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT)
        )
    return raw


# ---------------------------------------------------------------------------
# ROI
# ---------------------------------------------------------------------------

def crop_roi(frame, cx, cy, half):
    """Recorta el frame a un cuadrado centrado en (cx, cy) con radio half.

    Returns:
        crop   : imagen recortada (puede ser más pequeña en los bordes)
        x_off  : columna de inicio del recorte en el frame original
        y_off  : fila de inicio del recorte en el frame original
    """
    H, W = frame.shape[:2]
    x1 = max(0, int(cx) - half)
    y1 = max(0, int(cy) - half)
    x2 = min(W, int(cx) + half)
    y2 = min(H, int(cy) + half)
    return frame[y1:y2, x1:x2], x1, y1


# ---------------------------------------------------------------------------
# Detección de robot (ArUco) con ROI
# ---------------------------------------------------------------------------

def _aruco_in_crop(crop, detector, aruco_id):
    """Detecta el marcador aruco_id en el recorte.
    Retorna (cx_crop, cy_crop, angle_deg) o None.
    """
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return None
    for i, mid in enumerate(ids):
        if mid[0] == aruco_id:
            pts = corners[i].reshape(4, 2)
            cx = float(pts[:, 0].mean())
            cy = float(pts[:, 1].mean())
            vec = pts[1] - pts[0]
            angle = float(math.degrees(math.atan2(vec[1], vec[0])))
            return (cx, cy, angle)
    return None


def detect_robot_pos(frame, detector, aruco_id, roi_center):
    """Centroide del robot en coordenadas del frame completo, o None."""
    crop, x_off, y_off = crop_roi(frame, roi_center[0], roi_center[1], ROBOT_ROI_HALF)
    result = _aruco_in_crop(crop, detector, aruco_id)
    if result is None:
        return None
    cx_crop, cy_crop, _ = result
    return (cx_crop + x_off, cy_crop + y_off)


def detect_robot_heading(frame, detector, aruco_id, roi_center):
    """Orientación del marcador en grados [-180, 180), o None."""
    crop, x_off, y_off = crop_roi(frame, roi_center[0], roi_center[1], ROBOT_ROI_HALF)
    result = _aruco_in_crop(crop, detector, aruco_id)
    if result is None:
        return None
    return result[2]


# ---------------------------------------------------------------------------
# Kick point y presencia de pelota en ROI
# ---------------------------------------------------------------------------

# Píxeles naranjos mínimos en el ROI para considerar que la pelota está presente.
# Pelota de radio ~7 px → área ~154 px². Con detección parcial, 30 px es
# suficiente para evitar falsos positivos sin exigir detección perfecta.
BALL_MIN_PIXELS = 30


def compute_kick_point(frame, detector, aruco_id):
    """Retorna el kick point (kx, ky) del robot en el frame completo, o None.

    Usa detección ArUco en todo el frame (solo una vez, durante el setup).
    kick_point = centro_robot + KICK_POINT_OFFSET_PX × vector_heading.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return None
    for i, mid in enumerate(ids):
        if mid[0] == aruco_id:
            pts = corners[i].reshape(4, 2)
            cx = float(pts[:, 0].mean())
            cy = float(pts[:, 1].mean())
            vec = pts[1] - pts[0]
            angle_rad = math.atan2(vec[1], vec[0]) + math.radians(KICK_POINT_ANGLE_OFFSET_DEG)
            kx = cx + KICK_POINT_OFFSET_PX * math.cos(angle_rad)
            ky = cy + KICK_POINT_OFFSET_PX * math.sin(angle_rad)
            return (kx, ky)
    return None


def ball_present_in_roi(frame, ball_lower, ball_upper, roi_center):
    """True si hay suficientes píxeles naranjos en el ROI del kick point.

    Solo cuenta píxeles HSV en el recorte — sin contornos ni morfología.
    Más rápido que la detección completa; sirve para detectar presencia/ausencia.
    """
    crop, _, _ = crop_roi(frame, roi_center[0], roi_center[1], BALL_ROI_HALF)
    if crop.size == 0:
        return False
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, ball_lower, ball_upper)
    return cv2.countNonZero(mask) >= BALL_MIN_PIXELS


# ---------------------------------------------------------------------------
# Medición por tipo
# ---------------------------------------------------------------------------

def _avg_pos(readings):
    return (sum(p[0] for p in readings) / len(readings),
            sum(p[1] for p in readings) / len(readings))


def _avg_angle(readings):
    sins = sum(math.sin(math.radians(a)) for a in readings)
    coss = sum(math.cos(math.radians(a)) for a in readings)
    return math.degrees(math.atan2(sins / len(readings), coss / len(readings)))


def _angular_diff(a, b):
    diff = abs(a - b) % 360.0
    return diff if diff <= 180.0 else 360.0 - diff


def measure_motor(cap, persp, detect_fn, dist_fn, avg_fn,
                  send_fn, stop_fn, threshold):
    """Ensayo de motor: mide latencia hasta que el robot supera el umbral.

    detect_fn recibe (frame, roi_center) y retorna la medición (pos o ángulo).
    La posición inicial determina el ROI de todos los frames siguientes.
    """
    # Captura inicial sin ROI fijo (el ROI lo da la pos inicial detectada)
    readings = []
    misses = 0
    roi_center = None
    while len(readings) < INIT_FRAMES:
        frame = grab_frame(cap, persp)
        if frame is None:
            continue
        # Primer frame: ROI centrado en el frame completo para buscar el marker
        rc = roi_center if roi_center is not None else (
            CAMERA_PERSPECTIVE_WIDTH // 2, CAMERA_PERSPECTIVE_HEIGHT // 2)
        val = detect_fn(frame, rc)
        if val is not None:
            readings.append(val)
            # Actualizar el centro del ROI con la detección más reciente
            if isinstance(val, tuple):
                roi_center = (int(val[0]), int(val[1]))
        else:
            misses += 1
            if misses > INIT_FRAMES * 3:
                log.warning("  No se detectó el estado inicial. Reposicionar.")
                return None

    init = avg_fn(readings)
    if isinstance(init, tuple):
        roi_center = (int(init[0]), int(init[1]))

    for _ in range(2):
        cap.grab()

    t1 = time.perf_counter()
    send_fn()

    t2 = None
    deadline = t1 + MAX_WAIT_S
    while time.perf_counter() < deadline:
        frame = grab_frame(cap, persp)
        if frame is None:
            continue
        t_frame = time.perf_counter()
        val = detect_fn(frame, roi_center)
        if val is None:
            continue
        if dist_fn(val, init) >= threshold:
            t2 = t_frame
            break

    stop_fn()

    if t2 is None:
        log.warning("  Sin cambio detectado en %.1f s.", MAX_WAIT_S)
        return None
    return (t2 - t1) * 1000.0


def measure_kicker(cap, persp, detector, aruco_id, ball_lower, ball_upper, send_fn):
    """Ensayo de kicker: mide latencia hasta que la pelota desaparece del kick point.

    El ROI se calcula desde la posición y orientación del robot (ArUco), no
    buscando la pelota en el frame completo. El criterio de "éxito" es que la
    pelota deje de detectarse en ese ROI durante KICKER_ABSENT_FRAMES frames
    consecutivos: no se necesita localizarla en su nueva posición.

    Flujo:
        1. Detectar robot en frame completo (una vez) → calcular kick_point.
        2. Verificar que la pelota está en el ROI del kick_point.
        3. Enviar comando de kicker (t1).
        4. Leer frames: si la pelota ausente ≥ KICKER_ABSENT_FRAMES → t2.
        5. Latencia = t2 − t1.
    """
    # 1. Calcular kick point a partir del ArUco (detección en frame completo,
    #    solo una vez antes de t1 — no afecta la medición de latencia).
    kick_point = None
    misses = 0
    while kick_point is None:
        frame = grab_frame(cap, persp)
        if frame is None:
            continue
        kick_point = compute_kick_point(frame, detector, aruco_id)
        if kick_point is None:
            misses += 1
            if misses > 30:
                log.warning("  Robot %d no detectado. Verificar que el marcador sea visible.", aruco_id)
                return None

    roi_center = (int(kick_point[0]), int(kick_point[1]))
    log.info("  Kick point estimado: (%d, %d) — ROI ±%d px", *roi_center, BALL_ROI_HALF)

    # 2. Verificar que la pelota está en el ROI (esperar hasta INIT_FRAMES confirmaciones).
    confirmed = 0
    misses = 0
    while confirmed < INIT_FRAMES:
        frame = grab_frame(cap, persp)
        if frame is None:
            continue
        if ball_present_in_roi(frame, ball_lower, ball_upper, roi_center):
            confirmed += 1
        else:
            misses += 1
            if misses > INIT_FRAMES * 3:
                log.warning("  Pelota no detectada en el ROI del kicker. "
                            "Colocarla justo frente al solenoide.")
                return None

    for _ in range(2):
        cap.grab()      # vaciar buffer antes de medir

    # 3 & 4. Enviar comando y detectar desaparición.
    t1 = time.perf_counter()
    send_fn()

    t2 = None
    absent_count = 0
    deadline = t1 + MAX_WAIT_S
    while time.perf_counter() < deadline:
        frame = grab_frame(cap, persp)
        if frame is None:
            continue
        t_frame = time.perf_counter()
        if not ball_present_in_roi(frame, ball_lower, ball_upper, roi_center):
            absent_count += 1
            if absent_count >= KICKER_ABSENT_FRAMES:
                t2 = t_frame
                break
        else:
            absent_count = 0    # reset: detección puntual de ruido

    if t2 is None:
        log.warning("  Pelota no salió del ROI en %.1f s. ¿El kicker disparó?", MAX_WAIT_S)
        return None
    return (t2 - t1) * 1000.0


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--robot-id", type=int, required=True, choices=[0, 1, 2, 3],
                        help="ID del robot por marcador ArUco (0-3)")
    parser.add_argument("--serial-port", default="/dev/ttyUSB0")
    parser.add_argument("--camera-id", type=int, default=None)
    parser.add_argument("--trials", type=int, default=10,
                        help="Ensayos por comando (default 10)")
    parser.add_argument("--delta", type=float, default=8.0,
                        help="Umbral de desplazamiento px para motores_avanzar (default 8)")
    parser.add_argument("--delta-deg", type=float, default=5.0,
                        help="Umbral de orientación en ° para motores_giro (default 5)")
    parser.add_argument("--commands", nargs="+", choices=COMMANDS, default=COMMANDS,
                        metavar="CMD",
                        help=("Comandos a ejecutar. Default: todos. "
                              "Ejemplo: --commands motores_avanzar motores_giro"))
    args = parser.parse_args()

    aruco_id = args.robot_id
    firmware_id = aruco_id + 1
    prosa_id = aruco_id + 1

    camera_id = args.camera_id if args.camera_id is not None else get_camera_index()
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        sys.exit(f"No se pudo abrir la cámara {camera_id}")
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    persp = build_perspective_matrix()
    detector = create_aruco_detector(use_camera=True)
    ball_lower = np.array(RANGO_COLOR_NARANJO[0])
    ball_upper = np.array(RANGO_COLOR_NARANJO[1])

    rf = RFController(port=args.serial_port)
    if not rf.initialize():
        cap.release()
        sys.exit("No se pudo inicializar el enlace RF")
    time.sleep(2.0)

    # Cargar datos previos del mismo robot (ejecución en fases).
    LOG_DIR = ROOT_DIR / "LOG"
    prev_files = sorted(glob.glob(str(LOG_DIR / f"actuator_latency_r{prosa_id}_*.json")))
    prev_commands = {}
    if prev_files:
        try:
            with open(prev_files[-1], encoding="utf-8") as fh:
                prev_commands = json.load(fh).get("commands", {})
            log.info("Datos previos cargados: %s", list(prev_commands.keys()))
        except Exception:
            pass

    data = {"robot_id": prosa_id, "aruco_id": aruco_id,
            "motor_pwm": MOTOR_PWM, "commands": dict(prev_commands)}

    def det_pos(frame, rc):
        return detect_robot_pos(frame, detector, aruco_id, rc)

    def det_heading(frame, rc):
        return detect_robot_heading(frame, detector, aruco_id, rc)

    def pos_dist(cur, init):
        return math.hypot(cur[0] - init[0], cur[1] - init[1])

    try:
        for cmd in args.commands:
            log.info("=== Robot %d — %s ===", prosa_id, cmd)

            if cmd == "kicker":
                latencias = []
                n_fail = 0
                for t in range(args.trials):
                    input(f"  [Enter] ensayo {t + 1}/{args.trials}: "
                          f"colocar pelota frente al kicker del robot {prosa_id}...")
                    lat = measure_kicker(cap, persp, detector, aruco_id,
                                        ball_lower, ball_upper,
                                        lambda: rf.kick(firmware_id, KICK_POWER))
                    if lat is None:
                        n_fail += 1
                    else:
                        latencias.append(round(lat, 2))
                        log.info("  Latencia: %.1f ms", lat)
                    time.sleep(0.3)

            else:  # motores_avanzar o motores_giro
                if cmd == "motores_avanzar":
                    detect_fn = det_pos
                    dist_fn = pos_dist
                    avg_fn = _avg_pos
                    threshold = args.delta
                    send_fn = lambda: rf.set_motors(firmware_id, MOTOR_PWM, MOTOR_PWM)
                    objeto = f"robot {prosa_id} con espacio libre por delante (~5 cm)"
                    unidad = f"{args.delta:.0f} px"
                else:  # motores_giro
                    detect_fn = det_heading
                    dist_fn = _angular_diff
                    avg_fn = _avg_angle
                    threshold = args.delta_deg
                    send_fn = lambda: rf.set_motors(firmware_id, MOTOR_PWM, -MOTOR_PWM)
                    objeto = f"robot {prosa_id} (girará en su propio eje)"
                    unidad = f"{args.delta_deg:.0f}°"

                log.info("  Umbral: %s — ROI: ±%d px", unidad, ROBOT_ROI_HALF)
                latencias = []
                n_fail = 0
                for t in range(args.trials):
                    input(f"  [Enter] ensayo {t + 1}/{args.trials}: "
                          f"posicionar {objeto} y mantenerlo quieto...")
                    lat = measure_motor(cap, persp, detect_fn, dist_fn, avg_fn,
                                       send_fn, lambda: rf.stop_robot(firmware_id), threshold)
                    if lat is None:
                        n_fail += 1
                    else:
                        latencias.append(round(lat, 2))
                        log.info("  Latencia: %.1f ms", lat)
                    time.sleep(0.4)

            n_suc = len(latencias)
            mean = round(statistics.mean(latencias), 2) if latencias else None
            std = (round(statistics.stdev(latencias), 2)
                   if len(latencias) > 1 else (0.0 if latencias else None))
            data["commands"][cmd] = {
                "n_trials": args.trials,
                "n_success": n_suc,
                "lat_camara_ms": latencias,
                "lat_camara_mean_ms": mean,
                "lat_camara_std_ms": std,
            }
            mean_str = f"{mean} ± {std} ms" if mean is not None else "sin datos"
            log.info("  Resumen %s: %d/%d — %s", cmd, n_suc, args.trials, mean_str)

    finally:
        rf.stop_robot(firmware_id)
        rf.shutdown()
        cap.release()

    out = save_metrics(f"actuator_latency_r{prosa_id}", data)
    log.info("Guardado: %s", out)


if __name__ == "__main__":
    main()
