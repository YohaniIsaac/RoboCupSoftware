#!/usr/bin/env python3
"""Tasa de detección de marcadores ArUco con los robots EN MOVIMIENTO (OBJ 1).

La prueba de precisión (metrics_robot_precision.py) mide el robot estático. Este
script cubre el caso que falta: ¿se mantiene la detección cuando el marcador se
mueve y se difumina? Acciona los robots por RF con RÁFAGAS de movimiento
temporizadas (rotación en su sitio o desplazamiento corto, de duración fija)
mientras corre el mismo flujo de percepción de producción (undistort -> warp ->
ArUco con paralaje) y cuenta la detección por marcador, fotograma a fotograma.

Ráfagas temporizadas y de a un robot: una tecla dispara un movimiento de
duración fija (p. ej. girar 1 s, avanzar 0,5 s); un hilo de keepalive lo sostiene
reenviando el comando cada 30 ms durante esa ventana y lo detiene al expirar. El
firmware corta los motores si no recibe un comando en 100 ms, y el despachador RF
satura el enlace si se accionan los cuatro robots a la vez; por eso conviene
mover de a uno (la detección es independiente por marcador, así que la tasa por
robot no necesita movimiento simultáneo).

Dos problemas que la medición resuelve de raíz:

  1. Conteo solo "en movimiento": un marcador suma a su denominador únicamente
     en los fotogramas en que su robot está dentro de una ráfaga.

  2. Descarte de fuera de campo: si un marcador en ráfaga no se detecta, cuenta
     como FALLO solo si su última posición vista era interior y reciente; si
     venía saliendo por un borde, el fotograma se DESCARTA (ausencia != fallo).
     Además, C pausa el conteo y S es stop de emergencia.

La fila "cuatro simultáneos" del resultado es la fracción de fotogramas contados
en que los cuatro marcadores aparecen a la vez (disponibilidad; con los cuatro
sobre el campo, mayormente estáticos).

Uso:
    cd ~/git/RoboCupSoftware
    python scripts/integration/test_detection_in_motion.py
    python scripts/integration/test_detection_in_motion.py --serial-port /dev/ttyUSB0
    python scripts/integration/test_detection_in_motion.py --rot-secs 1.0 --trans-secs 0.5

Controles (ventana de cámara):
    1 / 2 / 3 / 4 : disparar una ráfaga en ese robot con el patrón actual
    A             : ráfaga en todos (puede pulsar con 4; mejor de a uno)
    F             : ciclar patron (rotar izq / rotar der / adelante / atras)
    + / -         : ajustar la duración del patrón actual
    C / ESPACIO   : iniciar/pausar el conteo
    S             : stop de emergencia (cancela ráfagas)
    R             : reiniciar acumuladores
    ESC           : detener, guardar y salir

Produce LOG/robot_detection_motion_<timestamp>.json (mismo esquema que consume
gen_tablas.py para tab:multi_robot: per_id_detection_rate_pct y percentages[4]).
"""
import sys
import time
import logging
import argparse
import threading
from pathlib import Path

ROOT_DIR = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))
sys.path.insert(0, str(ROOT_DIR / "scripts"))

import cv2
import numpy as np

from robot_soccer.config import (
    CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH,
    CAMERA_PERSPECTIVE_ENABLED, CAMERA_PERSPECTIVE_SRC_POINTS,
)
from robot_soccer.perception.player_tracking import (
    create_aruco_detector, deteccion_jugadores_aruco_tag,
)
from robot_soccer.utils.camera_undistort import load_intrinsics, undistort_frame
from robot_soccer.utils.camera_utils import get_camera_index
from robot_soccer.communication.rf_controller import RFController

from metrics.metrics_capture import save_metrics
from metrics.session_recorder import SessionRecorder

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-8s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

WINDOW = "Deteccion en movimiento"
ROBOT_IDS = {0, 1, 2, 3}

# Primitivas de movimiento: (nombre, PWM izq, PWM der, tipo). PWM con signo
# (-127..127), iguales a test_4robot_rf. La rotación va primera: es segura (no
# se traslada, no choca, no sale de cuadro) y da el mayor difuminado angular.
PATTERNS = [
    ("Rotar Izq", -40,  40, "rot"),
    ("Rotar Der",  40, -40, "rot"),
    ("Adelante",   50,  50, "trans"),
    ("Atras",     -50, -50, "trans"),
]

KEEPALIVE_PERIOD = 0.03    # s entre reenvíos durante una ráfaga (<100 ms del firmware)
EDGE_MARGIN_PX = 30        # margen al borde rectificado: dentro = interior
RECENT_FRAMES = 10         # antigüedad máxima de la última posición vista para "fallo"


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


def acquire(cap, matrix, detector, K, D):
    """Lee un fotograma y replica el pipeline de producción (undistort->warp->ArUco)."""
    ret, raw = cap.read()
    if not ret:
        return None, []
    raw = undistort_frame(raw, K, D)
    frame = cv2.warpPerspective(
        raw, matrix, (CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT)
    ) if matrix is not None else raw
    frame, datos = deteccion_jugadores_aruco_tag(
        frame, detector, allowed_ids=ROBOT_IDS, draw=True)
    return frame, datos


def _is_interior(pos, margin=EDGE_MARGIN_PX):
    """True si la posición está dentro del campo rectificado con margen."""
    x, y = pos
    return (margin <= x <= CAMERA_PERSPECTIVE_WIDTH - margin and
            margin <= y <= CAMERA_PERSPECTIVE_HEIGHT - margin)


def _draw_hud(view, now, motion_until, capturing, total, n_frames, pattern,
              dur, detected_ids, measured, n_all_detected):
    """Texto de estado sobre el fotograma (la detección ya viene dibujada)."""
    cap_color = (0, 255, 0) if capturing else (0, 0, 220)
    cap_text = f"CONTANDO {total}/{n_frames}" if capturing else "PAUSADO (C para contar)"
    cv2.putText(view, cap_text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, cap_color, 2)
    cv2.putText(view, f"patron: {pattern}  ({dur:.1f}s)", (10, 46),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    x0 = 10
    for pid in measured:
        det = pid in detected_ids
        rem = motion_until.get(pid + 1, 0) - now
        col = (0, 255, 0) if det else (0, 0, 220)
        tag = f"R{pid}:{rem:.1f}s" if rem > 0 else f"R{pid}:-"
        cv2.putText(view, tag, (x0, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
        x0 += 95

    if total > 0:
        sim = 100.0 * n_all_detected / total
        cv2.putText(view, f"4 simult.: {sim:.1f}% ({n_all_detected}/{total})",
                    (10, CAMERA_PERSPECTIVE_HEIGHT - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    cv2.putText(view, "1-4=rafaga A=todos F=patron +/-=dur C=contar S=stop ESC=guardar",
                (10, CAMERA_PERSPECTIVE_HEIGHT - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)


def main():
    parser = argparse.ArgumentParser(
        description="Tasa de detección de robots ArUco en movimiento (OBJ 1)."
    )
    parser.add_argument("--serial-port", default="/dev/ttyUSB0",
                        help="Puerto serial del transmisor RF (default: /dev/ttyUSB0).")
    parser.add_argument("--robots", nargs="+", type=int, default=[0, 1, 2, 3],
                        choices=[0, 1, 2, 3],
                        help="IDs Python de los robots a medir (default: 0 1 2 3).")
    parser.add_argument("--camera-id", type=int, default=None,
                        help="ID de cámara (auto-detecta DroidCam si se omite).")
    parser.add_argument("--frames", type=int, default=1000,
                        help="Fotogramas a contar antes de detener (default: 1000).")
    parser.add_argument("--rot-secs", type=float, default=1.0,
                        help="Duración de una ráfaga de rotación (default: 1.0 s).")
    parser.add_argument("--trans-secs", type=float, default=0.5,
                        help="Duración de una ráfaga de traslación (default: 0.5 s).")
    args = parser.parse_args()

    measured = sorted(set(args.robots))
    rot_secs = args.rot_secs
    trans_secs = args.trans_secs

    # --- Cámara y pipeline de percepción ---
    K, D = load_intrinsics()
    camera_id = args.camera_id
    if camera_id is None:
        camera_id = get_camera_index(prefer_droidcam=True, fallback_index=0)
        log.info("Cámara auto-detectada: /dev/video%d", camera_id)
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        log.error("No se pudo abrir la cámara %d", camera_id)
        return
    matrix = build_perspective_matrix()
    detector = create_aruco_detector(use_camera=True)

    # --- Enlace RF ---
    log.info("Conectando RF en %s ...", args.serial_port)
    rf = RFController(port=args.serial_port, enable_calibration=True)
    if not rf.initialize():
        log.error("No se pudo abrir %s — verifica el transmisor.", args.serial_port)
        cap.release()
        return
    connections = rf.test_connections()
    for pid in measured:
        fid = pid + 1
        ok = connections.get(f"robot_{fid}", False)
        log.info("  Robot %d (firmware %d): %s", pid, fid,
                 "disponible" if ok else "SIN RESPUESTA")

    pattern_idx = 0
    capturing = False

    # Estado de ráfagas (claves = firmware id 1..4)
    motion_until = {pid + 1: 0.0 for pid in measured}   # instante de fin de ráfaga
    burst_cmd = {pid + 1: (0, 0) for pid in measured}    # PWM congelado al disparar
    was_moving = {pid + 1: False for pid in measured}

    # Acumuladores (claves = id Python 0..3)
    per_id_moving = {pid: 0 for pid in measured}
    per_id_detected = {pid: 0 for pid in measured}
    last_seen = {}            # pid -> (x, y, frame_count)
    n_all_detected = 0        # fotogramas contados con los cuatro detectados
    total = 0                 # fotogramas contados (conteo activo)
    frame_count = 0

    # Keepalive en hilo propio: sostiene la ráfaga reenviando cada 30 ms (el
    # firmware corta a los 100 ms) y detiene el robot al expirar. Único que
    # llama a set_motors; el lazo de cámara nunca lo hace.
    stop_event = threading.Event()

    def _keepalive():
        while not stop_event.is_set():
            now = time.time()
            for pid in measured:
                fid = pid + 1
                if now < motion_until[fid]:
                    rf.set_motors(fid, *burst_cmd[fid])
                    was_moving[fid] = True
                elif was_moving[fid]:
                    rf.stop_robot(fid)
                    was_moving[fid] = False
            time.sleep(KEEPALIVE_PERIOD)

    ka_thread = threading.Thread(target=_keepalive, daemon=True)
    ka_thread.start()

    def start_burst(fid):
        """Dispara una ráfaga del patrón actual sobre un robot, con su duración."""
        _, pat_l, pat_r, kind = PATTERNS[pattern_idx]
        dur = rot_secs if kind == "rot" else trans_secs
        burst_cmd[fid] = (pat_l, pat_r)
        motion_until[fid] = time.time() + dur

    cv2.namedWindow(WINDOW)
    print("=" * 64)
    print("  DETECCIÓN EN MOVIMIENTO (ráfagas temporizadas, de a un robot)")
    print(f"  Robots: {measured} | objetivo {args.frames} fotogramas")
    print(f"  Rotación {rot_secs:.1f}s | Traslación {trans_secs:.1f}s (ajusta con +/-)")
    print("  F elige patrón; 1-4 dispara la ráfaga en ese robot; C cuenta.")
    print("  Coloca los cuatro sobre el campo y muévelos de a uno.")
    print("  C = contar | S = stop | ESC = guardar y salir")
    print("=" * 64)

    recorder = SessionRecorder("robot_detection_motion")
    video_path = None
    try:
        while True:
            frame, datos = acquire(cap, matrix, detector, K, D)
            if frame is None:
                continue
            frame_count += 1
            now = time.time()
            detected_ids = {d["id"] for d in datos if d["id"] in ROBOT_IDS}
            for d in datos:
                if d["id"] in ROBOT_IDS:
                    last_seen[d["id"]] = (d["x"], d["y"], frame_count)

            # --- Conteo (solo con C activo) ---
            if capturing:
                total += 1
                for pid in measured:
                    if now < motion_until[pid + 1]:        # robot en ráfaga
                        if pid in detected_ids:
                            per_id_moving[pid] += 1
                            per_id_detected[pid] += 1
                        else:
                            ls = last_seen.get(pid)
                            recent = ls is not None and (frame_count - ls[2]) <= RECENT_FRAMES
                            if recent and _is_interior((ls[0], ls[1])):
                                per_id_moving[pid] += 1   # fallo real (estaba en interior)
                            # si no: venía saliendo o sin rastro -> se descarta
                if all(pid in detected_ids for pid in measured):
                    n_all_detected += 1

            pattern = PATTERNS[pattern_idx][0]
            dur = rot_secs if PATTERNS[pattern_idx][3] == "rot" else trans_secs
            view = frame.copy()
            _draw_hud(view, now, motion_until, capturing, total, args.frames,
                      pattern, dur, detected_ids, measured, n_all_detected)
            cv2.imshow(WINDOW, view)
            recorder.write(view)

            if capturing and total >= args.frames:
                log.info("Alcanzados %d fotogramas contados.", total)
                break

            key = cv2.waitKey(1) & 0xFF
            if key == 27:                       # ESC
                break
            elif key in (ord("c"), ord("C"), ord(" ")):
                capturing = not capturing
                log.info("Conteo %s", "INICIADO" if capturing else "PAUSADO")
            elif key in (ord("s"), ord("S")):
                for pid in measured:
                    motion_until[pid + 1] = 0.0
                    rf.stop_robot(pid + 1)
                log.info("STOP de emergencia: ráfagas canceladas")
            elif key in (ord("f"), ord("F")):
                pattern_idx = (pattern_idx + 1) % len(PATTERNS)
                log.info("Patron: %s", PATTERNS[pattern_idx][0])
            elif key in (ord("+"), ord("=")):
                if PATTERNS[pattern_idx][3] == "rot":
                    rot_secs = min(5.0, round(rot_secs + 0.1, 1))
                else:
                    trans_secs = min(5.0, round(trans_secs + 0.1, 1))
            elif key == ord("-"):
                if PATTERNS[pattern_idx][3] == "rot":
                    rot_secs = max(0.1, round(rot_secs - 0.1, 1))
                else:
                    trans_secs = max(0.1, round(trans_secs - 0.1, 1))
            elif key in (ord("a"), ord("A")):
                for pid in measured:
                    start_burst(pid + 1)
                log.info("Ráfaga en TODOS (%s)", PATTERNS[pattern_idx][0])
            elif key in (ord("r"), ord("R")):
                per_id_moving = {pid: 0 for pid in measured}
                per_id_detected = {pid: 0 for pid in measured}
                n_all_detected = total = 0
                log.info("Acumuladores reiniciados")
            elif ord("1") <= key <= ord("4"):
                pid = key - ord("1")
                if pid in measured:
                    start_burst(pid + 1)
                    log.info("Ráfaga Robot %d: %s", pid, PATTERNS[pattern_idx][0])
    finally:
        stop_event.set()
        ka_thread.join(timeout=1.0)
        for pid in measured:
            rf.stop_robot(pid + 1)
        rf.shutdown()
        cap.release()
        video_path = recorder.ask_and_close(WINDOW)
        cv2.destroyAllWindows()

    if total == 0:
        log.warning("Sin fotogramas contados; no se guarda nada.")
        return

    rate = {
        str(pid): round(100.0 * per_id_detected[pid] / per_id_moving[pid], 1)
        for pid in measured if per_id_moving[pid] > 0
    }
    pct_all = round(100.0 * n_all_detected / total, 1) if total else None
    summary = {
        "mode": "multi_motion",
        "camera_id": camera_id,
        "video": str(video_path) if video_path else None,
        "robots_measured": measured,
        "rot_secs": rot_secs,
        "trans_secs": trans_secs,
        "total_frames": total,
        "per_id_frames_moving": {str(pid): per_id_moving[pid] for pid in measured},
        "per_id_detected": {str(pid): per_id_detected[pid] for pid in measured},
        "per_id_detection_rate_pct": rate,
        "frames_all_detected": n_all_detected,
        "percentages": {"4": pct_all},
    }
    out = save_metrics("robot_detection_motion", summary)
    log.info("Guardado: %s", out)
    print("\n--- Resumen ---")
    for pid in measured:
        print(f"  Robot {pid}: {rate.get(str(pid), 'sin datos')}% "
              f"({per_id_detected[pid]}/{per_id_moving[pid]} fotogramas en ráfaga)")
    print(f"  Cuatro simultáneos: {pct_all}% ({n_all_detected}/{total})")


if __name__ == "__main__":
    main()
