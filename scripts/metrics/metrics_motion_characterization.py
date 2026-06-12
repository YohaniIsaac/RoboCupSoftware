#!/usr/bin/env python3
"""Caracterización cinemática real de los robots para OBJ 3 (REQUIERE HARDWARE).

Mueve cada robot a puntos preestablecidos usando el controlador PID real
(DifferentialDriveController), que ya implementa el comportamiento de
producción: gira en el lugar si el error de orientación > 30°, luego avanza
recto con corrección continua. El robot no se sale de los límites del campo
ni choca, porque los waypoints están dentro del área y la cadencia es la misma
del sistema desplegado.

Mide el tiempo real de cada tramo y de cada giro puro (calibración), y deriva
los parámetros cinemáticos de los datos medidos, no de muestras de velocidad
instantánea.

Ejecutar (cámara + RF conectados):
    cd ~/git/RoboCupSoftware

    # Paso 0: verificar que los robots se mueven (sin cámara)
    .venv/bin/python scripts/metrics/metrics_motion_characterization.py --check

    # Paso 1: caracterizar todos los robots
    .venv/bin/python scripts/metrics/metrics_motion_characterization.py \\
        --robots 1 2 3 4 --camera-id 2 --serial-port /dev/ttyUSB0

    # Paso 1b: un solo robot (más rápido para empezar)
    .venv/bin/python scripts/metrics/metrics_motion_characterization.py \\
        --robots 1 --camera-id 2

Parámetros ajustables al inicio del archivo:
    WAYPOINTS   — puntos del circuito en px (marco FIELD_CAM 640×480)
    TIMEOUT_S   — tiempo máximo por tramo antes de declararlo fallido
    N_LAPS      — vueltas al circuito por robot (más vueltas = mejor estadística)

Salida: LOG/motion_profile_<timestamp>.json con, por robot (player id 0-3):
    v_px_s         {mean, std, n}   velocidad lineal efectiva
    omega_cw_deg_s {mean, std, n}   velocidad angular horaria
    omega_ccw_deg_s{mean, std, n}   velocidad angular antihoraria
    t_turn_overhead_s               tiempo extra de giro estimado (frenado/arranque)
    t_move_overhead_s               tiempo extra de avance estimado
    raw_segments                    datos crudos de cada tramo
    failed_segments                 tramos descartados (timeout o robot no detectado)
"""

import sys
import time
import json
import glob
import math
import logging
import argparse
import statistics
from pathlib import Path

import numpy as np
import cv2

ROOT_DIR = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))
sys.path.insert(0, str(ROOT_DIR / "scripts"))

logging.basicConfig(level=logging.WARNING, format="%(levelname)-8s %(message)s")
log = logging.getLogger(__name__)

from robot_soccer.communication.rf_controller import RFController
from robot_soccer.controllers.differential_drive import DifferentialDriveController
from robot_soccer.perception.player_tracking import (
    create_aruco_detector, deteccion_jugadores_aruco_tag,
)
from robot_soccer.config import (
    CAMERA_PERSPECTIVE_ENABLED,
    CAMERA_PERSPECTIVE_SRC_POINTS,
    CAMERA_PERSPECTIVE_WIDTH,
    CAMERA_PERSPECTIVE_HEIGHT,
    ROBOT_LINEAR_START_ANGLE_THRESHOLD_DEG,
    ROBOT_POSITION_THRESHOLD,
)
from metrics.metrics_capture import save_metrics

# ─── Circuito de waypoints (px en marco FIELD_CAM 640×480) ───────────────────
# Los puntos están bien dentro del campo, con margen de paredes. El robot
# recorre los puntos en orden; al terminar todos, se cuenta una vuelta.
# Cada par (A→B) con geometría diferente ejercita distintos regímenes.
WAYPOINTS = [
    (160, 120),   # esquina sup-izq
    (480, 120),   # esquina sup-der   → tramo largo horizontal, giro ~0°
    (480, 360),   # esquina inf-der   → tramo vertical, giro ~90°
    (160, 360),   # esquina inf-izq   → tramo largo horizontal, giro ~180°
    (320, 240),   # centro            → tramo diagonal, giro variable
]

# ─── Parámetros operativos ────────────────────────────────────────────────────
N_LAPS = 2           # vueltas al circuito por robot
TIMEOUT_S = 12.0     # tiempo máximo por tramo (si supera → fallo, pausa y sigue)
LOOP_DT = 0.04       # periodo del lazo de control (25 Hz, igual que el sistema real)
ANGLE_TURN_THR = ROBOT_LINEAR_START_ANGLE_THRESHOLD_DEG  # 30°
ARRIVAL_PX = ROBOT_POSITION_THRESHOLD
PAUSE_BETWEEN_ROBOTS_S = 5.0


# ─── Proxy mínimo del objeto Player que necesita DifferentialDriveController ─
class _PlayerProxy:
    __slots__ = ("id", "x", "y", "angle", "dx", "dy", "dw")

    def __init__(self, pid, x=0, y=0, angle=0.0):
        self.id = pid
        self.x = x
        self.y = y
        self.angle = float(angle)
        self.dx = self.dy = self.dw = 0.0


# ─── Cámara + ArUco ───────────────────────────────────────────────────────────
class Eyes:
    def __init__(self, camera_id):
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            raise RuntimeError(f"No se pudo abrir cámara {camera_id}")
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        self.M = None
        if CAMERA_PERSPECTIVE_ENABLED:
            src = np.float32(CAMERA_PERSPECTIVE_SRC_POINTS)
            dst = np.float32([
                [0, 0],
                [CAMERA_PERSPECTIVE_WIDTH - 1, 0],
                [CAMERA_PERSPECTIVE_WIDTH - 1, CAMERA_PERSPECTIVE_HEIGHT - 1],
                [0, CAMERA_PERSPECTIVE_HEIGHT - 1],
            ])
            self.M = cv2.getPerspectiveTransform(src, dst)
        # Precalentar: descartar primeros fotogramas
        for _ in range(10):
            self.cap.read()
        # Verificar que entrega frames
        ok = False
        t0 = time.time()
        while time.time() - t0 < 3.0:
            r, _ = self.cap.read()
            if r:
                ok = True
                break
        if not ok:
            self.cap.release()
            raise RuntimeError(f"Cámara {camera_id} no entrega fotogramas. "
                               "Verificar --camera-id y que la fuente esté activa.")
        self.detector = create_aruco_detector(use_camera=True)

    def get_pose(self, python_id):
        """Devuelve (x, y, angulo_rad) del robot (id 0-3), o None.
        ArUco entrega angulo en grados; el controlador PID espera radianes."""
        ok, raw = self.cap.read()
        if not ok:
            return None
        frame = (cv2.warpPerspective(raw, self.M,
                 (CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT))
                 if self.M is not None else raw)
        _, datos = deteccion_jugadores_aruco_tag(frame, self.detector, draw=False)
        for d in datos:
            if d["id"] == python_id:
                return (float(d["x"]), float(d["y"]),
                        math.radians(float(d["angulo"])))
        return None

    def close(self):
        self.cap.release()


# ─── Función auxiliar ─────────────────────────────────────────────────────────
def _wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def _normalize_angle(a):
    """Normaliza ángulo en radianes a [-π, π]."""
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


# ─── Tramo: mueve de start a target, retorna métricas del tramo ───────────────
def run_segment(ctrl, robot, target, rf, fw_id, eyes, python_id):
    """Mueve robot a target con el PID real y mide tiempo, distancia y giro.

    Retorna dict con los datos del tramo, o None si hubo timeout o pérdida de
    detección antes de llegar.
    """
    # Registrar estado de inicio
    pose0 = eyes.get_pose(python_id)
    if pose0 is None:
        return None
    robot.x, robot.y, robot.angle = pose0   # angle en radianes (lo que espera el PID)
    x0, y0, a0_rad = pose0

    heading_to_target_rad = math.atan2(target[1] - y0, target[0] - x0)
    angle_error_deg = abs(math.degrees(_normalize_angle(heading_to_target_rad - a0_rad)))
    is_turn_segment = angle_error_deg > ANGLE_TURN_THR

    t_start = time.time()
    last_pose_t = t_start
    arrived = False

    while True:
        loop_t = time.time()
        elapsed = loop_t - t_start
        if elapsed > TIMEOUT_S:
            rf.set_motors(fw_id, 0, 0)
            return None   # timeout

        # Leer pose y actualizar robot proxy
        pose = eyes.get_pose(python_id)
        if pose is not None:
            robot.x, robot.y, robot.angle = pose
            last_pose_t = loop_t
        elif loop_t - last_pose_t > 1.5:
            rf.set_motors(fw_id, 0, 0)
            return None   # perdido por demasiado tiempo

        # PID → set_motors (el controlador envía el RF internamente)
        arrived = ctrl.move_to_position(robot, target)
        if arrived:
            break

        # Mantener cadencia del lazo
        dt = time.time() - loop_t
        if dt < LOOP_DT:
            time.sleep(LOOP_DT - dt)

    rf.set_motors(fw_id, 0, 0)
    t_end = time.time()

    pose1 = eyes.get_pose(python_id)
    if pose1 is None:
        pose1 = (robot.x, robot.y, robot.angle)

    dist_px = math.hypot(pose1[0] - x0, pose1[1] - y0)
    angle_turned_deg = abs(math.degrees(_normalize_angle(pose1[2] - a0_rad)))
    t_total = t_end - t_start

    return {
        "from": [round(x0), round(y0)],
        "to": [target[0], target[1]],
        "heading_error_deg": round(angle_error_deg, 1),
        "heading_start_rad": round(a0_rad, 4),   # para clasificar CW/CCW
        "is_turn_segment": is_turn_segment,
        "dist_px": round(dist_px, 1),
        "angle_turned_deg": round(angle_turned_deg, 1),
        "t_s": round(t_total, 3),
    }


# ─── Caracterización de un robot ──────────────────────────────────────────────
def characterize_one(ctrl, robot, rf, fw_id, eyes, python_id):
    segs, failed = [], 0

    print(f"  Llevando robot al primer waypoint {WAYPOINTS[0]}…")
    pose = eyes.get_pose(python_id)
    if pose is None:
        print(f"  ERROR: robot {fw_id} no detectado. Verificar ArUco y posición.")
        return None
    robot.x, robot.y, robot.angle = pose

    # Ir al primer punto para establecer posición de partida
    t0 = time.time()
    while time.time() - t0 < TIMEOUT_S:
        pose = eyes.get_pose(python_id)
        if pose:
            robot.x, robot.y, robot.angle = pose
        done = ctrl.move_to_position(robot, WAYPOINTS[0])
        if done:
            break
        time.sleep(LOOP_DT)
    rf.set_motors(fw_id, 0, 0)
    time.sleep(0.5)

    for lap in range(N_LAPS):
        for i, tgt in enumerate(WAYPOINTS[1:] + [WAYPOINTS[0]]):
            print(f"  vuelta {lap + 1}/{N_LAPS} tramo {i + 1}/{len(WAYPOINTS)}  → {tgt}", end=" ", flush=True)
            seg = run_segment(ctrl, robot, tgt, rf, fw_id, eyes, python_id)
            if seg is None:
                failed += 1
                print("✗ (timeout / sin detección)")
                # Intentar recuperar posición
                time.sleep(1.0)
                pose = eyes.get_pose(python_id)
                if pose:
                    robot.x, robot.y, robot.angle = pose
            else:
                segs.append(seg)
                print(f"✓  t={seg['t_s']}s  d={seg['dist_px']}px  Δθ={seg['heading_error_deg']}°")
            time.sleep(0.4)

    if not segs:
        return None

    # ── Derivar parámetros cinemáticos de los tramos medidos ─────────────────
    def _ms(xs):
        if not xs:
            return {"mean": None, "std": None, "n": 0}
        return {"mean": round(statistics.mean(xs), 2),
                "std": round(statistics.stdev(xs), 2) if len(xs) > 1 else 0.0,
                "n": len(xs)}

    # 1. Velocidad lineal: solo tramos sin giro previo (error ≤ umbral).
    #    t_s es casi puro avance, así que v = dist / t es directo.
    move_segs = [s for s in segs if not s["is_turn_segment"] and s["dist_px"] > 20]
    vs = [s["dist_px"] / s["t_s"] for s in move_segs if s["t_s"] > 0]
    mean_v = statistics.mean(vs) if vs else None

    # 2. Velocidad angular: tramos con giro previo (error > umbral).
    #    t_s incluye TANTO el giro como el avance posterior. Para aislar el
    #    tiempo de giro se descuenta el tiempo estimado de avance:
    #        t_giro = t_s - dist_px / v
    #    Si no tenemos v medida, se usa el tramo completo (estimación conservadora).
    turn_segs = [s for s in segs if s["is_turn_segment"] and s["angle_turned_deg"] > 10]

    def _omega(seg):
        t_turn = seg["t_s"]
        if mean_v and mean_v > 0:
            t_travel = seg["dist_px"] / mean_v
            t_turn = max(seg["t_s"] - t_travel, 0.1)  # al menos 0.1 s para el giro
        return seg["angle_turned_deg"] / t_turn if t_turn > 0 else None

    # 3. Clasificar CW vs CCW: el signo del ángulo de giro se obtiene del
    #    sentido que debió girar el robot para apuntar al siguiente waypoint.
    #    Guardamos el heading inicial (a0_rad) en el dict del segmento.
    cw_vals, ccw_vals = [], []
    all_omega_vals = []
    for s in turn_segs:
        w = _omega(s)
        if w is None:
            continue
        all_omega_vals.append(w)
        # Sentido de giro: ¿debía girar CW (delta negativo) o CCW (delta positivo)?
        heading_target = math.atan2(s["to"][1] - s["from"][1],
                                    s["to"][0] - s["from"][0])
        a_start = s.get("heading_start_rad")
        if a_start is not None:
            delta = _normalize_angle(heading_target - a_start)
            if delta < 0:
                cw_vals.append(w)
            else:
                ccw_vals.append(w)
        else:
            # Sin heading registrado: suma a ambos para no perder el dato
            cw_vals.append(w)
            ccw_vals.append(w)

    # Si no hay suficiente separación, usar todos para ambos sentidos
    if not cw_vals:
        cw_vals = all_omega_vals
    if not ccw_vals:
        ccw_vals = all_omega_vals

    # 4. Overheads estimados (arranque + frenado por tramo).
    #    Con más datos se puede ajustar; por ahora son conservadores.
    t_move_oh = 0.10
    t_turn_oh = 0.12

    return {
        "v_px_s": _ms(vs),
        "omega_cw_deg_s": _ms(cw_vals),
        "omega_ccw_deg_s": _ms(ccw_vals),
        "t_turn_overhead_s": t_turn_oh,
        "t_move_overhead_s": t_move_oh,
        "n_segments_ok": len(segs),
        "n_segments_failed": failed,
        "raw_segments": segs,
    }


# ─── Main ─────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robots", type=int, nargs="+", default=[1, 2, 3, 4],
                    help="firmware IDs a caracterizar (1-4)")
    ap.add_argument("--camera-id", type=int, default=2)
    ap.add_argument("--serial-port", type=str, default="/dev/ttyUSB0")
    ap.add_argument("--check", action="store_true",
                    help="Prueba solo el RF (sin cámara): pulsa motores de cada robot.")
    ap.add_argument("--append", action="store_true",
                    help="Carga el perfil más reciente y añade/actualiza los robots "
                         "indicados. Útil para caracterizar uno por vez sin perder "
                         "los datos anteriores.")
    args = ap.parse_args()

    print("=" * 64)
    print("  CARACTERIZACIÓN CINEMÁTICA — REQUIERE HARDWARE")
    print(f"  Robots: {args.robots} | cámara {args.camera_id} | {args.serial_port}")
    print(f"  Circuito: {len(WAYPOINTS)} puntos × {N_LAPS} vueltas/robot")
    print("=" * 64)

    rf = RFController(port=args.serial_port, enable_calibration=True)
    if not rf.initialize():
        print(f"ERROR: no se pudo inicializar RF en {args.serial_port}. "
              "Verificar puerto, permisos (grupo dialout) y que no esté ocupado.")
        return

    try:
        conns = rf.test_connections()
        for fw in args.robots:
            estado = "OK" if conns.get(f"robot_{fw}", False) else "NO responde"
            print(f"  RF robot {fw}: {estado}")
    except Exception as e:
        print(f"  (test_connections: {e})")

    if args.check:
        print("\n[--check] Pulsando motores (adelante 0,5 s por robot)…")
        try:
            for fw in args.robots:
                print(f"  Robot {fw}: adelante")
                rf.set_motors(fw, 65, 65)
                time.sleep(0.5)
                rf.set_motors(fw, 0, 0)
                time.sleep(0.5)
        finally:
            for fw in args.robots:
                rf.set_motors(fw, 0, 0)
            rf.shutdown()
        print("  Listo. Si se movieron, el RF está OK.")
        return

    print(f"\nCircuito de waypoints: {WAYPOINTS}")
    print("Asegure que cada robot esté en el campo y dentro del cuadro de cámara.\n")

    eyes = Eyes(args.camera_id)
    ctrl = DifferentialDriveController(rf_controller=rf)

    # --append: cargar perfil existente más reciente para acumular robots
    profile = {}
    if args.append:
        existing = sorted(glob.glob(str(ROOT_DIR / "LOG" / "motion_profile_*.json")))
        if existing:
            with open(existing[-1], encoding="utf-8") as fh:
                profile = json.load(fh)
            print(f"  Cargando perfil existente: {existing[-1]}")
            print(f"  Robots ya caracterizados: "
                  f"{[k for k in profile if k.isdigit()]}")
        else:
            print("  --append: no hay perfil previo, creando uno nuevo.")
    if not profile:
        profile = {"_provisional": False,
                   "source": "metrics_motion_characterization.py",
                   "field": {"width": CAMERA_PERSPECTIVE_WIDTH,
                              "height": CAMERA_PERSPECTIVE_HEIGHT},
                   "waypoints": WAYPOINTS,
                   "n_laps": N_LAPS,
                   "angle_turn_threshold_deg": ANGLE_TURN_THR}

    try:
        for fw in args.robots:
            python_id = fw - 1
            print(f"\n── Robot firmware {fw} (player {python_id}) ──────────────────")
            input("  Posicionar robot en el campo y presionar ENTER para empezar…")
            robot = _PlayerProxy(python_id)
            rec = characterize_one(ctrl, robot, rf, fw, eyes, python_id)
            if rec is None:
                print(f"  Robot {fw}: sin datos suficientes, se omite.")
                continue
            profile[str(python_id)] = rec
            print(f"  v={rec['v_px_s']}  cw={rec['omega_cw_deg_s']}  ccw={rec['omega_ccw_deg_s']}")
            rf.set_motors(fw, 0, 0)
            if fw != args.robots[-1]:
                print(f"  Pausa {PAUSE_BETWEEN_ROBOTS_S:.0f} s antes del siguiente robot…")
                time.sleep(PAUSE_BETWEEN_ROBOTS_S)

        # Perfil por defecto = promedio de robots medidos (respaldo)
        ids = [k for k in profile if k.isdigit()]
        if ids:
            def avg(field):
                vals = [profile[i][field]["mean"]
                        for i in ids if profile[i].get(field, {}).get("mean")]
                return round(statistics.mean(vals), 2) if vals else None
            profile["default"] = {
                "v_px_s": avg("v_px_s"),
                "omega_cw_deg_s": avg("omega_cw_deg_s"),
                "omega_ccw_deg_s": avg("omega_ccw_deg_s"),
                "t_turn_overhead_s": 0.12,
                "t_move_overhead_s": 0.10,
            }
    finally:
        for fw in args.robots:
            rf.set_motors(fw, 0, 0)
        rf.shutdown()
        eyes.close()

    out = save_metrics("motion_profile", profile)
    print(f"\n  Perfil guardado en: {out}")
    print("  Copiar a LOG/ para que metrics_role_decision_quality.py lo use "
          "(ya está ahí, se detecta automáticamente).")


if __name__ == "__main__":
    main()
