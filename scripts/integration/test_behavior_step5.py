#!/usr/bin/env python3
"""Test de integracion: ciclo de partido completo con deteccion de gol y marcador.

PASO 5 del plan de integracion. Valida el ciclo completo de partido:
  - Atacante empuja pelota al arco rival (arco derecho para equipo rojo)
  - Sistema detecta cuando la pelota entra al arco
  - Marcador se actualiza y BT se pausa 3s
  - BT se reanuda automaticamente para el siguiente gol

Pipeline completo:
  Camara -> Perspectiva -> Deteccion robot 0 + robot 1 + pelota + zona de arco
         -> BehaviorManager([p0, p1]) -> PID -> RF -> Motores

Que se valida:
  - Ciclo completo: posicionarse -> avanzar -> patear -> detectar gol
  - Deteccion de zona de arco con coordenadas reales de FIELD_CAM
  - Marcador visual persistente entre goles
  - Pausa y reanudacion automatica del BT tras gol (sin intervencion manual)
  - Cooldown anti-spam: sin eventos duplicados si la pelota se queda en el arco

Arquitectura: 3 procesos
  1. Percepcion:    Detecta robots (ArUco) + pelota (HSV) + zona de gol
  2. Decision:      BehaviorManager([player0, player1])  <- src/robot_soccer/core/process/
  3. Visualizacion: Video con overlay: marcador, GOOOL!, rol A/D, targets, estado BT

Uso:
    python scripts/integration/test_behavior_step5.py
    python scripts/integration/test_behavior_step5.py --robot-ids 0 1
    python scripts/integration/test_behavior_step5.py --robot-ids 0 1 --serial-port /dev/ttyUSB0

Controles:
    ESPACIO: START/STOP - Activar o pausar el BehaviorManager
    ESC: Salir
"""

import sys
import math
import time
import logging
import argparse
import multiprocessing
from multiprocessing import Value, shared_memory
from pathlib import Path

ROOT_DIR = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))
sys.path.insert(0, str(ROOT_DIR / "scripts"))

from robot_soccer.config import (
    CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH,
    CAMERA_PERSPECTIVE_ENABLED, CAMERA_PERSPECTIVE_SRC_POINTS,
    RANGO_COLOR_NARANJO,
    FIELD_CAM,
)
from robot_soccer.utils.camera_utils import get_camera_index
from robot_soccer.core.process.decision_process import decision_process

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(processName)-12s] %(levelname)-8s %(message)s',
    datefmt='%H:%M:%S'
)
log = logging.getLogger(__name__)

SHM_NAME = "robot_behavior_step5"

BALL_RADIUS_PX   = 15    # radio aproximado de la pelota en px para deteccion de gol
GOAL_COOLDOWN_S  = 5.0   # cooldown minimo entre eventos de gol (evita duplicados)
POST_GOAL_PAUSE_S = 3.0  # segundos que el BT queda pausado tras un gol


# =============================================================================
# PROCESO 1: Percepcion multi-robot con deteccion de zona de gol
# =============================================================================

def perception_process_step5(control_pipe, viz_pipe, robot_ids, camera_id,
                              shm_name, frame_counter):
    """Detecta robots ArUco + pelota HSV + evento de gol por zona."""
    import cv2
    import numpy as np
    from robot_soccer.perception.player_tracking import create_aruco_detector

    log.info("Percepcion step5 iniciada para robots %s", robot_ids)
    robot_ids_set = set(robot_ids)

    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        log.error("No se pudo abrir camara %d", camera_id)
        return

    perspective_matrix = None
    if CAMERA_PERSPECTIVE_ENABLED:
        src = np.float32(CAMERA_PERSPECTIVE_SRC_POINTS)
        dst = np.float32([
            [0, 0],
            [CAMERA_PERSPECTIVE_WIDTH - 1, 0],
            [CAMERA_PERSPECTIVE_WIDTH - 1, CAMERA_PERSPECTIVE_HEIGHT - 1],
            [0, CAMERA_PERSPECTIVE_HEIGHT - 1]
        ])
        perspective_matrix = cv2.getPerspectiveTransform(src, dst)

    detector = create_aruco_detector(use_camera=True)

    frame_shape = (CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH, 3)
    shm = shared_memory.SharedMemory(name=shm_name)
    shared_array = np.ndarray(frame_shape, dtype=np.uint8, buffer=shm.buf)

    ball_lower = np.array(RANGO_COLOR_NARANJO[0])
    ball_upper = np.array(RANGO_COLOR_NARANJO[1])

    frame_count = 0
    detect_counts = {rid: 0 for rid in robot_ids}
    ball_detect_count = 0
    t0 = time.time()
    last_goal_time = 0.0

    try:
        while True:
            ret, raw_frame = cap.read()
            if not ret:
                continue

            if perspective_matrix is not None:
                frame = cv2.warpPerspective(
                    raw_frame, perspective_matrix,
                    (CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT)
                )
            else:
                frame = raw_frame

            frame_count += 1

            # Detectar TODOS los markers ArUco de interes
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            robots_data = {}
            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id in robot_ids_set:
                        pts = corners[i].reshape(4, 2)
                        cx = int(pts[:, 0].mean())
                        cy = int(pts[:, 1].mean())
                        vec = pts[1] - pts[0]
                        angle = float(np.degrees(np.arctan2(vec[1], vec[0])))
                        robots_data[int(marker_id)] = {'x': cx, 'y': cy, 'angulo': angle}
                        detect_counts[int(marker_id)] = detect_counts.get(int(marker_id), 0) + 1

            # Detectar pelota (HSV)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, ball_lower, ball_upper)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

            ball_detected = False
            ball_pos = None
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) > 10:
                    (bx, by), radius = cv2.minEnclosingCircle(largest)
                    if 2 <= radius <= 30:
                        ball_pos = (int(bx), int(by))
                        ball_detected = True
                        ball_detect_count += 1

            # Deteccion de gol por zona
            goal_event = None
            if ball_detected and ball_pos is not None:
                bx, by = ball_pos
                now = time.time()
                # Equipo rojo ataca a la derecha: gol en arco derecho
                if (bx >= FIELD_CAM.goal_right_x - BALL_RADIUS_PX and
                        FIELD_CAM.goal_right_top_y <= by <= FIELD_CAM.goal_right_bottom_y):
                    if now - last_goal_time >= GOAL_COOLDOWN_S:
                        goal_event = 'red'
                        last_goal_time = now
                        log.info("GOL! Equipo ROJO. Pelota en (%d, %d)", bx, by)
                # Gol en arco izquierdo (autogol o equipo azul en 2v2 futuro)
                elif (bx <= FIELD_CAM.goal_left_x + BALL_RADIUS_PX and
                        FIELD_CAM.goal_left_top_y <= by <= FIELD_CAM.goal_left_bottom_y):
                    if now - last_goal_time >= GOAL_COOLDOWN_S:
                        goal_event = 'blue'
                        last_goal_time = now
                        log.info("GOL! Equipo AZUL. Pelota en (%d, %d)", bx, by)

            # Escribir frame a shared memory
            np.copyto(shared_array, frame)
            with frame_counter.get_lock():
                frame_counter.value += 1

            # Enviar datos a decision (goal_event ignorado por decision_process)
            try:
                while control_pipe.poll():
                    _ = control_pipe.recv()
                control_pipe.send({
                    'robots': robots_data,
                    'ball_detected': ball_detected,
                    'ball_pos': ball_pos,
                    'timestamp': time.time(),
                    'goal_event': goal_event,
                })
            except Exception:
                pass

            # Enviar a visualizacion
            try:
                while viz_pipe.poll():
                    _ = viz_pipe.recv()
                viz_pipe.send({
                    'robots': robots_data,
                    'ball_detected': ball_detected,
                    'ball_pos': ball_pos,
                    'goal_event': goal_event,
                })
            except Exception:
                pass

            # Stats cada 5s
            elapsed = time.time() - t0
            if frame_count % 150 == 0 and elapsed > 0:
                fps = frame_count / elapsed
                r_rates = {rid: detect_counts.get(rid, 0) / frame_count * 100
                           for rid in robot_ids}
                b_rate = ball_detect_count / frame_count * 100
                rates_str = " | ".join(f"R{rid}={r_rates[rid]:.0f}%" for rid in robot_ids)
                log.info("FPS=%.1f | %s | Pelota=%.0f%%", fps, rates_str, b_rate)

    finally:
        cap.release()
        shm.close()
        log.info("Percepcion step5 finalizada")


# =============================================================================
# PROCESO 2: Decision — robot_soccer.core.process.decision_process
# =============================================================================
# Importado arriba; lanzado como proceso en main() con robot_ids=[0, 1].


# =============================================================================
# PROCESO 3: Visualizacion con marcador y overlay GOOOL!
# =============================================================================

def visualization_process_step5(perception_pipe, decision_pipe, keyboard_pipe,
                                 robot_ids, shm_name, frame_counter):
    """Muestra video con overlays: marcador, GOOOL!, rol A/D, targets, estado BT."""
    import cv2
    import numpy as np

    log.info("Visualizacion step5 iniciada")

    frame_shape = (CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH, 3)
    shm = shared_memory.SharedMemory(name=shm_name)
    shared_array = np.ndarray(frame_shape, dtype=np.uint8, buffer=shm.buf)
    last_frame_counter = 0
    last_frame = None

    # Estado de percepcion
    robots_perc = {}
    ball_pos_perc = None

    # Estado de decision
    players_state = {}
    ball_pos_ctrl = None
    behavior_active = False
    robot_available = False

    window_name = 'Test Behavior Step5 - Gol Detection'
    cv2.namedWindow(window_name)

    # Colores por robot
    ROBOT_COLORS = {
        robot_ids[0]: (0, 220, 0),     # verde — atacante inicial
        robot_ids[1]: (220, 100, 0),   # azul-naranja — defensor inicial
    }
    ROL_LABELS = {'atacante': 'A', 'defensor': 'D'}

    # Estado de marcador y gol
    score = {'red': 0, 'blue': 0}
    goal_display_until = 0.0
    last_goal_team = None
    goal_resume_sent = False

    # Tablero: reset automático una vez cuando la conexión RF está disponible
    prev_robot_available = False

    try:
        while True:
            now_viz = time.time()

            # Auto-reanuda BT despues de la pausa por gol
            if (goal_display_until > 0
                    and now_viz > goal_display_until
                    and not goal_resume_sent):
                try:
                    keyboard_pipe.send({'command': 'toggle'})
                except Exception:
                    pass
                goal_resume_sent = True
                log.info("Reanudando BT tras gol")

            # Leer frame
            current = frame_counter.value
            if current != last_frame_counter:
                last_frame = shared_array.copy()
                last_frame_counter = current

            # Leer metadata percepcion (incluye goal_event)
            if perception_pipe.poll():
                try:
                    data = perception_pipe.recv()
                    robots_perc = data.get('robots', {})
                    ball_pos_perc = data.get('ball_pos')
                    goal_event = data.get('goal_event')
                    if goal_event in ('red', 'blue'):
                        score[goal_event] += 1
                        last_goal_team = goal_event
                        goal_display_until = time.time() + POST_GOAL_PAUSE_S
                        goal_resume_sent = False
                        tablero_cmd = 2 if goal_event == 'red' else 3
                        try:
                            keyboard_pipe.send({'command': 'toggle'})
                            keyboard_pipe.send({'command': 'tablero', 'cmd': tablero_cmd})
                        except Exception:
                            pass
                        log.info("GOL %s! Marcador R%d - A%d",
                                 goal_event.upper(), score['red'], score['blue'])
                except Exception:
                    pass

            # Leer estado decision
            if decision_pipe.poll():
                try:
                    data = decision_pipe.recv()
                    players_state = data.get('players', {})
                    ball_pos_ctrl = data.get('ball_pos')
                    behavior_active = data.get('behavior_active', False)
                    robot_available = data.get('robot_available', False)
                    # Reset tablero una sola vez al conectar RF
                    if robot_available and not prev_robot_available:
                        try:
                            keyboard_pipe.send({'command': 'tablero', 'cmd': 4})  # reset goles
                            keyboard_pipe.send({'command': 'tablero', 'cmd': 5})  # reset tiempo
                            log.info("Tablero: goles y tiempo reseteados")
                        except Exception:
                            pass
                    prev_robot_available = robot_available
                except Exception:
                    pass

            if last_frame is not None:
                frame = last_frame.copy()
                h, w = frame.shape[:2]

                # Dibujar campo: linea central y arcos con colores de equipo
                cv2.line(frame, (w // 2, 0), (w // 2, h), (40, 80, 40), 1)
                # Arco izquierdo (azul)
                cv2.rectangle(frame,
                    (0, FIELD_CAM.goal_left_top_y),
                    (FIELD_CAM.goal_left_x, FIELD_CAM.goal_left_bottom_y),
                    (255, 80, 80), 2)
                # Arco derecho (rojo — objetivo del equipo rojo)
                cv2.rectangle(frame,
                    (FIELD_CAM.goal_right_x, FIELD_CAM.goal_right_top_y),
                    (w, FIELD_CAM.goal_right_bottom_y),
                    (80, 80, 255), 2)
                # Indicador de direccion de ataque
                cv2.arrowedLine(frame,
                    (w // 2 + 10, FIELD_CAM.goal_right_top_y - 20),
                    (w // 2 + 50, FIELD_CAM.goal_right_top_y - 20),
                    (80, 80, 255), 2, tipLength=0.4)
                cv2.putText(frame, "ROJO ->",
                    (w // 2 + 55, FIELD_CAM.goal_right_top_y - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (80, 80, 255), 1)

                ball_pos = ball_pos_ctrl or ball_pos_perc

                # Dibujar cada robot
                for rid in robot_ids:
                    pstate = players_state.get(rid, {})
                    rd = robots_perc.get(rid)

                    pos = pstate.get('pos')
                    angle_deg = pstate.get('angle_deg')
                    rol = pstate.get('rol', 'defensor')
                    has_ball = pstate.get('has_ball', False)
                    target = pstate.get('target')

                    # Usar datos de percepcion si decision aun no los tiene
                    if pos is None and rd:
                        pos = (rd['x'], rd['y'])
                        angle_deg = rd['angulo']

                    base_color = ROBOT_COLORS.get(rid, (180, 180, 180))
                    robot_color = (0, 255, 255) if has_ball else base_color

                    # Dibujar target (cruz)
                    if target:
                        tx, ty = target
                        arm = 8
                        cv2.line(frame, (tx - arm, ty), (tx + arm, ty),
                                 (255, 0, 255), 2, cv2.LINE_AA)
                        cv2.line(frame, (tx, ty - arm), (tx, ty + arm),
                                 (255, 0, 255), 2, cv2.LINE_AA)

                    if pos:
                        rx, ry = pos

                        # Linea robot → target (solo atacante)
                        if target and rol == 'atacante':
                            cv2.line(frame, (rx, ry), target,
                                     (200, 0, 200), 1, cv2.LINE_AA)
                        elif ball_pos and rol == 'atacante' and not target:
                            cv2.line(frame, (rx, ry), ball_pos,
                                     (200, 200, 0), 1, cv2.LINE_AA)

                        # Circulo del robot
                        cv2.circle(frame, (rx, ry), 16, robot_color, 2, cv2.LINE_AA)

                        # Flecha de angulo
                        if angle_deg is not None:
                            angle_rad = math.radians(angle_deg)
                            ex = int(rx + 25 * math.cos(angle_rad))
                            ey = int(ry + 25 * math.sin(angle_rad))
                            cv2.arrowedLine(frame, (rx, ry), (ex, ey),
                                           robot_color, 2, cv2.LINE_AA, tipLength=0.3)

                        # Etiqueta: ID + rol (A/D)
                        rol_label = ROL_LABELS.get(rol, '?')
                        label = f"{rid}{rol_label}"
                        cv2.putText(frame, label, (rx - 8, ry - 20),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.55, robot_color, 2)

                # Dibujar pelota
                if ball_pos:
                    bx, by = ball_pos
                    any_has_ball = any(
                        players_state.get(rid, {}).get('has_ball', False)
                        for rid in robot_ids
                    )
                    ball_color = (0, 255, 255) if any_has_ball else (0, 165, 255)
                    cv2.circle(frame, (bx, by), 12, ball_color, 2, cv2.LINE_AA)
                    cv2.circle(frame, (bx, by), 2, ball_color, -1, cv2.LINE_AA)

                # --- Overlay de estado (esquina superior izquierda) ---
                y = 25
                status = "BT: ACTIVO" if behavior_active else "BT: PAUSADO"
                color = (0, 255, 0) if behavior_active else (0, 0, 255)
                cv2.putText(frame, status, (10, y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

                y += 22
                rf_text = "RF: OK" if robot_available else "RF: --"
                rf_color = (0, 255, 0) if robot_available else (100, 100, 100)
                cv2.putText(frame, rf_text, (10, y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, rf_color, 1)

                # Estado por robot
                for rid in robot_ids:
                    y += 18
                    pstate = players_state.get(rid, {})
                    pos = pstate.get('pos')
                    rol = pstate.get('rol', '?')
                    last_action = pstate.get('last_action')
                    has_ball = pstate.get('has_ball', False)

                    base_color = ROBOT_COLORS.get(rid, (180, 180, 180))
                    r_color = (0, 255, 255) if has_ball else base_color
                    pos_str = f"({pos[0]},{pos[1]})" if pos else "--"
                    action_str = f" {str(last_action)[:16]}" if last_action else ""
                    label = f"R{rid}[{ROL_LABELS.get(rol,'?')}] {pos_str}{action_str}"
                    cv2.putText(frame, label, (10, y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.38, r_color, 1)

                # --- Marcador (parte superior, centrado) ---
                score_text = f"ROJO: {score['red']}   AZUL: {score['blue']}"
                (tw, _th), _ = cv2.getTextSize(score_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                cv2.putText(frame, score_text, ((w - tw) // 2, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

                cv2.putText(frame, "ESPACIO=Activar/Pausar  ESC=Salir",
                           (10, CAMERA_PERSPECTIVE_HEIGHT - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

                # --- Overlay GOOOL! ---
                if now_viz < goal_display_until and last_goal_team is not None:
                    overlay = frame.copy()
                    banner_y1 = h // 3
                    banner_y2 = 2 * h // 3
                    if last_goal_team == 'red':
                        banner_color = (0, 0, 180)
                        text_color = (80, 80, 255)
                        goal_label = "GOOOL!  ROJO"
                    else:
                        banner_color = (180, 0, 0)
                        text_color = (255, 80, 80)
                        goal_label = "GOOOL!  AZUL"
                    cv2.rectangle(overlay, (0, banner_y1), (w, banner_y2), banner_color, -1)
                    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
                    (gw, gh), _ = cv2.getTextSize(goal_label, cv2.FONT_HERSHEY_SIMPLEX, 2.0, 4)
                    gy = (banner_y1 + banner_y2 + gh) // 2
                    cv2.putText(frame, goal_label, ((w - gw) // 2, gy),
                                cv2.FONT_HERSHEY_SIMPLEX, 2.0, text_color, 4, cv2.LINE_AA)

                cv2.imshow(window_name, frame)
            else:
                placeholder = np.zeros(frame_shape, dtype=np.uint8)
                cv2.putText(placeholder, "Esperando frames...",
                           (CAMERA_PERSPECTIVE_WIDTH // 2 - 100,
                            CAMERA_PERSPECTIVE_HEIGHT // 2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2)
                cv2.imshow(window_name, placeholder)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                try:
                    keyboard_pipe.send({'command': 'exit'})
                except Exception:
                    pass
                break
            elif key == ord(' '):
                try:
                    keyboard_pipe.send({'command': 'toggle'})
                    keyboard_pipe.send({'command': 'tablero', 'cmd': 1})  # toggle cronometro
                except Exception:
                    pass

    except KeyboardInterrupt:
        pass
    finally:
        shm.close()
        cv2.destroyAllWindows()
        log.info("Visualizacion step5 finalizada")


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Test de integracion: ciclo de partido completo con deteccion de gol (Paso 5)')
    parser.add_argument('--robot-ids', nargs='+', type=int, default=[0, 1],
                        help='IDs de los robots (default: 0 1)')
    parser.add_argument('--serial-port', type=str, default='/dev/ttyUSB0',
                        help='Puerto serial (default: /dev/ttyUSB0)')
    parser.add_argument('--camera-id', type=int, default=None,
                        help='ID de camara (auto-detecta si no se especifica)')
    args = parser.parse_args()

    if args.camera_id is None:
        camera_id = get_camera_index(prefer_droidcam=True, fallback_index=0)
        log.info("Camara auto-detectada: /dev/video%d", camera_id)
    else:
        camera_id = args.camera_id

    robot_ids = args.robot_ids
    shm_name = f"{SHM_NAME}_{'_'.join(str(r) for r in robot_ids)}"

    # Limpiar SHM huerfana
    try:
        old = shared_memory.SharedMemory(name=shm_name)
        old.close()
        old.unlink()
    except FileNotFoundError:
        pass

    frame_size = CAMERA_PERSPECTIVE_HEIGHT * CAMERA_PERSPECTIVE_WIDTH * 3
    shm = shared_memory.SharedMemory(create=True, name=shm_name, size=frame_size)
    frame_counter = Value('i', 0)

    # Pipes
    perc_to_dec_s, perc_to_dec_r = multiprocessing.Pipe()
    perc_to_viz_s, perc_to_viz_r = multiprocessing.Pipe()
    dec_to_viz_s, dec_to_viz_r = multiprocessing.Pipe()
    viz_to_dec_s, viz_to_dec_r = multiprocessing.Pipe()

    log.info("=" * 60)
    log.info("  TEST INTEGRACION: BEHAVIOR MANAGER + GOL DETECTION (2 robots)")
    log.info("=" * 60)
    log.info("Robot IDs: %s | Puerto: %s | Camara: /dev/video%d",
             robot_ids, args.serial_port, camera_id)
    log.info("Roles iniciales: R%d=ATACANTE, R%d=DEFENSOR",
             robot_ids[0], robot_ids[1])
    log.info("Campo: FIELD_CAM (%dx%d) | Arco derecho: x>=%d",
             CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT, FIELD_CAM.goal_right_x)
    log.info("Pausa post-gol: %.1fs | Cooldown: %.1fs", POST_GOAL_PAUSE_S, GOAL_COOLDOWN_S)
    log.info("=" * 60)

    processes = []

    p1 = multiprocessing.Process(
        target=perception_process_step5,
        args=(perc_to_dec_s, perc_to_viz_s, robot_ids, camera_id,
              shm_name, frame_counter),
        name="Perception"
    )
    processes.append(p1)

    p2 = multiprocessing.Process(
        target=decision_process,
        args=(perc_to_dec_r, dec_to_viz_s, viz_to_dec_r,
              robot_ids, args.serial_port),
        name="Decision"
    )
    processes.append(p2)

    p3 = multiprocessing.Process(
        target=visualization_process_step5,
        args=(perc_to_viz_r, dec_to_viz_r, viz_to_dec_s,
              robot_ids, shm_name, frame_counter),
        name="Visualization"
    )
    processes.append(p3)

    try:
        for proc in processes:
            proc.start()
            log.info("  %s iniciado (PID: %d)", proc.name, proc.pid)

        log.info("")
        log.info("Sistema corriendo. Presiona ESPACIO en la ventana para activar BT.")
        log.info("Ambos robots deben ser visibles antes de activar.")
        log.info("=" * 60)

        for proc in processes:
            proc.join()

    except KeyboardInterrupt:
        log.info("Interrumpido por usuario")
        for proc in processes:
            if proc.is_alive():
                proc.terminate()
                proc.join(timeout=2)
    finally:
        try:
            shm.close()
            shm.unlink()
        except Exception:
            pass
        log.info("Test finalizado")


if __name__ == '__main__':
    main()
