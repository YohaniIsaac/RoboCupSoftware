#!/usr/bin/env python3
"""Test de integración: partido 2v2 completo con dos equipos autónomos.

PASO 7 del plan de integración. Valida el partido completo 2vs2:
  - Equipo rojo (robots 0, 1): ataca arco derecho
  - Equipo azul (robots 2, 3): ataca arco izquierdo
  - Ambos equipos con BehaviorManager independiente, RFController compartido
  - Detección de gol en ambos arcos con marcador y pausa automática

Pipeline:
  Cámara → Perspectiva → Detección (4 robots ArUco + pelota HSV + zonas de gol)
         → Decision2v2 (BM-rojo + BM-azul, serial único)
         → PID → RF → Motores

Arquitectura: 3 procesos
  1. Percepción:   4 ArUcos + pelota + detección de gol
  2. Decision 2v2: BM-rojo y BM-azul en el mismo proceso, un solo serial
  3. Visualización: campo con 4 robots, marcador, overlays de gol

Uso:
    python scripts/integration/test_behavior_2v2.py
    python scripts/integration/test_behavior_2v2.py --serial-port /dev/ttyUSB0

Controles (ventana OpenCV):
    ESPACIO : Activar/pausar AMBOS equipos
    R       : Activar/pausar solo equipo rojo   (debug: verificar un equipo)
    B       : Activar/pausar solo equipo azul   (debug: verificar un equipo)
    ESC     : Salir
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
    BALL_OUT_MARGIN_PX,
    RESET_POS,
)
from robot_soccer.utils.camera_utils import get_camera_index
from robot_soccer.core.process.decision_process import decision_process_2v2

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(processName)-12s] %(levelname)-8s %(message)s',
    datefmt='%H:%M:%S'
)
log = logging.getLogger(__name__)

SHM_NAME       = "robot_soccer_2v2"
BALL_RADIUS_PX = 15
GOAL_COOLDOWN_S  = 5.0

# Robots por equipo
TEAM_RED_IDS  = [0, 1]
TEAM_BLUE_IDS = [2, 3]
ALL_IDS       = TEAM_RED_IDS + TEAM_BLUE_IDS

# Colores (BGR): rojo oscuro, rojo claro, azul oscuro, azul claro
ROBOT_COLORS = {
    0: (60,  60, 220),
    1: (80,  80, 200),
    2: (220, 80,  60),
    3: (200, 100, 80),
}
TEAM_COLOR = {'red': (60, 60, 220), 'blue': (220, 80, 60)}
ROL_LABELS = {'atacante': 'A', 'defensor': 'D'}


# =============================================================================
# PROCESO 1: Percepción
# =============================================================================

def perception_process_2v2(control_pipe, viz_pipe, camera_id, shm_name, frame_counter):
    import cv2
    import numpy as np
    from robot_soccer.perception.player_tracking import create_aruco_detector

    log.info("Percepcion 2v2 iniciada — robots %s", ALL_IDS)

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
            [0, CAMERA_PERSPECTIVE_HEIGHT - 1],
        ])
        perspective_matrix = cv2.getPerspectiveTransform(src, dst)

    detector     = create_aruco_detector(use_camera=True)
    robot_ids_set = set(ALL_IDS)

    frame_shape  = (CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH, 3)
    shm          = shared_memory.SharedMemory(name=shm_name)
    shared_array = np.ndarray(frame_shape, dtype=np.uint8, buffer=shm.buf)

    ball_lower = np.array(RANGO_COLOR_NARANJO[0])
    ball_upper = np.array(RANGO_COLOR_NARANJO[1])

    frame_count   = 0
    detect_counts = {rid: 0 for rid in ALL_IDS}
    ball_det_count = 0
    t0 = time.time()
    last_goal_time = 0.0

    try:
        while True:
            ret, raw_frame = cap.read()
            if not ret:
                continue

            frame = (cv2.warpPerspective(raw_frame, perspective_matrix,
                     (CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT))
                     if perspective_matrix is not None else raw_frame)
            frame_count += 1

            # ArUco
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            robots_data = {}
            if ids is not None:
                for i, mid in enumerate(ids.flatten()):
                    if mid in robot_ids_set:
                        pts = corners[i].reshape(4, 2)
                        cx  = int(pts[:, 0].mean())
                        cy  = int(pts[:, 1].mean())
                        vec = pts[1] - pts[0]
                        ang = float(np.degrees(np.arctan2(vec[1], vec[0])))
                        robots_data[int(mid)] = {'x': cx, 'y': cy, 'angulo': ang}
                        detect_counts[int(mid)] = detect_counts.get(int(mid), 0) + 1

            # Pelota HSV
            hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, ball_lower, ball_upper)
            k    = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)

            ball_detected = False
            ball_pos      = None
            contours, _   = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) > 10:
                    (bx, by), radius = cv2.minEnclosingCircle(largest)
                    if 2 <= radius <= 30:
                        ball_pos      = (int(bx), int(by))
                        ball_detected = True
                        ball_det_count += 1

            # Detección de gol en ambos arcos
            goal_event = None
            if ball_detected and ball_pos is not None:
                bx, by = ball_pos
                now    = time.time()
                # Rojo anota: arco derecho
                if (bx >= FIELD_CAM.goal_right_x - BALL_RADIUS_PX and
                        FIELD_CAM.goal_right_top_y <= by <= FIELD_CAM.goal_right_bottom_y):
                    if now - last_goal_time >= GOAL_COOLDOWN_S:
                        goal_event     = 'red'
                        last_goal_time = now
                        log.info("GOL equipo ROJO en (%d, %d)", bx, by)
                # Azul anota: arco izquierdo
                elif (bx <= FIELD_CAM.goal_left_x + BALL_RADIUS_PX and
                        FIELD_CAM.goal_left_top_y <= by <= FIELD_CAM.goal_left_bottom_y):
                    if now - last_goal_time >= GOAL_COOLDOWN_S:
                        goal_event     = 'blue'
                        last_goal_time = now
                        log.info("GOL equipo AZUL en (%d, %d)", bx, by)

            # Pelota fuera de límites (excluye zonas de arco)
            ball_out = False
            if ball_detected and ball_pos is not None:
                bx, by = ball_pos
                m = BALL_OUT_MARGIN_PX
                if by < m or by > CAMERA_PERSPECTIVE_HEIGHT - m:
                    ball_out = True
                elif (bx < m and
                      not (FIELD_CAM.goal_left_top_y <= by <= FIELD_CAM.goal_left_bottom_y)):
                    ball_out = True
                elif (bx > CAMERA_PERSPECTIVE_WIDTH - m and
                      not (FIELD_CAM.goal_right_top_y <= by <= FIELD_CAM.goal_right_bottom_y)):
                    ball_out = True

            # Shared memory
            np.copyto(shared_array, frame)
            with frame_counter.get_lock():
                frame_counter.value += 1

            payload = {
                'robots': robots_data,
                'ball_detected': ball_detected,
                'ball_pos': ball_pos,
                'ball_out': ball_out,
                'goal_event': goal_event,
                'timestamp': time.time(),
            }
            for pipe in (control_pipe, viz_pipe):
                try:
                    while pipe.poll():
                        _ = pipe.recv()
                    pipe.send(payload)
                except Exception:
                    pass

            # Stats cada 5s
            elapsed = time.time() - t0
            if frame_count % 150 == 0 and elapsed > 0:
                fps    = frame_count / elapsed
                rates  = " | ".join(
                    f"R{rid}={detect_counts.get(rid,0)/frame_count*100:.0f}%"
                    for rid in ALL_IDS
                )
                b_rate = ball_det_count / frame_count * 100
                log.info("FPS=%.1f | %s | Pelota=%.0f%%", fps, rates, b_rate)

    finally:
        cap.release()
        shm.close()
        log.info("Percepcion 2v2 finalizada")


# =============================================================================
# PROCESO 3: Visualización
# =============================================================================

def visualization_process_2v2(perception_pipe, decision_pipe, keyboard_pipe,
                               shm_name, frame_counter):
    import cv2
    import numpy as np

    log.info("Visualizacion 2v2 iniciada")

    frame_shape  = (CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH, 3)
    shm          = shared_memory.SharedMemory(name=shm_name)
    shared_array = np.ndarray(frame_shape, dtype=np.uint8, buffer=shm.buf)

    last_frame_counter = 0
    last_frame = None

    robots_perc   = {}
    ball_pos_perc = None
    players_state = {}
    ball_pos_ctrl = None
    active_red    = False
    active_blue   = False
    robot_available = False
    prev_robot_available = False

    score          = {'red': 0, 'blue': 0}
    last_goal_team = None
    goal_reset_viz = False   # True → esperando SPACE tras gol
    ball_out_viz   = False   # True → pelota fuera de límites
    pre_init_viz   = True    # True → robots quietos, esperando ESPACIO1
    init_phase_viz = False   # True → robots organizándose (entre ESPACIO1 y ESPACIO2)
    init_ready_viz = False   # True → robots listos, esperando ESPACIO2 para iniciar
    ball_detected_viz = False

    window_name = 'Partido 2v2 — RoboCup'
    cv2.namedWindow(window_name)

    try:
        while True:
            now = time.time()

            # Frame
            current = frame_counter.value
            if current != last_frame_counter:
                last_frame         = shared_array.copy()
                last_frame_counter = current

            # Percepción
            if perception_pipe.poll():
                try:
                    data          = perception_pipe.recv()
                    robots_perc   = data.get('robots', {})
                    ball_pos_perc = data.get('ball_pos')
                    goal_event    = data.get('goal_event')
                    if goal_event in ('red', 'blue'):
                        score[goal_event] += 1
                        last_goal_team = goal_event
                        goal_reset_viz = True
                        tablero_cmd = 2 if goal_event == 'red' else 3
                        try:
                            keyboard_pipe.send({'command': 'goal_scored', 'team': goal_event})
                            keyboard_pipe.send({'command': 'tablero', 'cmd': tablero_cmd})
                        except Exception:
                            pass
                        log.info("GOL %s! Marcador R%d - A%d",
                                 goal_event.upper(), score['red'], score['blue'])
                except Exception:
                    pass

            # Decision
            if decision_pipe.poll():
                try:
                    data            = decision_pipe.recv()
                    players_state   = data.get('players', {})
                    ball_pos_ctrl   = data.get('ball_pos')
                    active_red      = data.get('active_red', False)
                    active_blue     = data.get('active_blue', False)
                    robot_available = data.get('robot_available', False)
                    ball_out_viz    = data.get('ball_out', False)
                    pre_init_viz      = data.get('pre_init', False)
                    init_phase_viz    = data.get('init_phase', False)
                    init_ready_viz    = data.get('init_ready', False)
                    ball_detected_viz = data.get('ball_detected', False)
                    if not data.get('goal_reset', False):
                        goal_reset_viz = False   # decision confirmó salida del reset
                    if robot_available and not prev_robot_available:
                        try:
                            keyboard_pipe.send({'command': 'tablero', 'cmd': 4})
                            keyboard_pipe.send({'command': 'tablero', 'cmd': 5})
                        except Exception:
                            pass
                    prev_robot_available = robot_available
                except Exception:
                    pass

            if last_frame is not None:
                frame = last_frame.copy()
                h, w  = frame.shape[:2]

                # Campo
                cv2.line(frame, (w // 2, 0), (w // 2, h), (40, 80, 40), 1)
                # Arco izquierdo — azul
                cv2.rectangle(frame,
                    (0, FIELD_CAM.goal_left_top_y),
                    (FIELD_CAM.goal_left_x, FIELD_CAM.goal_left_bottom_y),
                    TEAM_COLOR['blue'], 2)
                cv2.putText(frame, "<- AZUL",
                    (4, FIELD_CAM.goal_left_top_y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, TEAM_COLOR['blue'], 1)
                # Arco derecho — rojo
                cv2.rectangle(frame,
                    (FIELD_CAM.goal_right_x, FIELD_CAM.goal_right_top_y),
                    (w, FIELD_CAM.goal_right_bottom_y),
                    TEAM_COLOR['red'], 2)
                cv2.putText(frame, "ROJO ->",
                    (FIELD_CAM.goal_right_x + 2, FIELD_CAM.goal_right_top_y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, TEAM_COLOR['red'], 1)

                ball_pos = ball_pos_ctrl or ball_pos_perc

                # Robots
                for rid in ALL_IDS:
                    pstate   = players_state.get(rid, {})
                    rd       = robots_perc.get(rid)
                    pos      = pstate.get('pos')
                    angle_deg = pstate.get('angle_deg')
                    rol      = pstate.get('rol', 'defensor')
                    has_ball = pstate.get('has_ball', False)
                    target   = pstate.get('target')
                    team     = pstate.get('team', 'red' if rid in TEAM_RED_IDS else 'blue')

                    if pos is None and rd:
                        pos       = (rd['x'], rd['y'])
                        angle_deg = rd['angulo']

                    base_color   = ROBOT_COLORS.get(rid, (180, 180, 180))
                    robot_color  = (0, 255, 255) if has_ball else base_color

                    if target:
                        tx, ty = target
                        arm = 8
                        cv2.line(frame, (tx-arm, ty), (tx+arm, ty), (255, 0, 255), 2, cv2.LINE_AA)
                        cv2.line(frame, (tx, ty-arm), (tx, ty+arm), (255, 0, 255), 2, cv2.LINE_AA)

                    if pos:
                        rx, ry = pos
                        if target and rol == 'atacante':
                            cv2.line(frame, (rx, ry), target, (200, 0, 200), 1, cv2.LINE_AA)
                        cv2.circle(frame, (rx, ry), 16, robot_color, 2, cv2.LINE_AA)
                        if angle_deg is not None:
                            ar = math.radians(angle_deg)
                            ex = int(rx + 25 * math.cos(ar))
                            ey = int(ry + 25 * math.sin(ar))
                            cv2.arrowedLine(frame, (rx, ry), (ex, ey),
                                            robot_color, 2, cv2.LINE_AA, tipLength=0.3)
                        rol_l = ROL_LABELS.get(rol, '?')
                        cv2.putText(frame, f"{rid}{rol_l}", (rx - 8, ry - 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, robot_color, 2)

                # Pelota
                if ball_pos:
                    bx, by = ball_pos
                    any_has = any(players_state.get(r, {}).get('has_ball', False)
                                  for r in ALL_IDS)
                    bcol = (0, 255, 255) if any_has else (0, 165, 255)
                    cv2.circle(frame, (bx, by), 12, bcol, 2, cv2.LINE_AA)
                    cv2.circle(frame, (bx, by),  2, bcol, -1, cv2.LINE_AA)

                # Estado BT por equipo (esquina superior izquierda)
                def _bt_label(active, label):
                    return f"BT {label}: {'ACTIVO' if active else 'PAUSADO'}"
                y = 24
                for active, label, color in (
                    (active_red,  "ROJO", TEAM_COLOR['red']),
                    (active_blue, "AZUL", TEAM_COLOR['blue']),
                ):
                    c = color if active else (80, 80, 80)
                    cv2.putText(frame, _bt_label(active, label), (10, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1)
                    y += 18

                rf_col = (0, 220, 80) if robot_available else (80, 80, 80)
                cv2.putText(frame, "RF: OK" if robot_available else "RF: --",
                            (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, rf_col, 1)
                y += 16

                for rid in ALL_IDS:
                    pstate = players_state.get(rid, {})
                    pos    = pstate.get('pos')
                    rol    = pstate.get('rol', '?')
                    act    = pstate.get('last_action')
                    hball  = pstate.get('has_ball', False)
                    bc     = ROBOT_COLORS.get(rid, (160, 160, 160))
                    rc     = (0, 255, 255) if hball else bc
                    ps     = f"({pos[0]},{pos[1]})" if pos else "--"
                    as_    = f" {str(act)[:14]}" if act else ""
                    cv2.putText(frame, f"R{rid}[{ROL_LABELS.get(rol,'?')}] {ps}{as_}",
                                (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.37, rc, 1)
                    y += 16

                # Marcador centrado
                score_text = f"ROJO: {score['red']}    AZUL: {score['blue']}"
                (tw, _), _ = cv2.getTextSize(score_text, cv2.FONT_HERSHEY_SIMPLEX, 0.75, 2)
                cv2.putText(frame, score_text, ((w - tw) // 2, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)

                cv2.putText(frame, "ESPACIO=Ambos  R=Rojo  B=Azul  ESC=Salir",
                            (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1)

                # Overlay pre-juego: pre_init (quieto) / init_phase (organizando) / init_ready (listo)
                if pre_init_viz or init_phase_viz:
                    robots_seen = sum(
                        1 for rid in ALL_IDS
                        if players_state.get(rid, {}).get('pos') is not None
                    )
                    overlay = frame.copy()
                    cv2.rectangle(overlay, (0, h // 3), (w, 2 * h // 3), (15, 40, 15), -1)
                    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

                    if pre_init_viz:
                        if robots_seen == len(ALL_IDS):
                            main_txt = "ROBOTS DETECTADOS"
                            sub_txt  = "Presiona ESPACIO para organizar"
                            txt_col  = (80, 220, 80)
                        else:
                            main_txt = f"Detectando robots... ({robots_seen}/{len(ALL_IDS)})"
                            sub_txt  = "Posiciona todos los robots en el campo"
                            txt_col  = (80, 160, 220)
                    elif init_ready_viz:
                        ball_ok   = ball_detected_viz
                        main_txt  = "ROBOTS LISTOS"
                        sub_txt   = ("Presiona ESPACIO para iniciar el partido"
                                     if ball_ok else
                                     "Coloca la pelota en el campo y presiona ESPACIO")
                        txt_col   = (60, 220, 60) if ball_ok else (30, 160, 220)
                    else:
                        main_txt = "ORGANIZANDO ROBOTS..."
                        sub_txt  = "Espera que los robots lleguen a sus posiciones"
                        txt_col  = (80, 220, 80)

                    (tw_m, th_m), _ = cv2.getTextSize(main_txt, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)
                    cv2.putText(frame, main_txt,
                                ((w - tw_m) // 2, h // 2 - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, txt_col, 2, cv2.LINE_AA)
                    (tw_s, _), _ = cv2.getTextSize(sub_txt, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
                    cv2.putText(frame, sub_txt,
                                ((w - tw_s) // 2, h // 2 + th_m + 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1, cv2.LINE_AA)

                    # Hint de posición de pelota (centro del campo)
                    bx, by = w // 2, h // 2
                    hint_col = (40, 200, 40) if ball_detected_viz else (40, 140, 255)
                    cv2.circle(frame, (bx, by), 38, hint_col, 2, cv2.LINE_AA)
                    cv2.circle(frame, (bx, by), 10, hint_col, -1, cv2.LINE_AA)
                    cv2.line(frame, (bx - 14, by), (bx + 14, by), (0, 0, 0), 1, cv2.LINE_AA)
                    cv2.line(frame, (bx, by - 14), (bx, by + 14), (0, 0, 0), 1, cv2.LINE_AA)
                    pelota_lbl = "PELOTA OK" if ball_detected_viz else "PELOTA AQUI"
                    cv2.putText(frame, pelota_lbl,
                                (bx - 38, by + 54),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42, hint_col, 1, cv2.LINE_AA)

                    # Marcadores X en posiciones de inicio
                    for rid, rpos in RESET_POS.items():
                        rc  = TEAM_COLOR['red'] if rid in TEAM_RED_IDS else TEAM_COLOR['blue']
                        rx, ry = rpos
                        arm = 12
                        cv2.line(frame, (rx-arm, ry-arm), (rx+arm, ry+arm), rc, 2, cv2.LINE_AA)
                        cv2.line(frame, (rx+arm, ry-arm), (rx-arm, ry+arm), rc, 2, cv2.LINE_AA)
                        cv2.circle(frame, rpos, arm + 3, rc, 1, cv2.LINE_AA)

                # Overlay GOOOL! + PRESS SPACE + marcadores reset
                if goal_reset_viz and last_goal_team is not None:
                    overlay     = frame.copy()
                    banner_y1   = h // 3
                    banner_y2   = 2 * h // 3
                    banner_col  = ((0, 0, 160)   if last_goal_team == 'red'
                                   else (160, 0, 0))
                    text_col    = (TEAM_COLOR['red'] if last_goal_team == 'red'
                                   else TEAM_COLOR['blue'])
                    goal_label  = ("GOOOL!  ROJO" if last_goal_team == 'red'
                                   else "GOOOL!  AZUL")
                    cv2.rectangle(overlay, (0, banner_y1), (w, banner_y2), banner_col, -1)
                    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
                    (gw, gh), _ = cv2.getTextSize(goal_label, cv2.FONT_HERSHEY_SIMPLEX, 2.0, 4)
                    gy = (banner_y1 + banner_y2 + gh) // 2
                    cv2.putText(frame, goal_label, ((w - gw) // 2, gy),
                                cv2.FONT_HERSHEY_SIMPLEX, 2.0, text_col, 4, cv2.LINE_AA)
                    sub_text = "Presiona ESPACIO para continuar"
                    (sw, _), _ = cv2.getTextSize(sub_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                    cv2.putText(frame, sub_text, ((w - sw) // 2, banner_y2 + 24),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 1, cv2.LINE_AA)
                    # Marcadores X en posiciones de reset
                    for rid, rpos in RESET_POS.items():
                        rc  = TEAM_COLOR['red'] if rid in TEAM_RED_IDS else TEAM_COLOR['blue']
                        rx, ry = rpos
                        arm = 10
                        cv2.line(frame, (rx-arm, ry-arm), (rx+arm, ry+arm), rc, 2, cv2.LINE_AA)
                        cv2.line(frame, (rx+arm, ry-arm), (rx-arm, ry+arm), rc, 2, cv2.LINE_AA)
                        cv2.circle(frame, rpos, arm + 2, rc, 1, cv2.LINE_AA)

                # Overlay PELOTA FUERA
                if ball_out_viz:
                    hc = h // 2
                    cv2.rectangle(frame, (0, hc - 28), (w, hc + 28), (20, 20, 20), -1)
                    out_text = "PELOTA FUERA"
                    (tw_o, _), _ = cv2.getTextSize(out_text, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)
                    cv2.putText(frame, out_text, ((w - tw_o) // 2, hc + 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 2, cv2.LINE_AA)

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
                goal_reset_viz = False
                try:
                    keyboard_pipe.send({'command': 'toggle'})
                    # Enviar señal tablero sólo cuando el partido arranca de verdad
                    # (desde init_ready) o cuando se toggle durante juego activo
                    if init_ready_viz or (not pre_init_viz and not init_phase_viz):
                        keyboard_pipe.send({'command': 'tablero', 'cmd': 1})
                except Exception:
                    pass
            elif key in (ord('r'), ord('R')):
                try:
                    keyboard_pipe.send({'command': 'toggle_red'})
                except Exception:
                    pass
            elif key in (ord('b'), ord('B')):
                try:
                    keyboard_pipe.send({'command': 'toggle_blue'})
                except Exception:
                    pass

    except KeyboardInterrupt:
        pass
    finally:
        shm.close()
        cv2.destroyAllWindows()
        log.info("Visualizacion 2v2 finalizada")


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Test de integración: partido 2v2 con dos equipos autónomos (Paso 7)')
    parser.add_argument('--serial-port', default='/dev/ttyUSB0',
                        help='Puerto serial compartido (default: /dev/ttyUSB0)')
    parser.add_argument('--camera-id', type=int, default=None,
                        help='ID de cámara (auto-detecta si no se especifica)')
    args = parser.parse_args()

    if args.camera_id is None:
        camera_id = get_camera_index(prefer_droidcam=True, fallback_index=0)
        log.info("Cámara auto-detectada: /dev/video%d", camera_id)
    else:
        camera_id = args.camera_id

    shm_name = SHM_NAME

    try:
        old = shared_memory.SharedMemory(name=shm_name)
        old.close()
        old.unlink()
    except FileNotFoundError:
        pass

    frame_size = CAMERA_PERSPECTIVE_HEIGHT * CAMERA_PERSPECTIVE_WIDTH * 3
    shm        = shared_memory.SharedMemory(create=True, name=shm_name, size=frame_size)
    frame_counter = Value('i', 0)

    perc_to_dec_s, perc_to_dec_r = multiprocessing.Pipe()
    perc_to_viz_s, perc_to_viz_r = multiprocessing.Pipe()
    dec_to_viz_s,  dec_to_viz_r  = multiprocessing.Pipe()
    viz_to_dec_s,  viz_to_dec_r  = multiprocessing.Pipe()

    log.info("=" * 65)
    log.info("  PARTIDO 2v2 — RoboCup Soccer")
    log.info("=" * 65)
    log.info("Equipo ROJO  : robots %s  (arco derecho)", TEAM_RED_IDS)
    log.info("Equipo AZUL  : robots %s  (arco izquierdo)", TEAM_BLUE_IDS)
    log.info("Puerto serial: %s", args.serial_port)
    log.info("Cámara       : /dev/video%d", camera_id)
    log.info("Campo        : FIELD_CAM %dx%d", CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT)
    log.info("=" * 65)
    log.info("Secuencia de inicio:")
    log.info("  1. Verificar que los 4 robots aparecen en la visualizacion")
    log.info("  2. Colocar la pelota en el centro del campo (indicador naranja)")
    log.info("  3. Presionar ESPACIO: robots se organizan en posiciones de inicio")
    log.info("  4. Esperar a que el indicador diga ROBOTS LISTOS")
    log.info("  5. Presionar ESPACIO: partido inicia (solo si robots y pelota OK)")
    log.info("  (R / B: activar solo rojo / azul para pruebas)")
    log.info("=" * 65)

    p1 = multiprocessing.Process(
        target=perception_process_2v2,
        args=(perc_to_dec_s, perc_to_viz_s, camera_id, shm_name, frame_counter),
        name="Perception"
    )
    p2 = multiprocessing.Process(
        target=decision_process_2v2,
        args=(perc_to_dec_r, dec_to_viz_s, viz_to_dec_r,
              TEAM_RED_IDS, TEAM_BLUE_IDS, args.serial_port),
        name="Decision2v2"
    )
    p3 = multiprocessing.Process(
        target=visualization_process_2v2,
        args=(perc_to_viz_r, dec_to_viz_r, viz_to_dec_s, shm_name, frame_counter),
        name="Visualization"
    )

    processes = [p1, p2, p3]
    try:
        for proc in processes:
            proc.start()
            log.info("  %s iniciado (PID: %d)", proc.name, proc.pid)
        log.info("")
        log.info("Sistema corriendo. Presiona ESPACIO para organizar robots, luego ESPACIO de nuevo para iniciar.")
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
        log.info("Partido 2v2 finalizado")


if __name__ == '__main__':
    main()
