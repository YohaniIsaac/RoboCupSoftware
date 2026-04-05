#!/usr/bin/env python3
"""Test de integracion: 1 robot controlado por BehaviorManager.

PASO 2 del plan de integracion. Conecta el BehaviorManager al pipeline completo:
  Camara -> Perspectiva -> Deteccion robot + pelota -> BehaviorManager -> PID -> RF -> Motor

A diferencia del Paso 1 (PID directo hacia la pelota), aqui el arbol de
comportamiento decide QUE hacer: acercarse por el angulo optimo, capturar,
disparar al arco, etc.

Arquitectura: 3 procesos
  1. Percepcion: Detecta robot (ArUco) + pelota (HSV)
  2. Decision:   BehaviorManager toma decisiones y envia comandos RF
  3. Visualizacion: Muestra video con estado del arbol de comportamiento

Uso:
    python scripts/integration/test_behavior_1robot.py --robot-id 0
    python scripts/integration/test_behavior_1robot.py --robot-id 0 --serial-port /dev/ttyUSB0
    python scripts/integration/test_behavior_1robot.py --robot-id 0 --camera-id 2

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
    RANGO_COLOR_NARANJO, FIELD_CAM, CAPTURE_CONFIRM_DISTANCE_PX,
    DRIBBLER_HOLD_POWER,
    DRIBBLER_PULSE_ON_MS,
    DRIBBLER_PULSE_OFF_MS,
)
from robot_soccer.utils.camera_utils import get_camera_index

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(processName)-12s] %(levelname)-8s %(message)s',
    datefmt='%H:%M:%S'
)
# Desactivar DEBUG de BT/PID para reducir ruido (activar si se necesita depurar)
# logging.getLogger('robot_soccer.ai.behavior_tree.base').setLevel(logging.DEBUG)
# logging.getLogger('robot_soccer.controllers.robot_command_manager').setLevel(logging.DEBUG)
log = logging.getLogger(__name__)

SHM_NAME = "robot_behavior_1"


# =============================================================================
# PROCESO 1: Percepcion (robot + pelota) — igual que test_chase_ball
# =============================================================================

def perception_process(control_pipe, viz_pipe, robot_id, camera_id,
                       shm_name, frame_counter):
    """Detecta robot (ArUco) y pelota (HSV) en cada frame."""
    import cv2
    import numpy as np
    from robot_soccer.perception.player_tracking import create_aruco_detector

    log.info("Percepcion iniciada")

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
    detect_count = 0
    ball_detect_count = 0
    t0 = time.time()

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

            # Detectar robot (ArUco)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            robot_detected = False
            robot_data = None

            if ids is not None:
                for i, marker_id in enumerate(ids):
                    if marker_id[0] == robot_id:
                        pts = corners[i].reshape(4, 2)
                        cx = int(pts[:, 0].mean())
                        cy = int(pts[:, 1].mean())
                        vec = pts[1] - pts[0]
                        angle = float(np.degrees(np.arctan2(vec[1], vec[0])))
                        robot_data = {'x': cx, 'y': cy, 'angulo': angle}
                        robot_detected = True
                        detect_count += 1
                        break

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

            # Escribir frame a shared memory
            np.copyto(shared_array, frame)
            with frame_counter.get_lock():
                frame_counter.value += 1

            # Enviar datos a decision
            try:
                while control_pipe.poll():
                    _ = control_pipe.recv()
                control_pipe.send({
                    'robot_detected': robot_detected,
                    'robot_data': robot_data,
                    'ball_detected': ball_detected,
                    'ball_pos': ball_pos,
                    'timestamp': time.time()
                })
            except Exception:
                pass

            # Enviar a visualizacion
            try:
                while viz_pipe.poll():
                    _ = viz_pipe.recv()
                viz_pipe.send({
                    'robot_detected': robot_detected,
                    'robot_data': robot_data,
                    'ball_detected': ball_detected,
                    'ball_pos': ball_pos,
                })
            except Exception:
                pass

            # Stats cada 5s
            elapsed = time.time() - t0
            if frame_count % 150 == 0 and elapsed > 0:
                fps = frame_count / elapsed
                r_rate = detect_count / frame_count * 100
                b_rate = ball_detect_count / frame_count * 100
                log.info("FPS=%.1f | Robot=%.0f%% | Pelota=%.0f%%",
                         fps, r_rate, b_rate)

    finally:
        cap.release()
        shm.close()
        log.info("Percepcion finalizada")


# =============================================================================
# PROCESO 2: Decision (BehaviorManager)
# =============================================================================

def decision_process(perception_pipe, viz_state_pipe, keyboard_pipe,
                     robot_id, serial_port):
    """Instancia BehaviorManager y ejecuta arboles de comportamiento."""
    import numpy as np
    from robot_soccer.entities.player import Player
    from robot_soccer.entities.ball import Ball
    from robot_soccer.ai.behavior_tree.manager import BehaviorManager
    from robot_soccer.config import ROL_ATACANTE

    log.info("Decision iniciada para robot %d", robot_id)

    # Crear entidades del juego
    player = Player(robot_id, 0, 0, 0.0, team='red')
    player.set_rol(ROL_ATACANTE)
    ball = Ball(0, 0)

    # Crear BehaviorManager — gestiona internamente RFController
    robot_available = False
    try:
        behavior_manager = BehaviorManager(
            players=[player],
            ball=ball,
            team='red',
            use_real_robots=True,
            serial_port=serial_port,
            field=FIELD_CAM
        )
        robot_available = (behavior_manager.command_manager.rf_controller is not None
                           and behavior_manager.command_manager.use_real_robots)
        if robot_available:
            log.info("Robot %d disponible via RF", robot_id)
        else:
            log.warning("Robot %d NO disponible via RF — modo simulacion", robot_id)
    except Exception as e:
        log.error("Error inicializando BehaviorManager: %s", e)
        return

    player_initialized = False
    ball_initialized = False
    behavior_active = False
    running = True

    last_viz_time = 0
    VIZ_INTERVAL = 0.04   # ~25 Hz

    last_bt_tick = 0
    BT_TICK_INTERVAL = 0.1  # BT toma decisiones a 10 Hz, no a 100 Hz

    last_dribbler_keepalive = 0.0
    DRIBBLER_KEEPALIVE = 0.08  # 80ms < timeout firmware 100ms
                             # Esto da tiempo al PID de completar rotacion/movimiento
                             # entre cada decision del arbol de comportamiento
    dribbler_pulse_phase = 'on'   # fase actual del pulso: 'on' u 'off'
    dribbler_pulse_timer = 0.0    # tiempo de inicio de la fase actual
    prev_has_ball = False         # para detectar flanco de subida de _has_ball

    log.info("BehaviorManager listo. Presiona ESPACIO para activar comportamiento.")

    try:
        while running:
            now = time.time()

            # --- Recibir datos de percepcion ---
            if perception_pipe.poll():
                try:
                    data = perception_pipe.recv()

                    if data.get('robot_detected') and data.get('robot_data'):
                        rd = data['robot_data']
                        player.x = rd['x']
                        player.y = rd['y']
                        # Player.angle se mantiene en GRADOS para el BT
                        # (capture_ball, is_shot_possible usan operaciones en grados)
                        # La conversión a radianes se hace puntualmente para execute_commands()
                        player.angle = rd['angulo']
                        if not player_initialized:
                            log.info("Robot detectado en (%d, %d)", rd['x'], rd['y'])
                        player_initialized = True

                    if data.get('ball_detected') and data.get('ball_pos'):
                        bx, by = data['ball_pos']
                        ball.set_position(bx, by)
                        if not ball_initialized:
                            log.info("Pelota detectada en (%d, %d)", bx, by)
                        ball_initialized = True
                    elif ball_initialized:
                        # Pelota no visible — mantener ultima posicion conocida
                        pass

                except Exception:
                    pass

            # --- Recibir comandos de teclado ---
            if keyboard_pipe.poll():
                try:
                    cmd = keyboard_pipe.recv()
                    command = cmd.get('command', '')
                    if command == 'exit':
                        running = False
                    elif command == 'toggle':
                        behavior_active = not behavior_active
                        log.info("Comportamiento %s",
                                 "ACTIVADO" if behavior_active else "PAUSADO")
                        if not behavior_active and robot_available:
                            firmware_id = robot_id + 1
                            rf = behavior_manager.command_manager.rf_controller
                            rf.set_motors(firmware_id, 0, 0)
                            rf.set_dribbler(firmware_id, 0)  # apagar dribbler al pausar
                            player._has_ball = False  # resetear estado al pausar
                except Exception:
                    pass

            # --- Actualizar has_ball por proximidad (con histéresis) ---
            # Sin histéresis el flag se resetea cada frame cuando el robot rota
            # ligeramente al orientarse al arco, rompiendo orient_to_goal.
            # Umbral captura: CAPTURE_CONFIRM_DISTANCE_PX (mismo que BT)
            # Umbral liberación: 2x — permite rotación sin perder la pelota
            distance = None
            if player_initialized and ball_initialized:
                distance = float(player.distance_to_ball(ball))
                if distance < CAPTURE_CONFIRM_DISTANCE_PX:
                    player._has_ball = True
                elif distance > CAPTURE_CONFIRM_DISTANCE_PX * 2:
                    player._has_ball = False
                # Zona de histéresis: mantener valor actual

            # --- Actualizar contexto del juego ---
            if player_initialized and ball_initialized:
                # posesion: 0.0 si el robot tiene la pelota, 0.5 si esta libre
                posesion = 0.0 if player.has_ball() else 0.5

                # proximidad: siempre "cerca" con 1 robot sin rivales
                proximidad = 0.5

                # zona: posicion de la pelota en el campo (0=defensiva, 1=neutral, 2=ofensiva)
                zona = (ball.x / FIELD_CAM.width) * 2.0
                zona = max(0.0, min(2.0, zona))

                behavior_manager.update_game_context((posesion, proximidad, zona))

            # --- Dribbler keepalive con pulso intermitente ---
            if behavior_active and player._has_ball and robot_available:
                firmware_id = robot_id + 1
                rf = behavior_manager.command_manager.rf_controller

                # Resetear ciclo de pulso al capturar la pelota (flanco subida)
                if player._has_ball and not prev_has_ball:
                    dribbler_pulse_phase = 'on'
                    dribbler_pulse_timer = now

                pulse_on_s = DRIBBLER_PULSE_ON_MS / 1000.0
                pulse_off_s = DRIBBLER_PULSE_OFF_MS / 1000.0

                if pulse_off_s <= 0:
                    # Modo continuo
                    if now - last_dribbler_keepalive >= DRIBBLER_KEEPALIVE:
                        rf.set_dribbler(firmware_id, DRIBBLER_HOLD_POWER)
                        last_dribbler_keepalive = now
                else:
                    phase_elapsed = now - dribbler_pulse_timer

                    if dribbler_pulse_phase == 'on':
                        if now - last_dribbler_keepalive >= DRIBBLER_KEEPALIVE:
                            rf.set_dribbler(firmware_id, DRIBBLER_HOLD_POWER)
                            last_dribbler_keepalive = now
                        if phase_elapsed >= pulse_on_s:
                            rf.set_dribbler(firmware_id, 0)
                            dribbler_pulse_phase = 'off'
                            dribbler_pulse_timer = now
                    else:  # 'off'
                        if phase_elapsed >= pulse_off_s:
                            rf.set_dribbler(firmware_id, DRIBBLER_HOLD_POWER)
                            last_dribbler_keepalive = now
                            dribbler_pulse_phase = 'on'
                            dribbler_pulse_timer = now

            prev_has_ball = player._has_ball

            # --- Ejecutar comportamiento ---
            if behavior_active and player_initialized and ball_initialized:
                try:
                    # El BT toma decisiones a 10 Hz: setea el target (move_robot_to)
                    # BT usa player.angle en GRADOS (capture_ball, is_shot_possible)
                    if now - last_bt_tick >= BT_TICK_INTERVAL:
                        behavior_manager.update()
                        last_bt_tick = now

                    # execute_commands corre a 100 Hz: envia comandos RF cada loop.
                    # DifferentialDriveController espera robot.angle en RADIANES
                    # (hace: target_heading_rad - robot.angle directamente)
                    # Conversión temporal: grados → radianes → ejecutar → restaurar
                    angle_deg = player.angle
                    player.angle = math.radians(angle_deg)
                    behavior_manager.command_manager.execute_commands()
                    player.angle = angle_deg
                except Exception as e:
                    log.error("Error en BehaviorManager: %s", e)

            # --- Enviar estado a visualizacion ---
            if now - last_viz_time >= VIZ_INTERVAL:
                try:
                    blackboard = behavior_manager.blackboards.get(robot_id)
                    last_action = blackboard.last_action if blackboard else None

                    # Obtener target actual del command_manager
                    current_target = None
                    action_info = behavior_manager.command_manager.actions_in_progress.get(robot_id)
                    if action_info and 'target_pos' in action_info:
                        tp = action_info['target_pos']
                        try:
                            current_target = (int(tp[0]), int(tp[1]))
                        except Exception:
                            pass

                    viz_state_pipe.send({
                        'player_pos': (player.x, player.y) if player_initialized else None,
                        'robot_angle_deg': player.angle if player_initialized else None,
                        'ball_pos': (int(ball.x), int(ball.y)) if ball_initialized else None,
                        'behavior_active': behavior_active,
                        'has_ball': player.has_ball(),
                        'last_action': str(last_action) if last_action else None,
                        'current_target': current_target,
                        'distance': distance,
                        'robot_available': robot_available,
                        'timestamp': now
                    })
                    last_viz_time = now
                except Exception:
                    pass

            time.sleep(0.01)  # ~100 Hz

    finally:
        try:
            behavior_manager.shutdown()
        except Exception:
            pass
        log.info("Decision finalizada")


# =============================================================================
# PROCESO 3: Visualizacion
# =============================================================================

def visualization_process(perception_pipe, decision_pipe, keyboard_pipe,
                          shm_name, frame_counter):
    """Muestra video con overlays: robot, pelota, estado del behavior tree."""
    import cv2
    import numpy as np

    log.info("Visualizacion iniciada")

    frame_shape = (CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH, 3)
    shm = shared_memory.SharedMemory(name=shm_name)
    shared_array = np.ndarray(frame_shape, dtype=np.uint8, buffer=shm.buf)
    last_frame_counter = 0

    last_frame = None
    robot_data = None
    ball_pos_perc = None

    # Estado del decision process
    player_pos = None
    robot_angle_deg = None
    ball_pos_ctrl = None
    behavior_active = False
    has_ball = False
    last_action = None
    current_target = None
    distance = None
    robot_available = False

    window_name = 'Test Behavior 1 Robot'
    cv2.namedWindow(window_name)

    try:
        while True:
            # Leer frame
            current = frame_counter.value
            if current != last_frame_counter:
                last_frame = shared_array.copy()
                last_frame_counter = current

            # Leer metadata percepcion
            if perception_pipe.poll():
                try:
                    data = perception_pipe.recv()
                    robot_data = data.get('robot_data')
                    ball_pos_perc = data.get('ball_pos')
                except Exception:
                    pass

            # Leer estado decision
            if decision_pipe.poll():
                try:
                    data = decision_pipe.recv()
                    player_pos = data.get('player_pos')
                    robot_angle_deg = data.get('robot_angle_deg')
                    ball_pos_ctrl = data.get('ball_pos')
                    behavior_active = data.get('behavior_active', False)
                    has_ball = data.get('has_ball', False)
                    last_action = data.get('last_action')
                    current_target = data.get('current_target')
                    distance = data.get('distance')
                    robot_available = data.get('robot_available', False)
                except Exception:
                    pass

            if last_frame is not None:
                frame = last_frame.copy()

                # Dibujar campo: linea central y arcos
                h, w = frame.shape[:2]
                cv2.line(frame, (w // 2, 0), (w // 2, h), (40, 80, 40), 1)
                # Arco izquierdo
                cv2.rectangle(frame, (0, 196), (27, 295), (40, 80, 40), 1)
                # Arco derecho
                cv2.rectangle(frame, (616, 193), (640, 294), (40, 80, 40), 1)

                # Dibujar target del behavior tree (cruz magenta)
                if current_target:
                    tx, ty = current_target
                    arm = 10
                    cv2.line(frame, (tx - arm, ty), (tx + arm, ty),
                             (255, 0, 255), 2, cv2.LINE_AA)
                    cv2.line(frame, (tx, ty - arm), (tx, ty + arm),
                             (255, 0, 255), 2, cv2.LINE_AA)
                    cv2.circle(frame, (tx, ty), 3, (255, 0, 255), -1, cv2.LINE_AA)

                # Dibujar pelota
                ball_pos = ball_pos_ctrl or ball_pos_perc
                if ball_pos:
                    bx, by = ball_pos
                    color = (0, 255, 255) if has_ball else (0, 165, 255)
                    cv2.circle(frame, (bx, by), 12, color, 2, cv2.LINE_AA)
                    cv2.circle(frame, (bx, by), 2, color, -1, cv2.LINE_AA)

                # Dibujar robot
                rd = robot_data
                if rd:
                    rx, ry = rd['x'], rd['y']
                    r_color = (0, 255, 255) if has_ball else (0, 220, 0)
                    cv2.circle(frame, (rx, ry), 16, r_color, 2, cv2.LINE_AA)
                    angle_rad = math.radians(rd['angulo'])
                    ex = int(rx + 25 * math.cos(angle_rad))
                    ey = int(ry + 25 * math.sin(angle_rad))
                    cv2.arrowedLine(frame, (rx, ry), (ex, ey),
                                   r_color, 2, cv2.LINE_AA, tipLength=0.3)

                    # Linea robot -> target (si existe)
                    if current_target:
                        cv2.line(frame, (rx, ry), current_target,
                                (200, 0, 200), 1, cv2.LINE_AA)
                    elif ball_pos:
                        cv2.line(frame, (rx, ry), (ball_pos[0], ball_pos[1]),
                                (200, 200, 0), 1, cv2.LINE_AA)

                # --- Overlay de texto ---
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

                y += 18
                r_text = "Robot: OK" if rd else "Robot: --"
                r_color = (0, 255, 0) if rd else (0, 0, 200)
                cv2.putText(frame, r_text, (10, y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, r_color, 1)

                y += 18
                b_text = (f"Pelota: ({ball_pos[0]},{ball_pos[1]})"
                          if ball_pos else "Pelota: --")
                b_color = (0, 200, 255) if ball_pos else (0, 0, 200)
                cv2.putText(frame, b_text, (10, y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, b_color, 1)

                if distance is not None:
                    y += 18
                    cv2.putText(frame, f"Dist: {distance:.0f}px", (10, y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

                if has_ball:
                    y += 18
                    cv2.putText(frame, "TIENE PELOTA", (10, y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

                if last_action:
                    y += 18
                    # Truncar si es muy largo
                    action_text = f"Accion: {str(last_action)[:30]}"
                    cv2.putText(frame, action_text, (10, y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 255), 1)

                if current_target:
                    y += 18
                    cv2.putText(frame,
                               f"Target BT: ({current_target[0]},{current_target[1]})",
                               (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.40,
                               (255, 0, 255), 1)

                cv2.putText(frame, "ESPACIO=Activar/Pausar  ESC=Salir",
                           (10, CAMERA_PERSPECTIVE_HEIGHT - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

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
                except Exception:
                    pass

    except KeyboardInterrupt:
        pass
    finally:
        shm.close()
        cv2.destroyAllWindows()
        log.info("Visualizacion finalizada")


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Test de integracion: Robot controlado por BehaviorManager (Paso 2)')
    parser.add_argument('--robot-id', type=int, choices=[0, 1, 2, 3],
                        required=True, help='ID del robot (0-3)')
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

    robot_id = args.robot_id
    shm_name = f"{SHM_NAME}_{robot_id}"

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
    log.info("  TEST INTEGRACION: BEHAVIOR MANAGER (1 robot)")
    log.info("=" * 60)
    log.info("Robot ID: %d | Puerto: %s | Camara: /dev/video%d",
             robot_id, args.serial_port, camera_id)
    log.info("Rol: ATACANTE | Equipo: red | Campo: FIELD_CAM (640x480)")
    log.info("=" * 60)

    processes = []

    p1 = multiprocessing.Process(
        target=perception_process,
        args=(perc_to_dec_s, perc_to_viz_s, robot_id, camera_id,
              shm_name, frame_counter),
        name="Perception"
    )
    processes.append(p1)

    p2 = multiprocessing.Process(
        target=decision_process,
        args=(perc_to_dec_r, dec_to_viz_s, viz_to_dec_r,
              robot_id, args.serial_port),
        name="Decision"
    )
    processes.append(p2)

    p3 = multiprocessing.Process(
        target=visualization_process,
        args=(perc_to_viz_r, dec_to_viz_r, viz_to_dec_s,
              shm_name, frame_counter),
        name="Visualization"
    )
    processes.append(p3)

    try:
        for proc in processes:
            proc.start()
            log.info("  %s iniciado (PID: %d)", proc.name, proc.pid)

        log.info("")
        log.info("Sistema corriendo. Presiona ESPACIO en la ventana para activar BT.")
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
