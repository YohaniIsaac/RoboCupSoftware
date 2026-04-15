#!/usr/bin/env python3
"""Test de integracion: path planning RRT* con 1 robot.

PASO 2.5 del plan de integracion. Valida el pipeline de planificacion de rutas
end-to-end con un robot real:

  Camara -> Perspectiva -> ArUco -> [Planning RRT*] -> PID -> RF -> Motor

El planificador corre en un proceso separado para no bloquear el control PID.
La percepcion detecta TODOS los robots con ArUco. Los robots no controlados se
convierten en obstaculos dinamicos: si se mueven, se replannea automaticamente.

Arquitectura: 4 procesos
  1. Percepcion:  Detecta robot (ArUco), escribe frame a shared memory
  2. Planning:    Ejecuta RRT* Smart, produce waypoints
  3. Control:     Sigue waypoints via PID, envia comandos RF
  4. Visualizacion: Dibuja frame + ruta planificada + estado

Uso:
    python scripts/integration/test_path_planning_1robot.py --robot-id 0 --goal-x 500 --goal-y 400
    python scripts/integration/test_path_planning_1robot.py --robot-id 0 --goal-x 500 --goal-y 400 --serial-port /dev/ttyUSB0
    python scripts/integration/test_path_planning_1robot.py --robot-id 0  # pide goal interactivamente

Controles:
    ESPACIO: START/STOP - Activar o pausar el seguimiento de ruta
    R:       Forzar replanificacion (resetea posicion enviada al planner)
    ESC:     Salir
"""

import sys
import math
import time
import queue
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
    FIELD_CAM,
    RRT_STEP_LEN, RRT_GOAL_SAMPLE_RATE, RRT_SEARCH_RADIUS, RRT_ITER_MAX,
)
from robot_soccer.utils.camera_utils import get_camera_index

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(processName)-12s] %(levelname)-8s %(message)s',
    datefmt='%H:%M:%S'
)
log = logging.getLogger(__name__)

SHM_NAME = "robot_path_plan"

# Umbral de distancia (px) para enviar nueva posicion al planner
REPLAN_POSITION_THRESHOLD = 30
# Umbral de llegada a waypoints intermedios (px)
WAYPOINT_ARRIVAL_THRESHOLD = 20
# Radio (px) con que se representa cada robot obstáculo como circulo
ROBOT_OBSTACLE_RADIUS = 30
# Cuánto debe moverse un obstáculo (px) para disparar replanificacion
OBSTACLE_MOVE_THRESHOLD = 40
# Intervalo de envio de estado a visualizacion
VIZ_INTERVAL_S = 0.04


# =============================================================================
# PROCESO 1: Percepcion (ArUco)
# =============================================================================

def perception_process(ctrl_pipe, viz_pipe, robot_id, camera_id,
                       shm_name, frame_counter):
    """Detecta todos los robots con ArUco y escribe frames a shared memory.

    Detecta TODOS los marcadores ArUco visibles. El robot controlado se
    identifica por robot_id; el resto se incluye en all_robots para que
    el control los trate como obstaculos dinamicos.
    """
    import cv2
    import numpy as np
    from robot_soccer.perception.player_tracking import (
        create_aruco_detector, deteccion_jugadores_aruco_tag
    )

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

    frame_count = 0
    detect_count = 0
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

            # Detectar TODOS los robots con ArUco (draw=False para no modificar frame)
            _, all_robots = deteccion_jugadores_aruco_tag(frame, detector, draw=False)
            # all_robots = [{'id': int, 'x': int, 'y': int, 'angulo': float, ...}, ...]

            robot_data = next((r for r in all_robots if r['id'] == robot_id), None)
            robot_detected = robot_data is not None
            if robot_detected:
                detect_count += 1

            # Escribir frame a shared memory
            np.copyto(shared_array, frame)
            with frame_counter.get_lock():
                frame_counter.value += 1

            msg = {
                'robot_detected': robot_detected,
                'robot_data': robot_data,
                'all_robots': all_robots,    # Lista completa para obstaculos dinamicos
                'timestamp': time.time()
            }

            # Enviar a control (descartar stale)
            try:
                while ctrl_pipe.poll():
                    ctrl_pipe.recv()
                ctrl_pipe.send(msg)
            except Exception:
                pass

            # Enviar a visualizacion (descartar stale)
            try:
                while viz_pipe.poll():
                    viz_pipe.recv()
                viz_pipe.send(msg)
            except Exception:
                pass

            # Stats cada 5s
            elapsed = time.time() - t0
            if frame_count % 150 == 0 and elapsed > 0:
                fps = frame_count / elapsed
                r_rate = detect_count / frame_count * 100
                obs_count = sum(1 for r in all_robots if r['id'] != robot_id)
                log.info("FPS=%.1f | Robot=%.0f%% | Obstaculos detectados: %d",
                         fps, r_rate, obs_count)

    finally:
        cap.release()
        shm.close()
        log.info("Percepcion finalizada")


# =============================================================================
# PROCESO 2: Planning (RRT* Smart)
# =============================================================================

def planning_process(ctrl_to_plan_pipe, path_queue, goal_pos):
    """Ejecuta RRT* Smart en proceso separado. Nunca bloquea al control.

    Recibe posicion del robot Y lista de obstaculos actuales por pipe.
    Los obstaculos son dinamicos: incluyen robots detectados + estaticos del CLI.
    Pone el resultado en path_queue (maxsize=1, descarta paths obsoletos).
    """
    from robot_soccer.ai.path_planning.rrt_star_smart import RrtStarSmart

    log.info("Planning iniciado | Goal: %s", goal_pos)

    rrt = RrtStarSmart(
        step_len=RRT_STEP_LEN,
        goal_sample_rate=RRT_GOAL_SAMPLE_RATE,
        search_radius=RRT_SEARCH_RADIUS,
        iter_max=RRT_ITER_MAX,
        field=FIELD_CAM,        # Fix #1: usar coordenadas de camara (640x480)
    )

    try:
        while True:
            # Esperar nueva posicion+obstaculos del robot (bloquea hasta recibir)
            try:
                data = ctrl_to_plan_pipe.recv()
            except EOFError:
                break

            robot_pos = data['robot_pos']
            obstacles = data.get('obstacles', [])   # Dinamicos: otros robots + estaticos
            log.info("Planificando desde %s hacia %s | %d obstaculos ...",
                     robot_pos, goal_pos, len(obstacles))

            t0 = time.time()
            try:
                rrt.setup(robot_pos, goal_pos, obstacles, field=FIELD_CAM)
                rrt.planning()
            except Exception as e:
                log.error("Error en planning(): %s", e)
                continue

            elapsed = time.time() - t0

            path = rrt.path
            if path and len(path) > 0:
                # El path sale en orden goal→start; invertir a start→goal
                path = list(reversed(path))
                path = [(int(p[0]), int(p[1])) for p in path]
                log.info("Path encontrado: %d waypoints en %.2f s", len(path), elapsed)
                try:
                    # Descartar path anterior si la cola estaba llena
                    try:
                        path_queue.get_nowait()
                    except Exception:
                        pass
                    path_queue.put_nowait({'path': path})
                except Exception:
                    pass
            else:
                log.warning("RRT* no encontro ruta en %.2f s", elapsed)

    except KeyboardInterrupt:
        pass
    finally:
        log.info("Planning finalizado")


# =============================================================================
# PROCESO 3: Control (PID + RF)
# =============================================================================

def _obstacles_moved(last_positions, all_robots, robot_id):
    """Retorna True si algun obstaculo se movio mas de OBSTACLE_MOVE_THRESHOLD px,
    o si el numero de obstaculos cambio (aparecio/desaparecio un robot).
    """
    current_ids = {r['id'] for r in all_robots if r['id'] != robot_id}
    if current_ids != set(last_positions.keys()):
        return True
    for r in all_robots:
        if r['id'] == robot_id:
            continue
        if r['id'] in last_positions:
            lx, ly = last_positions[r['id']]
            if math.hypot(r['x'] - lx, r['y'] - ly) > OBSTACLE_MOVE_THRESHOLD:
                return True
    return False


def control_process(perc_pipe, ctrl_to_plan_pipe, path_queue,
                    viz_pipe, keyboard_pipe, robot_id, serial_port,
                    goal_pos, static_obstacles):
    """Sigue waypoints del path planner via PID y RF.

    Extrae robots obstaculos de la percepcion, los combina con static_obstacles
    y los envia al planner. Replannea si el robot se mueve o si los obstaculos
    cambian significativamente.
    """
    from robot_soccer.controllers.differential_drive import DifferentialDriveController
    from robot_soccer.communication.rf_controller import RFController

    log.info("Control iniciado para robot %d", robot_id)

    # --- RF ---
    rf_controller = None
    robot_available = False
    try:
        rf_controller = RFController(port=serial_port, enable_calibration=True)
        if rf_controller.initialize():
            connections = rf_controller.test_connections()
            robot_key = f'robot_{robot_id + 1}'
            robot_available = connections.get(robot_key, False)
            if robot_available:
                log.info("Robot %d disponible via RF", robot_id)
            else:
                log.warning("Robot %d NO responde via RF", robot_id)
    except Exception as e:
        log.warning("Error RF: %s — control sin RF", e)

    controller = DifferentialDriveController(rf_controller=rf_controller)
    firmware_id = robot_id + 1

    # --- Robot entity ---
    class RobotEntity:
        def __init__(self, rid):
            self.id = rid
            self.x = 0.0
            self.y = 0.0
            self.angle = 0.0  # radianes
            self.dx = self.dy = self.dw = 0.0

    robot = None
    current_path = []       # [(x, y), ...] en orden start→goal
    current_wp_idx = 0
    active = False
    goal_reached = False
    running = True

    # Posicion que se envio al planner la ultima vez (para filtrar threshold)
    last_sent_pos = None
    force_replan = False
    # Posiciones de obstaculos enviadas al planner la ultima vez {robot_id: (x,y)}
    last_obstacle_positions = {}
    # Obstaculos actuales (dinamicos + estaticos) para enviar al planner
    current_obstacles = list(static_obstacles)

    last_viz_time = 0
    robot_lost_time = None   # timestamp cuando se perdio deteccion
    ROBOT_LOST_TIMEOUT = 0.5  # s antes de detener motores por seguridad
    all_robots = []          # Ultimo listado completo de robots detectados

    log.info("Control listo. Presiona ESPACIO en la ventana para iniciar.")

    try:
        while running:
            now = time.time()

            # --- Percepcion ---
            if perc_pipe.poll():
                try:
                    data = perc_pipe.recv()
                    all_robots = data.get('all_robots', [])

                    if data.get('robot_detected') and data.get('robot_data'):
                        rd = data['robot_data']
                        if robot is None:
                            robot = RobotEntity(robot_id)
                            log.info("Robot detectado en (%d, %d)", rd['x'], rd['y'])
                        robot.x = rd['x']
                        robot.y = rd['y']
                        robot.angle = math.radians(rd['angulo'])
                        robot_lost_time = None
                    else:
                        all_robots = data.get('all_robots', [])
                        if robot is not None and robot_lost_time is None:
                            robot_lost_time = now

                    # Construir lista de obstaculos dinamica (otros robots + estaticos)
                    dynamic_obstacles = [
                        [r['x'], r['y'], ROBOT_OBSTACLE_RADIUS]
                        for r in all_robots if r['id'] != robot_id
                    ]
                    current_obstacles = dynamic_obstacles + static_obstacles

                except Exception:
                    pass

            # --- Teclado ---
            if keyboard_pipe.poll():
                try:
                    cmd = keyboard_pipe.recv()
                    command = cmd.get('command', '')
                    if command == 'exit':
                        running = False
                    elif command == 'toggle':
                        active = not active
                        goal_reached = False
                        log.info("Control %s", "ACTIVADO" if active else "PAUSADO")
                        if not active and robot_available:
                            rf_controller.set_motors(firmware_id, 0, 0)
                    elif command == 'replan':
                        force_replan = True
                        last_sent_pos = None
                        log.info("Replanificacion forzada")
                except Exception:
                    pass

            # --- Path nuevo del planner ---
            try:
                result = path_queue.get_nowait()
                new_path = result.get('path', [])
                if new_path:
                    current_path = new_path
                    current_wp_idx = 0
                    goal_reached = False
                    # Resetear estado PID al recibir nueva ruta
                    controller._pid_state.pop(robot_id, None)
                    log.info("Ruta actualizada: %d waypoints", len(current_path))
            except Exception:
                pass

            # --- Enviar posicion+obstaculos al planner si algo cambio ---
            if robot is not None and active and not goal_reached:
                new_pos = (int(robot.x), int(robot.y))
                robot_moved = (
                    force_replan or
                    last_sent_pos is None or
                    math.hypot(new_pos[0] - last_sent_pos[0],
                               new_pos[1] - last_sent_pos[1]) > REPLAN_POSITION_THRESHOLD
                )
                obs_changed = _obstacles_moved(
                    last_obstacle_positions, all_robots, robot_id
                )
                if robot_moved or obs_changed:
                    try:
                        while ctrl_to_plan_pipe.poll():
                            ctrl_to_plan_pipe.recv()
                        ctrl_to_plan_pipe.send({
                            'robot_pos': new_pos,
                            'obstacles': current_obstacles,
                        })
                        last_sent_pos = new_pos
                        last_obstacle_positions = {
                            r['id']: (r['x'], r['y'])
                            for r in all_robots if r['id'] != robot_id
                        }
                        force_replan = False
                        if obs_changed:
                            log.info("Obstaculos cambiaron -> replanificando")
                    except Exception:
                        pass

            # --- Ejecucion PID ---
            robot_ok = robot is not None and (
                robot_lost_time is None or
                now - robot_lost_time < ROBOT_LOST_TIMEOUT
            )

            if active and robot_ok and current_path and not goal_reached:
                wp = current_path[current_wp_idx]
                is_last_wp = (current_wp_idx == len(current_path) - 1)

                # Umbral: mas tolerante en waypoints intermedios, preciso en el final
                threshold = None if is_last_wp else WAYPOINT_ARRIVAL_THRESHOLD

                arrived = controller.move_to_position(robot, wp,
                                                      target_angle=None)

                # Para waypoints intermedios usar threshold propio sin esperar al PID
                if not is_last_wp and not arrived:
                    dist = math.hypot(robot.x - wp[0], robot.y - wp[1])
                    if dist < WAYPOINT_ARRIVAL_THRESHOLD:
                        arrived = True

                if arrived:
                    if is_last_wp:
                        log.info("GOAL ALCANZADO en (%d, %d)", wp[0], wp[1])
                        goal_reached = True
                        active = False
                        if robot_available:
                            rf_controller.set_motors(firmware_id, 0, 0)
                    else:
                        current_wp_idx += 1
                        # Resetear estado PID para el nuevo waypoint
                        controller._pid_state.pop(robot_id, None)
                        log.info("WP %d/%d alcanzado -> siguiente: %s",
                                 current_wp_idx, len(current_path),
                                 current_path[current_wp_idx])

            elif active and not robot_ok and robot_available:
                # Robot perdido: detener por seguridad
                rf_controller.set_motors(firmware_id, 0, 0)

            # --- Visualizacion ---
            if now - last_viz_time >= VIZ_INTERVAL_S:
                try:
                    wp_current = current_path[current_wp_idx] if (
                        current_path and current_wp_idx < len(current_path)
                    ) else None

                    dist_to_goal = None
                    if robot is not None:
                        gx, gy = goal_pos
                        dist_to_goal = math.hypot(robot.x - gx, robot.y - gy)

                    # Estado texto para overlay
                    if goal_reached:
                        estado = "GOAL!"
                    elif not active:
                        estado = "PAUSADO"
                    elif not current_path:
                        estado = "PLANIFICANDO..."
                    else:
                        estado = f"WP {current_wp_idx + 1}/{len(current_path)}"

                    while viz_pipe.poll():
                        viz_pipe.recv()
                    viz_pipe.send({
                        'robot_pos': (int(robot.x), int(robot.y)) if robot else None,
                        'robot_angle_deg': math.degrees(robot.angle) if robot else None,
                        'path': current_path,
                        'current_wp': wp_current,
                        'current_wp_idx': current_wp_idx,
                        'total_wps': len(current_path),
                        'goal_pos': goal_pos,
                        'active': active,
                        'goal_reached': goal_reached,
                        'robot_available': robot_available,
                        'estado': estado,
                        'dist_to_goal': dist_to_goal,
                        'timestamp': now,
                    })
                    last_viz_time = now
                except Exception:
                    pass

            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            if robot_available:
                rf_controller.set_motors(firmware_id, 0, 0)
        except Exception:
            pass
        log.info("Control finalizado")


# =============================================================================
# PROCESO 4: Visualizacion
# =============================================================================

def visualization_process(perc_pipe, ctrl_pipe, keyboard_pipe,
                          shm_name, frame_counter, robot_id):
    """Muestra frame con ruta planificada, todos los robots y estado.

    Robot controlado: circulo verde. Robots obstáculo: circulo rojo.
    """
    import cv2
    import numpy as np

    log.info("Visualizacion iniciada")

    frame_shape = (CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH, 3)
    shm = shared_memory.SharedMemory(name=shm_name)
    shared_array = np.ndarray(frame_shape, dtype=np.uint8, buffer=shm.buf)
    last_frame_counter = 0
    last_frame = None

    # Estado de percepcion
    robot_data_perc = None
    all_robots_perc = []    # Todos los robots detectados (para dibujar obstaculos)

    # Estado de control
    robot_pos = None
    robot_angle_deg = None
    path = []
    current_wp = None
    current_wp_idx = 0
    total_wps = 0
    goal_pos = None
    active = False
    goal_reached = False
    robot_available = False
    estado = "Esperando..."
    dist_to_goal = None

    window_name = "Test Path Planning 1 Robot"
    cv2.namedWindow(window_name)

    try:
        while True:
            # Frame
            current = frame_counter.value
            if current != last_frame_counter:
                last_frame = shared_array.copy()
                last_frame_counter = current

            # Datos de percepcion (robot controlado + todos los robots)
            if perc_pipe.poll():
                try:
                    data = perc_pipe.recv()
                    robot_data_perc = data.get('robot_data')
                    all_robots_perc = data.get('all_robots', [])
                except Exception:
                    pass

            # Datos de control
            if ctrl_pipe.poll():
                try:
                    data = ctrl_pipe.recv()
                    robot_pos = data.get('robot_pos')
                    robot_angle_deg = data.get('robot_angle_deg')
                    path = data.get('path', [])
                    current_wp = data.get('current_wp')
                    current_wp_idx = data.get('current_wp_idx', 0)
                    total_wps = data.get('total_wps', 0)
                    goal_pos = data.get('goal_pos')
                    active = data.get('active', False)
                    goal_reached = data.get('goal_reached', False)
                    robot_available = data.get('robot_available', False)
                    estado = data.get('estado', '')
                    dist_to_goal = data.get('dist_to_goal')
                except Exception:
                    pass

            if last_frame is not None:
                frame = last_frame.copy()
                h, w = frame.shape[:2]

                # Campo: linea central
                cv2.line(frame, (w // 2, 0), (w // 2, h), (40, 80, 40), 1)

                # --- Ruta planificada (lineas verdes) ---
                if len(path) >= 2:
                    for i in range(1, len(path)):
                        pt1 = path[i - 1]
                        pt2 = path[i]
                        # Segmentos recorridos: gris; pendientes: verde
                        if i <= current_wp_idx:
                            color = (80, 80, 80)
                        else:
                            color = (0, 200, 0)
                        cv2.line(frame, pt1, pt2, color, 2, cv2.LINE_AA)

                    # Puntos de waypoint
                    for i, wp in enumerate(path):
                        if i < current_wp_idx:
                            cv2.circle(frame, wp, 3, (60, 60, 60), -1)
                        elif i == current_wp_idx:
                            cv2.circle(frame, wp, 5, (0, 255, 255), -1)
                        else:
                            cv2.circle(frame, wp, 3, (0, 180, 0), -1)

                # --- Goal (circulo amarillo) ---
                if goal_pos:
                    gx, gy = goal_pos
                    gcolor = (0, 255, 0) if goal_reached else (0, 215, 255)
                    cv2.circle(frame, (gx, gy), 16, gcolor, 2, cv2.LINE_AA)
                    cv2.circle(frame, (gx, gy), 3, gcolor, -1)
                    cv2.putText(frame, "GOAL", (gx + 10, gy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, gcolor, 1)

                # --- Waypoint activo (cruz cyan) ---
                if current_wp and not goal_reached:
                    tx, ty = current_wp
                    arm = 10
                    cv2.line(frame, (tx - arm, ty), (tx + arm, ty),
                             (255, 255, 0), 2, cv2.LINE_AA)
                    cv2.line(frame, (tx, ty - arm), (tx, ty + arm),
                             (255, 255, 0), 2, cv2.LINE_AA)

                # --- Todos los robots ---
                for r in all_robots_perc:
                    rx, ry = r['x'], r['y']
                    angle_rad = math.radians(r['angulo'])
                    ex = int(rx + 25 * math.cos(angle_rad))
                    ey = int(ry + 25 * math.sin(angle_rad))

                    if r['id'] == robot_id:
                        # Robot controlado: verde (activo) / naranja (pausado)
                        r_color = (0, 255, 0) if active else (0, 140, 255)
                        cv2.circle(frame, (rx, ry), 16, r_color, 2, cv2.LINE_AA)
                        cv2.arrowedLine(frame, (rx, ry), (ex, ey),
                                        r_color, 2, cv2.LINE_AA, tipLength=0.3)
                        cv2.putText(frame, f"R{r['id']}", (rx + 12, ry - 12),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, r_color, 1)
                        # Linea robot -> waypoint activo
                        if current_wp and active and not goal_reached:
                            cv2.line(frame, (rx, ry), current_wp,
                                     (100, 255, 100), 1, cv2.LINE_AA)
                    else:
                        # Robot obstáculo: rojo + circulo de radio de seguridad
                        cv2.circle(frame, (rx, ry), 16, (0, 0, 220), 2, cv2.LINE_AA)
                        cv2.circle(frame, (rx, ry), ROBOT_OBSTACLE_RADIUS,
                                   (0, 0, 120), 1, cv2.LINE_AA)
                        cv2.arrowedLine(frame, (rx, ry), (ex, ey),
                                        (0, 0, 180), 1, cv2.LINE_AA, tipLength=0.3)
                        cv2.putText(frame, f"OBS#{r['id']}", (rx + 12, ry - 12),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 80, 220), 1)

                # --- Overlay de texto ---
                y = 25
                # Estado principal
                if goal_reached:
                    s_color = (0, 255, 0)
                elif not active:
                    s_color = (0, 140, 255)
                elif estado == "PLANIFICANDO...":
                    s_color = (0, 215, 255)
                else:
                    s_color = (0, 255, 150)
                cv2.putText(frame, estado, (10, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, s_color, 2)

                y += 22
                rf_text = "RF: OK" if robot_available else "RF: --"
                rf_color = (0, 255, 0) if robot_available else (80, 80, 80)
                cv2.putText(frame, rf_text, (10, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, rf_color, 1)

                y += 18
                rd = robot_data_perc
                r_text = "Robot: OK" if rd else "Robot: --"
                r_color = (0, 255, 0) if rd else (0, 0, 200)
                cv2.putText(frame, r_text, (10, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, r_color, 1)

                y += 18
                obs_count = sum(1 for r in all_robots_perc if r['id'] != robot_id)
                obs_color = (0, 80, 220) if obs_count > 0 else (80, 80, 80)
                cv2.putText(frame, f"Obstaculos: {obs_count}", (10, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, obs_color, 1)

                if total_wps > 0:
                    y += 18
                    cv2.putText(frame, f"Waypoints: {current_wp_idx}/{total_wps}",
                                (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                                (200, 200, 200), 1)

                if dist_to_goal is not None:
                    y += 18
                    cv2.putText(frame, f"Dist goal: {dist_to_goal:.0f}px",
                                (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                                (200, 200, 200), 1)

                if goal_pos:
                    y += 18
                    cv2.putText(frame, f"Goal: {goal_pos}",
                                (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                                (0, 215, 255), 1)

                cv2.putText(frame,
                            "ESPACIO=Start/Stop  R=Replan  ESC=Salir",
                            (10, CAMERA_PERSPECTIVE_HEIGHT - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1)

                cv2.imshow(window_name, frame)
            else:
                placeholder = np.zeros(frame_shape, dtype=np.uint8)
                cv2.putText(placeholder, "Esperando camara...",
                            (CAMERA_PERSPECTIVE_WIDTH // 2 - 110,
                             CAMERA_PERSPECTIVE_HEIGHT // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2)
                cv2.imshow(window_name, placeholder)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:       # ESC
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
            elif key == ord('r') or key == ord('R'):
                try:
                    keyboard_pipe.send({'command': 'replan'})
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
        description='Test de integracion: Path Planning RRT* con 1 robot (Paso 2.5)')
    parser.add_argument('--robot-id', type=int, choices=[0, 1, 2, 3],
                        required=True, help='ID del robot (0-3)')
    parser.add_argument('--serial-port', type=str, default='/dev/ttyUSB0',
                        help='Puerto serial (default: /dev/ttyUSB0)')
    parser.add_argument('--camera-id', type=int, default=None,
                        help='ID de camara (auto-detecta si no se especifica)')
    parser.add_argument('--goal-x', type=int, default=None,
                        help='Coordenada X del goal en px (0-640)')
    parser.add_argument('--goal-y', type=int, default=None,
                        help='Coordenada Y del goal en px (0-480)')
    parser.add_argument('--obstacle', type=str, action='append', default=[],
                        metavar='SPEC',
                        help=(
                            'Agregar obstaculo. Formatos:\n'
                            '  Circulo:     "cx,cy,radio"\n'
                            '  Rectangulo:  "cx,cy,hw,hh,angulo_deg"\n'
                            'Ejemplo: --obstacle "320,240,50" --obstacle "100,100,30,20,45"'
                        ))
    args = parser.parse_args()

    if args.camera_id is None:
        camera_id = get_camera_index(prefer_droidcam=True, fallback_index=0)
        log.info("Camara auto-detectada: /dev/video%d", camera_id)
    else:
        camera_id = args.camera_id

    # Resolver goal
    if args.goal_x is not None and args.goal_y is not None:
        goal_pos = (args.goal_x, args.goal_y)
    else:
        print(f"\nCampo de camara: {CAMERA_PERSPECTIVE_WIDTH}x{CAMERA_PERSPECTIVE_HEIGHT} px")
        print("Ingresa el punto de llegada (goal):")
        try:
            gx = int(input(f"  Goal X (0-{CAMERA_PERSPECTIVE_WIDTH}): ").strip())
            gy = int(input(f"  Goal Y (0-{CAMERA_PERSPECTIVE_HEIGHT}): ").strip())
        except (ValueError, KeyboardInterrupt):
            print("Goal invalido. Usando centro del campo derecho (500, 240).")
            gx, gy = 500, 240
        goal_pos = (gx, gy)

    # Parsear obstaculos desde CLI
    obstacles = []
    for spec in args.obstacle:
        try:
            parts = [float(v) for v in spec.split(',')]
            if len(parts) == 3:
                obstacles.append(parts)   # circulo: [cx, cy, r]
            elif len(parts) == 5:
                # rectangulo: convertir angulo de grados a radianes
                cx, cy, hw, hh, angle_deg = parts
                obstacles.append([cx, cy, hw, hh, math.radians(angle_deg)])
            else:
                log.warning("Obstaculo ignorado (formato invalido): %s", spec)
        except ValueError:
            log.warning("Obstaculo ignorado (no se pudo parsear): %s", spec)

    robot_id = args.robot_id
    shm_name = f"{SHM_NAME}_{robot_id}"

    log.info("=" * 60)
    log.info("  TEST PATH PLANNING (1 robot)")
    log.info("=" * 60)
    log.info("Robot ID: %d | Puerto: %s | Camara: /dev/video%d",
             robot_id, args.serial_port, camera_id)
    log.info("Goal: %s | Obstaculos: %d", goal_pos, len(obstacles))
    log.info("Campo: FIELD_CAM (%dx%d px)", CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT)
    log.info("=" * 60)

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
    perc_to_ctrl_s, perc_to_ctrl_r = multiprocessing.Pipe()
    perc_to_viz_s, perc_to_viz_r = multiprocessing.Pipe()
    ctrl_to_plan_s, ctrl_to_plan_r = multiprocessing.Pipe()
    ctrl_to_viz_s, ctrl_to_viz_r = multiprocessing.Pipe()
    viz_to_ctrl_s, viz_to_ctrl_r = multiprocessing.Pipe()

    # Queue para path (maxsize=1: siempre el mas reciente)
    path_queue = multiprocessing.Queue(maxsize=1)

    processes = []

    p1 = multiprocessing.Process(
        target=perception_process,
        args=(perc_to_ctrl_s, perc_to_viz_s, robot_id, camera_id,
              shm_name, frame_counter),
        name="Perception"
    )
    processes.append(p1)

    p2 = multiprocessing.Process(
        target=planning_process,
        args=(ctrl_to_plan_r, path_queue, goal_pos),   # sin obstacles: llegan por pipe
        name="Planning"
    )
    processes.append(p2)

    p3 = multiprocessing.Process(
        target=control_process,
        args=(perc_to_ctrl_r, ctrl_to_plan_s, path_queue,
              ctrl_to_viz_s, viz_to_ctrl_r, robot_id, args.serial_port,
              goal_pos, obstacles),                     # obstacles estaticos del CLI
        name="Control"
    )
    processes.append(p3)

    p4 = multiprocessing.Process(
        target=visualization_process,
        args=(perc_to_viz_r, ctrl_to_viz_r, viz_to_ctrl_s,
              shm_name, frame_counter, robot_id),       # robot_id para distinguir colores
        name="Visualization"
    )
    processes.append(p4)

    try:
        for proc in processes:
            proc.start()
            log.info("  %s iniciado (PID: %d)", proc.name, proc.pid)

        log.info("")
        log.info("Sistema corriendo.")
        log.info("Presiona ESPACIO para activar el seguimiento de ruta.")
        log.info("El planner calculara la ruta automaticamente al detectar el robot.")
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
