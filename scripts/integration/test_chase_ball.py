#!/usr/bin/env python3
"""Test de integracion: Robot persigue pelota.

PASO 1 del plan de integracion. Verifica el pipeline completo:
  Camara -> Perspectiva -> Deteccion robot + pelota -> PID -> RF -> Motor

Arquitectura: 3 procesos
  1. Percepcion: Detecta robot (ArUco) + pelota (HSV) en el mismo frame
  2. Control: PID mueve robot hacia pelota via RF
  3. Visualizacion: Muestra video con overlays

Uso:
    python scripts/integration/test_chase_ball.py --robot-id 0
    python scripts/integration/test_chase_ball.py --robot-id 0 --camera-id 2
    python scripts/integration/test_chase_ball.py --robot-id 0 --serial-port /dev/ttyUSB0

Controles:
    ESPACIO: START/STOP - Iniciar o pausar persecucion
    X: Detener robot inmediatamente
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

from metrics.metrics_capture import save_metrics
from metrics.session_recorder import SessionRecorder

from robot_soccer.config import (
    CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH,
    CAMERA_PERSPECTIVE_ENABLED, CAMERA_PERSPECTIVE_SRC_POINTS,
    RANGO_COLOR_NARANJO,
)
from robot_soccer.utils.camera_utils import get_camera_index

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(processName)-12s] %(levelname)-8s %(message)s',
    datefmt='%H:%M:%S'
)
log = logging.getLogger(__name__)

SHM_NAME = "robot_chase_ball"


# =============================================================================
# PROCESO 1: Percepcion (robot + pelota)
# =============================================================================

def perception_process(control_pipe, viz_pipe, robot_id, camera_id,
                       shm_name, frame_counter,
                       total_frames_val=None, robot_detections_val=None,
                       ball_detections_val=None):
    """Detecta robot (ArUco) y pelota (HSV) en cada frame."""
    import cv2
    import numpy as np
    from robot_soccer.perception.player_tracking import create_aruco_detector

    log.info("Percepcion iniciada")

    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        log.error("No se pudo abrir camara %d", camera_id)
        return

    # Perspectiva
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

    # ArUco detector
    detector = create_aruco_detector(use_camera=True)

    # Shared memory para frames
    frame_shape = (CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH, 3)
    shm = shared_memory.SharedMemory(name=shm_name)
    shared_array = np.ndarray(frame_shape, dtype=np.uint8, buffer=shm.buf)

    # Ball color range
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

            # Transformar perspectiva
            if perspective_matrix is not None:
                frame = cv2.warpPerspective(
                    raw_frame, perspective_matrix,
                    (CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT)
                )
            else:
                frame = raw_frame

            frame_count += 1
            if total_frames_val is not None:
                with total_frames_val.get_lock():
                    total_frames_val.value += 1

            # --- Detectar robot (ArUco) ---
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
                        if robot_detections_val is not None:
                            with robot_detections_val.get_lock():
                                robot_detections_val.value += 1
                        break

            # --- Detectar pelota (HSV) ---
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
                area = cv2.contourArea(largest)
                if area > 10:  # Filtro minimo de area
                    (bx, by), radius = cv2.minEnclosingCircle(largest)
                    if 2 <= radius <= 30:
                        ball_pos = (int(bx), int(by))
                        ball_detected = True
                        ball_detect_count += 1
                        if ball_detections_val is not None:
                            with ball_detections_val.get_lock():
                                ball_detections_val.value += 1

            # --- Escribir frame a shared memory ---
            np.copyto(shared_array, frame)
            with frame_counter.get_lock():
                frame_counter.value += 1

            # --- Enviar datos a control ---
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

            # --- Enviar metadata a visualizacion ---
            try:
                while viz_pipe.poll():
                    _ = viz_pipe.recv()
                viz_pipe.send({
                    'robot_detected': robot_detected,
                    'robot_data': robot_data,
                    'ball_detected': ball_detected,
                    'ball_pos': ball_pos,
                    'timestamp': time.time()
                })
            except Exception:
                pass

            # Stats log cada 5s
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
# PROCESO 2: Control (PID + RF)
# =============================================================================

def control_process(perception_pipe, viz_state_pipe, keyboard_pipe,
                    robot_id, serial_port, rf_available_val=None):
    """Recibe posiciones, ejecuta PID, envia comandos RF."""
    from robot_soccer.controllers.differential_drive import DifferentialDriveController
    from robot_soccer.communication.rf_controller import RFController
    from robot_soccer.config import ROBOT_POSITION_THRESHOLD

    log.info("Control iniciado para robot %d", robot_id)

    # RF
    rf_controller = None
    robot_available = False
    try:
        rf_controller = RFController(port=serial_port, enable_calibration=True)
        if rf_controller.initialize():
            connections = rf_controller.test_connections()
            robot_key = f'robot_{robot_id + 1}'
            robot_available = connections.get(robot_key, False)
            if rf_available_val is not None:
                rf_available_val.value = int(robot_available)
            if robot_available:
                log.info("Robot %d disponible via RF", robot_id)
            else:
                log.warning("Robot %d NO responde via RF", robot_id)
    except Exception as e:
        log.warning("Error RF: %s", e)

    controller = DifferentialDriveController(rf_controller=rf_controller)

    # Robot entity simple
    class RobotEntity:
        def __init__(self, rid):
            self.id = rid
            self.x = 0.0
            self.y = 0.0
            self.angle = 0.0
            self.dx = self.dy = self.dw = 0.0
        def update(self, x, y, angle_rad):
            self.x = x
            self.y = y
            self.angle = angle_rad

    robot = None
    ball_pos = None
    chase_active = False
    robot_stopped = True
    running = True

    last_state_time = 0
    STATE_INTERVAL = 0.04  # ~25 Hz

    try:
        while running:
            now = time.time()

            # Recibir datos de percepcion
            if perception_pipe.poll():
                try:
                    data = perception_pipe.recv()
                    if data.get('robot_detected') and data.get('robot_data'):
                        rd = data['robot_data']
                        angle_rad = math.radians(rd['angulo'])
                        if robot is None:
                            robot = RobotEntity(robot_id)
                            log.info("Robot detectado en (%d, %d)", rd['x'], rd['y'])
                        robot.update(rd['x'], rd['y'], angle_rad)
                    else:
                        robot = None

                    if data.get('ball_detected') and data.get('ball_pos'):
                        ball_pos = data['ball_pos']
                    else:
                        ball_pos = None
                except Exception:
                    pass

            # Recibir comandos de teclado
            if keyboard_pipe.poll():
                try:
                    cmd = keyboard_pipe.recv()
                    command = cmd.get('command', '')
                    if command == 'exit':
                        running = False
                    elif command == 'toggle_chase':
                        chase_active = not chase_active
                        robot_stopped = False
                        log.info("Persecucion %s",
                                 "INICIADA" if chase_active else "PAUSADA")
                    elif command == 'stop':
                        chase_active = False
                        robot_stopped = False
                        log.info("Robot detenido manualmente")
                except Exception:
                    pass

            # Ejecutar control
            if chase_active and robot and ball_pos:
                reached = controller.move_to_position(robot, ball_pos)
                if reached:
                    log.info("Pelota alcanzada en (%d, %d)", ball_pos[0], ball_pos[1])
                    # No desactivar chase: seguir persiguiendo si la pelota se mueve
                    robot_stopped = False
            else:
                if robot and robot_available and not robot_stopped:
                    firmware_id = robot_id + 1
                    rf_controller.set_motors(firmware_id, 0, 0)
                    robot_stopped = True

            # Enviar estado a visualizacion
            if now - last_state_time >= STATE_INTERVAL:
                try:
                    # Calcular error angular hacia pelota
                    angle_error_deg = None
                    distance = None
                    if robot and ball_pos:
                        dx = ball_pos[0] - robot.x
                        dy = ball_pos[1] - robot.y
                        distance = math.sqrt(dx * dx + dy * dy)
                        target_heading = math.atan2(dy, dx)
                        angle_error = target_heading - robot.angle
                        while angle_error > math.pi:
                            angle_error -= 2 * math.pi
                        while angle_error < -math.pi:
                            angle_error += 2 * math.pi
                        angle_error_deg = math.degrees(angle_error)

                    viz_state_pipe.send({
                        'robot_pos': (int(robot.x), int(robot.y)) if robot else None,
                        'robot_angle_deg': math.degrees(robot.angle) if robot else None,
                        'ball_pos': ball_pos,
                        'chase_active': chase_active,
                        'angle_error_deg': angle_error_deg,
                        'distance': distance,
                        'robot_available': robot_available,
                        'timestamp': now
                    })
                    last_state_time = now
                except Exception:
                    pass

            time.sleep(0.005)  # ~200 Hz

    finally:
        if robot_available and rf_controller:
            rf_controller.set_motors(robot_id + 1, 0, 0)
        if rf_controller:
            rf_controller.shutdown()
        log.info("Control finalizado")


# =============================================================================
# PROCESO 3: Visualizacion
# =============================================================================

def visualization_process(perception_pipe, control_pipe, keyboard_pipe,
                          shm_name, frame_counter):
    """Muestra video con overlays de robot, pelota y estado."""
    import cv2
    import numpy as np

    log.info("Visualizacion iniciada")

    frame_shape = (CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH, 3)
    shm = shared_memory.SharedMemory(name=shm_name)
    shared_array = np.ndarray(frame_shape, dtype=np.uint8, buffer=shm.buf)
    last_frame_counter = 0

    last_frame = None
    robot_data = None
    ball_pos = None
    robot_detected = False
    ball_detected = False

    # Estado del control
    ctrl_robot_pos = None
    ctrl_ball_pos = None
    chase_active = False
    angle_error_deg = None
    distance = None
    robot_available = False

    window_name = 'Test Chase Ball'
    cv2.namedWindow(window_name)
    recorder = SessionRecorder("test_chase_ball")

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
                    robot_detected = data.get('robot_detected', False)
                    robot_data = data.get('robot_data')
                    ball_detected = data.get('ball_detected', False)
                    ball_pos = data.get('ball_pos')
                except Exception:
                    pass

            # Leer estado control
            if control_pipe.poll():
                try:
                    data = control_pipe.recv()
                    ctrl_robot_pos = data.get('robot_pos')
                    ctrl_ball_pos = data.get('ball_pos')
                    chase_active = data.get('chase_active', False)
                    angle_error_deg = data.get('angle_error_deg')
                    distance = data.get('distance')
                    robot_available = data.get('robot_available', False)
                except Exception:
                    pass

            if last_frame is not None:
                frame = last_frame.copy()

                # Dibujar pelota
                if ball_pos:
                    bx, by = ball_pos
                    cv2.circle(frame, (bx, by), 12, (0, 165, 255), 2, cv2.LINE_AA)
                    cv2.circle(frame, (bx, by), 2, (0, 165, 255), -1, cv2.LINE_AA)

                # Dibujar robot
                if robot_data:
                    rx, ry = robot_data['x'], robot_data['y']
                    cv2.circle(frame, (rx, ry), 16, (0, 220, 0), 2, cv2.LINE_AA)
                    angle_rad = math.radians(robot_data['angulo'])
                    ex = int(rx + 25 * math.cos(angle_rad))
                    ey = int(ry + 25 * math.sin(angle_rad))
                    cv2.arrowedLine(frame, (rx, ry), (ex, ey),
                                   (0, 220, 0), 2, cv2.LINE_AA, tipLength=0.3)

                    # Linea robot -> pelota
                    if ball_pos:
                        cv2.line(frame, (rx, ry), ball_pos,
                                (200, 200, 0), 1, cv2.LINE_AA)

                # Info overlay
                y_text = 25
                status = "PERSIGUIENDO" if chase_active else "DETENIDO"
                color = (0, 255, 0) if chase_active else (0, 0, 255)
                cv2.putText(frame, status, (10, y_text),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

                y_text += 25
                r_text = "Robot: OK" if robot_detected else "Robot: --"
                r_color = (0, 255, 0) if robot_detected else (0, 0, 200)
                cv2.putText(frame, r_text, (10, y_text),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, r_color, 1)

                y_text += 20
                b_text = f"Pelota: ({ball_pos[0]},{ball_pos[1]})" if ball_pos else "Pelota: --"
                b_color = (0, 200, 255) if ball_detected else (0, 0, 200)
                cv2.putText(frame, b_text, (10, y_text),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, b_color, 1)

                if distance is not None:
                    y_text += 20
                    cv2.putText(frame, f"Dist: {distance:.0f}px", (10, y_text),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

                if angle_error_deg is not None:
                    y_text += 20
                    cv2.putText(frame, f"Err ang: {angle_error_deg:+.1f} deg",
                               (10, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                               (255, 255, 255), 1)

                rf_text = "RF: OK" if robot_available else "RF: --"
                rf_color = (0, 255, 0) if robot_available else (0, 0, 200)
                cv2.putText(frame, rf_text, (CAMERA_PERSPECTIVE_WIDTH - 80, 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, rf_color, 1)

                # Controles
                cv2.putText(frame, "ESPACIO=Start/Stop  X=Parar  ESC=Salir",
                           (10, CAMERA_PERSPECTIVE_HEIGHT - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

                cv2.imshow(window_name, frame)
                recorder.write(frame)
            else:
                placeholder = np.zeros(frame_shape, dtype=np.uint8)
                cv2.putText(placeholder, "Esperando frames...",
                           (CAMERA_PERSPECTIVE_WIDTH // 2 - 100,
                            CAMERA_PERSPECTIVE_HEIGHT // 2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2)
                cv2.imshow(window_name, placeholder)

            # Teclado
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                try:
                    keyboard_pipe.send({'command': 'exit', 'timestamp': time.time()})
                except Exception:
                    pass
                break
            elif key == ord(' '):
                try:
                    keyboard_pipe.send({'command': 'toggle_chase',
                                        'timestamp': time.time()})
                except Exception:
                    pass
            elif key == ord('x') or key == ord('X'):
                try:
                    keyboard_pipe.send({'command': 'stop',
                                        'timestamp': time.time()})
                except Exception:
                    pass

    except KeyboardInterrupt:
        pass
    finally:
        shm.close()
        cv2.destroyAllWindows()
        recorder.close()
        log.info("Visualizacion finalizada")


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Test de integracion: Robot persigue pelota (Paso 1)')
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

    # Crear shared memory
    frame_size = CAMERA_PERSPECTIVE_HEIGHT * CAMERA_PERSPECTIVE_WIDTH * 3
    shm = shared_memory.SharedMemory(create=True, name=shm_name, size=frame_size)
    frame_counter = Value('i', 0)

    # Contadores de métricas compartidos entre procesos
    total_frames_val = Value('i', 0)
    robot_detections_val = Value('i', 0)
    ball_detections_val = Value('i', 0)
    rf_available_val = Value('i', 0)
    t_start = time.time()

    # Pipes
    perc_to_ctrl_s, perc_to_ctrl_r = multiprocessing.Pipe()
    perc_to_viz_s, perc_to_viz_r = multiprocessing.Pipe()
    ctrl_to_viz_s, ctrl_to_viz_r = multiprocessing.Pipe()
    viz_to_ctrl_s, viz_to_ctrl_r = multiprocessing.Pipe()

    log.info("=" * 60)
    log.info("  TEST INTEGRACION: ROBOT PERSIGUE PELOTA")
    log.info("=" * 60)
    log.info("Robot ID: %d", robot_id)
    log.info("Puerto: %s", args.serial_port)
    log.info("Camara: /dev/video%d", camera_id)
    log.info("=" * 60)

    processes = []

    p1 = multiprocessing.Process(
        target=perception_process,
        args=(perc_to_ctrl_s, perc_to_viz_s, robot_id, camera_id,
              shm_name, frame_counter,
              total_frames_val, robot_detections_val, ball_detections_val),
        name="Perception"
    )
    processes.append(p1)

    p2 = multiprocessing.Process(
        target=control_process,
        args=(perc_to_ctrl_r, ctrl_to_viz_s, viz_to_ctrl_r,
              robot_id, args.serial_port, rf_available_val),
        name="Control"
    )
    processes.append(p2)

    p3 = multiprocessing.Process(
        target=visualization_process,
        args=(perc_to_viz_r, ctrl_to_viz_r, viz_to_ctrl_s,
              shm_name, frame_counter),
        name="Visualization"
    )
    processes.append(p3)

    try:
        for proc in processes:
            proc.start()
            log.info("  %s iniciado (PID: %d)", proc.name, proc.pid)

        log.info("")
        log.info("Sistema corriendo. Presiona ESPACIO para iniciar persecucion.")
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

        # Guardar métricas en LOG/
        elapsed = time.time() - t_start
        fc = total_frames_val.value
        rc = robot_detections_val.value
        bc = ball_detections_val.value
        metrics = {
            "duration_s": round(elapsed, 2),
            "total_frames": fc,
            "fps_avg": round(fc / elapsed, 2) if elapsed > 0 else 0,
            "robot_detection_count": rc,
            "robot_detection_rate_pct": round(rc / fc * 100, 2) if fc > 0 else 0,
            "ball_detection_count": bc,
            "ball_detection_rate_pct": round(bc / fc * 100, 2) if fc > 0 else 0,
            "robot_id": robot_id,
            "rf_available": bool(rf_available_val.value),
            "camera_id": camera_id,
            "serial_port": args.serial_port,
        }
        try:
            out = save_metrics("test_chase_ball", metrics)
            log.info("Metricas guardadas en %s", out)
        except Exception as e:
            log.warning("No se pudieron guardar metricas: %s", e)

        log.info("Test finalizado")


if __name__ == '__main__':
    main()
