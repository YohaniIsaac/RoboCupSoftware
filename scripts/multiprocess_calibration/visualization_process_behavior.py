"""Proceso de visualización para calibración de umbrales de comportamiento (Arquitectura de 3 procesos).

Este proceso maneja TODA la interfaz de usuario:
- Recibe frames procesados desde percepción (shared memory)
- Recibe metadata de percepción (pipe)
- Recibe estado del controlador behavior desde control (pipe)
- Dibuja overlays (robot, waypoint, orientación)
- Muestra panel de información con parámetros behavior
- Captura teclas y mouse
- Envía comandos al proceso de control

Diseñado para NO bloquear el proceso de control crítico.
FPS objetivo: 28-40 (limitado por percepción y cv2.imshow)
"""

import logging
import sys
import time
import math
from multiprocessing import shared_memory
from pathlib import Path

import cv2
import numpy as np

ROOT_DIR = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))

from robot_soccer.config import CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH

log = logging.getLogger(__name__)


def visualization_loop_behavior(perception_pipe, control_state_pipe, keyboard_pipe,
                                 shm_name: str = None, frame_counter=None):
    """Bucle principal del proceso de visualización para calibración behavior.

    Args:
        perception_pipe: Pipe para recibir metadata desde percepción (sin frames)
        control_state_pipe: Pipe para recibir estado desde control (behavior params, waypoint, etc.)
        keyboard_pipe: Pipe para enviar comandos a control (teclado/mouse)
        shm_name: Nombre de la shared memory para leer frames
        frame_counter: multiprocessing.Value('i') contador atómico de frames
    """
    log.info("🖥️  Proceso de visualización iniciado (Behavior - 3 procesos)")
    log.info("   Responsabilidades:")
    log.info("     ✓ Leer frames de shared memory")
    log.info("     ✓ Recibir metadata de percepción (pipe)")
    log.info("     ✓ Recibir estado de control")
    log.info("     ✓ Mostrar UI con cv2.imshow()")
    log.info("     ✓ Capturar teclas y mouse")
    log.info("     ✓ Enviar comandos a control")

    frame_shape = (CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH, 3)
    shm = shared_memory.SharedMemory(name=shm_name)
    shared_array = np.ndarray(frame_shape, dtype=np.uint8, buffer=shm.buf)
    last_frame_counter = 0
    log.info(f"📦 Conectado a shared memory: {shm_name}")

    last_frame = None
    last_robot_detected = False
    last_robot_data = None
    last_perception_stats = {}

    last_behavior_params = {
        'position_threshold': 16,
        'angle_threshold': 7,
        'linear_start_angle_threshold': 30,
        'max_angular_correction_pwm': 10
    }
    last_target_waypoint = None
    last_movement_active = False
    last_robot_id = 0
    last_robot_available = False
    last_robot_pos = None

    window_name = 'Calibracion Behavior (3 Procesos)'
    panel_height = 370

    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and y > panel_height:
            actual_y = y - panel_height
            try:
                if keyboard_pipe.poll():
                    _ = keyboard_pipe.recv()
                keyboard_pipe.send({
                    'command': 'set_waypoint',
                    'waypoint': [x, actual_y],
                    'timestamp': time.time()
                })
                log.info(f"🎯 Waypoint establecido: ({x}, {actual_y})")
            except Exception as e:
                log.warning(f"⚠️  Error enviando waypoint: {e}")

    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_callback)

    log.info("✅ Ventana de visualización creada")

    try:
        while True:
            current_counter = frame_counter.value
            if current_counter != last_frame_counter:
                last_frame = shared_array.copy()
                last_frame_counter = current_counter

            if perception_pipe.poll():
                try:
                    perception_data = perception_pipe.recv()
                    last_robot_detected = perception_data.get('robot_detected', False)
                    last_robot_data = perception_data.get('robot_data')
                    last_perception_stats = perception_data.get('stats', {})
                except Exception as e:
                    log.warning(f"⚠️  Error recibiendo metadata: {e}")

            if control_state_pipe.poll():
                try:
                    control_data = control_state_pipe.recv()
                    last_behavior_params = control_data.get('behavior_params', last_behavior_params)
                    last_target_waypoint = control_data.get('target_waypoint')
                    last_movement_active = control_data.get('movement_active', False)
                    last_robot_id = control_data.get('robot_id', 0)
                    last_robot_available = control_data.get('robot_available', False)
                    last_robot_pos = control_data.get('robot_pos')
                except Exception as e:
                    log.warning(f"⚠️  Error recibiendo estado control: {e}")

            if last_frame is not None:
                vis_frame = last_frame.copy()

                if last_robot_pos:
                    rx, ry = last_robot_pos
                    cv2.circle(vis_frame, (rx, ry), 20, (0, 255, 0), 2)
                    if last_robot_data and 'angulo' in last_robot_data:
                        angle_rad = math.radians(last_robot_data['angulo'])
                        end_x = int(rx + 30 * math.cos(angle_rad))
                        end_y = int(ry + 30 * math.sin(angle_rad))
                        cv2.line(vis_frame, (rx, ry), (end_x, end_y), (0, 255, 0), 2)

                if last_target_waypoint:
                    wx, wy = last_target_waypoint
                    cv2.circle(vis_frame, (wx, wy), 15, (0, 255, 0), 2)
                    cv2.circle(vis_frame, (wx, wy), 3, (0, 255, 0), -1)
                    if last_robot_pos:
                        cv2.line(vis_frame, last_robot_pos, (wx, wy), (255, 255, 0), 1)

                status_text = "MOVING" if last_movement_active else "STOPPED"
                status_color = (0, 255, 0) if last_movement_active else (0, 0, 255)
                cv2.putText(vis_frame, status_text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, status_color, 2)

                cv2.imshow(window_name, vis_frame)
            else:
                placeholder = np.zeros((panel_height + 480, 640, 3), dtype=np.uint8)
                cv2.putText(placeholder, "Esperando frames...", (200, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (128, 128, 128), 2)
                cv2.imshow(window_name, placeholder)

            panel = _draw_behavior_panel(last_behavior_params, last_robot_pos,
                                         last_target_waypoint, last_robot_available,
                                         last_robot_detected)
            cv2.imshow('Behavior Control Panel', panel)

            key = cv2.waitKey(1) & 0xFF

            if key != 255:
                command = None
                param = None
                delta = 0
                waypoint_delta = None

                if key == 27:
                    command = 'exit'
                    log.info("ESC presionado - Enviando comando de salida")
                elif key == ord(' '):
                    command = 'toggle_movement'
                elif key == ord('x') or key == ord('X'):
                    command = 'cancel_waypoint'
                elif key == ord('\r') or key == ord('\n'):
                    command = 'save_params'
                elif key == ord('9'):
                    command = 'adjust_threshold'
                    param = 'position_threshold'
                    delta = 1
                elif key == ord('0'):
                    command = 'adjust_threshold'
                    param = 'position_threshold'
                    delta = -1
                elif key == ord('-'):
                    command = 'adjust_threshold'
                    param = 'angle_threshold'
                    delta = -1
                elif key == ord('='):
                    command = 'adjust_threshold'
                    param = 'angle_threshold'
                    delta = 1
                elif key == ord('['):
                    command = 'adjust_threshold'
                    param = 'linear_start_angle_threshold'
                    delta = -1
                elif key == ord(']'):
                    command = 'adjust_threshold'
                    param = 'linear_start_angle_threshold'
                    delta = 1
                elif key == ord(','):
                    command = 'adjust_threshold'
                    param = 'max_angular_correction_pwm'
                    delta = -1
                elif key == ord('.'):
                    command = 'adjust_threshold'
                    param = 'max_angular_correction_pwm'
                    delta = 1
                elif key in [82, 84, 81, 83] and last_robot_pos:
                    command = 'move_waypoint'
                    waypoint_delta = [0, 0]
                    if key == 82:
                        waypoint_delta = [0, -10]
                    elif key == 84:
                        waypoint_delta = [0, 10]
                    elif key == 81:
                        waypoint_delta = [-10, 0]
                    elif key == 83:
                        waypoint_delta = [10, 0]

                if command:
                    try:
                        if keyboard_pipe.poll():
                            _ = keyboard_pipe.recv()

                        msg = {'command': command, 'timestamp': time.time()}
                        if param:
                            msg['param'] = param
                            msg['delta'] = delta
                        if waypoint_delta:
                            msg['delta'] = waypoint_delta

                        keyboard_pipe.send(msg)

                        if command == 'exit':
                            break
                    except Exception as e:
                        log.warning(f"⚠️  Error enviando comando: {e}")

    except KeyboardInterrupt:
        log.info("⏹️  Proceso de visualización detenido por usuario")
    except Exception as e:
        log.error(f"❌ Error en proceso de visualización: {e}", exc_info=True)
    finally:
        shm.close()
        cv2.destroyAllWindows()
        log.info("🔌 Ventana cerrada, shared memory desconectada")


def _draw_behavior_panel(behavior_params, robot_pos, waypoint, robot_available, robot_detected):
    """Dibuja el panel de control de comportamiento."""
    panel = np.zeros((370, 650, 3), dtype=np.uint8)

    y_offset = 30
    line_height = 25

    cv2.putText(panel, "=== CALIBRACION DE COMPORTAMIENTO ===", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    y_offset += line_height * 2

    if robot_available:
        status_color = (0, 255, 0)
        status_text = f"Robot detectado" if robot_detected else "Robot NO visible"
    else:
        status_color = (0, 0, 255)
        status_text = "Robot NO conectado"

    cv2.putText(panel, status_text, (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1)
    y_offset += line_height * 2

    cv2.putText(panel, "=== UMBRALES DE PRECISION ===", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    y_offset += line_height

    cv2.putText(panel, f"Posicion: {behavior_params['position_threshold']}px (9/0)", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y_offset += line_height
    cv2.putText(panel, "  -> Distancia para considerar waypoint alcanzado", (20, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
    y_offset += int(line_height * 1.5)

    cv2.putText(panel, f"Angular: {behavior_params['angle_threshold']}deg (-/=)", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y_offset += line_height
    cv2.putText(panel, "  -> Error angular aceptable", (20, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
    y_offset += line_height * 2

    cv2.putText(panel, "=== POLITICA DE MOVIMIENTO ===", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    y_offset += line_height

    cv2.putText(panel, f"Inicio lineal: {behavior_params['linear_start_angle_threshold']}deg ([/])", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y_offset += line_height
    cv2.putText(panel, "  -> Angulo max para moverse linealmente", (20, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
    y_offset += line_height
    cv2.putText(panel, "  -> Mayor valor = menos giros en lugar", (20, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
    y_offset += line_height * 2

    cv2.putText(panel, "=== CORRECCION ANGULAR ===", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    y_offset += line_height

    cv2.putText(panel, f"Max correccion: {behavior_params['max_angular_correction_pwm']} PWM (,/.)", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y_offset += line_height
    cv2.putText(panel, "  -> Max diferencia L/R durante mov. lineal", (20, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
    y_offset += line_height * 2

    if waypoint:
        cv2.putText(panel, f"Waypoint: ({waypoint[0]}, {waypoint[1]})", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    else:
        cv2.putText(panel, "Sin waypoint (usar flechas o click)", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
    y_offset += line_height * 2

    cv2.putText(panel, "=== CONTROLES ===", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
    y_offset += line_height

    controls = [
        "Flechas: Mover waypoint",
        "Click: Establecer waypoint",
        "ESPACIO: START/STOP",
        "X: Cancelar waypoint",
        "ENTER: Guardar a config.py",
        "ESC: Salir",
    ]

    for control in controls:
        cv2.putText(panel, control, (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        y_offset += line_height

    return panel
