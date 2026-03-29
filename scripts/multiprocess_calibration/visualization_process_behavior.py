"""Proceso de visualización para calibración de umbrales de comportamiento (Arquitectura de 3 procesos).

Este proceso maneja TODA la interfaz de usuario:
- Recibe frames procesados desde percepción (shared memory)
- Recibe metadata de percepción (pipe)
- Recibe estado del controlador behavior desde control (pipe)
- Dibuja overlays (robot, waypoint, orientación, error angular)
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
    """Bucle principal del proceso de visualización para calibración behavior."""
    log.info("Proceso de visualización iniciado (Behavior - 3 procesos)")

    frame_shape = (CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH, 3)
    shm = shared_memory.SharedMemory(name=shm_name)
    shared_array = np.ndarray(frame_shape, dtype=np.uint8, buffer=shm.buf)
    last_frame_counter = 0
    log.info(f"Conectado a shared memory: {shm_name}")

    last_frame = None
    last_robot_detected = False
    last_robot_data = None

    last_behavior_params = {
        'position_threshold': 16,
        'angle_threshold': 7,
        'linear_start_angle_threshold': 30,
        'max_angular_correction_pwm': 10,
        'capture_activate_px': 38,
        'capture_overshoot_px': 15,
        'capture_confirm_px': 20,
        'creep_speed_pwm': 30,
        'dribble_pwm_factor': 1.0,
    }
    last_target_waypoint = None
    last_movement_active = False
    last_robot_id = 0
    last_robot_available = False
    last_robot_pos = None
    last_robot_angle_deg = None
    last_angle_error_deg = None
    last_target_heading_deg = None
    last_movement_mode = None
    last_capture_phase = 'idle'
    last_ball_waypoint = None
    last_overshoot_target = None
    last_dribbler_on = False

    # Waypoint persistence: keeps showing where the target was after arrival
    reached_waypoint = None       # waypoint that was reached (persists until new one is set)
    prev_target_waypoint = None   # to detect transition to None

    zoom_level = 2.0
    zoom_min = 1.0
    zoom_max = 4.0
    zoom_center = None

    window_name = 'Calibracion Behavior (3 Procesos)'
    panel_height = 450
    frame_height = CAMERA_PERSPECTIVE_HEIGHT  # 480
    frame_width = CAMERA_PERSPECTIVE_WIDTH    # 640

    # Layout: zoom (left, 640x480) + right column (320x480) with mini full view (320x240)
    zoom_w, zoom_h = frame_width, frame_height           # 640x480
    mini_w, mini_h = frame_width // 2, frame_height // 2  # 320x240
    right_col_w = mini_w                                   # 320
    total_w = zoom_w + right_col_w                         # 960

    def mouse_callback(event, x, y, flags, param):
        nonlocal reached_waypoint
        if event != cv2.EVENT_LBUTTONDOWN or y <= panel_height:
            return
        actual_y = y - panel_height

        if x < zoom_w:
            # Click on zoom view: map to full-frame coords
            if zoom_center is not None:
                zx, zy = zoom_center
                crop_h = int(frame_height / zoom_level)
                crop_w = int(frame_width / zoom_level)
                x1 = max(0, zx - crop_w // 2)
                y1 = max(0, zy - crop_h // 2)
                x2 = min(frame_width, x1 + crop_w)
                y2 = min(frame_height, y1 + crop_h)
                if x2 - x1 < crop_w:
                    x1 = max(0, x2 - crop_w)
                if y2 - y1 < crop_h:
                    y1 = max(0, y2 - crop_h)
                wx = int(x1 + x * crop_w / zoom_w)
                wy = int(y1 + actual_y * crop_h / zoom_h)
            else:
                wx, wy = x, actual_y
        elif actual_y < mini_h:
            # Click on mini full view (top-right)
            rx = x - zoom_w
            wx = int(rx * frame_width / mini_w)
            wy = int(actual_y * frame_height / mini_h)
        else:
            return  # Click on info area below mini view, ignore

        wx = max(0, min(frame_width - 1, wx))
        wy = max(0, min(frame_height - 1, wy))

        # New waypoint set → clear reached marker
        reached_waypoint = None

        try:
            if keyboard_pipe.poll():
                _ = keyboard_pipe.recv()
            keyboard_pipe.send({
                'command': 'set_waypoint',
                'waypoint': [wx, wy],
                'timestamp': time.time()
            })
            log.info(f"Waypoint establecido: ({wx}, {wy})")
        except Exception as e:
            log.warning(f"Error enviando waypoint: {e}")

    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_callback)
    log.info("Ventana de visualización creada")

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
                except Exception as e:
                    log.warning(f"Error recibiendo metadata: {e}")

            if control_state_pipe.poll():
                try:
                    control_data = control_state_pipe.recv()
                    last_behavior_params = control_data.get('behavior_params', last_behavior_params)
                    new_target = control_data.get('target_waypoint')
                    last_movement_active = control_data.get('movement_active', False)
                    last_robot_id = control_data.get('robot_id', 0)
                    last_robot_available = control_data.get('robot_available', False)
                    last_robot_pos = control_data.get('robot_pos')
                    last_robot_angle_deg = control_data.get('robot_angle_deg')
                    last_angle_error_deg = control_data.get('angle_error_deg')
                    last_target_heading_deg = control_data.get('target_heading_deg')
                    last_movement_mode = control_data.get('movement_mode')
                    last_capture_phase = control_data.get('capture_phase', 'idle')
                    last_ball_waypoint = control_data.get('ball_waypoint')
                    last_overshoot_target = control_data.get('overshoot_target')
                    last_dribbler_on = control_data.get('dribbler_on', False)

                    # Detect waypoint reached: target went from something to None
                    if prev_target_waypoint is not None and new_target is None:
                        reached_waypoint = list(prev_target_waypoint)
                    # Detect new waypoint set: clear reached marker
                    if new_target is not None and (prev_target_waypoint is None or
                            new_target != prev_target_waypoint):
                        reached_waypoint = None

                    prev_target_waypoint = list(new_target) if new_target else None
                    last_target_waypoint = new_target
                except Exception as e:
                    log.warning(f"Error recibiendo estado control: {e}")

            if last_frame is not None:
                if last_robot_pos:
                    zoom_center = tuple(last_robot_pos)
                elif last_target_waypoint and zoom_center is None:
                    zoom_center = tuple(last_target_waypoint)

                full_frame = last_frame.copy()

                # --- Draw overlays on full frame ---
                _draw_overlays(full_frame, last_robot_pos, last_robot_data,
                               last_target_waypoint, last_robot_angle_deg,
                               last_angle_error_deg, last_target_heading_deg,
                               last_movement_mode, last_behavior_params,
                               last_movement_active, reached_waypoint,
                               last_ball_waypoint, last_overshoot_target,
                               last_capture_phase, last_dribbler_on)

                # --- Create zoom view (full resolution 640x480, no distortion) ---
                if zoom_center:
                    zx, zy = int(zoom_center[0]), int(zoom_center[1])
                    crop_h = int(frame_height / zoom_level)
                    crop_w = int(frame_width / zoom_level)
                    x1 = max(0, zx - crop_w // 2)
                    y1 = max(0, zy - crop_h // 2)
                    x2 = min(frame_width, x1 + crop_w)
                    y2 = min(frame_height, y1 + crop_h)
                    if x2 - x1 < crop_w:
                        x1 = max(0, x2 - crop_w)
                    if y2 - y1 < crop_h:
                        y1 = max(0, y2 - crop_h)

                    zoom_raw = last_frame[y1:y2, x1:x2].copy()
                    zoom_frame = cv2.resize(zoom_raw, (zoom_w, zoom_h),
                                           interpolation=cv2.INTER_LINEAR)

                    _draw_overlays_zoom(zoom_frame, last_robot_pos, last_robot_data,
                                        last_target_waypoint, last_robot_angle_deg,
                                        last_angle_error_deg, last_target_heading_deg,
                                        last_movement_mode, last_behavior_params,
                                        last_movement_active, reached_waypoint,
                                        x1, y1, crop_w, crop_h,
                                        zoom_w, zoom_h, zoom_level,
                                        last_ball_waypoint, last_overshoot_target,
                                        last_capture_phase, last_dribbler_on)
                else:
                    zoom_frame = full_frame.copy()
                    x1 = y1 = x2 = y2 = 0

                # --- Mini full view (320x240, proportional, no distortion) ---
                mini_full = cv2.resize(full_frame, (mini_w, mini_h),
                                       interpolation=cv2.INTER_AREA)

                # Draw zoom region rectangle on mini view
                if zoom_center:
                    sx = mini_w / frame_width
                    sy = mini_h / frame_height
                    cv2.rectangle(mini_full,
                                  (int(x1 * sx), int(y1 * sy)),
                                  (int(x2 * sx), int(y2 * sy)),
                                  (0, 255, 255), 1)

                # --- Build right column (320x480): mini on top, info below ---
                right_col = np.zeros((zoom_h, right_col_w, 3), dtype=np.uint8)
                right_col[0:mini_h, 0:mini_w] = mini_full

                # Info area below mini view
                info_y = mini_h + 20
                cv2.putText(right_col, "FULL VIEW", (5, mini_h + 15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

                # Status + capture phase
                phase_labels = {
                    'idle': 'STOPPED',
                    'approach': 'FASE 1: APPROACH',
                    'capture': 'FASE 2: CAPTURE',
                }
                if last_movement_active:
                    status_text = phase_labels.get(last_capture_phase, 'MOVING')
                else:
                    if last_capture_phase == 'approach':
                        status_text = 'FASE 1 OK → D'
                    elif last_capture_phase == 'capture':
                        status_text = 'FASE 2 OK'
                    else:
                        status_text = 'STOPPED'
                status_color = (0, 255, 0) if last_movement_active else (0, 100, 255)
                cv2.putText(right_col, status_text, (5, info_y + 15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 2)

                # Dribbler indicator
                if last_dribbler_on:
                    cv2.putText(right_col, "DRIBBLER ON", (200, info_y + 15),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 80), 2)

                # Angular error info
                if last_angle_error_deg is not None and last_movement_active:
                    cv2.putText(right_col, f"Err: {last_angle_error_deg:+.1f} deg",
                               (5, info_y + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                               (255, 255, 255), 1)
                    if last_movement_mode:
                        mode_labels = {'rotating': 'GIRANDO', 'linear': 'LINEAL',
                                       'arrived_angle': 'ALINEADO'}
                        mode_color = (0, 100, 255) if last_movement_mode == 'rotating' else (0, 255, 200)
                        cv2.putText(right_col, mode_labels.get(last_movement_mode, ''),
                                   (5, info_y + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                                   mode_color, 1)

                # Distance info
                if last_robot_pos and last_target_waypoint:
                    dx = last_target_waypoint[0] - last_robot_pos[0]
                    dy = last_target_waypoint[1] - last_robot_pos[1]
                    dist = math.sqrt(dx * dx + dy * dy)
                    pos_thresh = last_behavior_params['position_threshold']
                    dist_color = (0, 255, 0) if dist < pos_thresh else (255, 255, 255)
                    cv2.putText(right_col, f"Dist: {dist:.0f}px (th={pos_thresh})",
                               (5, info_y + 85), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                               dist_color, 1)

                # Reached waypoint info
                if reached_waypoint:
                    cv2.putText(right_col, f"Llegada: ({reached_waypoint[0]}, {reached_waypoint[1]})",
                               (5, info_y + 110), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                               (150, 150, 255), 1)
                    if last_robot_pos:
                        dx = last_robot_pos[0] - reached_waypoint[0]
                        dy = last_robot_pos[1] - reached_waypoint[1]
                        offset = math.sqrt(dx * dx + dy * dy)
                        cv2.putText(right_col, f"Error pos: {offset:.1f}px",
                                   (5, info_y + 130), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                                   (150, 150, 255), 1)

                # Zoom label on left frame
                cv2.putText(zoom_frame, f"ZOOM x{zoom_level:.1f}", (5, 18),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

                # Combine: zoom (left 640x480) + right column (320x480)
                video_frame = np.hstack([zoom_frame, right_col])

                panel = _draw_behavior_panel(last_behavior_params, last_robot_pos,
                                             last_target_waypoint, last_robot_available,
                                             last_robot_detected, last_angle_error_deg,
                                             last_movement_mode, reached_waypoint,
                                             panel_height, total_w)
                combined = np.vstack([panel, video_frame])
                cv2.imshow(window_name, combined)
            else:
                panel = _draw_behavior_panel(last_behavior_params, last_robot_pos,
                                             last_target_waypoint, last_robot_available,
                                             last_robot_detected, None, None, None,
                                             panel_height, total_w)
                placeholder = np.zeros((zoom_h, total_w, 3), dtype=np.uint8)
                cv2.putText(placeholder, "Esperando frames...",
                           (total_w // 2 - 100, zoom_h // 2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2)
                combined = np.vstack([panel, placeholder])
                cv2.imshow(window_name, combined)

            key = cv2.waitKey(1) & 0xFF
            if key != 255:
                command = None
                param = None
                delta = 0
                waypoint_delta = None

                if key == 27:
                    command = 'exit'
                elif key == ord(' '):
                    command = 'toggle_movement'
                elif key == ord('d') or key == ord('D'):
                    command = 'start_capture'
                elif key == ord('x') or key == ord('X'):
                    command = 'cancel_waypoint'
                    reached_waypoint = None
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
                elif key == ord('u') or key == ord('U'):
                    command = 'adjust_threshold'
                    param = 'capture_activate_px'
                    delta = 1
                elif key == ord('j') or key == ord('J'):
                    command = 'adjust_threshold'
                    param = 'capture_activate_px'
                    delta = -1
                elif key == ord('i') or key == ord('I'):
                    command = 'adjust_threshold'
                    param = 'capture_overshoot_px'
                    delta = 1
                elif key == ord('k') or key == ord('K'):
                    command = 'adjust_threshold'
                    param = 'capture_overshoot_px'
                    delta = -1
                elif key == ord('o') or key == ord('O'):
                    command = 'adjust_threshold'
                    param = 'capture_confirm_px'
                    delta = 1
                elif key == ord('l') or key == ord('L'):
                    command = 'adjust_threshold'
                    param = 'capture_confirm_px'
                    delta = -1
                elif key == ord('n') or key == ord('N'):
                    command = 'adjust_threshold'
                    param = 'creep_speed_pwm'
                    delta = -1
                elif key == ord('m') or key == ord('M'):
                    command = 'adjust_threshold'
                    param = 'creep_speed_pwm'
                    delta = 1
                elif key in [82, 84, 81, 83] and last_robot_pos:
                    command = 'move_waypoint'
                    reached_waypoint = None  # New waypoint movement clears reached
                    waypoint_delta = [0, 0]
                    if key == 82:
                        waypoint_delta = [0, -10]
                    elif key == 84:
                        waypoint_delta = [0, 10]
                    elif key == 81:
                        waypoint_delta = [-10, 0]
                    elif key == 83:
                        waypoint_delta = [10, 0]

                elif key == ord('+'):
                    zoom_level = min(zoom_max, zoom_level + 0.5)
                elif key == ord('_'):
                    zoom_level = max(zoom_min, zoom_level - 0.5)
                elif key == ord('v') or key == ord('V'):
                    if last_robot_pos:
                        zoom_center = tuple(last_robot_pos)
                elif key == ord('w') or key == ord('W'):
                    if last_target_waypoint:
                        zoom_center = tuple(last_target_waypoint)
                elif key == ord('c') or key == ord('C'):
                    zoom_center = None
                elif key == ord('r') or key == ord('R'):
                    zoom_level = 2.0
                    if last_robot_pos:
                        zoom_center = tuple(last_robot_pos)

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
                        log.warning(f"Error enviando comando: {e}")

    except KeyboardInterrupt:
        log.info("Proceso de visualización detenido por usuario")
    except Exception as e:
        log.error(f"Error en proceso de visualización: {e}", exc_info=True)
    finally:
        shm.close()
        cv2.destroyAllWindows()
        log.info("Ventana cerrada, shared memory desconectada")


def _draw_overlays(frame, robot_pos, robot_data, waypoint,
                   robot_angle_deg, angle_error_deg, target_heading_deg,
                   movement_mode, behavior_params, movement_active,
                   reached_waypoint,
                   ball_waypoint=None, overshoot_target=None,
                   capture_phase='idle', dribbler_on=False):
    """Draw robot, waypoint, trajectory and angular info on a frame."""
    # Draw reached waypoint (persistent marker, dimmer)
    if reached_waypoint:
        rwx, rwy = int(reached_waypoint[0]), int(reached_waypoint[1])
        cross_size = 8
        color_reached = (120, 120, 200)  # dim blue-ish
        cv2.line(frame, (rwx - cross_size, rwy), (rwx + cross_size, rwy),
                color_reached, 1, cv2.LINE_AA)
        cv2.line(frame, (rwx, rwy - cross_size), (rwx, rwy + cross_size),
                color_reached, 1, cv2.LINE_AA)
        cv2.putText(frame, "llegada", (rwx + 10, rwy - 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.3, color_reached, 1, cv2.LINE_AA)

    # --- Capture circles around ball_waypoint ---
    bw = ball_waypoint if ball_waypoint else waypoint
    if bw:
        bx, by = int(bw[0]), int(bw[1])
        activate_r = behavior_params.get('capture_activate_px', 38)
        confirm_r = behavior_params.get('capture_confirm_px', 20)

        # Outer circle: capture_activate_px (yellow) — dribbler activates here
        cv2.circle(frame, (bx, by), activate_r, (0, 200, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, f"activar {activate_r}px", (bx + activate_r + 3, by - 3),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.28, (0, 200, 255), 1, cv2.LINE_AA)

        # Inner circle: capture_confirm_px (green) — capture confirmed here
        cv2.circle(frame, (bx, by), confirm_r, (0, 255, 100), 1, cv2.LINE_AA)
        cv2.putText(frame, f"confirm {confirm_r}px", (bx + confirm_r + 3, by + 12),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.28, (0, 255, 100), 1, cv2.LINE_AA)

    # Overshoot target point (cyan dot + position_threshold circle)
    if overshoot_target:
        ox, oy = int(overshoot_target[0]), int(overshoot_target[1])
        cv2.circle(frame, (ox, oy), 4, (255, 255, 0), -1, cv2.LINE_AA)
        pos_thresh = behavior_params.get('position_threshold', 32)
        cv2.circle(frame, (ox, oy), pos_thresh, (255, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, f"overshoot (PID para a {pos_thresh}px)",
                   (ox + pos_thresh + 3, oy + 4),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.28, (255, 255, 0), 1, cv2.LINE_AA)

    # Draw active waypoint as a subtle cross
    if waypoint:
        wx, wy = int(waypoint[0]), int(waypoint[1])
        cross_size = 8
        color_wp = (100, 255, 100)
        cv2.line(frame, (wx - cross_size, wy), (wx + cross_size, wy),
                color_wp, 1, cv2.LINE_AA)
        cv2.line(frame, (wx, wy - cross_size), (wx, wy + cross_size),
                color_wp, 1, cv2.LINE_AA)

    # Draw robot
    if robot_pos:
        rx, ry = robot_pos
        cv2.circle(frame, (rx, ry), 14, (0, 200, 0), 2, cv2.LINE_AA)

        # Robot heading arrow
        if robot_data and 'angulo' in robot_data:
            angle_rad = math.radians(robot_data['angulo'])
            arrow_len = 25
            end_x = int(rx + arrow_len * math.cos(angle_rad))
            end_y = int(ry + arrow_len * math.sin(angle_rad))
            cv2.arrowedLine(frame, (rx, ry), (end_x, end_y),
                           (0, 200, 0), 2, cv2.LINE_AA, tipLength=0.3)

        # Trajectory line + target heading + angular error
        if waypoint:
            wx, wy = int(waypoint[0]), int(waypoint[1])
            cv2.line(frame, (rx, ry), (wx, wy), (200, 200, 0), 1, cv2.LINE_AA)

            if target_heading_deg is not None:
                th_rad = math.radians(target_heading_deg)
                th_len = 20
                thx = int(rx + th_len * math.cos(th_rad))
                thy = int(ry + th_len * math.sin(th_rad))
                cv2.arrowedLine(frame, (rx, ry), (thx, thy),
                               (255, 255, 0), 1, cv2.LINE_AA, tipLength=0.35)

            if angle_error_deg is not None and robot_data and 'angulo' in robot_data:
                _draw_angle_error_arc(frame, rx, ry, robot_data['angulo'],
                                      angle_error_deg, movement_mode)


def _draw_angle_error_arc(frame, cx, cy, robot_angle_deg, angle_error_deg, movement_mode):
    """Draw an arc showing the angular error between robot heading and target heading."""
    arc_radius = 30
    start_angle = robot_angle_deg
    end_angle = start_angle + angle_error_deg

    if movement_mode == 'rotating':
        arc_color = (0, 100, 255)
    elif movement_mode == 'linear':
        arc_color = (0, 255, 200)
    else:
        arc_color = (0, 255, 0)

    if abs(angle_error_deg) > 1:
        cv2.ellipse(frame, (cx, cy), (arc_radius, arc_radius),
                    0, start_angle, end_angle, arc_color, 2, cv2.LINE_AA)

        text_angle = math.radians(start_angle + angle_error_deg / 2)
        tx = int(cx + (arc_radius + 12) * math.cos(text_angle))
        ty = int(cy + (arc_radius + 12) * math.sin(text_angle))
        cv2.putText(frame, f"{angle_error_deg:+.0f}", (tx - 12, ty + 4),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.32, arc_color, 1, cv2.LINE_AA)


def _draw_overlays_zoom(frame, robot_pos, robot_data, waypoint,
                         robot_angle_deg, angle_error_deg, target_heading_deg,
                         movement_mode, behavior_params, movement_active,
                         reached_waypoint,
                         crop_x1, crop_y1, crop_w, crop_h,
                         out_w, out_h, zoom_level,
                         ball_waypoint=None, overshoot_target=None,
                         capture_phase='idle', dribbler_on=False):
    """Draw overlays on the zoom frame with coordinate transformation."""
    sx = out_w / crop_w
    sy = out_h / crop_h

    def to_zoom(px, py):
        return int((px - crop_x1) * sx), int((py - crop_y1) * sy)

    def in_bounds(zx, zy):
        return -50 <= zx < out_w + 50 and -50 <= zy < out_h + 50

    # Reached waypoint (persistent dim marker)
    if reached_waypoint:
        zwx, zwy = to_zoom(reached_waypoint[0], reached_waypoint[1])
        if in_bounds(zwx, zwy):
            cross_size = int(10 * zoom_level)
            color_reached = (120, 120, 200)
            cv2.line(frame, (zwx - cross_size, zwy), (zwx + cross_size, zwy),
                    color_reached, 1, cv2.LINE_AA)
            cv2.line(frame, (zwx, zwy - cross_size), (zwx, zwy + cross_size),
                    color_reached, 1, cv2.LINE_AA)
            cv2.putText(frame, "llegada", (zwx + cross_size + 3, zwy - 3),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.35, color_reached, 1, cv2.LINE_AA)

    # --- Capture circles around ball_waypoint (zoom coords) ---
    bw = ball_waypoint if ball_waypoint else waypoint
    if bw:
        zbx, zby = to_zoom(bw[0], bw[1])
        if in_bounds(zbx, zby):
            activate_r = int(behavior_params.get('capture_activate_px', 38) * sx)
            confirm_r = int(behavior_params.get('capture_confirm_px', 20) * sx)

            # Outer: capture_activate_px (yellow)
            cv2.circle(frame, (zbx, zby), activate_r, (0, 200, 255), 1, cv2.LINE_AA)
            cv2.putText(frame, f"activar", (zbx + activate_r + 3, zby - 3),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 200, 255), 1, cv2.LINE_AA)

            # Inner: capture_confirm_px (green)
            cv2.circle(frame, (zbx, zby), confirm_r, (0, 255, 100), 1, cv2.LINE_AA)
            cv2.putText(frame, f"confirm", (zbx + confirm_r + 3, zby + 14),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 255, 100), 1, cv2.LINE_AA)

    # Overshoot target (cyan dot + position_threshold circle)
    if overshoot_target:
        zox, zoy = to_zoom(overshoot_target[0], overshoot_target[1])
        if in_bounds(zox, zoy):
            dot_r = max(3, int(4 * sx))
            cv2.circle(frame, (zox, zoy), dot_r, (255, 255, 0), -1, cv2.LINE_AA)
            pos_thresh = int(behavior_params.get('position_threshold', 32) * sx)
            cv2.circle(frame, (zox, zoy), pos_thresh, (255, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(frame, "overshoot", (zox + pos_thresh + 3, zoy + 4),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.32, (255, 255, 0), 1, cv2.LINE_AA)

    # Waypoint cross
    if waypoint:
        zwx, zwy = to_zoom(waypoint[0], waypoint[1])
        if in_bounds(zwx, zwy):
            cross_size = int(10 * zoom_level)
            color_wp = (100, 255, 100)
            cv2.line(frame, (zwx - cross_size, zwy), (zwx + cross_size, zwy),
                    color_wp, 1, cv2.LINE_AA)
            cv2.line(frame, (zwx, zwy - cross_size), (zwx, zwy + cross_size),
                    color_wp, 1, cv2.LINE_AA)

    # Robot
    if robot_pos:
        zrx, zry = to_zoom(robot_pos[0], robot_pos[1])
        if in_bounds(zrx, zry):
            robot_r = int(14 * sx)
            cv2.circle(frame, (zrx, zry), robot_r, (0, 200, 0), 2, cv2.LINE_AA)

            # Heading arrow
            if robot_data and 'angulo' in robot_data:
                angle_rad = math.radians(robot_data['angulo'])
                arrow_len = int(30 * sx)
                end_x = int(zrx + arrow_len * math.cos(angle_rad))
                end_y = int(zry + arrow_len * math.sin(angle_rad))
                cv2.arrowedLine(frame, (zrx, zry), (end_x, end_y),
                               (0, 200, 0), 2, cv2.LINE_AA, tipLength=0.25)

            # Target heading arrow
            if waypoint and target_heading_deg is not None:
                th_rad = math.radians(target_heading_deg)
                th_len = int(25 * sx)
                thx = int(zrx + th_len * math.cos(th_rad))
                thy = int(zry + th_len * math.sin(th_rad))
                cv2.arrowedLine(frame, (zrx, zry), (thx, thy),
                               (255, 255, 0), 1, cv2.LINE_AA, tipLength=0.3)

            # Trajectory line
            if waypoint:
                zwx, zwy = to_zoom(waypoint[0], waypoint[1])
                cv2.line(frame, (zrx, zry), (zwx, zwy), (200, 200, 0), 1, cv2.LINE_AA)

            # Angular error arc (scaled)
            if angle_error_deg is not None and robot_data and 'angulo' in robot_data:
                arc_radius = int(35 * sx)
                start_angle = robot_data['angulo']
                end_angle = start_angle + angle_error_deg

                if movement_mode == 'rotating':
                    arc_color = (0, 100, 255)
                elif movement_mode == 'linear':
                    arc_color = (0, 255, 200)
                else:
                    arc_color = (0, 255, 0)

                if abs(angle_error_deg) > 1:
                    cv2.ellipse(frame, (zrx, zry), (arc_radius, arc_radius),
                                0, start_angle, end_angle, arc_color, 2, cv2.LINE_AA)

                    text_angle = math.radians(start_angle + angle_error_deg / 2)
                    tx = int(zrx + (arc_radius + 15) * math.cos(text_angle))
                    ty = int(zry + (arc_radius + 15) * math.sin(text_angle))
                    cv2.putText(frame, f"{angle_error_deg:+.0f}",
                               (tx - 14, ty + 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, arc_color, 1, cv2.LINE_AA)


def _draw_behavior_panel(behavior_params, robot_pos, waypoint, robot_available,
                          robot_detected, angle_error_deg, movement_mode,
                          reached_waypoint, panel_height=450, panel_width=960):
    """Dibuja el panel de control de comportamiento."""
    panel = np.zeros((panel_height, panel_width, 3), dtype=np.uint8)

    y = 25
    lh = 22

    cv2.putText(panel, "CALIBRACION DE COMPORTAMIENTO", (10, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    y += lh + 8

    # Status
    if robot_available:
        status_color = (0, 255, 0) if robot_detected else (0, 100, 255)
        status_text = "Robot detectado" if robot_detected else "Robot NO visible"
    else:
        status_color = (0, 0, 255)
        status_text = "Robot NO conectado"
    cv2.putText(panel, status_text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, status_color, 1)
    y += lh + 6

    # Angular error display
    if angle_error_deg is not None:
        err_color = (0, 255, 0) if abs(angle_error_deg) < behavior_params['angle_threshold'] else (0, 100, 255)
        cv2.putText(panel, f"Error angular: {angle_error_deg:+.1f} deg", (10, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, err_color, 1)
        if movement_mode:
            mode_label = {'rotating': 'GIRANDO', 'linear': 'LINEAL', 'arrived_angle': 'ALINEADO'}
            cv2.putText(panel, mode_label.get(movement_mode, movement_mode),
                       (280, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, err_color, 1)
    y += lh + 4

    # --- Left column: thresholds ---
    col_left_x = 10
    col_right_x = panel_width // 2

    cv2.putText(panel, "UMBRALES DE PRECISION", (col_left_x, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    y_left = y + lh

    cv2.putText(panel, f"Posicion: {behavior_params['position_threshold']}px", (col_left_x, y_left),
               cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(panel, "(9/0)", (250, y_left), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)
    y_left += lh

    cv2.putText(panel, f"Angular: {behavior_params['angle_threshold']} deg", (col_left_x, y_left),
               cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(panel, "(-/=)", (250, y_left), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)
    y_left += lh + 6

    cv2.putText(panel, "POLITICA DE MOVIMIENTO", (col_left_x, y_left),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    y_left += lh

    cv2.putText(panel, f"Inicio lineal: {behavior_params['linear_start_angle_threshold']} deg", (col_left_x, y_left),
               cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(panel, "([/])", (250, y_left), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)
    y_left += int(lh * 0.8)
    cv2.putText(panel, "Angulo max para avanzar (mayor = menos giros)", (20, y_left),
               cv2.FONT_HERSHEY_SIMPLEX, 0.33, (150, 150, 150), 1)
    y_left += lh + 4

    cv2.putText(panel, f"Max correccion: {behavior_params['max_angular_correction_pwm']} PWM", (col_left_x, y_left),
               cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(panel, "(,/.)", (250, y_left), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)
    y_left += int(lh * 0.8)
    cv2.putText(panel, "(no usado - Dual PID v+w lo reemplaza)", (20, y_left),
               cv2.FONT_HERSHEY_SIMPLEX, 0.33, (100, 100, 150), 1)
    y_left += lh + 4

    cv2.putText(panel, "CAPTURA DE BALON", (col_left_x, y_left),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    y_left += lh

    cv2.putText(panel, f"Activar: {behavior_params.get('capture_activate_px', 38)}px", (col_left_x, y_left),
               cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(panel, "(U/J)", (250, y_left), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)
    y_left += lh

    cv2.putText(panel, f"Overshoot: {behavior_params.get('capture_overshoot_px', 15)}px", (col_left_x, y_left),
               cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(panel, "(I/K)", (250, y_left), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)
    y_left += lh

    cv2.putText(panel, f"Confirmar: {behavior_params.get('capture_confirm_px', 20)}px", (col_left_x, y_left),
               cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(panel, "(O/L)", (250, y_left), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)
    y_left += lh

    cv2.putText(panel, f"Creep: {behavior_params.get('creep_speed_pwm', 30)} PWM", (col_left_x, y_left),
               cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(panel, "(N/M)", (250, y_left), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)

    # --- Right column: waypoint info + controls ---
    y_right = y + lh

    # Waypoint info
    if waypoint:
        cv2.putText(panel, f"Waypoint: ({waypoint[0]}, {waypoint[1]})", (col_right_x, y_right),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
    else:
        cv2.putText(panel, "Sin waypoint (click o flechas)", (col_right_x, y_right),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (128, 128, 128), 1)
    y_right += lh

    if reached_waypoint:
        cv2.putText(panel, f"Llegada: ({reached_waypoint[0]}, {reached_waypoint[1]})", (col_right_x, y_right),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 255), 1)
        y_right += lh
        if robot_pos:
            dx = robot_pos[0] - reached_waypoint[0]
            dy = robot_pos[1] - reached_waypoint[1]
            offset = math.sqrt(dx * dx + dy * dy)
            cv2.putText(panel, f"Error de posicion: {offset:.1f}px", (col_right_x, y_right),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 255), 1)
    y_right += lh + 8

    # Controls
    cv2.putText(panel, "CONTROLES", (col_right_x, y_right),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 1)
    y_right += lh

    controls = [
        "Click: Waypoint (= posicion pelota)",
        "ESPACIO: Fase 1 (approach → parar a activate_px)",
        "D: Fase 2 (dribbler ON + creep a overshoot)",
        "X: Cancelar  |  ENTER: Guardar config",
        "U/J: Activar  I/K: Overshoot  O/L: Confirm",
        "N/M: Creep PWM  9/0: PosThresh",
        "+/_: Zoom in/out  V/W/C/R: Zoom ctrl",
        "ESC: Salir",
    ]

    for ctrl in controls:
        cv2.putText(panel, ctrl, (col_right_x, y_right),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.33, (200, 200, 200), 1)
        y_right += lh - 3

    return panel
