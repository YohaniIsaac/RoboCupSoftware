"""Proceso de control para calibración de umbrales de comportamiento.

Este proceso se ejecuta independientemente y:
- Recibe posiciones de robots desde el proceso de percepción
- Maneja la interfaz gráfica con panel de umbrales
- Permite ajustar umbrales de comportamiento con el teclado
- Envía comandos RF al robot seleccionado
"""

import sys
import time
import math
import logging
from pathlib import Path

import cv2
import numpy as np

# Agregar src al path
ROOT_DIR = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))

from robot_soccer.controllers.differential_drive import DifferentialDriveController
from robot_soccer.controllers.robot_calibration import RobotCalibration
from robot_soccer.communication.rf_controller import RFController
from robot_soccer.config import (
    ROBOT_POSITION_THRESHOLD,
    ROBOT_ANGLE_THRESHOLD_DEG,
    ROBOT_LINEAR_START_ANGLE_THRESHOLD_DEG,
    MAX_ANGULAR_CORRECTION_PWM,
    CAPTURE_ACTIVATE_DISTANCE_PX,
    CAPTURE_OVERSHOOT_PX,
    CAPTURE_CONFIRM_DISTANCE_PX,
    CAPTURE_CREEP_SPEED_PWM,
    DRIBBLE_PWM_FACTOR,
    DRIBBLER_CAPTURE_POWER,
    DRIBBLER_HOLD_POWER,
    DRIBBLER_FW_ON_MS,
    DRIBBLER_FW_OFF_MS,
    BEHIND_BALL_APPROACH_PX,
    CONTACT_SETTLE_TIME_S,
    CAMERA_PERSPECTIVE_WIDTH,
    CAMERA_PERSPECTIVE_HEIGHT,
    FIELD_CAM,
    STUCK_MOVEMENT_THRESHOLD_PX,
    STUCK_DETECTION_WINDOW_S,
    STUCK_BOOST_INCREMENT,
    STUCK_BOOST_MAX,
    STUCK_AUTO_KICK,
    KICK_POINT_OFFSET_PX,
    KICK_POINT_TOLERANCE_PX,
    KICK_POINT_ANGLE_OFFSET_DEG,
)

from robot_soccer.utils.robot_logger import robot_status_logger

log = logging.getLogger(__name__)


class RobotEntity:
    """Entidad de robot para el controlador."""

    def __init__(self, robot_id, x, y, angle):
        self.id = robot_id
        self.x = x
        self.y = y
        self.angle = angle
        self.dx = 0.0
        self.dy = 0.0
        self.dw = 0.0

    def update(self, x, y, angle):
        """Actualiza la posición del robot."""
        self.x = x
        self.y = y
        self.angle = angle


def control_loop_behavior(robot_positions_pipe, frame_pipe, robot_id, serial_port):
    """Bucle principal del proceso de control de comportamiento.

    Args:
        robot_positions_pipe: Pipe para recibir posiciones de robots desde percepción
        frame_pipe: Pipe para recibir frames procesados desde percepción
        robot_id: ID del robot a controlar
        serial_port: Puerto serial para comunicación RF
    """
    log.info(f"🎮 Proceso de control de comportamiento iniciado para Robot ID {robot_id}")

    # Inicializar RF controller
    rf_controller = None
    robot_available = False
    try:
        log.info("🔌 Iniciando comunicación RF...")
        rf_controller = RFController(port=serial_port, enable_calibration=True)
        if rf_controller.initialize():
            log.info("✅ Conexión Serial establecida")

            connections = rf_controller.test_connections()
            robot_key = f'robot_{robot_id + 1}'
            robot_available = connections.get(robot_key, False)

            if robot_available:
                log.info(f"✅ Robot {robot_id} disponible via RF")
            else:
                log.warning(f"⚠️  Robot {robot_id} NO responde via RF")
        else:
            log.warning("⚠️  No se pudo conectar al transmisor")
    except Exception as e:
        log.warning(f"⚠️  Error RF: {e}")

    # Parámetros de comportamiento
    behavior_params = {
        'position_threshold': ROBOT_POSITION_THRESHOLD,
        'angle_threshold': ROBOT_ANGLE_THRESHOLD_DEG,
        'linear_start_angle_threshold': ROBOT_LINEAR_START_ANGLE_THRESHOLD_DEG,
        'max_angular_correction_pwm': MAX_ANGULAR_CORRECTION_PWM,
        'capture_activate_px': CAPTURE_ACTIVATE_DISTANCE_PX,
        'capture_overshoot_px': CAPTURE_OVERSHOOT_PX,
        'capture_confirm_px': CAPTURE_CONFIRM_DISTANCE_PX,
        'creep_speed_pwm': CAPTURE_CREEP_SPEED_PWM,
        'kick_point_offset_px': KICK_POINT_OFFSET_PX,
        'kick_point_tolerance_px': KICK_POINT_TOLERANCE_PX,
        'kick_point_angle_offset_deg': KICK_POINT_ANGLE_OFFSET_DEG,
        'dribble_pwm_factor': DRIBBLE_PWM_FACTOR,
        'dribbler_capture_power': DRIBBLER_CAPTURE_POWER,
        'dribbler_hold_power': DRIBBLER_HOLD_POWER,
        'dribbler_fw_on_ms': DRIBBLER_FW_ON_MS,
        'dribbler_fw_off_ms': DRIBBLER_FW_OFF_MS,
        'stuck_movement_threshold_px': STUCK_MOVEMENT_THRESHOLD_PX,
        'stuck_detection_window_s':    STUCK_DETECTION_WINDOW_S,
        'stuck_boost_increment':       STUCK_BOOST_INCREMENT,
        'stuck_boost_max':             STUCK_BOOST_MAX,
        'stuck_auto_kick':             STUCK_AUTO_KICK,
    }

    # Controlador
    controller = DifferentialDriveController(rf_controller=rf_controller)
    _update_behavior_controller(controller, behavior_params)

    # Estado
    robot = None
    target_waypoint = None
    movement_active = False
    running = True
    last_frame = None

    # Debounce para tecla ESPACIO
    last_space_time = 0

    # Flag para saber si ya enviamos comando de detención
    robot_stopped = False

    # Crear ventanas
    cv2.namedWindow('Robot View', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Behavior Control Panel', cv2.WINDOW_NORMAL)

    log.info("✅ Interfaz iniciada - Esperando datos de percepción...")

    try:
        while running:
            # Recibir posiciones de robots (sin bloqueo)
            if robot_positions_pipe.poll():
                try:
                    data = robot_positions_pipe.recv()
                    robots_list = data['robots']

                    # Buscar nuestro robot en la lista por ID
                    robot_found = None
                    for r in robots_list:
                        if r['id'] == robot_id:
                            robot_found = r
                            break

                    # Actualizar robot si está detectado
                    if robot_found:
                        angle_rad = math.radians(robot_found['angulo'])
                        if robot is None:
                            robot = RobotEntity(robot_id, robot_found['x'], robot_found['y'], angle_rad)
                            log.info(f"🤖 Robot {robot_id} detectado en ({robot_found['x']:.0f}, {robot_found['y']:.0f})")
                        else:
                            robot.update(robot_found['x'], robot_found['y'], angle_rad)
                    else:
                        if robot is not None:
                            log.warning(f"⚠️  Robot {robot_id} perdido")
                        robot = None

                except Exception as e:
                    log.error(f"Error recibiendo posiciones: {e}")

            # Recibir frame procesado (sin bloqueo)
            if frame_pipe.poll():
                try:
                    last_frame = frame_pipe.recv()
                except Exception as e:
                    log.error(f"Error recibiendo frame: {e}")

            # Control de movimiento si hay waypoint activo
            if movement_active and robot and target_waypoint:
                reached = controller.move_to_position(
                    robot,
                    tuple(target_waypoint)
                )
                if reached:
                    log.info(f"✅ Waypoint alcanzado en ({target_waypoint[0]}, {target_waypoint[1]})")
                    target_waypoint = None
                    movement_active = False
                    robot_stopped = False
            else:
                # Detener robot si no hay movimiento activo
                if robot and robot_available and not robot_stopped:
                    firmware_id = robot_id + 1
                    rf_controller.set_motors(firmware_id, 0, 0)
                    robot_stopped = True

            # Visualización
            if last_frame is not None:
                vis_frame = last_frame.copy()

                # Dibujar waypoint
                if target_waypoint:
                    cv2.circle(vis_frame, tuple(target_waypoint), 15, (0, 255, 0), 2)
                    cv2.circle(vis_frame, tuple(target_waypoint), 3, (0, 255, 0), -1)

                    # Dibujar línea de robot a waypoint
                    if robot:
                        cv2.line(
                            vis_frame,
                            (int(robot.x), int(robot.y)),
                            tuple(target_waypoint),
                            (255, 255, 0), 1
                        )

                # Overlay kick_point: punto donde se asume que impacta el solenoide.
                # El usuario coloca la pelota delante del robot en posición ideal
                # de kick (dribbler tocando) y ajusta el offset (F/V) hasta que la
                # cruz coincida con el centro de la pelota. El círculo es la
                # tolerancia (H/B) para confirmar contacto.
                if robot:
                    rx, ry = int(robot.x), int(robot.y)
                    kp_off = behavior_params.get('kick_point_offset_px', KICK_POINT_OFFSET_PX)
                    kp_tol = behavior_params.get('kick_point_tolerance_px', KICK_POINT_TOLERANCE_PX)
                    kpx = int(rx + kp_off * math.cos(robot.angle))
                    kpy = int(ry + kp_off * math.sin(robot.angle))
                    kp_col = (40, 220, 220)
                    # Línea punteada robot -> kick_point
                    steps = 6
                    for s in range(steps):
                        if s % 2 == 0:
                            x0 = int(rx + (kpx - rx) * s / steps)
                            y0 = int(ry + (kpy - ry) * s / steps)
                            x1 = int(rx + (kpx - rx) * (s + 1) / steps)
                            y1 = int(ry + (kpy - ry) * (s + 1) / steps)
                            cv2.line(vis_frame, (x0, y0), (x1, y1),
                                     kp_col, 1, cv2.LINE_AA)
                    cv2.line(vis_frame, (kpx - 5, kpy), (kpx + 5, kpy),
                             kp_col, 2, cv2.LINE_AA)
                    cv2.line(vis_frame, (kpx, kpy - 5), (kpx, kpy + 5),
                             kp_col, 2, cv2.LINE_AA)
                    cv2.circle(vis_frame, (kpx, kpy), max(1, int(kp_tol)),
                               kp_col, 1, cv2.LINE_AA)

                # Indicador de movimiento
                status_text = "MOVING" if movement_active else "STOPPED"
                status_color = (0, 255, 0) if movement_active else (0, 0, 255)
                cv2.putText(vis_frame, status_text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, status_color, 2)

                cv2.imshow('Robot View', vis_frame)

            # Panel de control
            panel = _draw_behavior_panel(behavior_params, robot, target_waypoint, robot_available)
            cv2.imshow('Behavior Control Panel', panel)

            # Manejo de teclado
            key = cv2.waitKey(1) & 0xFF

            if key != 255:  # Si se presionó alguna tecla
                result = _handle_keyboard_behavior(
                    key, robot, target_waypoint, behavior_params,
                    movement_active, last_space_time, controller
                )

                if result['action'] == 'quit':
                    running = False
                elif result['action'] == 'toggle_movement':
                    movement_active = result['movement_active']
                    last_space_time = result['last_space_time']
                    robot_stopped = False
                elif result['action'] == 'cancel_waypoint':
                    target_waypoint = None
                    movement_active = False
                    robot_stopped = False
                elif result['action'] == 'save':
                    _save_behavior_to_config(behavior_params)
                elif result['action'] == 'update_behavior':
                    _update_behavior_controller(controller, behavior_params)

                if 'waypoint' in result:
                    target_waypoint = result['waypoint']

            time.sleep(0.01)  # 100 Hz loop

    finally:
        # Limpieza
        if robot and robot_available and rf_controller:
            firmware_id = robot_id + 1
            rf_controller.set_motors(firmware_id, 0, 0)
            log.info("🛑 Robot detenido")

        if rf_controller:
            rf_controller.shutdown()
            log.info("🔌 Conexión RF cerrada")

        cv2.destroyAllWindows()
        log.info("🎮 Proceso de control de comportamiento finalizado")


def _update_behavior_controller(controller, behavior_params):
    """Actualiza los parámetros de comportamiento del controlador en tiempo real."""
    controller.position_threshold = behavior_params['position_threshold']
    controller.angle_threshold = math.radians(behavior_params['angle_threshold'])
    controller.dribble_pwm_factor = behavior_params.get('dribble_pwm_factor', 1.0)
    # MAX_ANGULAR_CORRECTION ya no se usa (reemplazado por Dual PID v+ω)
    controller.stuck_movement_threshold_px = behavior_params.get('stuck_movement_threshold_px', STUCK_MOVEMENT_THRESHOLD_PX)
    controller.stuck_detection_window_s    = behavior_params.get('stuck_detection_window_s',    STUCK_DETECTION_WINDOW_S)
    controller.stuck_boost_increment       = behavior_params.get('stuck_boost_increment',       STUCK_BOOST_INCREMENT)
    controller.stuck_boost_max             = behavior_params.get('stuck_boost_max',             STUCK_BOOST_MAX)


def _set_controller_creep_speed(controller, creep_pwm):
    """Limita velocidad lineal para fase 2 (creep/captura de balón).

    Usa max_linear_pwm_override del controlador: capea v DESPUÉS del perfil
    de velocidad pero ANTES de combinar con omega. Así:
    - Velocidad lineal <= creep_pwm (el robot va lento)
    - Corrección angular sigue usando robot_max_speed (el PID angular funciona)
    - El perfil de rampa sigue activo (desacelera al acercarse)
    """
    controller.max_linear_pwm_override = creep_pwm


def _restore_controller_speed(controller):
    """Restaura velocidad normal del controlador tras fase 2."""
    controller.max_linear_pwm_override = None


def _draw_behavior_panel(behavior_params, robot, waypoint, robot_available):
    """Dibuja el panel de control de comportamiento."""
    panel = np.zeros((700, 650, 3), dtype=np.uint8)

    y_offset = 30
    line_height = 25

    # Título
    cv2.putText(panel, "=== CALIBRACION DE COMPORTAMIENTO ===", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    y_offset += line_height * 2

    # Estado del robot
    if robot_available:
        status_color = (0, 255, 0)
        status_text = f"Robot detectado" if robot else "Robot NO visible"
    else:
        status_color = (0, 0, 255)
        status_text = "Robot NO conectado"

    cv2.putText(panel, status_text, (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1)
    y_offset += line_height * 2

    # Umbrales de precisión
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

    # Política de movimiento
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

    # Corrección angular
    cv2.putText(panel, "=== CORRECCION ANGULAR ===", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    y_offset += line_height

    cv2.putText(panel, f"Max correccion: {behavior_params['max_angular_correction_pwm']} PWM (,/.)", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y_offset += line_height
    cv2.putText(panel, "  -> Max diferencia L/R durante mov. lineal", (20, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
    y_offset += line_height
    cv2.putText(panel, "  -> Mayor valor = mas capacidad de corregir", (20, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
    y_offset += line_height * 2

    # Captura de balón
    cv2.putText(panel, "=== CAPTURA DE BALON ===", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    y_offset += line_height

    cv2.putText(panel, f"Activar dribbler: {behavior_params['capture_activate_px']}px (U/J)", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y_offset += line_height
    cv2.putText(panel, "  -> Distancia al balon para activar dribbler", (20, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
    y_offset += int(line_height * 1.2)

    cv2.putText(panel, f"Overshoot: {behavior_params['capture_overshoot_px']}px (I/K)", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y_offset += line_height
    cv2.putText(panel, "  -> Px mas alla del balon como objetivo PID", (20, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
    y_offset += int(line_height * 1.2)

    cv2.putText(panel, f"Confirmar captura: {behavior_params['capture_confirm_px']}px (O/L)", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y_offset += line_height
    cv2.putText(panel, "  -> Distancia al balon para confirmar captura", (20, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
    y_offset += int(line_height * 1.2)

    cv2.putText(panel, f"Creep speed: {behavior_params['creep_speed_pwm']} PWM (N/M)", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y_offset += line_height
    cv2.putText(panel, "  -> Velocidad de motores durante fase 2 (creep)", (20, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
    y_offset += int(line_height * 1.2)

    kp_off = behavior_params.get('kick_point_offset_px', KICK_POINT_OFFSET_PX)
    kp_tol = behavior_params.get('kick_point_tolerance_px', KICK_POINT_TOLERANCE_PX)
    cv2.putText(panel, f"Kick point offset: {kp_off}px (F/V)", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (40, 220, 220), 1)
    y_offset += line_height
    cv2.putText(panel, "  -> Centro marker -> punto impacto solenoide", (20, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
    y_offset += int(line_height * 1.2)
    cv2.putText(panel, f"Kick point tol:    {kp_tol}px (H/B)", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (40, 220, 220), 1)
    y_offset += line_height
    cv2.putText(panel, "  -> Tolerancia bola<->kick_point para disparar", (20, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
    y_offset += int(line_height * 1.2)

    factor = behavior_params.get('dribble_pwm_factor', 1.0)
    cv2.putText(panel, f"Dribble PWM factor: {factor:.2f}x (T/Y)", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y_offset += line_height

    cap_pwr = behavior_params.get('dribbler_capture_power', 25)
    hold_pwr = behavior_params.get('dribbler_hold_power', 115)
    pulse_on = behavior_params.get('dribbler_fw_on_ms', 65)
    pulse_off = behavior_params.get('dribbler_fw_off_ms', 15)
    pulse_mode = "CONTINUO" if pulse_off == 0 else f"ON {pulse_on}ms / OFF {pulse_off}ms"
    cv2.putText(panel, f"Dribbler capture: {cap_pwr} PWM (1/2)", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y_offset += line_height
    cv2.putText(panel, f"Dribbler hold:    {hold_pwr} PWM (3/4)", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y_offset += line_height
    cv2.putText(panel, f"Pulso: {pulse_mode}  (5/6: ON  7/8: OFF)", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 100), 1)
    y_offset += line_height * 2

    # Información del waypoint
    if waypoint:
        cv2.putText(panel, f"Waypoint: ({waypoint[0]}, {waypoint[1]})", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    else:
        cv2.putText(panel, "Sin waypoint (usar flechas)", (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
    y_offset += line_height * 2

    # Controles
    cv2.putText(panel, "=== CONTROLES ===", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
    y_offset += line_height

    controls = [
        "Flechas: Mover waypoint",
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


def _handle_keyboard_behavior(key, robot, target_waypoint, behavior_params,
                               movement_active, last_space_time, controller):
    """Maneja eventos de teclado para calibración de comportamiento."""
    result = {'action': 'none'}

    # Teclas especiales
    if key == 27:  # ESC
        log.info("ESC presionado - Saliendo...")
        result['action'] = 'quit'
        return result

    elif key == ord('\r') or key == ord('\n'):  # ENTER
        result['action'] = 'save'
        return result

    elif key == ord(' '):  # ESPACIO - Toggle movimiento
        current_time = time.time()
        if current_time - last_space_time > 0.3:  # Debounce 300ms
            movement_active = not movement_active
            status = "INICIADO" if movement_active else "PAUSADO"
            log.info(f"{'▶️' if movement_active else '⏸️'} Movimiento {status}")
            result.update({
                'action': 'toggle_movement',
                'movement_active': movement_active,
                'last_space_time': current_time
            })
        return result

    elif key == ord('x') or key == ord('X'):  # Cancelar waypoint
        if target_waypoint:
            log.info("❌ Waypoint cancelado")
            result['action'] = 'cancel_waypoint'
        return result

    # Movimiento del waypoint con flechas
    elif key in [82, 0] and robot:  # Arriba
        if target_waypoint is None:
            target_waypoint = [int(robot.x), int(robot.y)]
            log.info("🎯 Waypoint creado: (%d, %d)", target_waypoint[0], target_waypoint[1])
        target_waypoint[1] -= 10
        log.info("⬆️  Waypoint → (%d, %d)", target_waypoint[0], target_waypoint[1])
        result['waypoint'] = target_waypoint
    elif key in [84, 1] and robot:  # Abajo
        if target_waypoint is None:
            target_waypoint = [int(robot.x), int(robot.y)]
            log.info("🎯 Waypoint creado: (%d, %d)", target_waypoint[0], target_waypoint[1])
        target_waypoint[1] += 10
        log.info("⬇️  Waypoint → (%d, %d)", target_waypoint[0], target_waypoint[1])
        result['waypoint'] = target_waypoint
    elif key in [81, 2] and robot:  # Izquierda
        if target_waypoint is None:
            target_waypoint = [int(robot.x), int(robot.y)]
            log.info("🎯 Waypoint creado: (%d, %d)", target_waypoint[0], target_waypoint[1])
        target_waypoint[0] -= 10
        log.info("⬅️  Waypoint → (%d, %d)", target_waypoint[0], target_waypoint[1])
        result['waypoint'] = target_waypoint
    elif key in [83, 3] and robot:  # Derecha
        if target_waypoint is None:
            target_waypoint = [int(robot.x), int(robot.y)]
            log.info("🎯 Waypoint creado: (%d, %d)", target_waypoint[0], target_waypoint[1])
        target_waypoint[0] += 10
        log.info("➡️  Waypoint → (%d, %d)", target_waypoint[0], target_waypoint[1])
        result['waypoint'] = target_waypoint

    # Threshold de posición (±1 px)
    elif key == ord('9'):
        behavior_params['position_threshold'] = min(50, behavior_params['position_threshold'] + 1)
        result['action'] = 'update_behavior'
        log.info(f"position_threshold: {behavior_params['position_threshold']}px")
    elif key == ord('0'):
        behavior_params['position_threshold'] = max(5, behavior_params['position_threshold'] - 1)
        result['action'] = 'update_behavior'
        log.info(f"position_threshold: {behavior_params['position_threshold']}px")

    # Threshold angular (±1 deg)
    elif key == ord('-'):
        behavior_params['angle_threshold'] = max(1, behavior_params['angle_threshold'] - 1)
        result['action'] = 'update_behavior'
        log.info(f"angle_threshold: {behavior_params['angle_threshold']}°")
    elif key == ord('='):
        behavior_params['angle_threshold'] = min(30, behavior_params['angle_threshold'] + 1)
        result['action'] = 'update_behavior'
        log.info(f"angle_threshold: {behavior_params['angle_threshold']}°")

    # Threshold de inicio lineal (±1 deg)
    elif key == ord('['):
        behavior_params['linear_start_angle_threshold'] = max(5, behavior_params['linear_start_angle_threshold'] - 1)
        result['action'] = 'update_behavior'
        log.info(f"linear_start_angle_threshold: {behavior_params['linear_start_angle_threshold']}°")
    elif key == ord(']'):
        behavior_params['linear_start_angle_threshold'] = min(90, behavior_params['linear_start_angle_threshold'] + 1)
        result['action'] = 'update_behavior'
        log.info(f"linear_start_angle_threshold: {behavior_params['linear_start_angle_threshold']}°")

    # Corrección angular máxima (±1 PWM)
    elif key == ord(','):
        behavior_params['max_angular_correction_pwm'] = max(0, behavior_params['max_angular_correction_pwm'] - 1)
        result['action'] = 'update_behavior'
        log.info(f"max_angular_correction_pwm: {behavior_params['max_angular_correction_pwm']} PWM")
    elif key == ord('.'):
        behavior_params['max_angular_correction_pwm'] = min(50, behavior_params['max_angular_correction_pwm'] + 1)
        result['action'] = 'update_behavior'
        log.info(f"max_angular_correction_pwm: {behavior_params['max_angular_correction_pwm']} PWM")

    # Captura: distancia de activación (U/J ±1 px)
    elif key == ord('u') or key == ord('U'):
        behavior_params['capture_activate_px'] = min(100, behavior_params['capture_activate_px'] + 1)
        result['action'] = 'update_behavior'
        log.info(f"capture_activate_px: {behavior_params['capture_activate_px']}px")
    elif key == ord('j') or key == ord('J'):
        behavior_params['capture_activate_px'] = max(10, behavior_params['capture_activate_px'] - 1)
        result['action'] = 'update_behavior'
        log.info(f"capture_activate_px: {behavior_params['capture_activate_px']}px")

    # Captura: overshoot (I/K ±1 px)
    elif key == ord('i') or key == ord('I'):
        behavior_params['capture_overshoot_px'] = min(50, behavior_params['capture_overshoot_px'] + 1)
        result['action'] = 'update_behavior'
        log.info(f"capture_overshoot_px: {behavior_params['capture_overshoot_px']}px")
    elif key == ord('k') or key == ord('K'):
        behavior_params['capture_overshoot_px'] = max(0, behavior_params['capture_overshoot_px'] - 1)
        result['action'] = 'update_behavior'
        log.info(f"capture_overshoot_px: {behavior_params['capture_overshoot_px']}px")

    # Captura: distancia de confirmación (O/L ±1 px)
    elif key == ord('o') or key == ord('O'):
        behavior_params['capture_confirm_px'] = min(50, behavior_params['capture_confirm_px'] + 1)
        result['action'] = 'update_behavior'
        log.info(f"capture_confirm_px: {behavior_params['capture_confirm_px']}px")
    elif key == ord('l') or key == ord('L'):
        behavior_params['capture_confirm_px'] = max(5, behavior_params['capture_confirm_px'] - 1)
        result['action'] = 'update_behavior'
        log.info(f"capture_confirm_px: {behavior_params['capture_confirm_px']}px")

    # Captura: velocidad de creep (N/M ±1 PWM)
    elif key == ord('n') or key == ord('N'):
        behavior_params['creep_speed_pwm'] = max(10, behavior_params['creep_speed_pwm'] - 1)
        result['action'] = 'update_behavior'
        log.info(f"creep_speed_pwm: {behavior_params['creep_speed_pwm']} PWM")
    elif key == ord('m') or key == ord('M'):
        behavior_params['creep_speed_pwm'] = min(60, behavior_params['creep_speed_pwm'] + 1)
        result['action'] = 'update_behavior'
        log.info(f"creep_speed_pwm: {behavior_params['creep_speed_pwm']} PWM")

    # Kick_point: offset desde marker ArUco hasta punto de impacto del solenoide (F/V ±1 px)
    elif key == ord('f') or key == ord('F'):
        behavior_params['kick_point_offset_px'] = min(80, behavior_params['kick_point_offset_px'] + 1)
        result['action'] = 'update_behavior'
        log.info(f"kick_point_offset_px: {behavior_params['kick_point_offset_px']}px")
    elif key == ord('v') or key == ord('V'):
        behavior_params['kick_point_offset_px'] = max(0, behavior_params['kick_point_offset_px'] - 1)
        result['action'] = 'update_behavior'
        log.info(f"kick_point_offset_px: {behavior_params['kick_point_offset_px']}px")

    # Kick_point: tolerancia bola↔kick_point para confirmar contacto (H/B ±1 px)
    elif key == ord('h') or key == ord('H'):
        behavior_params['kick_point_tolerance_px'] = min(40, behavior_params['kick_point_tolerance_px'] + 1)
        result['action'] = 'update_behavior'
        log.info(f"kick_point_tolerance_px: {behavior_params['kick_point_tolerance_px']}px")
    elif key == ord('b') or key == ord('B'):
        behavior_params['kick_point_tolerance_px'] = max(1, behavior_params['kick_point_tolerance_px'] - 1)
        result['action'] = 'update_behavior'
        log.info(f"kick_point_tolerance_px: {behavior_params['kick_point_tolerance_px']}px")

    # Dribble PWM factor (T/Y ±0.05)
    elif key == ord('t') or key == ord('T'):
        behavior_params['dribble_pwm_factor'] = round(max(0.5, behavior_params['dribble_pwm_factor'] - 0.05), 2)
        result['action'] = 'update_behavior'
        log.info(f"dribble_pwm_factor: {behavior_params['dribble_pwm_factor']}")
    elif key == ord('y') or key == ord('Y'):
        behavior_params['dribble_pwm_factor'] = round(min(2.0, behavior_params['dribble_pwm_factor'] + 0.05), 2)
        result['action'] = 'update_behavior'
        log.info(f"dribble_pwm_factor: {behavior_params['dribble_pwm_factor']}")

    # Dribbler capture power (1/2 ±1 PWM)
    elif key == ord('1'):
        behavior_params['dribbler_capture_power'] = max(0, behavior_params['dribbler_capture_power'] - 1)
        result['action'] = 'update_behavior'
        log.info(f"dribbler_capture_power: {behavior_params['dribbler_capture_power']} PWM")
    elif key == ord('2'):
        behavior_params['dribbler_capture_power'] = min(255, behavior_params['dribbler_capture_power'] + 1)
        result['action'] = 'update_behavior'
        log.info(f"dribbler_capture_power: {behavior_params['dribbler_capture_power']} PWM")

    # Dribbler hold power (3/4 ±1 PWM)
    elif key == ord('3'):
        behavior_params['dribbler_hold_power'] = max(0, behavior_params['dribbler_hold_power'] - 1)
        result['action'] = 'update_behavior'
        log.info(f"dribbler_hold_power: {behavior_params['dribbler_hold_power']} PWM")
    elif key == ord('4'):
        behavior_params['dribbler_hold_power'] = min(255, behavior_params['dribbler_hold_power'] + 1)
        result['action'] = 'update_behavior'
        log.info(f"dribbler_hold_power: {behavior_params['dribbler_hold_power']} PWM")

    # Dribbler firmware oscilación ON (5/6 ±5ms; byte 0-255, persiste vía comando 'C')
    elif key == ord('5'):
        behavior_params['dribbler_fw_on_ms'] = max(10, behavior_params['dribbler_fw_on_ms'] - 5)
        result['action'] = 'update_behavior'
        log.info(f"dribbler_fw_on_ms: {behavior_params['dribbler_fw_on_ms']}ms")
    elif key == ord('6'):
        behavior_params['dribbler_fw_on_ms'] = min(255, behavior_params['dribbler_fw_on_ms'] + 5)
        result['action'] = 'update_behavior'
        log.info(f"dribbler_fw_on_ms: {behavior_params['dribbler_fw_on_ms']}ms")

    # Dribbler firmware oscilación OFF (7/8 ±5ms, 0=continuo; byte 0-255)
    elif key == ord('7'):
        behavior_params['dribbler_fw_off_ms'] = max(0, behavior_params['dribbler_fw_off_ms'] - 5)
        result['action'] = 'update_behavior'
        log.info(f"dribbler_fw_off_ms: {behavior_params['dribbler_fw_off_ms']}ms (0=continuo)")
    elif key == ord('8'):
        behavior_params['dribbler_fw_off_ms'] = min(255, behavior_params['dribbler_fw_off_ms'] + 5)
        result['action'] = 'update_behavior'
        log.info(f"dribbler_fw_off_ms: {behavior_params['dribbler_fw_off_ms']}ms (0=continuo)")

    return result


def _save_behavior_to_config(behavior_params):
    """Guarda parámetros de comportamiento a config.py."""
    config_path = ROOT_DIR / "src" / "robot_soccer" / "config.py"

    log.info("💾 Guardando parámetros de comportamiento a config.py...")

    with open(config_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    replacements = {
        'ROBOT_POSITION_THRESHOLD': f"{behavior_params['position_threshold']}",
        'ROBOT_ANGLE_THRESHOLD_DEG': f"{behavior_params['angle_threshold']}",
        'ROBOT_LINEAR_START_ANGLE_THRESHOLD_DEG': f"{behavior_params['linear_start_angle_threshold']:.1f}",
        'MAX_ANGULAR_CORRECTION_PWM': f"{behavior_params['max_angular_correction_pwm']}",
        'CAPTURE_ACTIVATE_DISTANCE_PX': f"{behavior_params['capture_activate_px']}",
        'CAPTURE_OVERSHOOT_PX': f"{behavior_params['capture_overshoot_px']}",
        'CAPTURE_CONFIRM_DISTANCE_PX': f"{behavior_params['capture_confirm_px']}",
        'CAPTURE_CREEP_SPEED_PWM': f"{behavior_params['creep_speed_pwm']}",
        'KICK_POINT_OFFSET_PX':        f"{int(behavior_params.get('kick_point_offset_px', KICK_POINT_OFFSET_PX))}",
        'KICK_POINT_TOLERANCE_PX':     f"{int(behavior_params.get('kick_point_tolerance_px', KICK_POINT_TOLERANCE_PX))}",
        'KICK_POINT_ANGLE_OFFSET_DEG': f"{behavior_params.get('kick_point_angle_offset_deg', KICK_POINT_ANGLE_OFFSET_DEG):.1f}",
        'DRIBBLE_PWM_FACTOR': f"{behavior_params.get('dribble_pwm_factor', 1.0)}",
        'DRIBBLER_CAPTURE_POWER': f"{behavior_params.get('dribbler_capture_power', 25)}",
        'DRIBBLER_HOLD_POWER': f"{behavior_params.get('dribbler_hold_power', 115)}",
        'DRIBBLER_FW_ON_MS': f"{behavior_params.get('dribbler_fw_on_ms', 65)}",
        'DRIBBLER_FW_OFF_MS': f"{behavior_params.get('dribbler_fw_off_ms', 15)}",
        'BEHIND_BALL_APPROACH_PX': f"{behavior_params.get('behind_ball_approach_px', BEHIND_BALL_APPROACH_PX)}",
        'CONTACT_SETTLE_TIME_S': f"{behavior_params.get('contact_settle_time_s', CONTACT_SETTLE_TIME_S)}",
        'STUCK_MOVEMENT_THRESHOLD_PX': f"{int(behavior_params.get('stuck_movement_threshold_px', STUCK_MOVEMENT_THRESHOLD_PX))}",
        'STUCK_DETECTION_WINDOW_S':    f"{behavior_params.get('stuck_detection_window_s', STUCK_DETECTION_WINDOW_S):.1f}",
        'STUCK_BOOST_INCREMENT':       f"{int(behavior_params.get('stuck_boost_increment', STUCK_BOOST_INCREMENT))}",
        'STUCK_BOOST_MAX':             f"{int(behavior_params.get('stuck_boost_max', STUCK_BOOST_MAX))}",
        'STUCK_AUTO_KICK':             f"{behavior_params.get('stuck_auto_kick', STUCK_AUTO_KICK)}",
    }

    new_lines = []
    for line in lines:
        modified = False
        for param_name, new_value in replacements.items():
            if line.strip().startswith(param_name + ' ='):
                if '#' in line:
                    comment = line[line.index('#'):]
                else:
                    comment = '\n'
                new_line = f"{param_name} = {new_value}  {comment}"
                new_lines.append(new_line)
                modified = True
                log.info(f"  {param_name} = {new_value}")
                break
        if not modified:
            new_lines.append(line)

    with open(config_path, 'w', encoding='utf-8') as f:
        f.writelines(new_lines)

    log.info("✅ Parámetros de comportamiento guardados")


def control_loop_behavior_pure(perception_pipe, control_state_pipe, keyboard_pipe, robot_id, serial_port):
    """Proceso de control behavior PURO (Arquitectura de 3 procesos).

    Este proceso maneja SOLO el control de movimiento:
    - Recibe datos de posición desde percepción (pipe)
    - Recibe comandos desde visualización (pipe: waypoint, teclado)
    - Ejecuta control PID para alcanzar waypoints
    - Envía comandos RF al robot
    - Envía estado a visualización (pipe)
    - NO hace visualización (sin cv2.imshow, sin waitKey)

    Args:
        perception_pipe: Pipe para recibir posiciones desde percepción
        control_state_pipe: Pipe para enviar estado a visualización
        keyboard_pipe: Pipe para recibir comandos desde visualización (teclado/mouse)
        robot_id: ID del robot a controlar
        serial_port: Puerto serial para comunicación RF
    """
    log.info(f"🎮 Proceso de control behavior puro iniciado para Robot ID {robot_id}")

    rf_controller = None
    robot_available = False
    try:
        log.info("🔌 Iniciando comunicación RF...")
        rf_controller = RFController(port=serial_port, enable_calibration=True)
        if rf_controller.initialize():
            log.info("✅ Conexión Serial establecida")
            connections = rf_controller.test_connections()
            robot_key = f'robot_{robot_id + 1}'
            robot_available = connections.get(robot_key, False)
            if robot_available:
                log.info(f"✅ Robot {robot_id} disponible via RF")
            else:
                log.warning(f"⚠️  Robot {robot_id} NO responde via RF")
        else:
            log.warning("⚠️  No se pudo conectar al transmisor")
    except Exception as e:
        log.warning(f"⚠️  Error RF: {e}")

    behavior_params = {
        'position_threshold': ROBOT_POSITION_THRESHOLD,
        'angle_threshold': ROBOT_ANGLE_THRESHOLD_DEG,
        'linear_start_angle_threshold': ROBOT_LINEAR_START_ANGLE_THRESHOLD_DEG,
        'max_angular_correction_pwm': MAX_ANGULAR_CORRECTION_PWM,
        'capture_activate_px': CAPTURE_ACTIVATE_DISTANCE_PX,
        'capture_overshoot_px': CAPTURE_OVERSHOOT_PX,
        'capture_confirm_px': CAPTURE_CONFIRM_DISTANCE_PX,
        'creep_speed_pwm': CAPTURE_CREEP_SPEED_PWM,
        'kick_point_offset_px': KICK_POINT_OFFSET_PX,
        'kick_point_tolerance_px': KICK_POINT_TOLERANCE_PX,
        'kick_point_angle_offset_deg': KICK_POINT_ANGLE_OFFSET_DEG,
        'dribble_pwm_factor': DRIBBLE_PWM_FACTOR,
        'dribbler_capture_power': DRIBBLER_CAPTURE_POWER,
        'dribbler_hold_power': DRIBBLER_HOLD_POWER,
        'dribbler_fw_on_ms': DRIBBLER_FW_ON_MS,
        'dribbler_fw_off_ms': DRIBBLER_FW_OFF_MS,
        'behind_ball_approach_px': BEHIND_BALL_APPROACH_PX,
        'contact_settle_time_s': CONTACT_SETTLE_TIME_S,
        'stuck_movement_threshold_px': STUCK_MOVEMENT_THRESHOLD_PX,
        'stuck_detection_window_s':    STUCK_DETECTION_WINDOW_S,
        'stuck_boost_increment':       STUCK_BOOST_INCREMENT,
        'stuck_boost_max':             STUCK_BOOST_MAX,
        'stuck_auto_kick':             STUCK_AUTO_KICK,
    }

    controller = DifferentialDriveController(rf_controller=rf_controller)
    controller.position_threshold = behavior_params['position_threshold']
    controller.angle_threshold = math.radians(behavior_params['angle_threshold'])
    controller.dribble_pwm_factor = behavior_params['dribble_pwm_factor']

    robot = None
    target_waypoint = None
    movement_active = False
    running = True
    robot_stopped = False

    # --- Estado de captura ---
    # 'idle':              sin movimiento
    # 'approach':          fase 1a — PID de posición hacia DETRÁS de la pelota
    # 'approach_align':    fase 1b — rotación en sitio para apuntar a la pelota
    #                       (ángulo calculado desde posición real del robot al terminar 1a)
    # 'capture_creeping':  fase 2a — creep lento directo hacia la pelota
    # 'capture_settling':  fase 2b — contacto confirmado, esperando asentamiento
    # 'confirmed':         contacto y asentamiento OK
    capture_phase = 'idle'
    ball_waypoint = None       # posición del waypoint (simula la pelota)
    behind_ball_target = None  # posición calculada detrás de la pelota (fase 1a)
    behind_ball_target_angle_deg = None  # ángulo calculado al terminar 1a (robot → pelota real)
    overshoot_target = None    # compatibilidad con visualización existente
    contact_start_time = None  # timestamp cuando se confirmó el contacto
    settle_elapsed = 0.0       # segundos transcurridos en fase settling
    dist_to_ball_now = 0.0     # distancia robot-pelota actual
    dribbler_on = False        # siempre False (sin dribbler), para compatibilidad
    firmware_id = robot_id + 1

    last_state_send_time = 0
    STATE_SEND_INTERVAL = 0.04  # ~25 Hz
    last_cmd_type = 'idle'       # Último tipo de comando enviado a motores
    last_creep_log_time = 0.0    # Para logs periódicos durante Fase 2

    log.info("✅ Control iniciado - Esperando datos de percepción...")

    try:
        while running:
            current_time = time.time()

            if perception_pipe.poll():
                try:
                    data = perception_pipe.recv()
                    robot_detected = data.get('robot_detected', False)
                    robot_data = data.get('robot_data', None)

                    if robot_detected and robot_data:
                        angle_rad = math.radians(robot_data['angulo'])
                        if robot is None:
                            robot = RobotEntity(robot_id, robot_data['x'], robot_data['y'], angle_rad)
                            log.info(f"Robot {robot_id} detectado en ({robot_data['x']:.0f}, {robot_data['y']:.0f})")
                        else:
                            robot.update(robot_data['x'], robot_data['y'], angle_rad)
                    else:
                        if robot is not None:
                            log.warning(f"Robot {robot_id} perdido")
                        robot = None

                except Exception as e:
                    log.error(f"Error recibiendo posiciones: {e}")

            if keyboard_pipe.poll():
                try:
                    cmd = keyboard_pipe.recv()
                    command = cmd.get('command', '')

                    if command == 'exit':
                        running = False
                    elif command == 'toggle_movement':
                        if not movement_active and target_waypoint:
                            # FASE 1: navegar a posición DETRÁS de la pelota (sin PID al waypoint directo)
                            # El waypoint representa la pelota; calculamos behind_pos desde el lado opuesto al arco
                            bx, by = float(target_waypoint[0]), float(target_waypoint[1])
                            # El arco rival para equipo rojo está a la derecha
                            goal_x = float(CAMERA_PERSPECTIVE_WIDTH)
                            goal_y = float(CAMERA_PERSPECTIVE_HEIGHT) / 2.0
                            gtb_x, gtb_y = bx - goal_x, by - goal_y
                            dist_gtb = math.sqrt(gtb_x**2 + gtb_y**2)
                            if dist_gtb > 1.0:
                                unit_x, unit_y = gtb_x / dist_gtb, gtb_y / dist_gtb
                                approach = behavior_params['behind_ball_approach_px']
                                bhx = int(bx + unit_x * approach)
                                bhy = int(by + unit_y * approach)
                                bhx = max(15, min(CAMERA_PERSPECTIVE_WIDTH - 15, bhx))
                                bhy = max(15, min(CAMERA_PERSPECTIVE_HEIGHT - 15, bhy))
                                behind_ball_target = [bhx, bhy]
                                target_waypoint = [bhx, bhy]
                            else:
                                bhx, bhy = int(bx), int(by)
                                behind_ball_target = list(target_waypoint)
                            # El ángulo se calculará FRESCO cuando se alcance la posición
                            behind_ball_target_angle_deg = None
                            ball_waypoint = [int(bx), int(by)]
                            overshoot_target = None
                            movement_active = True
                            capture_phase = 'approach'
                            controller.position_threshold = behavior_params['position_threshold']
                            robot_stopped = False
                            log.info("▶️  FASE 1a: Yendo DETRÁS de la pelota → (%d,%d) | approach=%dpx",
                                     target_waypoint[0], target_waypoint[1],
                                     behavior_params['behind_ball_approach_px'])
                        else:
                            # Pausar
                            movement_active = False
                            robot_stopped = False
                            if capture_phase == 'capture_creeping':
                                _restore_controller_speed(controller)
                                if rf_controller:
                                    rf_controller.set_motors(firmware_id, 0, 0)
                            capture_phase = 'idle'
                            overshoot_target = None
                            behind_ball_target_angle_deg = None
                            controller.position_threshold = behavior_params['position_threshold']
                            log.info("⏸️  Movimiento PAUSADO")
                    elif command == 'start_capture':
                        # FASE 2: PID con velocidad limitada hacia overshoot target
                        # Solo permite si Fase 1b (alineación) completó
                        if (not movement_active and ball_waypoint and robot
                                and capture_phase == 'approach_align'):
                            # Calcular overshoot target (más allá de la pelota hacia el arco)
                            goal_center = FIELD_CAM.goal_right_center
                            dx_bg = goal_center[0] - ball_waypoint[0]
                            dy_bg = goal_center[1] - ball_waypoint[1]
                            dist_bg = math.sqrt(dx_bg**2 + dy_bg**2)
                            if dist_bg > 0:
                                ov_x = ball_waypoint[0] + (dx_bg / dist_bg) * behavior_params['capture_overshoot_px']
                                ov_y = ball_waypoint[1] + (dy_bg / dist_bg) * behavior_params['capture_overshoot_px']
                            else:
                                ov_x, ov_y = float(ball_waypoint[0]), float(ball_waypoint[1])
                            overshoot_target = [int(ov_x), int(ov_y)]

                            # Activar PID con velocidad limitada
                            _set_controller_creep_speed(controller, behavior_params['creep_speed_pwm'])
                            capture_phase = 'capture_creeping'
                            contact_start_time = None
                            robot_stopped = False
                            dist_to_ball_now = math.sqrt(
                                (ball_waypoint[0] - robot.x)**2 + (ball_waypoint[1] - robot.y)**2
                            )
                            log.info("▶️  FASE 2: PID creep | speed=%d PWM | dist=%.0fpx | overshoot=(%d,%d)",
                                     behavior_params['creep_speed_pwm'], dist_to_ball_now,
                                     overshoot_target[0], overshoot_target[1])
                        elif capture_phase in ('approach', 'idle') and not movement_active:
                            log.warning("⚠️  Completa fase 1 primero (ESPACIO): posición + alineación")
                        elif capture_phase == 'approach' and movement_active:
                            log.info("Fase 1a en curso (acercamiento) — espera a que complete")
                        elif capture_phase == 'approach_align' and movement_active:
                            log.info("Fase 1b en curso (alineando) — espera a que complete")
                        elif capture_phase == 'capture_creeping':
                            log.info("Ya en fase 2 (creeping)")
                        elif capture_phase in ('capture_settling', 'confirmed'):
                            log.info("Ya hay contacto — presiona X para reiniciar")
                        else:
                            log.warning("Estado inesperado: %s | movement=%s", capture_phase, movement_active)
                    elif command == 'start_rotate':
                        # FASE 3: Disparar solenoide (kick) tras confirmar posesión
                        if capture_phase == 'confirmed':
                            if rf_controller:
                                rf_controller.kick(firmware_id)
                                log.info("💥 FASE 3: KICK disparado (robot %d, firmware_id=%d)",
                                         robot_id, firmware_id)
                            else:
                                log.info("💥 FASE 3: KICK (simulación — sin RF)")
                            capture_phase = 'idle'
                            movement_active = False
                            robot_stopped = True   # evitar stop de alta prioridad en esta iteración
                            contact_start_time = None
                            overshoot_target = None
                            _restore_controller_speed(controller)
                        elif capture_phase in ('capture_creeping', 'capture_settling'):
                            log.warning("⚠️  Espera a CONFIRMADO antes de disparar (fase actual: %s)", capture_phase)
                        else:
                            log.warning("⚠️  Completa fase 1+2 primero. Estado actual: %s", capture_phase)
                    elif command == 'cancel_waypoint':
                        target_waypoint = None
                        ball_waypoint = None
                        overshoot_target = None
                        behind_ball_target = None
                        behind_ball_target_angle_deg = None
                        movement_active = False
                        robot_stopped = False
                        if capture_phase in ('capture_creeping',):
                            _restore_controller_speed(controller)
                            if rf_controller:
                                rf_controller.set_motors(firmware_id, 0, 0)
                        capture_phase = 'idle'
                        contact_start_time = None
                        controller.position_threshold = behavior_params['position_threshold']
                        log.info("❌ Waypoint cancelado — reiniciando")
                    elif command == 'set_waypoint':
                        waypoint = cmd.get('waypoint')
                        if waypoint:
                            target_waypoint = list(waypoint)
                            ball_waypoint = list(waypoint)
                            overshoot_target = None
                            behind_ball_target = None
                            behind_ball_target_angle_deg = None
                            if capture_phase in ('capture_creeping',) and rf_controller:
                                rf_controller.set_motors(firmware_id, 0, 0)
                            capture_phase = 'idle'
                            contact_start_time = None
                            log.info(f"🎯 Waypoint (pelota) establecido: ({target_waypoint[0]}, {target_waypoint[1]})")
                    elif command == 'move_waypoint':
                        delta = cmd.get('delta', [0, 0])
                        if target_waypoint is None and robot:
                            target_waypoint = [int(robot.x), int(robot.y)]
                        if target_waypoint:
                            target_waypoint[0] += delta[0]
                            target_waypoint[1] += delta[1]
                            ball_waypoint = list(target_waypoint)
                            log.info(f"⬆️⬇️⬅️➡️ Waypoint → ({target_waypoint[0]}, {target_waypoint[1]})")
                    elif command == 'adjust_threshold':
                        param = cmd.get('param')
                        delta = cmd.get('delta', 0)
                        if param in behavior_params:
                            _param_bounds = {
                                'position_threshold': (5, 50),
                                'angle_threshold': (1, 30),
                                'linear_start_angle_threshold': (5, 90),
                                'max_angular_correction_pwm': (0, 50),
                                'capture_activate_px': (10, 100),
                                'capture_overshoot_px': (0, 50),
                                'capture_confirm_px': (5, 50),
                                'creep_speed_pwm': (10, 80),
                                'kick_point_offset_px':        (0, 80),
                                'kick_point_tolerance_px':     (1, 40),
                                'kick_point_angle_offset_deg': (-30, 30),
                                'dribble_pwm_factor': (0.5, 2.0),
                                'dribbler_capture_power': (0, 255),
                                'dribbler_hold_power': (0, 255),
                                'dribbler_fw_on_ms': (10, 255),
                                'dribbler_fw_off_ms': (0, 255),
                                'behind_ball_approach_px': (30, 120),
                                'contact_settle_time_s': (0.05, 2.0),
                                'stuck_movement_threshold_px': (2, 30),
                                'stuck_detection_window_s':    (0.2, 3.0),
                                'stuck_boost_increment':       (1, 15),
                                'stuck_boost_max':             (5, 60),
                            }
                            lo, hi = _param_bounds.get(param, (0, 9999))
                            behavior_params[param] = max(lo, min(hi, round(behavior_params[param] + delta, 2)))
                            controller.position_threshold = behavior_params['position_threshold']
                            controller.angle_threshold = math.radians(behavior_params['angle_threshold'])
                            controller.dribble_pwm_factor = behavior_params.get('dribble_pwm_factor', 1.0)
                            if param in ('stuck_movement_threshold_px', 'stuck_detection_window_s',
                                         'stuck_boost_increment', 'stuck_boost_max'):
                                _update_behavior_controller(controller, behavior_params)
                            log.info(f"  {param}: {behavior_params[param]}")
                    elif command == 'save_params':
                        _save_behavior_to_config(behavior_params)
                    elif command == 'update_behavior':
                        controller.position_threshold = behavior_params['position_threshold']
                        controller.angle_threshold = math.radians(behavior_params['angle_threshold'])

                except Exception as e:
                    log.error(f"Error recibiendo comando: {e}")

            # --- Fase 2a: PID con velocidad limitada hacia overshoot target ---
            if capture_phase == 'capture_creeping' and robot and overshoot_target:
                bx = ball_waypoint[0] - robot.x
                by = ball_waypoint[1] - robot.y
                dist_to_ball_now = math.sqrt(bx * bx + by * by)
                confirm_px = behavior_params['capture_confirm_px']
                if dist_to_ball_now < confirm_px:
                    # Contacto confirmado: detener PID, empezar asentamiento
                    _restore_controller_speed(controller)
                    if rf_controller:
                        rf_controller.set_motors(firmware_id, 0, 0)
                    last_cmd_type = 'stop'
                    capture_phase = 'capture_settling'
                    contact_start_time = current_time
                    settle_elapsed = 0.0
                    log.info("🤝 CONTACTO confirmado | dist=%.1fpx | Asentando %.2fs...",
                             dist_to_ball_now, behavior_params['contact_settle_time_s'])
                else:
                    # PID con velocidad limitada: corrección angular activa
                    controller.move_to_position(robot, tuple(overshoot_target))
                    last_cmd_type = f'pid_creep_{behavior_params["creep_speed_pwm"]}'
                    # Log periódico cada 500ms para monitorear alineación durante PID creep
                    if current_time - last_creep_log_time >= 0.5:
                        robot_deg = math.degrees(robot.angle)
                        ang_to_ball = math.degrees(math.atan2(by, bx))
                        err_creep = ((ang_to_ball - robot_deg + 180) % 360) - 180
                        log.info("🏃 FASE 2 PID | dist=%.0fpx | Robot=%.1f° | →Pelota=%.1f° | err=%+.1f° | overshoot=(%d,%d)",
                                 dist_to_ball_now, robot_deg, ang_to_ball, err_creep,
                                 overshoot_target[0], overshoot_target[1])
                        last_creep_log_time = current_time
                    # Auto-kick disparado por el controlador cuando stuck_boost llegó al máximo
                    if behavior_params.get('stuck_auto_kick', STUCK_AUTO_KICK):
                        _kicked = controller._pid_state.get(robot_id, {}).pop(
                            'stuck_auto_kicked', False
                        )
                        if _kicked:
                            robot_status_logger.emit_event(
                                robot_id, "AUTO-KICK: captura → idle"
                            )
                            capture_phase = 'idle'
                            movement_active = False
                            robot_stopped = True   # evitar stop de alta prioridad en esta iteración
                            contact_start_time = None
                            overshoot_target = None
                            _restore_controller_speed(controller)

            # --- Fase 2b: Asentamiento (robot quieto, esperando estabilización) ---
            elif capture_phase == 'capture_settling' and robot:
                bx = ball_waypoint[0] - robot.x
                by = ball_waypoint[1] - robot.y
                dist_to_ball_now = math.sqrt(bx * bx + by * by)
                settle_elapsed = current_time - contact_start_time
                confirm_px = behavior_params['capture_confirm_px']

                if dist_to_ball_now > confirm_px * 2:
                    # Pelota escapó durante asentamiento
                    capture_phase = 'idle'
                    contact_start_time = None
                    log.info("⚠️  Pelota escapó en asentamiento (dist=%.0fpx) → idle. Reinicia con X.",
                             dist_to_ball_now)
                elif settle_elapsed >= behavior_params['contact_settle_time_s']:
                    capture_phase = 'confirmed'
                    log.info("✅ ASENTAMIENTO OK (%.2fs) → CONFIRMADO | dist_final=%.1fpx",
                             settle_elapsed, dist_to_ball_now)

            # --- Fase 1a: PID de posición hacia detrás de la pelota ---
            if movement_active and robot and target_waypoint and capture_phase == 'approach':
                last_cmd_type = 'pid_mover'
                reached = controller.move_to_position(robot, tuple(target_waypoint))
                if reached:
                    # Calcular ángulo FRESCO desde posición real del robot a pelota actual
                    dx_a = ball_waypoint[0] - robot.x
                    dy_a = ball_waypoint[1] - robot.y
                    behind_ball_target_angle_deg = math.degrees(math.atan2(dy_a, dx_a))
                    capture_phase = 'approach_align'
                    robot_stopped = False
                    log.info("📍 FASE 1b: Posición OK | Alineando hacia pelota → %.1f° "
                             "(robot en %.1f°)",
                             behind_ball_target_angle_deg, math.degrees(robot.angle))

            # --- Fase 1b: Rotación en sitio para apuntar hacia la pelota ---
            elif movement_active and robot and capture_phase == 'approach_align':
                last_cmd_type = 'pid_girar'
                aligned = controller.rotate_to_angle(robot, behind_ball_target_angle_deg)
                if aligned:
                    dist_bh = math.sqrt(
                        (ball_waypoint[0] - robot.x)**2 + (ball_waypoint[1] - robot.y)**2
                    )
                    robot_deg = math.degrees(robot.angle)
                    bx_r = ball_waypoint[0] - robot.x
                    by_r = ball_waypoint[1] - robot.y
                    angle_to_ball = math.degrees(math.atan2(by_r, bx_r))
                    err_robot_ball = ((angle_to_ball - robot_deg + 180) % 360) - 180

                    # ── Retry si error a pelota demasiado grande ──────────────
                    retry_tol = behavior_params['angle_threshold'] * 2
                    if abs(err_robot_ball) > retry_tol:
                        behind_ball_target_angle_deg = angle_to_ball
                        robot_stopped = False
                        log.info("🔁 Fase 1b RETRY: err=%.1f° > %.1f° | nuevo obj=%.1f°",
                                 err_robot_ball, retry_tol, angle_to_ball)
                    else:
                        # ── Cono del arco rival ───────────────────────────────
                        gx = float(FIELD_CAM.goal_right_x)
                        g_top_y    = float(FIELD_CAM.goal_right_top_y)
                        g_bottom_y = float(FIELD_CAM.goal_right_bottom_y)
                        ang_top    = math.degrees(math.atan2(g_top_y    - robot.y, gx - robot.x))
                        ang_bottom = math.degrees(math.atan2(g_bottom_y - robot.y, gx - robot.x))
                        ang_center = (ang_top + ang_bottom) / 2.0
                        half_cone  = abs(ang_bottom - ang_top) / 2.0
                        err_robot_goal = ((ang_center - robot_deg + 180) % 360) - 180
                        err_ball_goal  = ((angle_to_ball - ang_center + 180) % 360) - 180
                        robot_in_cone  = abs(err_robot_goal) <= half_cone
                        ball_in_cone   = abs(err_ball_goal)  <= half_cone

                        log.info("✅ FASE 1 completada | dist_pelota=%.0fpx", dist_bh)
                        log.info("   Robot=%.1f° | →Pelota=%.1f° (err=%+.1f°)",
                                 robot_deg, angle_to_ball, err_robot_ball)
                        log.info("   Arco: centro=%.1f° cono=[%.1f°..%.1f°] (±%.1f°)",
                                 ang_center, ang_top, ang_bottom, half_cone)
                        log.info("   Robot dentro del cono: %s (err=%+.1f°) | "
                                 "Pelota dentro del cono: %s (err=%+.1f°)",
                                 "✓ SÍ" if robot_in_cone else "✗ NO", err_robot_goal,
                                 "✓ SÍ" if ball_in_cone  else "✗ NO", err_ball_goal)
                        log.info("   Presiona D para fase 2 (creep).")
                        movement_active = False
                        robot_stopped = False

            # --- Sin movimiento activo: detener robot (excepto fases de creep/settling) ---
            elif capture_phase not in ('capture_creeping', 'capture_settling', 'confirmed'):
                if robot and rf_controller and not robot_stopped:
                    rf_controller.set_motors(firmware_id, 0, 0)
                    last_cmd_type = 'stop'
                    robot_stopped = True

            if current_time - last_state_send_time >= STATE_SEND_INTERVAL:
                try:
                    # Calcular datos de navegación para visualización
                    angle_error_deg = None
                    target_heading_deg = None
                    movement_mode = None
                    robot_angle_deg = None

                    if robot and target_waypoint:
                        dx = target_waypoint[0] - robot.x
                        dy = target_waypoint[1] - robot.y
                        target_heading_rad = math.atan2(dy, dx)
                        target_heading_deg = math.degrees(target_heading_rad)
                        angle_error_rad = target_heading_rad - robot.angle
                        # Normalize to [-pi, pi]
                        while angle_error_rad > math.pi:
                            angle_error_rad -= 2 * math.pi
                        while angle_error_rad < -math.pi:
                            angle_error_rad += 2 * math.pi
                        angle_error_deg = math.degrees(angle_error_rad)

                        enter_thresh = behavior_params['linear_start_angle_threshold']
                        angle_thresh = behavior_params['angle_threshold']
                        if abs(angle_error_deg) > enter_thresh:
                            movement_mode = 'rotating'
                        elif abs(angle_error_deg) > angle_thresh:
                            movement_mode = 'linear'
                        else:
                            movement_mode = 'arrived_angle'

                    if robot:
                        robot_angle_deg = math.degrees(robot.angle)

                    control_state_pipe.send({
                        'behavior_params': behavior_params.copy(),
                        'target_waypoint': target_waypoint,
                        'movement_active': movement_active,
                        'robot_id': robot_id,
                        'robot_available': robot_available,
                        'robot_detected': robot is not None,
                        'robot_pos': (int(robot.x), int(robot.y)) if robot else None,
                        'robot_angle_deg': robot_angle_deg,
                        'angle_error_deg': angle_error_deg,
                        'target_heading_deg': target_heading_deg,
                        'movement_mode': movement_mode,
                        'capture_phase': capture_phase,
                        'ball_waypoint': ball_waypoint,
                        'overshoot_target': overshoot_target,
                        'dribbler_on': dribbler_on,
                        # Nuevos campos sin dribbler
                        'behind_ball_target': behind_ball_target,
                        'dist_to_ball': round(dist_to_ball_now, 1) if ball_waypoint and robot else None,
                        'settle_elapsed': round(settle_elapsed, 2) if capture_phase == 'capture_settling' else None,
                        'last_cmd_type': last_cmd_type,
                        'stuck_boost': controller._pid_state.get(robot_id, {}).get('stuck_boost', 0),
                        'stuck_auto_kick': behavior_params.get('stuck_auto_kick', STUCK_AUTO_KICK),
                        'timestamp': current_time
                    })
                    last_state_send_time = current_time
                except Exception as e:
                    log.warning(f"⚠️  Error enviando estado: {e}")

            time.sleep(0.01)

    finally:
        if rf_controller:
            if dribbler_on:
                rf_controller.set_dribbler(firmware_id, 0)
                log.info("Dribbler OFF (cleanup)")
            if robot and robot_available:
                rf_controller.set_motors(firmware_id, 0, 0)
                log.info("🛑 Robot detenido")
            rf_controller.shutdown()
            log.info("🔌 Conexión RF cerrada")

        log.info("🎮 Proceso de control behavior puro finalizado")
