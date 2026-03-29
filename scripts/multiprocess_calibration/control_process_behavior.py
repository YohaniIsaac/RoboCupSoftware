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
)

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
        'dribble_pwm_factor': DRIBBLE_PWM_FACTOR,
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

    factor = behavior_params.get('dribble_pwm_factor', 1.0)
    cv2.putText(panel, f"Dribble PWM factor: {factor:.2f}x (T/Y)", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y_offset += line_height
    cv2.putText(panel, "  -> Multiplicador PWM con posesion (>1 = mas potencia)", (20, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
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

    # Dribble PWM factor (T/Y ±0.05)
    elif key == ord('t') or key == ord('T'):
        behavior_params['dribble_pwm_factor'] = round(max(0.5, behavior_params['dribble_pwm_factor'] - 0.05), 2)
        result['action'] = 'update_behavior'
        log.info(f"dribble_pwm_factor: {behavior_params['dribble_pwm_factor']}")
    elif key == ord('y') or key == ord('Y'):
        behavior_params['dribble_pwm_factor'] = round(min(2.0, behavior_params['dribble_pwm_factor'] + 0.05), 2)
        result['action'] = 'update_behavior'
        log.info(f"dribble_pwm_factor: {behavior_params['dribble_pwm_factor']}")

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
        'DRIBBLE_PWM_FACTOR': f"{behavior_params.get('dribble_pwm_factor', 1.0)}",
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
        'dribble_pwm_factor': DRIBBLE_PWM_FACTOR,
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

    # --- Estado de captura (2 fases) ---
    # 'idle': sin movimiento
    # 'approach': fase 1 — moverse hacia waypoint, parar a capture_activate_px
    # 'capture': fase 2 — dribbler ON + creep hacia overshoot target
    capture_phase = 'idle'
    ball_waypoint = None      # posición original del waypoint (simula la pelota)
    overshoot_target = None   # punto overshoot calculado
    dribbler_on = False
    last_dribbler_keepalive = 0.0
    DRIBBLER_KEEPALIVE = 0.08  # 80ms < firmware timeout 100ms
    firmware_id = robot_id + 1

    last_state_send_time = 0
    STATE_SEND_INTERVAL = 0.04  # ~25 Hz

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
                            # Iniciar fase 1: approach — parar a capture_activate_px
                            movement_active = True
                            capture_phase = 'approach'
                            ball_waypoint = list(target_waypoint)
                            overshoot_target = None
                            controller.position_threshold = behavior_params['capture_activate_px']
                            robot_stopped = False
                            log.info("▶️  FASE 1: Approach → parar a %dpx del waypoint",
                                     behavior_params['capture_activate_px'])
                        else:
                            # Pausar — apagar dribbler si estaba activo
                            movement_active = False
                            robot_stopped = False
                            if dribbler_on and rf_controller:
                                rf_controller.set_dribbler(firmware_id, 0.0)
                                dribbler_on = False
                                log.info("Dribbler OFF (pausado)")
                            if capture_phase == 'capture':
                                _restore_controller_speed(controller)
                            capture_phase = 'idle'
                            overshoot_target = None
                            controller.position_threshold = behavior_params['position_threshold']
                            log.info("⏸️  Movimiento PAUSADO")
                    elif command == 'start_capture':
                        # Fase 2: dribbler ON + creep a overshoot
                        if not movement_active and ball_waypoint and robot and capture_phase in ('idle', 'approach'):
                            # Calcular overshoot: desde robot a través de ball_waypoint
                            dx = ball_waypoint[0] - robot.x
                            dy = ball_waypoint[1] - robot.y
                            dist = math.sqrt(dx * dx + dy * dy)
                            if dist > 0:
                                fwd_x = dx / dist
                                fwd_y = dy / dist
                                overshoot_target = [
                                    int(ball_waypoint[0] + fwd_x * behavior_params['capture_overshoot_px']),
                                    int(ball_waypoint[1] + fwd_y * behavior_params['capture_overshoot_px'])
                                ]
                                target_waypoint = list(overshoot_target)
                                capture_phase = 'capture'
                                movement_active = True
                                robot_stopped = False
                                controller.position_threshold = behavior_params['position_threshold']
                                # Forzar velocidad baja para creep
                                _set_controller_creep_speed(controller, behavior_params['creep_speed_pwm'])
                                # Activar dribbler
                                if rf_controller:
                                    rf_controller.set_dribbler(firmware_id, 1.0)
                                    dribbler_on = True
                                    last_dribbler_keepalive = time.time()
                                log.info("▶️  FASE 2: Capture | dribbler ON | overshoot=(%d,%d) | dist_final≈%dpx",
                                         overshoot_target[0], overshoot_target[1],
                                         max(0, behavior_params['capture_overshoot_px'] - behavior_params['position_threshold']))
                            else:
                                log.warning("Robot ya está en el waypoint, no se puede calcular dirección")
                        elif capture_phase == 'capture':
                            log.info("Ya en fase capture")
                        else:
                            log.warning("Necesitas un waypoint y completar fase 1 primero")
                    elif command == 'cancel_waypoint':
                        target_waypoint = None
                        ball_waypoint = None
                        overshoot_target = None
                        movement_active = False
                        robot_stopped = False
                        if capture_phase == 'capture':
                            _restore_controller_speed(controller)
                        capture_phase = 'idle'
                        controller.position_threshold = behavior_params['position_threshold']
                        if dribbler_on and rf_controller:
                            rf_controller.set_dribbler(firmware_id, 0.0)
                            dribbler_on = False
                        log.info("❌ Waypoint cancelado")
                    elif command == 'set_waypoint':
                        waypoint = cmd.get('waypoint')
                        if waypoint:
                            target_waypoint = list(waypoint)
                            ball_waypoint = list(waypoint)
                            overshoot_target = None
                            capture_phase = 'idle'
                            if dribbler_on and rf_controller:
                                rf_controller.set_dribbler(firmware_id, 0.0)
                                dribbler_on = False
                            log.info(f"🎯 Waypoint establecido: ({target_waypoint[0]}, {target_waypoint[1]})")
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
                                'creep_speed_pwm': (10, 60),
                                'dribble_pwm_factor': (0.5, 2.0),
                            }
                            lo, hi = _param_bounds.get(param, (0, 9999))
                            behavior_params[param] = max(lo, min(hi, round(behavior_params[param] + delta, 2)))
                            controller.position_threshold = behavior_params['position_threshold']
                            controller.angle_threshold = math.radians(behavior_params['angle_threshold'])
                            controller.dribble_pwm_factor = behavior_params.get('dribble_pwm_factor', 1.0)
                            log.info(f"  {param}: {behavior_params[param]}")
                    elif command == 'save_params':
                        _save_behavior_to_config(behavior_params)
                    elif command == 'update_behavior':
                        controller.position_threshold = behavior_params['position_threshold']
                        controller.angle_threshold = math.radians(behavior_params['angle_threshold'])

                except Exception as e:
                    log.error(f"Error recibiendo comando: {e}")

            # --- Dribbler keepalive (cada 80ms cuando activo) ---
            if dribbler_on and rf_controller and current_time - last_dribbler_keepalive >= DRIBBLER_KEEPALIVE:
                rf_controller.set_dribbler(firmware_id, 1.0)
                last_dribbler_keepalive = current_time

            if movement_active and robot and target_waypoint:
                reached = controller.move_to_position(robot, tuple(target_waypoint))
                if reached:
                    if capture_phase == 'approach':
                        log.info("✅ FASE 1 completada — robot a %dpx del waypoint. Presiona D para fase 2.",
                                 behavior_params['capture_activate_px'])
                        movement_active = False
                        robot_stopped = False
                    elif capture_phase == 'capture':
                        # Fase 2 completada — restaurar velocidades normales
                        _restore_controller_speed(controller)
                        dist_to_ball = 0
                        if ball_waypoint:
                            bx = ball_waypoint[0] - robot.x
                            by = ball_waypoint[1] - robot.y
                            dist_to_ball = math.sqrt(bx * bx + by * by)
                        log.info("✅ FASE 2 completada — dist al waypoint=%.1fpx (confirm=%dpx)",
                                 dist_to_ball, behavior_params['capture_confirm_px'])
                        if dist_to_ball < behavior_params['capture_confirm_px']:
                            log.info("✅ CAPTURA sería CONFIRMADA a esta distancia")
                        else:
                            log.info("⚠️  CAPTURA NO confirmada — ajustar overshoot (I/K) o confirm (O/L)")
                        movement_active = False
                        robot_stopped = False
                    else:
                        log.info(f"✅ Waypoint alcanzado en ({target_waypoint[0]}, {target_waypoint[1]})")
                        target_waypoint = None
                        movement_active = False
                        robot_stopped = False
            else:
                if robot and robot_available and not robot_stopped:
                    rf_controller.set_motors(firmware_id, 0, 0)
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
                        'timestamp': current_time
                    })
                    last_state_send_time = current_time
                except Exception as e:
                    log.warning(f"⚠️  Error enviando estado: {e}")

            time.sleep(0.01)

    finally:
        if rf_controller:
            if dribbler_on:
                rf_controller.set_dribbler(firmware_id, 0.0)
                log.info("Dribbler OFF (cleanup)")
            if robot and robot_available:
                rf_controller.set_motors(firmware_id, 0, 0)
                log.info("🛑 Robot detenido")
            rf_controller.shutdown()
            log.info("🔌 Conexión RF cerrada")

        log.info("🎮 Proceso de control behavior puro finalizado")
