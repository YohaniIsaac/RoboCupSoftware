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
    controller.MAX_ANGULAR_CORRECTION = behavior_params['max_angular_correction_pwm']


def _draw_behavior_panel(behavior_params, robot, waypoint, robot_available):
    """Dibuja el panel de control de comportamiento."""
    panel = np.zeros((600, 650, 3), dtype=np.uint8)

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
