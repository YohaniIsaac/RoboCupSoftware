"""Proceso de control para calibración de parámetros PID.

Este proceso se ejecuta independientemente y:
- Recibe posiciones de robots desde el proceso de percepción
- Maneja la interfaz gráfica con panel de parámetros PID
- Permite ajustar los 6 parámetros PID con el teclado
- Envía comandos RF al robot seleccionado
- Actualiza el controlador en tiempo real
"""

import logging
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# Agregar src al path
ROOT_DIR = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))

from robot_soccer.communication.rf_controller import RFController
from robot_soccer.config import (
    PID_ANGLE_KD,
    PID_ANGLE_KI,
    PID_ANGLE_KP,
    PID_POSITION_KD,
    PID_POSITION_KI,
    PID_POSITION_KP,
    ROBOT_ANGLE_THRESHOLD_DEG,
    ROBOT_POSITION_THRESHOLD,
)
from robot_soccer.controllers.differential_drive import DifferentialDriveController

log = logging.getLogger(__name__)


class RobotEntity:
    """Entidad de robot para el controlador."""

    def __init__(self, robot_id, x, y, angle):
        """Inicializa entidad de robot.

        Args:
            robot_id: ID del robot
            x: Posición X en píxeles
            y: Posición Y en píxeles
            angle: Ángulo en radianes
        """
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


def control_loop_pid(robot_positions_pipe, frame_pipe, robot_id, serial_port):
    """Bucle principal del proceso de control PID.

    Args:
        robot_positions_pipe: Pipe para recibir posiciones de robots desde percepción
        frame_pipe: Pipe para recibir frames procesados desde percepción
        robot_id: ID del robot a controlar
        serial_port: Puerto serial para comunicación RF
    """
    log.info(f"🎮 Proceso de control PID iniciado para Robot ID {robot_id}")

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

    # Parámetros PID de calibración
    pid_params = {
        'kp_pos': PID_POSITION_KP,
        'ki_pos': PID_POSITION_KI,
        'kd_pos': PID_POSITION_KD,
        'kp_angle': PID_ANGLE_KP,
        'ki_angle': PID_ANGLE_KI,
        'kd_angle': PID_ANGLE_KD,
        'position_threshold': ROBOT_POSITION_THRESHOLD,
        'angle_threshold': ROBOT_ANGLE_THRESHOLD_DEG,
    }

    # Controlador
    controller = DifferentialDriveController(rf_controller=rf_controller)
    _update_pid_controller(controller, pid_params)

    # Estado
    robot = None
    target_waypoint: list | None = None
    movement_active = False
    running = True
    last_frame = None

    # Debounce para tecla ESPACIO
    last_space_time = 0

    # Flag para saber si ya enviamos comando de detención
    robot_stopped = False

    # Crear ventanas
    cv2.namedWindow('Robot View', cv2.WINDOW_NORMAL)
    cv2.namedWindow('PID Control Panel', cv2.WINDOW_NORMAL)

    log.info("✅ Interfaz iniciada - Esperando datos de percepción...")

    try:
        while running:
            # Recibir posiciones de robots (sin bloqueo)
            if robot_positions_pipe.poll():
                try:
                    data = robot_positions_pipe.recv()
                    robots_data = data['robots']

                    # Buscar robot en la lista
                    robot_found = False
                    for r in robots_data:
                        if r['id'] == robot_id:
                            robot_found = True
                            if robot is None:
                                robot = RobotEntity(robot_id, r['x'], r['y'], r['angulo'])
                                log.info(f"🤖 Robot {robot_id} detectado en ({r['x']:.0f}, {r['y']:.0f})")
                            else:
                                robot.update(r['x'], r['y'], r['angulo'])
                            break

                    # Si no se encontró, marcar como perdido
                    if not robot_found and robot is not None:
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
                # pylint: disable=unsubscriptable-object  # target_waypoint is validated above
                reached = controller.move_to_waypoint(
                    robot,
                    target_waypoint[0],
                    target_waypoint[1]
                )
                if reached:
                    log.info(f"✅ Waypoint alcanzado en ({target_waypoint[0]}, {target_waypoint[1]})")
                    target_waypoint = None
                    movement_active = False
                    robot_stopped = False
            else:
                # Detener robot si no hay movimiento activo
                if robot and robot_available and not robot_stopped:
                    controller.stop(robot)
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

            # Panel de control PID
            panel = _draw_pid_panel(pid_params, robot, target_waypoint, robot_available)
            cv2.imshow('PID Control Panel', panel)

            # Manejo de teclado
            key = cv2.waitKey(1) & 0xFF

            if key != 255:  # Si se presionó alguna tecla
                result = _handle_keyboard_pid(
                    key, robot, target_waypoint, pid_params,
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
                    _save_pid_to_config(pid_params)
                elif result['action'] == 'update_pid':
                    _update_pid_controller(controller, pid_params)
                elif result['action'] == 'reset_integral':
                    _reset_integral_terms(controller)
                    log.info("🔄 Términos integrales reseteados (anti-windup)")

                if 'waypoint' in result:
                    target_waypoint = result['waypoint']

            time.sleep(0.01)  # 100 Hz loop

    finally:
        # Limpieza
        if robot and robot_available:
            controller.stop(robot)
            log.info("🛑 Robot detenido")

        if rf_controller:
            rf_controller.cleanup()
            log.info("🔌 Conexión RF cerrada")

        cv2.destroyAllWindows()
        log.info("🎮 Proceso de control PID finalizado")


def _update_pid_controller(controller, pid_params):
    """Actualiza los parámetros PID del controlador en tiempo real."""
    controller.kp_pos = pid_params['kp_pos']
    controller.ki_pos = pid_params['ki_pos']
    controller.kd_pos = pid_params['kd_pos']
    controller.kp_angle = pid_params['kp_angle']
    controller.ki_angle = pid_params['ki_angle']
    controller.kd_angle = pid_params['kd_angle']
    controller.position_threshold = pid_params['position_threshold']
    controller.angle_threshold = math.radians(pid_params['angle_threshold'])


def _reset_integral_terms(controller):
    """Resetea los términos integrales del PID (útil para anti-windup)."""
    controller.integral_pos = (0, 0)
    controller.integral_angle = 0


def _draw_pid_panel(pid_params, robot, waypoint, robot_available):
    """Dibuja el panel de control PID."""
    panel = np.zeros((800, 600, 3), dtype=np.uint8)

    y_offset = 30
    line_height = 25

    # Título
    cv2.putText(panel, "=== CALIBRACION PID ===", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    y_offset += line_height * 2

    # Estado del robot
    if robot_available:
        status_color = (0, 255, 0)
        status_text = "Robot detectado" if robot else "Robot NO visible"
    else:
        status_color = (0, 0, 255)
        status_text = "Robot NO conectado"

    cv2.putText(panel, status_text, (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1)
    y_offset += line_height * 2

    # PID de Posición
    cv2.putText(panel, "=== PID DE POSICION ===", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    y_offset += line_height

    cv2.putText(panel, f"Kp: {pid_params['kp_pos']:.4f} (q/a)", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y_offset += line_height

    cv2.putText(panel, f"Ki: {pid_params['ki_pos']:.4f} (w/s)", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y_offset += line_height

    cv2.putText(panel, f"Kd: {pid_params['kd_pos']:.4f} (e/d)", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y_offset += line_height * 2

    # PID Angular
    cv2.putText(panel, "=== PID ANGULAR ===", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    y_offset += line_height

    cv2.putText(panel, f"Kp: {pid_params['kp_angle']:.4f} (r/f)", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y_offset += line_height

    cv2.putText(panel, f"Ki: {pid_params['ki_angle']:.4f} (t/g)", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y_offset += line_height

    cv2.putText(panel, f"Kd: {pid_params['kd_angle']:.4f} (y/h)", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y_offset += line_height * 2

    # Thresholds
    cv2.putText(panel, "=== THRESHOLDS ===", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    y_offset += line_height

    cv2.putText(panel, f"Posicion: {pid_params['position_threshold']}px (9/0)", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y_offset += line_height

    cv2.putText(panel, f"Angular: {pid_params['angle_threshold']}deg (-/=)", (10, y_offset),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
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
        "C: Resetear integral (anti-windup)",
        "ENTER: Guardar a config.py",
        "ESC: Salir",
        "",
        "SHIFT + tecla: Ajuste fino (x0.1)",
    ]

    for control in controls:
        cv2.putText(panel, control, (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        y_offset += line_height

    return panel


def _handle_keyboard_pid(key, robot, target_waypoint, pid_params,
                         movement_active, last_space_time, controller):
    """Maneja eventos de teclado para calibración PID.

    Returns:
        dict con 'action' y otros campos según la acción
    """
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

    elif key == ord('c'):  # Resetear integral (minúscula)
        result['action'] = 'reset_integral'
        return result

    # Movimiento del waypoint con flechas
    elif key in [82, 0] and robot:  # Arriba
        if target_waypoint is None:
            target_waypoint = [int(robot.x), int(robot.y)]
            log.info("🎯 Waypoint creado en posición del robot: (%d, %d)", target_waypoint[0], target_waypoint[1])
        target_waypoint[1] -= 10
        log.info("⬆️  Waypoint → (%d, %d)", target_waypoint[0], target_waypoint[1])
        result['waypoint'] = target_waypoint
    elif key in [84, 1] and robot:  # Abajo
        if target_waypoint is None:
            target_waypoint = [int(robot.x), int(robot.y)]
            log.info("🎯 Waypoint creado en posición del robot: (%d, %d)", target_waypoint[0], target_waypoint[1])
        target_waypoint[1] += 10
        log.info("⬇️  Waypoint → (%d, %d)", target_waypoint[0], target_waypoint[1])
        result['waypoint'] = target_waypoint
    elif key in [81, 2] and robot:  # Izquierda
        if target_waypoint is None:
            target_waypoint = [int(robot.x), int(robot.y)]
            log.info("🎯 Waypoint creado en posición del robot: (%d, %d)", target_waypoint[0], target_waypoint[1])
        target_waypoint[0] -= 10
        log.info("⬅️  Waypoint → (%d, %d)", target_waypoint[0], target_waypoint[1])
        result['waypoint'] = target_waypoint
    elif key in [83, 3] and robot:  # Derecha
        if target_waypoint is None:
            target_waypoint = [int(robot.x), int(robot.y)]
            log.info("🎯 Waypoint creado en posición del robot: (%d, %d)", target_waypoint[0], target_waypoint[1])
        target_waypoint[0] += 10
        log.info("➡️  Waypoint → (%d, %d)", target_waypoint[0], target_waypoint[1])
        result['waypoint'] = target_waypoint

    # Ajustes PID de Posición (±0.001)
    elif key == ord('q'):  # Kp_pos +
        pid_params['kp_pos'] = min(1.0, pid_params['kp_pos'] + 0.001)
        result['action'] = 'update_pid'
        log.info(f"kp_pos: {pid_params['kp_pos']:.4f}")
    elif key == ord('a'):  # Kp_pos -
        pid_params['kp_pos'] = max(0.0, pid_params['kp_pos'] - 0.001)
        result['action'] = 'update_pid'
        log.info(f"kp_pos: {pid_params['kp_pos']:.4f}")
    elif key == ord('w'):  # Ki_pos +
        pid_params['ki_pos'] = min(1.0, pid_params['ki_pos'] + 0.001)
        result['action'] = 'update_pid'
        log.info(f"ki_pos: {pid_params['ki_pos']:.4f}")
    elif key == ord('s'):  # Ki_pos -
        pid_params['ki_pos'] = max(0.0, pid_params['ki_pos'] - 0.001)
        result['action'] = 'update_pid'
        log.info(f"ki_pos: {pid_params['ki_pos']:.4f}")
    elif key == ord('e'):  # Kd_pos +
        pid_params['kd_pos'] = min(1.0, pid_params['kd_pos'] + 0.001)
        result['action'] = 'update_pid'
        log.info(f"kd_pos: {pid_params['kd_pos']:.4f}")
    elif key == ord('d'):  # Kd_pos -
        pid_params['kd_pos'] = max(0.0, pid_params['kd_pos'] - 0.001)
        result['action'] = 'update_pid'
        log.info(f"kd_pos: {pid_params['kd_pos']:.4f}")

    # Ajustes PID Angular (±0.001)
    elif key == ord('r'):  # Kp_angle +
        pid_params['kp_angle'] = min(1.0, pid_params['kp_angle'] + 0.001)
        result['action'] = 'update_pid'
        log.info(f"kp_angle: {pid_params['kp_angle']:.4f}")
    elif key == ord('f'):  # Kp_angle -
        pid_params['kp_angle'] = max(0.0, pid_params['kp_angle'] - 0.001)
        result['action'] = 'update_pid'
        log.info(f"kp_angle: {pid_params['kp_angle']:.4f}")
    elif key == ord('t'):  # Ki_angle +
        pid_params['ki_angle'] = min(1.0, pid_params['ki_angle'] + 0.001)
        result['action'] = 'update_pid'
        log.info(f"ki_angle: {pid_params['ki_angle']:.4f}")
    elif key == ord('g'):  # Ki_angle -
        pid_params['ki_angle'] = max(0.0, pid_params['ki_angle'] - 0.001)
        result['action'] = 'update_pid'
        log.info(f"ki_angle: {pid_params['ki_angle']:.4f}")
    elif key == ord('y'):  # Kd_angle +
        pid_params['kd_angle'] = min(1.0, pid_params['kd_angle'] + 0.001)
        result['action'] = 'update_pid'
        log.info(f"kd_angle: {pid_params['kd_angle']:.4f}")
    elif key == ord('h'):  # Kd_angle -
        pid_params['kd_angle'] = max(0.0, pid_params['kd_angle'] - 0.001)
        result['action'] = 'update_pid'
        log.info(f"kd_angle: {pid_params['kd_angle']:.4f}")

    # Thresholds (±1)
    elif key == ord('9'):  # position_threshold +
        pid_params['position_threshold'] = min(50, pid_params['position_threshold'] + 1)
        result['action'] = 'update_pid'
        log.info(f"position_threshold: {pid_params['position_threshold']}px")
    elif key == ord('0'):  # position_threshold -
        pid_params['position_threshold'] = max(5, pid_params['position_threshold'] - 1)
        result['action'] = 'update_pid'
        log.info(f"position_threshold: {pid_params['position_threshold']}px")
    elif key == ord('-'):  # angle_threshold -
        pid_params['angle_threshold'] = max(1, pid_params['angle_threshold'] - 1)
        result['action'] = 'update_pid'
        log.info(f"angle_threshold: {pid_params['angle_threshold']}°")
    elif key == ord('='):  # angle_threshold +
        pid_params['angle_threshold'] = min(30, pid_params['angle_threshold'] + 1)
        result['action'] = 'update_pid'
        log.info(f"angle_threshold: {pid_params['angle_threshold']}°")

    return result


def _save_pid_to_config(pid_params):
    """Guarda parámetros PID a config.py."""
    config_path = ROOT_DIR / "src" / "robot_soccer" / "config.py"

    log.info("💾 Guardando parámetros PID a config.py...")

    with open(config_path, encoding='utf-8') as f:
        lines = f.readlines()

    replacements = {
        'PID_POSITION_KP': f"{pid_params['kp_pos']:.4f}",
        'PID_POSITION_KI': f"{pid_params['ki_pos']:.4f}",
        'PID_POSITION_KD': f"{pid_params['kd_pos']:.4f}",
        'PID_ANGLE_KP': f"{pid_params['kp_angle']:.4f}",
        'PID_ANGLE_KI': f"{pid_params['ki_angle']:.4f}",
        'PID_ANGLE_KD': f"{pid_params['kd_angle']:.4f}",
        'ROBOT_POSITION_THRESHOLD': f"{pid_params['position_threshold']}",
        'ROBOT_ANGLE_THRESHOLD_DEG': f"{pid_params['angle_threshold']}",
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

    log.info("✅ Parámetros PID guardados")
