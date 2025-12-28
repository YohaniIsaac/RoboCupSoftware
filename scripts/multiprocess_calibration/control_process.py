"""Proceso de control para calibración de velocidad.

Este proceso se ejecuta independientemente y:
- Recibe posiciones de robots desde el proceso de percepción
- Maneja la interfaz gráfica
- Permite ajustar parámetros con el teclado
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
    ROBOT_MAX_LINEAR_SPEED,
    ROBOT_MIN_LINEAR_SPEED,
    ROBOT_LINEAR_ARRIVAL_DISTANCE,
    ROBOT_LINEAR_NEAR_MIN,
    ROBOT_MAX_ROTATION_SPEED,
    ROBOT_MIN_ROTATION_SPEED,
    ROBOT_ROTATION_ARRIVAL_ANGLE_DEG,
    ROBOT_ROTATION_NEAR_MIN,
    MOTOR_DEAD_ZONE_PWM,
    MOTOR_MIN_MOVEMENT_PWM,
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


def control_loop(robot_positions_pipe, frame_pipe, robot_id, serial_port):
    """Bucle principal del proceso de control.

    Args:
        robot_positions_pipe: Pipe para recibir posiciones de robots desde percepción
        frame_pipe: Pipe para recibir frames procesados desde percepción
        robot_id: ID del robot a controlar
        serial_port: Puerto serial para comunicación RF
    """
    log.info(f"🎮 Proceso de control iniciado para Robot ID {robot_id}")

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

    # Cargar calibración del robot
    robot_calibration = RobotCalibration()
    max_speed_left, max_speed_right, bias = robot_calibration.get_calibration(robot_id)
    log.info(f"📊 Calibración Robot {robot_id}: L={max_speed_left:.2f}, R={max_speed_right:.2f}, Bias={bias:.3f}")

    # Parámetros de calibración
    params = {
        'max_linear_speed': ROBOT_MAX_LINEAR_SPEED,
        'min_linear_speed': ROBOT_MIN_LINEAR_SPEED,
        'linear_arrival_distance': ROBOT_LINEAR_ARRIVAL_DISTANCE,
        'linear_near_min': ROBOT_LINEAR_NEAR_MIN,
        'max_rotation_speed': ROBOT_MAX_ROTATION_SPEED,
        'min_rotation_speed': ROBOT_MIN_ROTATION_SPEED,
        'rotation_arrival_angle': ROBOT_ROTATION_ARRIVAL_ANGLE_DEG,
        'rotation_near_min': ROBOT_ROTATION_NEAR_MIN,
        'position_threshold': ROBOT_POSITION_THRESHOLD,
        'angle_threshold': ROBOT_ANGLE_THRESHOLD_DEG,
        # Calibración de motores
        'calib_max_speed_left': max_speed_left,
        'calib_max_speed_right': max_speed_right,
        'calib_bias': bias,
        'robot_id': robot_id,  # Para guardar calibración
    }

    # Controlador
    controller = DifferentialDriveController(rf_controller=rf_controller)
    _update_controller(controller, params)

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
    cv2.namedWindow('Control Panel', cv2.WINDOW_NORMAL)

    log.info("✅ Interfaz iniciada - Esperando datos de percepción...")

    try:
        while running:
            # Recibir posiciones de robots (sin bloqueo)
            if robot_positions_pipe.poll():
                try:
                    data = robot_positions_pipe.recv()
                    robots_data = data['robots']

                    # Buscar robot específico
                    for robot_data in robots_data:
                        if robot_data['id'] == robot_id:
                            if robot is None:
                                robot = RobotEntity(
                                    robot_data['id'],
                                    robot_data['x'],
                                    robot_data['y'],
                                    robot_data['angulo']
                                )
                                log.info(f"🤖 Robot ID {robot_id} detectado")
                            else:
                                robot.update(
                                    robot_data['x'],
                                    robot_data['y'],
                                    robot_data['angulo']
                                )
                            break
                except Exception as e:
                    log.debug(f"Error recibiendo posiciones: {e}")

            # Recibir frame procesado (sin bloqueo)
            if frame_pipe.poll():
                try:
                    last_frame = frame_pipe.recv()
                except Exception as e:
                    log.debug(f"Error recibiendo frame: {e}")

            # Si hay waypoint y robot, y movimiento está activo, mover
            if target_waypoint and robot and movement_active:
                reached = controller.move_to_position(robot, tuple(target_waypoint))
                robot_stopped = False  # Robot en movimiento
                if reached:
                    log.info("✅ Waypoint alcanzado!")
                    target_waypoint = None
                    movement_active = False
            elif robot and rf_controller and not robot_stopped:
                # Si NO hay movimiento activo, detener el robot UNA SOLA VEZ
                firmware_id = robot.id + 1
                rf_controller.set_motors(firmware_id, 0, 0)
                robot_stopped = True  # Marcar que ya detuvimos

            # Dibujar interfaz
            if last_frame is not None:
                display_frame = draw_robot_view(last_frame.copy(), robot, robot_id, target_waypoint)
            else:
                display_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(display_frame, "Esperando frames de percepcion...",
                           (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            panel = create_control_panel(params, robot, robot_id, robot_available, target_waypoint, movement_active)

            cv2.imshow('Robot View', display_frame)
            cv2.imshow('Control Panel', panel)

            # Procesar teclas (30ms = ~33 FPS, reduce carga de comunicación serial)
            key = cv2.waitKey(30) & 0xFF
            result = process_key(key, params, controller, robot, target_waypoint, movement_active)

            if result['action'] == 'quit':
                running = False
            elif result['action'] == 'update_params':
                params = result['params']
                _update_controller(controller, params)
            elif result['action'] == 'set_waypoint':
                target_waypoint = result['waypoint']
            elif result['action'] == 'cancel_waypoint':
                # Detener robot INMEDIATAMENTE al cancelar
                if robot and rf_controller:
                    firmware_id = robot.id + 1
                    rf_controller.set_motors(firmware_id, 0, 0)
                target_waypoint = None
                movement_active = False
                robot_stopped = True
                log.info("❌ Waypoint cancelado - Robot detenido")
            elif result['action'] == 'toggle_movement':
                # Debounce: solo procesar si han pasado al menos 300ms desde la última pulsación
                current_time = time.time()
                if (current_time - last_space_time) >= 0.3:
                    last_space_time = current_time
                    if target_waypoint and robot:
                        movement_active = not movement_active

                        # Calcular información de posición/objetivo
                        dx = target_waypoint[0] - int(robot.x)
                        dy = target_waypoint[1] - int(robot.y)
                        dist = math.sqrt(dx**2 + dy**2)

                        if movement_active:
                            robot_stopped = False  # Permitir movimiento
                            log.info("▶️  Movimiento INICIADO")
                            log.info("📍 Robot(%d, %d) → Target(%d, %d) | dx=%d dy=%d dist=%.1f",
                                    int(robot.x), int(robot.y),
                                    target_waypoint[0], target_waypoint[1],
                                    dx, dy, dist)
                        else:
                            # PAUSADO: Detener robot INMEDIATAMENTE
                            if rf_controller:
                                firmware_id = robot.id + 1
                                rf_controller.set_motors(firmware_id, 0, 0)
                                robot_stopped = True
                            log.info("⏸️  Movimiento PAUSADO - Robot detenido")
                            log.info("📍 Robot(%d, %d) → Target(%d, %d) | dx=%d dy=%d dist=%.1f",
                                    int(robot.x), int(robot.y),
                                    target_waypoint[0], target_waypoint[1],
                                    dx, dy, dist)
                    else:
                        log.warning("⚠️  Primero crea un waypoint con las flechas")
            elif result['action'] == 'save':
                save_to_config(params)
                save_to_calibration(params)

            # Actualizar referencias
            if result.get('waypoint') is not None:
                target_waypoint = result['waypoint']
            # movement_active se maneja directamente en las acciones, no en result

    except KeyboardInterrupt:
        log.info("⏹️  Proceso de control detenido por usuario")
    finally:
        # Detener robot
        if robot and rf_controller:
            firmware_id = robot.id + 1
            rf_controller.set_motors(firmware_id, 0, 0)
            log.info("⏹️  Robot detenido")

        cv2.destroyAllWindows()

        if rf_controller:
            rf_controller.shutdown()
            log.info("🔌 Comunicación RF cerrada")


def _update_controller(controller, params):
    """Actualiza el controlador con los parámetros actuales."""
    # Parámetros de velocidad lineal
    controller.max_smooth_speed = params['max_linear_speed']
    controller.min_motor_speed = params['min_linear_speed']
    controller.distance_near = params['linear_arrival_distance']
    controller.linear_near_min = params['linear_near_min']

    # Parámetros de velocidad angular
    controller.max_rotation_speed = params['max_rotation_speed']
    controller.min_rotation_speed = params['min_rotation_speed']
    controller.angle_near = math.radians(params['rotation_arrival_angle'])
    controller.rotation_near_min = params['rotation_near_min']

    # Thresholds
    controller.position_threshold = params['position_threshold']
    controller.angle_threshold = math.radians(params['angle_threshold'])

    # Calibración de motores L/R
    robot_id = params['robot_id']
    if controller.rf_controller:
        controller.rf_controller.update_robot_calibration(
            robot_id,
            params['calib_max_speed_left'],
            params['calib_max_speed_right'],
            params['calib_bias']
        )
        log.debug(f"🔧 Calibración actualizada en RF: L={params['calib_max_speed_left']:.3f}, R={params['calib_max_speed_right']:.3f}")


def draw_robot_view(frame, robot, robot_id, target_waypoint):
    """Dibuja visualización sobre el frame."""
    # Dibujar waypoint
    if target_waypoint:
        # Convertir a tupla para asegurar formato correcto en OpenCV
        waypoint_tuple = tuple(target_waypoint) if isinstance(target_waypoint, list) else target_waypoint

        cv2.circle(frame, waypoint_tuple, 10, (0, 255, 0), 2)
        cv2.circle(frame, waypoint_tuple, 3, (0, 255, 0), -1)
        cv2.putText(frame, "OBJETIVO",
                   (target_waypoint[0] + 15, target_waypoint[1] - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if robot:
            robot_pos = (int(robot.x), int(robot.y))
            cv2.line(frame, robot_pos, waypoint_tuple, (255, 0, 255), 1)

            # DEBUG: Dibujar info de coordenadas
            cv2.putText(frame, f"WP: ({target_waypoint[0]}, {target_waypoint[1]})",
                       (10, frame.shape[0] - 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(frame, f"Robot: ({int(robot.x)}, {int(robot.y)})",
                       (10, frame.shape[0] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    # Marcar robot controlado
    if robot:
        cv2.circle(frame, (int(robot.x), int(robot.y)), 60, (0, 255, 255), 3)
        cv2.putText(frame, f"CONTROLANDO ID {robot_id}",
                   (int(robot.x) - 80, int(robot.y) - 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    return frame


def create_control_panel(params, robot, robot_id, robot_available, target_waypoint, movement_active):
    """Crea panel de información."""
    panel = np.zeros((920, 650, 3), dtype=np.uint8)  # Aumentado de 680 a 920 para más espacio

    # Título
    cv2.putText(panel, "CALIBRACION PWM (0-255)", (10, 30),
               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    y = 65
    # Estado
    robot_color = (0, 255, 0) if robot else (0, 0, 255)
    cv2.putText(panel, f"Robot ID {robot_id}: {'OK' if robot else 'NO DETECTADO'}",
               (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, robot_color, 2)

    y += 30
    rf_color = (0, 255, 0) if robot_available else (0, 0, 255)
    cv2.putText(panel, f"RF: {'CONECTADO' if robot_available else 'DESCONECTADO'}",
               (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, rf_color, 2)

    # Referencias de motor DC
    y += 40
    cv2.putText(panel, "=== REFERENCIAS MOTOR DC ===", (10, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 100), 2)
    y += 25
    cv2.putText(panel, f"Dead zone: PWM < {MOTOR_DEAD_ZONE_PWM}", (20, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 100, 100), 1)
    y += 20
    cv2.putText(panel, f"Movimiento: PWM >= {MOTOR_MIN_MOVEMENT_PWM}", (20, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 255, 100), 1)

    # Parámetros
    y += 40
    cv2.putText(panel, "=== VELOCIDAD LINEAL (PWM) ===", (10, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 255), 2)
    y += 30
    cv2.putText(panel, f"Max: {params['max_linear_speed']} PWM (w/s)", (20, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y += 25
    cv2.putText(panel, f"Min LEJOS: {params['min_linear_speed']} PWM (a/d)", (20, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    y += 40
    cv2.putText(panel, "=== VELOCIDAD ROTACION (PWM) ===", (10, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 200), 2)
    y += 30
    cv2.putText(panel, f"Max: {params['max_rotation_speed']} PWM (q/e)", (20, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y += 25
    cv2.putText(panel, f"Min LEJOS: {params['min_rotation_speed']} PWM (z/c)", (20, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    y += 25
    cv2.putText(panel, f"Min rampa: {params['rotation_near_min']} PWM (7/8)", (20, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 150), 1)
    y += 25
    cv2.putText(panel, f"Inicia: {params['rotation_arrival_angle']:.1f}deg (3/4)", (20, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 150), 1)

    y += 40
    cv2.putText(panel, "=== RAMPA LINEAL ===", (10, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 150, 255), 2)
    y += 30
    cv2.putText(panel, f"Inicia: {params['linear_arrival_distance']}px (1/2)", (20, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 150), 1)
    y += 25
    cv2.putText(panel, f"Min rampa: {params['linear_near_min']} PWM (5/6)", (20, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 150), 1)

    y += 40
    cv2.putText(panel, "=== THRESHOLDS ===", (10, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 255, 150), 2)
    y += 30
    cv2.putText(panel, f"Posicion: {params['position_threshold']}px (9/0)", (20, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 150), 1)
    y += 25
    cv2.putText(panel, f"Angular: {params['angle_threshold']}deg (-/=)", (20, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 150), 1)

    y += 40
    cv2.putText(panel, "=== CALIBRACION MOTORES L/R ===", (10, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 180, 100), 2)
    y += 30
    cv2.putText(panel, f"Motor Left:  {params['calib_max_speed_left']:.3f} ([/])", (20, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 255, 200), 1)
    y += 25
    cv2.putText(panel, f"Motor Right: {params['calib_max_speed_right']:.3f} (;/')", (20, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 255, 200), 1)
    y += 25
    ratio = params['calib_max_speed_right'] / params['calib_max_speed_left'] if params['calib_max_speed_left'] > 0 else 0
    cv2.putText(panel, f"Ratio R/L: {ratio:.3f}", (20, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 200, 255), 1)

    y += 40
    cv2.putText(panel, "=== CONTROLES ===", (10, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 100), 2)
    y += 28
    cv2.putText(panel, "Flechas: Mover waypoint", (20, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    y += 23
    cv2.putText(panel, "ESPACIO: START/STOP", (20, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 100), 1)
    y += 23
    cv2.putText(panel, "ENTER: Guardar | ESC: Salir", (20, y),
               cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 255, 100), 1)

    # Estado movimiento
    y += 40
    if target_waypoint and robot:
        status_text = "MOVIMIENTO: ACTIVO" if movement_active else "MOVIMIENTO: PAUSADO"
        status_color = (0, 255, 0) if movement_active else (0, 165, 255)
        cv2.putText(panel, status_text, (10, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)

    return panel


def process_key(key, params, controller, robot, target_waypoint, movement_active):
    """Procesa teclas y retorna acción.

    Args:
        key: Tecla presionada
        params: Parámetros de calibración
        controller: Controlador del robot
        robot: Objeto robot
        target_waypoint: Waypoint objetivo
        movement_active: Si el movimiento está activo

    Returns:
        dict: Resultado con acción y parámetros actualizados
    """
    # NO incluir movement_active aquí, se maneja en el loop principal
    result = {'action': 'none', 'params': params, 'waypoint': target_waypoint}

    # Si no hay tecla presionada, no hacer nada
    if key == 255:
        return result

    # Teclas de control
    if key == 27:  # ESC
        result['action'] = 'quit'
    elif key == 13:  # ENTER
        result['action'] = 'save'
    elif key == 32:  # ESPACIO
        result['action'] = 'toggle_movement'
    elif key == ord('x'):
        result['action'] = 'cancel_waypoint'

    # Flechas - mover waypoint
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

    # Ajustes de velocidad lineal (±1 PWM)
    elif key == ord('w'):
        params['max_linear_speed'] = min(255, params['max_linear_speed'] + 1)
        result['action'] = 'update_params'
        log.info(f"max_linear_speed: {params['max_linear_speed']} PWM")
    elif key == ord('s'):
        params['max_linear_speed'] = max(0, params['max_linear_speed'] - 1)
        result['action'] = 'update_params'
        log.info(f"max_linear_speed: {params['max_linear_speed']} PWM")
    elif key == ord('a'):
        params['min_linear_speed'] = max(0, params['min_linear_speed'] - 1)
        result['action'] = 'update_params'
        log.info(f"min_linear_speed: {params['min_linear_speed']} PWM")
    elif key == ord('d'):
        params['min_linear_speed'] = min(255, params['min_linear_speed'] + 1)
        result['action'] = 'update_params'
        log.info(f"min_linear_speed: {params['min_linear_speed']} PWM")

    # Ajustes de velocidad rotación (±1 PWM)
    elif key == ord('q'):
        params['max_rotation_speed'] = min(255, params['max_rotation_speed'] + 1)
        result['action'] = 'update_params'
        log.info(f"max_rotation_speed: {params['max_rotation_speed']} PWM")
    elif key == ord('e'):
        params['max_rotation_speed'] = max(0, params['max_rotation_speed'] - 1)
        result['action'] = 'update_params'
        log.info(f"max_rotation_speed: {params['max_rotation_speed']} PWM")
    elif key == ord('z'):
        params['min_rotation_speed'] = max(0, params['min_rotation_speed'] - 1)
        result['action'] = 'update_params'
        log.info(f"min_rotation_speed: {params['min_rotation_speed']} PWM")
    elif key == ord('c'):
        params['min_rotation_speed'] = min(255, params['min_rotation_speed'] + 1)
        result['action'] = 'update_params'
        log.info(f"min_rotation_speed: {params['min_rotation_speed']} PWM")

    # === PARÁMETROS DE RAMPA Y THRESHOLDS ===

    # Inicio de rampa lineal (±1px)
    elif key == ord('1'):
        params['linear_arrival_distance'] = min(200, params['linear_arrival_distance'] + 1)
        result['action'] = 'update_params'
        log.info(f"linear_arrival_distance: {params['linear_arrival_distance']}px")
    elif key == ord('2'):
        params['linear_arrival_distance'] = max(10, params['linear_arrival_distance'] - 1)
        result['action'] = 'update_params'
        log.info(f"linear_arrival_distance: {params['linear_arrival_distance']}px")

    # Inicio de rampa angular (±1°)
    elif key == ord('3'):
        params['rotation_arrival_angle'] = min(90, params['rotation_arrival_angle'] + 1)
        result['action'] = 'update_params'
        log.info(f"rotation_arrival_angle: {params['rotation_arrival_angle']:.1f}°")
    elif key == ord('4'):
        params['rotation_arrival_angle'] = max(5, params['rotation_arrival_angle'] - 1)
        result['action'] = 'update_params'
        log.info(f"rotation_arrival_angle: {params['rotation_arrival_angle']:.1f}°")

    # Velocidad mínima en rampa lineal (±1 PWM)
    elif key == ord('5'):
        params['linear_near_min'] = min(params['min_linear_speed'], params['linear_near_min'] + 1)
        result['action'] = 'update_params'
        log.info(f"linear_near_min: {params['linear_near_min']} PWM")
    elif key == ord('6'):
        params['linear_near_min'] = max(MOTOR_DEAD_ZONE_PWM, params['linear_near_min'] - 1)
        result['action'] = 'update_params'
        log.info(f"linear_near_min: {params['linear_near_min']} PWM")

    # Velocidad mínima en rampa angular (±1 PWM)
    elif key == ord('7'):
        params['rotation_near_min'] = min(params['min_rotation_speed'], params['rotation_near_min'] + 1)
        result['action'] = 'update_params'
        log.info(f"rotation_near_min: {params['rotation_near_min']} PWM")
    elif key == ord('8'):
        params['rotation_near_min'] = max(MOTOR_DEAD_ZONE_PWM, params['rotation_near_min'] - 1)
        result['action'] = 'update_params'
        log.info(f"rotation_near_min: {params['rotation_near_min']} PWM")

    # Threshold de posición (±1px)
    elif key == ord('9'):
        params['position_threshold'] = min(50, params['position_threshold'] + 1)
        result['action'] = 'update_params'
        log.info(f"position_threshold: {params['position_threshold']}px")
    elif key == ord('0'):
        params['position_threshold'] = max(5, params['position_threshold'] - 1)
        result['action'] = 'update_params'
        log.info(f"position_threshold: {params['position_threshold']}px")

    # Threshold angular (±1°)
    elif key == ord('-'):
        params['angle_threshold'] = max(1, params['angle_threshold'] - 1)
        result['action'] = 'update_params'
        log.info(f"angle_threshold: {params['angle_threshold']:.1f}°")
    elif key == ord('='):
        params['angle_threshold'] = min(30, params['angle_threshold'] + 1)
        result['action'] = 'update_params'
        log.info(f"angle_threshold: {params['angle_threshold']:.1f}°")

    # === CALIBRACIÓN DE MOTORES L/R (incremento ±0.01) ===
    elif key == ord('['):
        params['calib_max_speed_left'] = max(0.0, params['calib_max_speed_left'] - 0.01)
        result['action'] = 'update_params'
        log.info(f"⚙️  calib_max_speed_left: {params['calib_max_speed_left']:.3f}")
    elif key == ord(']'):
        params['calib_max_speed_left'] = min(1.0, params['calib_max_speed_left'] + 0.01)
        result['action'] = 'update_params'
        log.info(f"⚙️  calib_max_speed_left: {params['calib_max_speed_left']:.3f}")
    elif key == ord(';'):
        params['calib_max_speed_right'] = max(0.0, params['calib_max_speed_right'] - 0.01)
        result['action'] = 'update_params'
        log.info(f"⚙️  calib_max_speed_right: {params['calib_max_speed_right']:.3f}")
    elif key == ord("'"):
        params['calib_max_speed_right'] = min(1.0, params['calib_max_speed_right'] + 0.01)
        result['action'] = 'update_params'
        log.info(f"⚙️  calib_max_speed_right: {params['calib_max_speed_right']:.3f}")

    return result


def save_to_config(params):
    """Guarda parámetros a config.py."""
    config_path = ROOT_DIR / "src" / "robot_soccer" / "config.py"

    log.info("💾 Guardando parámetros a config.py...")

    with open(config_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    replacements = {
        'ROBOT_MIN_LINEAR_SPEED': f"{params['min_linear_speed']}",  # PWM (int)
        'ROBOT_MAX_LINEAR_SPEED': f"{params['max_linear_speed']}",  # PWM (int)
        'ROBOT_LINEAR_ARRIVAL_DISTANCE': f"{params['linear_arrival_distance']}",  # píxeles (int)
        'ROBOT_LINEAR_NEAR_MIN': f"{params['linear_near_min']}",  # PWM (int)
        'ROBOT_MIN_ROTATION_SPEED': f"{params['min_rotation_speed']}",  # PWM (int)
        'ROBOT_MAX_ROTATION_SPEED': f"{params['max_rotation_speed']}",  # PWM (int)
        'ROBOT_ROTATION_ARRIVAL_ANGLE_DEG': f"{params['rotation_arrival_angle']:.1f}",  # grados (float)
        'ROBOT_ROTATION_NEAR_MIN': f"{params['rotation_near_min']}",  # PWM (int)
        'ROBOT_POSITION_THRESHOLD': f"{params['position_threshold']}",  # píxeles (int)
        'ROBOT_ANGLE_THRESHOLD_DEG': f"{params['angle_threshold']}",  # grados (int)
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
                break
        if not modified:
            new_lines.append(line)

    with open(config_path, 'w', encoding='utf-8') as f:
        f.writelines(new_lines)

    log.info("✅ Parámetros guardados")


def save_to_calibration(params):
    """Guarda calibración de motores L/R al archivo robot_calibration.json."""
    robot_id = params['robot_id']
    max_speed_left = params['calib_max_speed_left']
    max_speed_right = params['calib_max_speed_right']
    bias = params['calib_bias']

    log.info(f"💾 Guardando calibración Robot {robot_id} a robot_calibration.json...")

    robot_calibration = RobotCalibration()
    robot_calibration.set_calibration(robot_id, max_speed_left, max_speed_right, bias)
    robot_calibration.save()

    log.info(f"✅ Calibración guardada: L={max_speed_left:.3f}, R={max_speed_right:.3f}, Bias={bias:.3f}")
