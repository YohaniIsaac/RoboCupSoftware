"""Proceso de control PID PURO (Arquitectura de 3 procesos).

Este proceso maneja SOLO el control PID y comunicación RF:
- Recibe datos de posición desde percepción (ultra-rápido)
- Recibe comandos desde visualización (teclado/mouse)
- Ejecuta control PID para alcanzar waypoints
- Envía comandos RF al robot
- Envía estado a visualización
- NO hace visualización (sin cv2.imshow, sin generación de frames)

Diseñado para máxima frecuencia de control sin bloqueos de UI.
FPS objetivo: 100-200 Hz (vs 50-80 Hz con visualización integrada)
"""

import logging
import math
import sys
import time
from pathlib import Path

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
    """Entidad de robot con tracking de velocidad para predicción lineal."""

    # Máximo tiempo (s) usando predicción antes de considerar robot perdido
    MAX_PREDICTION_TIME = 0.5

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

        # Tracking de velocidad para predicción lineal
        self.vx = 0.0  # px/s
        self.vy = 0.0  # px/s
        self.last_detection_time = time.time()
        self.is_predicted = False  # True cuando posición es predicha
        # Posición real de última detección (para predecir siempre desde aquí)
        self._real_x = x
        self._real_y = y

    def update(self, x, y, angle):
        """Actualiza posición con datos reales de detección y calcula velocidad."""
        now = time.time()
        dt = now - self.last_detection_time

        # Calcular velocidad solo si dt es razonable (evitar spikes)
        if 0.005 < dt < 0.5:
            self.vx = (x - self._real_x) / dt
            self.vy = (y - self._real_y) / dt

        self.x = x
        self.y = y
        self._real_x = x
        self._real_y = y
        self.angle = angle
        self.last_detection_time = now
        self.is_predicted = False

    def predict(self):
        """Predice posición actual usando velocidad lineal desde última detección real.

        Returns:
            bool: True si la predicción es válida, False si expiró (>MAX_PREDICTION_TIME)
        """
        now = time.time()
        dt = now - self.last_detection_time

        if dt > self.MAX_PREDICTION_TIME:
            return False

        # Predicción lineal SIEMPRE desde la última posición real detectada
        self.x = self._real_x + self.vx * dt
        self.y = self._real_y + self.vy * dt
        # Ángulo se mantiene (no predecimos rotación)
        self.is_predicted = True
        return True


def _update_pid_controller(controller, pid_params):
    """Actualiza los parámetros PID del controlador.

    Args:
        controller: DifferentialDriveController instance
        pid_params: Dict con parámetros PID
    """
    controller.kp_pos = pid_params['kp_pos']
    controller.ki_pos = pid_params['ki_pos']
    controller.kd_pos = pid_params['kd_pos']
    controller.kp_angle = pid_params['kp_angle']
    controller.ki_angle = pid_params['ki_angle']
    controller.kd_angle = pid_params['kd_angle']
    controller.position_threshold = pid_params['position_threshold']
    controller.angle_threshold = pid_params['angle_threshold']


def _save_pid_to_config(pid_params):
    """Guarda parámetros PID actuales a config.py.

    Args:
        pid_params: Dict con parámetros PID
    """
    config_file = ROOT_DIR / "src" / "robot_soccer" / "config.py"

    # Leer archivo
    with open(config_file, 'r') as f:
        lines = f.readlines()

    # Actualizar líneas con nuevos valores
    updates = {
        'PID_POSITION_KP': pid_params['kp_pos'],
        'PID_POSITION_KI': pid_params['ki_pos'],
        'PID_POSITION_KD': pid_params['kd_pos'],
        'PID_ANGLE_KP': pid_params['kp_angle'],
        'PID_ANGLE_KI': pid_params['ki_angle'],
        'PID_ANGLE_KD': pid_params['kd_angle'],
    }

    for i, line in enumerate(lines):
        for key, value in updates.items():
            if line.strip().startswith(key + ' ='):
                lines[i] = f"{key} = {value}\n"

    # Escribir archivo
    with open(config_file, 'w') as f:
        f.writelines(lines)

    log.info("✅ Parámetros PID guardados a config.py")


def _load_default_pid():
    """Carga valores PID por defecto desde config.py.

    Returns:
        Dict con parámetros PID por defecto
    """
    return {
        'kp_pos': PID_POSITION_KP,
        'ki_pos': PID_POSITION_KI,
        'kd_pos': PID_POSITION_KD,
        'kp_angle': PID_ANGLE_KP,
        'ki_angle': PID_ANGLE_KI,
        'kd_angle': PID_ANGLE_KD,
        'position_threshold': ROBOT_POSITION_THRESHOLD,
        'angle_threshold': math.radians(ROBOT_ANGLE_THRESHOLD_DEG),  # Convertir a radianes
    }


def control_loop_pid(robot_positions_pipe, control_state_pipe, keyboard_pipe, robot_id, serial_port):
    """Bucle principal del proceso de control PID PURO (sin visualización).

    Args:
        robot_positions_pipe: Pipe para recibir posiciones desde percepción
        control_state_pipe: Pipe para enviar estado a visualización
        keyboard_pipe: Pipe para recibir comandos desde visualización
        robot_id: ID del robot a controlar
        serial_port: Puerto serial para comunicación RF

    Recibe por robot_positions_pipe (desde Percepción):
        {
            'robot_detected': bool,
            'robot_data': {'x': int, 'y': int, 'angulo': float} or None,
            'stats': {...},
            'timestamp': float
        }

    Recibe por keyboard_pipe (desde Visualización):
        {
            'command': str,  # 'adjust_pid', 'set_waypoint', 'save_pid', etc.
            'param': str or None,  # 'kp_pos', 'ki_angle', etc.
            'delta': float or None,
            'waypoint': [x, y] or None,
            'timestamp': float
        }

    Envía por control_state_pipe (a Visualización):
        {
            'pid_params': {...},
            'target_waypoint': [x, y] or None,
            'movement_active': bool,
            'robot_id': int,
            'timestamp': float
        }
    """
    log.info(f"🎮 Proceso de control PID PURO iniciado (Robot ID {robot_id})")
    log.info("   Modo: 100-200 Hz sin visualización")
    log.info("   Responsabilidades:")
    log.info("     ✓ Recibir posiciones de percepción")
    log.info("     ✓ Recibir comandos de visualización")
    log.info("     ✓ Ejecutar PID para alcanzar waypoints")
    log.info("     ✓ Enviar comandos RF al robot")
    log.info("     ✓ Enviar estado a visualización")

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
    pid_params = _load_default_pid()

    # Controlador
    controller = DifferentialDriveController(rf_controller=rf_controller)
    _update_pid_controller(controller, pid_params)

    # Estado del control
    robot = None
    target_waypoint = None
    movement_active = False
    running = True

    # Flag para saber si ya enviamos comando de detención
    robot_stopped = False

    # Contador de iteraciones para estadísticas
    iteration_count = 0
    start_time = time.time()
    last_stats_time = start_time

    log.info("✅ Control PID iniciado - Esperando datos...")

    try:
        while running:
            iteration_count += 1

            # ===== RECIBIR DATOS DE PERCEPCIÓN =====
            robot_detected = False
            robot_data = None

            if robot_positions_pipe.poll():
                try:
                    perception_data = robot_positions_pipe.recv()
                    robot_detected = perception_data.get('robot_detected', False)
                    robot_data = perception_data.get('robot_data', None)

                    # Actualizar entidad de robot si detectado
                    if robot_detected and robot_data:
                        if robot is None:
                            robot = RobotEntity(
                                robot_id,
                                robot_data['x'],
                                robot_data['y'],
                                math.radians(robot_data['angulo'])
                            )
                            log.info(f"🤖 Robot {robot_id} detectado en ({robot_data['x']}, {robot_data['y']})")
                        else:
                            robot.update(
                                robot_data['x'],
                                robot_data['y'],
                                math.radians(robot_data['angulo'])
                            )
                    # Robot no detectado: usar predicción lineal si existe
                    elif robot is not None:
                        if not robot.predict():
                            # Predicción expiró (>0.5s sin detección)
                            log.warning(f"⚠️  Robot {robot_id} perdido (predicción expirada)")
                            robot = None

                except Exception as e:
                    log.error(f"Error recibiendo posiciones: {e}")

            # Si no hay datos nuevos de percepción, seguir prediciendo
            elif robot is not None and robot.is_predicted:
                if not robot.predict():
                    log.warning(f"⚠️  Robot {robot_id} perdido (predicción expirada)")
                    robot = None

            # ===== RECIBIR COMANDOS DESDE VISUALIZACIÓN =====
            if keyboard_pipe.poll():
                try:
                    command_data = keyboard_pipe.recv()
                    command = command_data.get('command')

                    if command == 'adjust_pid':
                        # Ajustar parámetro PID
                        param = command_data.get('param')
                        delta = command_data.get('delta', 0.0)
                        if param in pid_params:
                            pid_params[param] = max(0.0, pid_params[param] + delta)
                            _update_pid_controller(controller, pid_params)
                            log.debug(f"PID {param} ajustado a {pid_params[param]:.5f}")

                    elif command == 'set_waypoint':
                        # Establecer waypoint
                        waypoint = command_data.get('waypoint')
                        if waypoint and robot:
                            target_waypoint = waypoint
                            movement_active = True
                            log.info(f"🎯 Waypoint establecido: ({waypoint[0]}, {waypoint[1]})")

                    elif command == 'toggle_movement':
                        # Toggle movimiento
                        if robot and target_waypoint:
                            movement_active = not movement_active
                            log.info(f"🔄 Movimiento: {'ACTIVO' if movement_active else 'DETENIDO'}")

                    elif command == 'save_pid':
                        # Guardar PID a config.py
                        _save_pid_to_config(pid_params)

                    elif command == 'reset_pid':
                        # Reset PID a valores por defecto
                        pid_params = _load_default_pid()
                        _update_pid_controller(controller, pid_params)
                        log.info("🔄 PID reseteado a valores por defecto")

                    elif command == 'exit':
                        # Salir del bucle
                        log.info("🛑 Comando de salida recibido")
                        running = False

                except Exception as e:
                    log.error(f"Error procesando comando: {e}")

            # ===== EJECUTAR CONTROL PID =====
            if movement_active and robot and robot_available and target_waypoint:
                try:
                    reached = controller.move_to_position(robot, tuple(target_waypoint))

                    if reached:
                        log.info(f"✅ Waypoint alcanzado: {target_waypoint}")
                        movement_active = False
                        robot_stopped = False

                except Exception as e:
                    log.error(f"Error en movimiento: {e}")

            # Detener robot si movimiento no está activo
            elif robot and robot_available and not robot_stopped:
                firmware_id = robot_id + 1
                rf_controller.set_motors(firmware_id, 0, 0)
                robot_stopped = True

            # ===== ENVIAR ESTADO A VISUALIZACIÓN =====
            try:
                # Vaciar pipe si tiene datos viejos
                if control_state_pipe.poll():
                    _ = control_state_pipe.recv()

                # Enviar estado actual
                control_state_pipe.send({
                    'pid_params': pid_params.copy(),
                    'target_waypoint': target_waypoint,
                    'movement_active': movement_active,
                    'robot_id': robot_id,
                    'robot_predicted': robot.is_predicted if robot else False,
                    'timestamp': time.time()
                })
            except Exception:
                # Continuar aunque falle el envío
                pass

            # ===== ESTADÍSTICAS DE RENDIMIENTO =====
            current_time = time.time()
            if current_time - last_stats_time >= 5.0:
                elapsed = current_time - last_stats_time
                control_hz = iteration_count / elapsed
                log.info(f"📊 Control: {control_hz:.1f} Hz (objetivo: 100-200 Hz)")
                iteration_count = 0
                last_stats_time = current_time

            # Pausa mínima para permitir ~100-200 Hz
            # No usar sleep - el polling de pipes ya introduce suficiente delay
            # time.sleep(0.001)  # Removido para máximo rendimiento

    except KeyboardInterrupt:
        log.info("⏹️  Proceso de control detenido por usuario")
    except Exception as e:
        log.error(f"❌ Error en proceso de control: {e}", exc_info=True)
    finally:
        # Detener robot
        if rf_controller and robot_available:
            firmware_id = robot_id + 1
            rf_controller.set_motors(firmware_id, 0, 0)
            log.info("🛑 Robot detenido")

        # Cerrar RF controller
        if rf_controller:
            rf_controller.shutdown()
            log.info("🔌 RF controller cerrado")
