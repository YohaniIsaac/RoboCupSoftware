"""Controlador de radiofrecuencia para robots de fútbol.

Este módulo integra el protocolo de comandos y la comunicación serial
para proporcionar una interfaz de alto nivel para controlar robots
de fútbol mediante comunicación por radiofrecuencia a través de Arduino.
"""

import logging
import sys
import time
import math
from pathlib import Path

# Agregar path para importar módulos del proyecto
sys.path.insert(0, str(Path(__file__).parent.parent))

# pylint: disable=wrong-import-position
from .serial_manager import SerialManager
from .command_protocol import RobotCommandProtocol
from ..controllers.robot_calibration import get_calibration_manager

log = logging.getLogger(__name__)


class RFController:
    """Controlador para la comunicación por radiofrecuencia con robots de fútbol.

    Esta clase integra el gestor serial y el protocolo de comandos para
    proporcionar una interfaz unificada para enviar comandos a los robots
    a través de un Arduino conectado por puerto serial.

    Attributes:
        serial_manager (SerialManager): Gestor de comunicación serial.
        protocol (RobotCommandProtocol): Protocolo de formateo de comandos.
        logger (Logger): Logger para registrar eventos y errores.
    """

    def __init__(self, port="/dev/ttyUSB0", enable_calibration=True):
        """Inicializa el controlador RF.

        Args:
            port (str): Puerto serial donde está conectado el Arduino.
                Defaults to '/dev/ttyUSB0'.
            enable_calibration (bool): Si es True, aplica calibración individual por robot.
                Defaults to True.
        """
        self.serial_manager = SerialManager(port=port)
        self.protocol = RobotCommandProtocol()
        self.enable_calibration = enable_calibration

        # Cargar gestor de calibración
        if self.enable_calibration:
            self.calibration = get_calibration_manager()
            log.info("Calibración de motores habilitada")
        else:
            self.calibration = None
            log.info("Calibración de motores deshabilitada")

        # Guardar último comando por robot para evitar logs repetidos
        self.last_commands = {}  # {robot_id: (left_val, right_val)}

        # Rate limiter: Mínimo 15ms entre comandos al mismo robot
        # (firmware tiene delay(10), más overhead de procesamiento)
        self.last_send_time = {}  # {robot_id: timestamp}
        self.MIN_COMMAND_INTERVAL = 0.015  # 15ms mínimo entre comandos

        # Detección de zona de rampa para prioridad media
        self.last_speed_magnitude = {}  # {robot_id: magnitude}
        self.RAMP_DECELERATION_THRESHOLD = 0.15  # Reducción >15% indica entrada a rampa

    def initialize(self):
        """Inicializa la comunicación con el Arduino.

        Establece la conexión serial y prepara el sistema para
        enviar comandos a los robots.

        Returns:
            bool: True si la inicialización fue exitosa, False en caso contrario.
        """
        return self.serial_manager.connect()

    def shutdown(self):
        """Cierra la comunicación con el Arduino.

        Desconecta el puerto serial y libera los recursos asociados.
        """
        self.serial_manager.disconnect()

    def set_motors(self, robot_id, left_speed, right_speed):
        """Establece las velocidades de los motores de un robot.

        Convierte las velocidades normalizadas a valores compatibles con
        Arduino y envía el comando correspondiente.

        Implementa rate limiting para evitar saturar el buffer serial:
        - Solo envía si han pasado MIN_COMMAND_INTERVAL (15ms) desde el último comando
        - O si es un comando de detención (prioridad alta)
        - O si el cambio es significativo (>= 5 en PWM)

        Args:
            robot_id (int): ID del robot (1-4).
            left_speed (float): Velocidad del motor izquierdo (-1.0 a 1.0).
            right_speed (float): Velocidad del motor derecho (-1.0 a 1.0).

        Returns:
            bool: True si el comando se envió correctamente, False en caso contrario.
        """
        # Convertir velocidades normalizadas (-1 a 1) a valores para Arduino (-127 a 127)
        # Limitado a este rango debido a la conversión uint8_t en el transmisor RF
        left_val = int(left_speed * 127)
        right_val = int(right_speed * 127)

        # Aplicar calibración individual si está habilitada
        if self.enable_calibration and self.calibration:
            left_val_calibrated, right_val_calibrated = self.calibration.apply_calibration(
                robot_id, left_val, right_val
            )
            left_val = left_val_calibrated
            right_val = right_val_calibrated

        # ===== RATE LIMITING =====
        # Verificar si debemos enviar este comando
        current_time = time.time()
        last_cmd = self.last_commands.get(robot_id, (None, None))
        last_time = self.last_send_time.get(robot_id, 0)

        # Condiciones para ENVIAR:
        is_stop_command = (left_val == 0 and right_val == 0)  # Detención tiene prioridad
        is_significant_change = (
            last_cmd[0] is None or
            abs(left_val - last_cmd[0]) >= 5 or
            abs(right_val - last_cmd[1]) >= 5
        )
        enough_time_passed = (current_time - last_time) >= self.MIN_COMMAND_INTERVAL

        # Solo enviar si se cumple alguna condición
        if not (is_stop_command or is_significant_change or enough_time_passed):
            return True  # Ignorar comando (demasiado pronto y cambio pequeño)

        # ===== LOGGING =====
        # Solo loguear cuando se DETIENE el robot
        if is_stop_command and last_cmd != (0, 0):
            log.info("⏹️  Robot %i DETENIDO", robot_id)

        # Actualizar comandos y tiempos
        self.last_commands[robot_id] = (left_val, right_val)
        self.last_send_time[robot_id] = current_time

        # Generar comando
        command = self.protocol.format_motor_command(robot_id, left_val, right_val)

        # ===== SISTEMA DE 3 PRIORIDADES =====
        # Calcular magnitud de velocidad (para detectar desaceleración)
        current_magnitude = math.sqrt(left_val**2 + right_val**2) / 127.0  # Normalizar a 0-1
        last_magnitude = self.last_speed_magnitude.get(robot_id, current_magnitude)

        # Detectar desaceleración significativa (entrada a rampa)
        is_decelerating = (last_magnitude - current_magnitude) > self.RAMP_DECELERATION_THRESHOLD

        # Actualizar magnitud para próxima iteración
        self.last_speed_magnitude[robot_id] = current_magnitude

        # PRIORIDAD ALTA: Detención (0, 0)
        if is_stop_command:
            success = self.serial_manager.send_priority_command(command, priority='high')
            log.info("🚨 ALTA prioridad (detención) para Robot %i", robot_id)

        # PRIORIDAD MEDIA: Entrada a rampa (desaceleración >15%)
        elif is_decelerating and not is_stop_command:
            success = self.serial_manager.send_priority_command(command, priority='medium')
            log.info("🔶 MEDIA prioridad (rampa) para Robot %i: mag %.2f → %.2f (Δ=%.2f%%)",
                     robot_id, last_magnitude, current_magnitude,
                     (last_magnitude - current_magnitude) * 100)

        # PRIORIDAD NORMAL: Movimiento regular
        else:
            success = self.serial_manager.send_command(command)

        return success

    def kick(self, robot_id, power=1.0):
        """Activa el mecanismo de pateo de un robot.

        Convierte la potencia normalizada a valores compatibles con Arduino
        y envía el comando de pateo.

        Args:
            robot_id (int): ID del robot (1-4).
            power (float): Potencia del pateo (0.0-1.0). Defaults to 1.0.

        Returns:
            bool: True si el comando se envió correctamente, False en caso contrario.
        """
        # Convertir potencia normalizada (0-1) a valor para Arduino (0-255)
        power_val = int(power * 255)

        # Generar comando
        command = self.protocol.format_kick_command(robot_id, power_val)

        # Enviar comando
        success = self.serial_manager.send_command(command)
        if success:
            log.debug("Robot %i: Pateo activado con potencia %.2f", robot_id, power)

        return success

    def set_dribbler(self, robot_id, power=1.0):
        """Establece la potencia del dribbler de un robot.

        El dribbler es el mecanismo que permite al robot mantener
        control sobre la pelota durante el movimiento.

        Args:
            robot_id (int): ID del robot (1-4).
            power (float): Potencia del dribbler (0.0-1.0). Defaults to 1.0.

        Returns:
            bool: True si el comando se envió correctamente, False en caso contrario.
        """
        # Convertir potencia normalizada (0-1) a valor para Arduino (0-255)
        power_val = int(power * 255)

        # Generar comando
        command = self.protocol.format_dribbler_command(robot_id, power_val)

        # Enviar comando
        success = self.serial_manager.send_command(command)
        if success:
            log.debug("Robot %i: Dribbler ajustado a %.2f", robot_id, power)

        return success

    def stop_robot(self, robot_id):
        """Detiene todos los motores de un robot.

        Envía un comando de parada que detiene inmediatamente todos
        los sistemas de movimiento del robot especificado.

        Args:
            robot_id (int): ID del robot (1-4).

        Returns:
            bool: True si el comando se envió correctamente, False en caso contrario.
        """
        # Generar comando
        command = self.protocol.format_stop_command(robot_id)

        # Enviar comando
        success = self.serial_manager.send_command(command)
        if success:
            log.debug("Robot %i: Detenido", robot_id)

        return success

    def test_connections(self):
        """Prueba la conexión RF con todos los dispositivos.

        Envía un comando 'ping' al transmisor que verifica la conexión
        con el tablero y todos los robots.

        Returns:
            dict: Diccionario con el estado de cada dispositivo.
                  Formato: {'tablero': bool, 'robot_1': bool, ...}
        """
        log.info("Probando conexiones RF...")

        # Enviar comando ping (el transmisor responde con ~11 líneas)
        # Usamos expected_lines=None para leer todo lo disponible
        responses = self.serial_manager.send_command_sync("ping", timeout=5.0, expected_lines=None)

        # Debug: mostrar respuestas recibidas
        log.debug("Ping recibió %d respuestas:", len(responses))
        for i, resp in enumerate(responses):
            log.debug("  [%d] %s", i, resp)

        # Parsear respuestas
        connections = {
            'tablero': False,
            'robot_1': False,
            'robot_2': False,
            'robot_3': False,
            'robot_4': False
        }

        # El firmware envía "✓ Robot X responded!" o "✗ Robot X NO RESPONSE"
        for response in responses:
            if 'Tablero' in response and 'responded' in response:
                connections['tablero'] = True
            elif 'Robot 1' in response and 'responded' in response:
                connections['robot_1'] = True
            elif 'Robot 2' in response and 'responded' in response:
                connections['robot_2'] = True
            elif 'Robot 3' in response and 'responded' in response:
                connections['robot_3'] = True
            elif 'Robot 4' in response and 'responded' in response:
                connections['robot_4'] = True

        return connections
