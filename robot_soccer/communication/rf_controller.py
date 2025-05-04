"""
Integrará el protocolo y la comunicación serial
"""

from robot_soccer.utils.logger import get_logger
from .serial_manager import SerialManager
from .command_protocol import RobotCommandProtocol


class RFController:
    """
    Controlador para la comunicación por radiofrecuencia.
    Integra el gestor serial y el protocolo de comandos.
    """

    def __init__(self, port='/dev/ttyUSB0'):
        """
        Inicializa el controlador RF.

        Args:
            port: Puerto serial donde está conectado el Arduino
        """
        self.serial_manager = SerialManager(port=port)
        self.protocol = RobotCommandProtocol()
        self.logger = get_logger("communication.rf_controller")

    def initialize(self):
        """
        Inicializa la comunicación con el Arduino.

        Returns:
            bool: True si la inicialización fue exitosa
        """
        return self.serial_manager.connect()

    def shutdown(self):
        """
        Cierra la comunicación con el Arduino.
        """
        self.serial_manager.disconnect()

    def set_motors(self, robot_id, left_speed, right_speed):
        """
        Establece las velocidades de los motores de un robot.

        Args:
            robot_id: ID del robot (1-4)
            left_speed: Velocidad del motor izquierdo (-1 a 1)
            right_speed: Velocidad del motor derecho (-1 a 1)

        Returns:
            bool: True si el comando se envió correctamente
        """
        # Convertir velocidades normalizadas (-1 a 1) a valores para Arduino (-255 a 255)
        left_val = int(left_speed * 255)
        right_val = int(right_speed * 255)

        # Generar comando
        command = self.protocol.format_motor_command(robot_id, left_val, right_val)

        # Enviar comando
        success = self.serial_manager.send_command(command)
        if success:
            self.logger.debug(f"Robot {robot_id}: Motores ajustados a L={left_speed:.2f}, R={right_speed:.2f}")

        return success

    def kick(self, robot_id, power=1.0):
        """
        Activa el mecanismo de pateo de un robot.

        Args:
            robot_id: ID del robot (1-4)
            power: Potencia del pateo (0-1)

        Returns:
            bool: True si el comando se envió correctamente
        """
        # Convertir potencia normalizada (0-1) a valor para Arduino (0-255)
        power_val = int(power * 255)

        # Generar comando
        command = self.protocol.format_kick_command(robot_id, power_val)

        # Enviar comando
        success = self.serial_manager.send_command(command)
        if success:
            self.logger.debug(f"Robot {robot_id}: Pateo activado con potencia {power:.2f}")

        return success

    def set_dribbler(self, robot_id, power=1.0):
        """
        Establece la potencia del dribbler de un robot.

        Args:
            robot_id: ID del robot (1-4)
            power: Potencia del dribbler (0-1)

        Returns:
            bool: True si el comando se envió correctamente
        """
        # Convertir potencia normalizada (0-1) a valor para Arduino (0-255)
        power_val = int(power * 255)

        # Generar comando
        command = self.protocol.format_dribbler_command(robot_id, power_val)

        # Enviar comando
        success = self.serial_manager.send_command(command)
        if success:
            self.logger.debug(f"Robot {robot_id}: Dribbler ajustado a {power:.2f}")

        return success

    def stop_robot(self, robot_id):
        """
        Detiene todos los motores de un robot.

        Args:
            robot_id: ID del robot (1-4)

        Returns:
            bool: True si el comando se envió correctamente
        """
        # Generar comando
        command = self.protocol.format_stop_command(robot_id)

        # Enviar comando
        success = self.serial_manager.send_command(command)
        if success:
            self.logger.debug(f"Robot {robot_id}: Detenido")

        return success
