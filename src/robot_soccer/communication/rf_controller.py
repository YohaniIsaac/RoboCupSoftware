"""Controlador de radiofrecuencia para robots de fútbol.

Este módulo integra el protocolo de comandos y la comunicación serial
para proporcionar una interfaz de alto nivel para controlar robots
de fútbol mediante comunicación por radiofrecuencia a través de Arduino.
"""

import logging
from .serial_manager import SerialManager
from .command_protocol import RobotCommandProtocol

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

    def __init__(self, port="/dev/ttyUSB0"):
        """Inicializa el controlador RF.

        Args:
            port (str): Puerto serial donde está conectado el Arduino.
                Defaults to '/dev/ttyUSB0'.
        """
        self.serial_manager = SerialManager(port=port)
        self.protocol = RobotCommandProtocol()

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

        Args:
            robot_id (int): ID del robot (1-4).
            left_speed (float): Velocidad del motor izquierdo (-1.0 a 1.0).
            right_speed (float): Velocidad del motor derecho (-1.0 a 1.0).

        Returns:
            bool: True si el comando se envió correctamente, False en caso contrario.
        """
        # Convertir velocidades normalizadas (-1 a 1) a valores para Arduino (-255 a 255)
        left_val = int(left_speed * 255)
        right_val = int(right_speed * 255)

        # Generar comando
        command = self.protocol.format_motor_command(robot_id, left_val, right_val)

        # Enviar comando
        success = self.serial_manager.send_command(command)
        if success:
            log.debug(
                "Robot %i: Motores ajustados a L=%.2f, R=%.2f", robot_id, left_speed, right_speed
            )

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
