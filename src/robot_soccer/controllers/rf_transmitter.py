"""Controlador RF para enviar comandos a los robots y al tablero.

Este módulo maneja la comunicación serial con el Arduino transmisor,
que a su vez envía comandos por RF a los robots y al tablero.
"""

import time
from enum import Enum
from typing import Optional

import serial


class RobotCommand(Enum):
    """Comandos disponibles para los robots."""
    FORWARD = 'F'
    BACKWARD = 'B'
    LEFT = 'L'
    RIGHT = 'R'
    KICK = 'P'
    ROLLER_ON = 'D'
    ROLLER_OFF = 'S'
    POWER_OFF = 'Q'


class TableroCommand(Enum):
    """Comandos disponibles para el tablero."""
    TOGGLE_PAUSE = 1
    GOAL_TEAM1 = 2
    GOAL_TEAM2 = 3
    RESET_GOALS = 4
    RESET_TIME = 5


class RFTransmitter:
    """Controlador para el transmisor RF.

    Comunica con el Arduino transmisor vía serial para enviar
    comandos RF a los robots y al tablero.
    """

    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 115200):
        """Inicializa el transmisor RF.

        Args:
            port: Puerto serial del Arduino transmisor
            baudrate: Velocidad de comunicación serial (default: 115200)
        """
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.connected = False

    def connect(self) -> bool:
        """Establece conexión con el transmisor.

        Returns:
            True si la conexión fue exitosa, False en caso contrario
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            time.sleep(2)  # Esperar a que Arduino se reinicie
            self.connected = True
            print(f"Conectado al transmisor en {self.port}")
            return True
        except serial.SerialException as e:
            print(f"Error al conectar: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """Cierra la conexión con el transmisor."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.connected = False
            print("Desconectado del transmisor")

    def send_robot_command(self, robot_id: int, command: RobotCommand) -> bool:
        """Envía un comando a un robot específico.

        Args:
            robot_id: ID del robot (1-4)
            command: Comando a enviar

        Returns:
            True si el comando se envió correctamente, False en caso contrario
        """
        if not self.connected or not self.serial:
            print("Error: No conectado al transmisor")
            return False

        if robot_id < 1 or robot_id > 4:
            print(f"Error: ID de robot inválido: {robot_id}")
            return False

        try:
            # Formato: R[ID][CMD]\n
            message = f"R{robot_id}{command.value}\n"
            self.serial.write(message.encode())
            self.serial.flush()

            # Leer respuesta
            response = self.serial.readline().decode().strip()
            if response.startswith("OK"):
                return True
            print(f"Error del transmisor: {response}")
            return False

        except Exception as e:
            print(f"Error al enviar comando: {e}")
            return False

    def send_tablero_command(self, command: TableroCommand) -> bool:
        """Envía un comando al tablero.

        Args:
            command: Comando a enviar

        Returns:
            True si el comando se envió correctamente, False en caso contrario
        """
        if not self.connected or not self.serial:
            print("Error: No conectado al transmisor")
            return False

        try:
            # Formato: T[CMD]\n
            message = f"T{command.value}\n"
            self.serial.write(message.encode())
            self.serial.flush()

            # Leer respuesta
            response = self.serial.readline().decode().strip()
            if response.startswith("OK"):
                return True
            print(f"Error del transmisor: {response}")
            return False

        except Exception as e:
            print(f"Error al enviar comando: {e}")
            return False

    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()


# Funciones de conveniencia
def move_robot(transmitter: RFTransmitter, robot_id: int, command: RobotCommand):
    """Mueve un robot con el comando especificado."""
    transmitter.send_robot_command(robot_id, command)


def kick_ball(transmitter: RFTransmitter, robot_id: int):
    """Hace que un robot patee la pelota."""
    transmitter.send_robot_command(robot_id, RobotCommand.KICK)


def toggle_roller(transmitter: RFTransmitter, robot_id: int, enable: bool):
    """Activa o desactiva el rodillo de un robot."""
    cmd = RobotCommand.ROLLER_ON if enable else RobotCommand.ROLLER_OFF
    transmitter.send_robot_command(robot_id, cmd)


def register_goal(transmitter: RFTransmitter, team: int):
    """Registra un gol en el tablero."""
    if team == 1:
        transmitter.send_tablero_command(TableroCommand.GOAL_TEAM1)
    elif team == 2:
        transmitter.send_tablero_command(TableroCommand.GOAL_TEAM2)
