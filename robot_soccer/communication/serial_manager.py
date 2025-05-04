"""
Gestionar la comunicación serial con el Arduino
"""
import serial
import time
import threading
from robot_soccer.utils.logger import get_logger


class SerialManager:
    """
    Gestor de comunicación serial con Arduino.
    Permite enviar comandos a los robots mediante RF.
    """

    def __init__(self, port='/dev/ttyUSB0', baud_rate=115200, timeout=1):
        """
        Inicializa la comunicación serial con el Arduino.

        Args:
            port: Puerto serial donde está conectado el Arduino
            baud_rate: Velocidad de comunicación
            timeout: Tiempo de espera para operaciones de lectura
        """
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial = None
        self.is_connected = False
        self.command_queue = []
        self.lock = threading.Lock()
        self.worker_thread = None
        self.running = False

        self.logger = get_logger("communication.serial_manager")

    def connect(self):
        """
        Establece la conexión serial con el Arduino.

        Returns:
            bool: True si la conexión fue exitosa
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=self.timeout
            )
            time.sleep(2)  # Dar tiempo para que Arduino se reinicie
            self.is_connected = True
            self.logger.info(f"Conectado al puerto {self.port} a {self.baud_rate} baudios")

            # Iniciar hilo de trabajo
            self.running = True
            self.worker_thread = threading.Thread(target=self._worker)
            self.worker_thread.daemon = True
            self.worker_thread.start()

            return True
        except serial.SerialException as e:
            self.logger.error(f"Error al conectar: {e}")
            self.is_connected = False
            return False

    def disconnect(self):
        """
        Cierra la conexión serial.
        """
        self.running = False
        if self.worker_thread:
            self.worker_thread.join(timeout=1)

        if self.serial and self.serial.is_open:
            self.serial.close()
            self.is_connected = False
            self.logger.info("Desconectado del puerto serial")

    def send_command(self, command):
        """
        Envía un comando al Arduino.

        Args:
            command: Comando a enviar (debe terminar con '\n')

        Returns:
            bool: True si el comando se envió correctamente
        """
        if not self.is_connected:
            self.logger.error("No hay conexión con Arduino")
            return False

        # Asegurar que el comando termine con salto de línea
        if not command.endswith('\n'):
            command += '\n'

        with self.lock:
            self.command_queue.append(command)

        return True

    def _worker(self):
        """
        Hilo de trabajo para enviar comandos y recibir respuestas.
        """
        while self.running:
            if not self.command_queue:
                time.sleep(0.01)  # Pequeña pausa si no hay comandos
                continue

            with self.lock:
                if self.command_queue:
                    command = self.command_queue.pop(0)
                else:
                    continue

            try:
                self.serial.write(command.encode('utf-8'))
                self.logger.debug(f"Enviado: {command.strip()}")

                # Leer respuesta (si la hay)
                response = self.serial.readline().decode('utf-8').strip()
                if response:
                    self.logger.debug(f"Recibido: {response}")

            except Exception as e:
                self.logger.error(f"Error en comunicación: {e}")
                self.is_connected = False
                break
