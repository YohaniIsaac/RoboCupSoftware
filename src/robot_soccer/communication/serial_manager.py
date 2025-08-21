
"""Gestión de comunicación serial con Arduino para control de robots.

Este módulo proporciona la clase SerialManager que maneja la comunicación
serial con un Arduino para enviar comandos a robots de fútbol mediante
radiofrecuencia.
"""
import logging
import time
import threading
import serial

log = logging.getLogger(__name__)


class SerialManager:
    """Gestor de comunicación serial con Arduino para control de robots.

    Esta clase maneja la comunicación bidireccional con un Arduino conectado
    via puerto serial. Utiliza una cola de comandos con threading para
    permitir operaciones no-bloqueantes y procesamiento concurrente.

    Attributes:
        port (str): Puerto serial del dispositivo Arduino.
        baud_rate (int): Velocidad de comunicación en baudios.
        timeout (int): Tiempo límite para operaciones de lectura.
        serial (serial.Serial): Objeto de conexión serial.
        is_connected (bool): Estado de la conexión serial.
        command_queue (list): Cola de comandos pendientes de envío.
        lock (threading.Lock): Mutex para sincronización de hilos.
        worker_thread (threading.Thread): Hilo de trabajo para procesamiento.
        running (bool): Bandera de control para el hilo de trabajo.
    """

    def __init__(self, port='/dev/ttyUSB0', baud_rate=115200, timeout=1):
        """Inicializa el gestor de comunicación serial.

        Args:
            port (str, optional): Puerto serial donde está conectado el Arduino.
                Defaults to '/dev/ttyUSB0'.
            baud_rate (int, optional): Velocidad de comunicación en baudios.
                Defaults to 115200.
            timeout (int, optional): Tiempo de espera para operaciones de
                lectura en segundos. Defaults to 1.
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

    def connect(self):
        """Establece la conexión serial con el Arduino.

        Inicializa la conexión serial, espera la reinicialización del Arduino
        y arranca el hilo de trabajo para procesamiento de comandos.

        Returns:
            bool: True si la conexión fue exitosa, False en caso contrario.

        Raises:
            serial.SerialException: Cuando no se puede establecer la conexión
                serial con el dispositivo especificado.
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=self.timeout
            )
            time.sleep(2)  # Dar tiempo para que Arduino se reinicie
            self.is_connected = True
            log.info("Conectado al puerto %i a %f baudios", self.port, self.baud_rate)

            # Iniciar hilo de trabajo
            self.running = True
            self.worker_thread = threading.Thread(target=self._worker)
            self.worker_thread.daemon = True
            self.worker_thread.start()

            return True
        except serial.SerialException as e:
            log.error("Error al conectar: %s", e)
            self.is_connected = False
            return False

    def disconnect(self):
        """Cierra la conexión serial y detiene el hilo de trabajo.

        Finaliza el hilo de trabajo de forma segura, espera su terminación
        y cierra el puerto serial liberando los recursos asociados.
        """
        self.running = False
        if self.worker_thread:
            self.worker_thread.join(timeout=1)

        if self.serial and self.serial.is_open:
            self.serial.close()
            self.is_connected = False
            log.info("Desconectado del puerto serial")

    def send_command(self, command):
        r"""Envía un comando al Arduino de forma asíncrona.

        Agrega el comando a la cola de envío para ser procesado por el
        hilo de trabajo. Asegura que el comando termine con salto de línea.

        Args:
            command (str): Comando a enviar al Arduino. Se añadirá '\n'
                automáticamente si no lo incluye.

        Returns:
            bool: True si el comando se agregó correctamente a la cola,
                False si no hay conexión activa.

        Note:
            Esta función es thread-safe y no bloquea al llamador.
            El envío real se realiza de forma asíncrona.
        """
        if not self.is_connected:
            log.error("No hay conexión con Arduino")
            return False

        # Asegurar que el comando termine con salto de línea
        if not command.endswith('\n'):
            command += '\n'

        with self.lock:
            self.command_queue.append(command)

        return True

    def _worker(self):
        """Hilo de trabajo para enviar comandos y recibir respuestas.

        Procesa continuamente la cola de comandos, enviando cada comando
        al Arduino y registrando las respuestas recibidas. Maneja errores
        de comunicación y actualiza el estado de conexión según corresponda.

        Note:
            Este método se ejecuta en un hilo separado y no debe ser
            llamado directamente. Es iniciado automáticamente por connect().
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
                log.debug("Enviado: %s", command.strip())

                # Leer respuesta (si la hay)
                response = self.serial.readline().decode('utf-8').strip()
                if response:
                    log.debug("Recibido: %s", response)

            except Exception as e:
                log.error("Error en comunicación: %s", e)
                self.is_connected = False
                break
