
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

# Cambiar a True para loguear cada comando escrito al serial (diagnóstico)
DEBUG_SERIAL_TX = False


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
        self.command_queue = []  # Cola NORMAL de comandos
        self.medium_priority_queue = []  # Cola MEDIA (entrada a rampa)
        self.high_priority_queue = []  # Cola ALTA (detención de emergencia)
        self.lock = threading.Lock()
        self.worker_thread = None
        self.running = False
        self.response_buffer = []  # Buffer para respuestas del Arduino
        # Telemetría/diagnóstico (D1): se clasifican las respuestas YA leídas por el worker
        # (observación, no se envía nada extra). _telem_lines acotado; _telem_err cuenta ERRORs.
        self._telem_lines = []     # [(ts, linea_TELEM)]
        self._telem_err = 0        # nº de ERROR de entrega desde el último drain

        # Límite de tamaño de cola (medida de seguridad)
        # Con "Last Command Wins" las colas no deberían crecer mucho
        # Máximo ~4 robots × 2 comandos pendientes = 8 comandos razonable
        self.MAX_QUEUE_SIZE = 10
        self.queue_overflow_warned = False

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
            log.info("Conectado al puerto %s a %d baudios", self.port, self.baud_rate)

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

    @staticmethod
    def _extract_robot_id(command):
        """Extrae el robot_id de un comando.

        Args:
            command (str): Comando en formato "M,{id},{left},{right}",
                           "D,{id},{power}" o "R{id}{cmd}"

        Returns:
            int or None: ID del robot o None si no se puede extraer
        """
        try:
            if command.startswith('M,'):
                # Formato motor: M,1,50,-50
                return int(command.split(',')[1])
            if command.startswith('D,'):
                # Formato dribbler: D,1,255
                return int(command.split(',')[1])
            if command.startswith('R') and len(command) >= 3:
                # Formato robot discreto: R1F
                return int(command[1])
        except (ValueError, IndexError):
            pass
        return None

    @staticmethod
    def _command_subsystem(command):
        """Clasifica el subsistema de un comando para el 'Last Command Wins'.

        El dedup por robot debe ser POR SUBSISTEMA: un comando de motor (M,) y uno
        de dribbler (D,) del mismo robot son ortogonales (ruedas vs rodillo) y NO
        deben pisarse en la cola. Sin esta distinción, un M,id borra un D,id pendiente
        del mismo robot (y viceversa); durante el movimiento eso descarta el
        re-encendido del rodillo y la pelota se suelta (el motor refresca el watchdog
        compartido del firmware, que mantiene el último PWM del dribbler en 0).

        Args:
            command (str): Comando en formato "M,{id},...", "D,{id},...", "R{id}{cmd}".

        Returns:
            str or None: 'motor', 'dribbler', 'kick', 'discrete', o None (sin robot,
                p.ej. tablero): None recae en el dedup por-robot legacy.
        """
        if command.startswith('M,'):
            return 'motor'
        if command.startswith('D,'):
            return 'dribbler'
        if command.startswith('R') and len(command) >= 3:
            # R{id}P = kick; otros R{id}{F/B/L/R/S/Q} = discreto (no usados en el juego)
            return 'kick' if command[2] == 'P' else 'discrete'
        return None

    def _remove_old_commands_for_robot(self, queue, robot_id, subsystem=None):
        """Elimina comandos viejos del mismo robot (y subsistema, si se indica) de una cola.

        Args:
            queue (list): Cola de comandos a limpiar
            robot_id (int): ID del robot cuyos comandos viejos se eliminarán
            subsystem (str, optional): Si se indica, solo elimina comandos de ESE
                subsistema (motor/dribbler/...). None = todos los del robot (legacy).
                El dedup por (robot, subsistema) evita que un comando de motor borre
                uno de dribbler del mismo robot.

        Returns:
            int: Número de comandos eliminados
        """
        original_len = len(queue)
        if subsystem is None:
            # Legacy: mantener solo comandos de OTROS robots
            queue[:] = [cmd for cmd in queue if self._extract_robot_id(cmd) != robot_id]
        else:
            # Mantener todo salvo los del MISMO (robot, subsistema)
            queue[:] = [
                cmd for cmd in queue
                if not (self._extract_robot_id(cmd) == robot_id
                        and self._command_subsystem(cmd) == subsystem)
            ]
        removed = original_len - len(queue)
        return removed

    def send_command(self, command):
        r"""Envía un comando al Arduino de forma asíncrona.

        Agrega el comando a la cola de envío para ser procesado por el
        hilo de trabajo. Asegura que el comando termine con salto de línea.

        IMPORTANTE: Si ya hay comandos en cola para el mismo robot,
        se eliminan (solo se mantiene el comando más reciente por robot).

        Args:
            command (str): Comando a enviar al Arduino. Se añadirá '\n'
                automáticamente si no lo incluye.

        Returns:
            bool: True si el comando se agregó correctamente a la cola,
                False si no hay conexión activa.

        Note:
            Esta función es thread-safe y no bloquea al llamador.
            El envío real se realiza de forma asíncrona.
            Implementa "Last Command Wins" para evitar queue overflow.
        """
        if not self.is_connected:
            log.error("No hay conexión con Arduino")
            return False

        # Asegurar que el comando termine con salto de línea
        if not command.endswith('\n'):
            command += '\n'

        with self.lock:
            # Extraer robot_id del nuevo comando
            robot_id = self._extract_robot_id(command)

            if robot_id is not None:
                # Last Command Wins POR SUBSISTEMA: un comando de motor no borra uno de
                # dribbler del mismo robot (ortogonales). Antes el dedup era solo por
                # robot y el motor borraba el re-encendido del rodillo.
                subsystem = self._command_subsystem(command)
                removed = self._remove_old_commands_for_robot(
                    self.command_queue, robot_id, subsystem)
                if removed > 0:
                    log.debug("🗑️  Robot %d (%s): %d comandos viejos eliminados (last wins)",
                             robot_id, subsystem, removed)

            # Agregar nuevo comando
            self.command_queue.append(command)

            # Advertencia si la cola crece demasiado (indica problema de rendimiento)
            queue_size = len(self.command_queue)
            total_queued = queue_size + len(self.medium_priority_queue) + len(self.high_priority_queue)
            if queue_size > self.MAX_QUEUE_SIZE and not self.queue_overflow_warned:
                log.warning(
                    "⚠️  Cola saturada: normal=%d media=%d alta=%d (total=%d, límite=%d)",
                    queue_size, len(self.medium_priority_queue),
                    len(self.high_priority_queue), total_queued, self.MAX_QUEUE_SIZE
                )
                self.queue_overflow_warned = True
            elif queue_size <= self.MAX_QUEUE_SIZE // 2:
                self.queue_overflow_warned = False  # Reset warning

        return True

    def send_priority_command(self, command, priority='high'):
        r"""Envía un comando con prioridad al Arduino.

        Sistema de 3 niveles de prioridad:
        - HIGH (alta): Detención de emergencia
          → Limpia TODAS las colas (normal + media)
          → Elimina comandos viejos del mismo robot en cola alta
          → Se procesa INMEDIATAMENTE
        - MEDIUM (media): Entrada a rampa de desaceleración
          → Limpia solo cola normal
          → Elimina comandos viejos del mismo robot en cola media
          → Se procesa antes que comandos normales
        - NORMAL: Movimiento regular (usar send_command())

        Args:
            command (str): Comando a enviar. Se añadirá '\n' automáticamente.
            priority (str): 'high' o 'medium'. Default: 'high'.

        Returns:
            bool: True si el comando se agregó correctamente,
                False si no hay conexión activa.

        Note:
            - HIGH: Solo para detención de emergencia
            - MEDIUM: Para cambios críticos de velocidad (rampa)
            - Implementa "Last Command Wins" en cada cola de prioridad
        """
        if not self.is_connected:
            log.error("No hay conexión con Arduino")
            return False

        # Asegurar que el comando termine con salto de línea
        if not command.endswith('\n'):
            command += '\n'

        # Extraer robot_id y subsistema para el dedup (Last Command Wins por subsistema):
        # un stop/decel de motor no descarta el dribbler del mismo robot en la cola normal.
        robot_id = self._extract_robot_id(command)
        subsystem = self._command_subsystem(command)

        with self.lock:
            if priority == 'high':
                # PRIORIDAD ALTA: limpiar comandos pendientes solo de ESTE robot.
                # No limpiar comandos de otros robots (multi-robot: cada uno
                # gestiona su propia cola independientemente).
                if robot_id is not None:
                    removed_n = self._remove_old_commands_for_robot(self.command_queue, robot_id, subsystem)
                    removed_m = self._remove_old_commands_for_robot(self.medium_priority_queue, robot_id, subsystem)
                    total_cleared = removed_n + removed_m
                    if total_cleared > 0:
                        log.debug("🚨 Robot %d: %d comandos previos descartados (stop)",
                                 robot_id, total_cleared)
                else:
                    # Sin robot_id conocido → limpiar todo (fallback de seguridad)
                    total_cleared = len(self.command_queue) + len(self.medium_priority_queue)
                    if total_cleared > 0:
                        log.debug("🚨 EMERGENCIA (robot_id=None): %d comandos descartados",
                                 total_cleared)
                        self.command_queue.clear()
                        self.medium_priority_queue.clear()

                # Eliminar comandos viejos del MISMO robot y subsistema en cola alta
                if robot_id is not None:
                    removed = self._remove_old_commands_for_robot(self.high_priority_queue, robot_id, subsystem)
                    if removed > 0:
                        log.debug("🚨 Robot %d: %d comandos ALTA viejos eliminados", robot_id, removed)

                # Agregar a cola de alta prioridad
                self.high_priority_queue.append(command)
                log.debug("⚡ ALTA prioridad encolado: %s", command.strip())

            elif priority == 'medium':
                # PRIORIDAD MEDIA: Entrada a rampa (desaceleración)
                # Eliminar comandos del MISMO robot y subsistema en colas normal + media
                if robot_id is not None:
                    removed_normal = self._remove_old_commands_for_robot(self.command_queue, robot_id, subsystem)
                    removed_medium = self._remove_old_commands_for_robot(self.medium_priority_queue, robot_id, subsystem)
                    total_removed = removed_normal + removed_medium
                    if total_removed > 0:
                        log.debug("🔶 Robot %d MEDIA: %d comandos eliminados (N=%d M=%d)",
                                 robot_id, total_removed, removed_normal, removed_medium)

                # Agregar a cola de media prioridad
                self.medium_priority_queue.append(command)
                log.debug("🔶 MEDIA prioridad encolado: %s", command.strip())

            else:
                log.warning("⚠️  Prioridad inválida '%s', usando 'high'", priority)
                # Eliminar comandos viejos en cola alta
                if robot_id is not None:
                    self._remove_old_commands_for_robot(self.high_priority_queue, robot_id)
                self.high_priority_queue.append(command)

        return True

    def send_command_sync(self, command, timeout=3.0, expected_lines=1):
        """Envía un comando y espera la(s) respuesta(s) de forma sincrónica.

        Args:
            command (str): Comando a enviar al Arduino.
            timeout (float): Tiempo máximo de espera en segundos.
            expected_lines (int): Número de líneas de respuesta esperadas.
                Si es None, lee hasta que no haya más datos (útil para ping).

        Returns:
            list: Lista de respuestas recibidas, o lista vacía si hay timeout/error.
        """
        if not self.is_connected:
            log.error("No hay conexión con Arduino")
            return []

        # Limpiar buffer de respuestas previas
        with self.lock:
            self.response_buffer.clear()

        # Enviar comando
        if not self.send_command(command):
            return []

        # Pequeño delay para que el worker thread procese el comando
        time.sleep(0.05)

        # Esperar respuestas
        responses = []
        start_time = time.time()
        no_data_timeout = 0.5  # Timeout de 500ms sin datos nuevos (aumentado para ping)

        if expected_lines is None:
            # Modo: leer hasta que no haya más datos
            last_response_time = time.time()

            while True:
                # Timeout absoluto
                if time.time() - start_time > timeout:
                    log.warning("Timeout absoluto esperando respuesta para comando: %s", command.strip())
                    break

                # Timeout por falta de datos nuevos
                if time.time() - last_response_time > no_data_timeout:
                    break

                with self.lock:
                    if self.response_buffer:
                        responses.append(self.response_buffer.pop(0))
                        last_response_time = time.time()

                time.sleep(0.01)
        else:
            # Modo original: esperar número fijo de líneas
            while len(responses) < expected_lines:
                if time.time() - start_time > timeout:
                    log.warning("Timeout esperando respuesta para comando: %s", command.strip())
                    break

                with self.lock:
                    if self.response_buffer:
                        responses.append(self.response_buffer.pop(0))

                time.sleep(0.01)

        return responses

    def _classify(self, line):
        """Clasifica una respuesta del transmisor para telemetría (D1). LLAMAR BAJO self.lock.

        Observación pura: la línea ya la leyó el worker; solo se mira el prefijo y se guarda
        (TELEM, de baja frecuencia) o se cuenta (ERROR de entrega). O(1), sin re-lock.
        """
        if line.startswith('TELEM'):
            self._telem_lines.append((time.time(), line))
            if len(self._telem_lines) > 128:        # acotar memoria
                del self._telem_lines[0]
        elif 'ERROR' in line:
            self._telem_err += 1

    def drain_telemetry(self):
        """Devuelve y limpia las líneas TELEM acumuladas + el contador de ERROR (D1).

        Lo consume el RFController a baja frecuencia para loguear/volcar la telemetría.

        Returns:
            (list[(ts, str)], int): líneas TELEM con timestamp, y nº de ERROR desde el último drain.
        """
        with self.lock:
            lines = self._telem_lines
            self._telem_lines = []
            err = self._telem_err
            self._telem_err = 0
        return lines, err

    def _worker(self):
        """Hilo de trabajo para enviar comandos y recibir respuestas.

        Procesa continuamente las colas de comandos con sistema de 3 prioridades:
        1. ALTA (high_priority_queue): Detención de emergencia
        2. MEDIA (medium_priority_queue): Entrada a rampa
        3. NORMAL (command_queue): Movimiento regular

        Orden de procesamiento: ALTA → MEDIA → NORMAL

        Note:
            Este método se ejecuta en un hilo separado y no debe ser
            llamado directamente. Es iniciado automáticamente por connect().
        """
        while self.running:
            # Verificar si hay comandos en CUALQUIER cola
            with self.lock:
                has_high = len(self.high_priority_queue) > 0
                has_medium = len(self.medium_priority_queue) > 0
                has_normal = len(self.command_queue) > 0

            if not has_high and not has_medium and not has_normal:
                # Drenar buffer serial mientras idle (evita acumulación
                # de respuestas de motor no leídas)
                if self.serial and self.serial.in_waiting > 0:
                    try:
                        response = self.serial.readline().decode('utf-8').strip()
                        if response:
                            with self.lock:
                                self.response_buffer.append(response)
                                self._classify(response)
                    except Exception:
                        pass
                else:
                    time.sleep(0.002)  # Pausa mínima si no hay datos
                continue

            # Obtener comando según prioridad (ALTA → MEDIA → NORMAL)
            priority_level = 'normal'
            with self.lock:
                if self.high_priority_queue:
                    # PRIORIDAD ALTA: procesar inmediatamente
                    command = self.high_priority_queue.pop(0)
                    priority_level = 'high'
                elif self.medium_priority_queue:
                    # PRIORIDAD MEDIA: procesar antes que normales
                    command = self.medium_priority_queue.pop(0)
                    priority_level = 'medium'
                elif self.command_queue:
                    # PRIORIDAD NORMAL: procesar solo si no hay otros
                    command = self.command_queue.pop(0)
                    priority_level = 'normal'
                else:
                    continue

            try:
                self.serial.write(command.encode('utf-8'))
                if DEBUG_SERIAL_TX:
                    prefix = {'high': '[TX-H]', 'medium': '[TX-M]', 'normal': '[TX]'}.get(
                        priority_level, '[TX]')
                    log.info("%s %s", prefix, command.strip())
                else:
                    if priority_level == 'high':
                        log.debug("⚡ ALTA enviado: %s", command.strip())
                    elif priority_level == 'medium':
                        log.debug("🔶 MEDIA enviado: %s", command.strip())
                    else:
                        log.debug("Enviado: %s", command.strip())

                # Leer respuestas disponibles
                # Motor commands: timeout corto (respuesta llega en <3ms a 115200)
                # Otros commands (ping, etc): timeout largo (RF roundtrips ~50ms)
                # M, (motor) y D, (dribbler) son comandos de robot: el transmisor responde
                # OK de inmediato (radio.write + Serial.print). Solo ping/diagnóstico tienen
                # roundtrips RF lentos (~50ms). Antes D, caía en la rama lenta y cada comando
                # de dribbler bloqueaba el worker hasta 50ms, retrasando el drenado de motores.
                is_fast_robot_command = command.strip().startswith(('M,', 'D,'))
                max_wait_time = 0.1 if is_fast_robot_command else 0.5
                start_read_time = time.time()
                no_data_timeout = 0.005 if is_fast_robot_command else 0.05

                last_response_time = time.time()

                while True:
                    # Timeout absoluto
                    if time.time() - start_read_time > max_wait_time:
                        break

                    # Timeout por falta de datos (si no hay respuestas en 50ms, asumimos que terminó)
                    if time.time() - last_response_time > no_data_timeout:
                        break

                    # Verificar si hay datos disponibles
                    if self.serial.in_waiting > 0:
                        try:
                            response = self.serial.readline().decode('utf-8').strip()
                            if response:
                                log.debug("Recibido: %s", response)
                                with self.lock:
                                    self.response_buffer.append(response)
                                    self._classify(response)
                                last_response_time = time.time()
                        except Exception as read_error:
                            log.warning("Error leyendo respuesta: %s", read_error)
                            break
                    else:
                        time.sleep(0.001)

            except Exception as e:
                log.error("Error en comunicación: %s", e)
                self.is_connected = False
                break
