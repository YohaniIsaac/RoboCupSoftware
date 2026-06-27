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
from ..controllers.robot_calibration_multipoint import (
    get_calibration_manager_multipoint as get_calibration_manager
)
from ..config import DRIBBLER_MAX_PWM, is_dribbler_enabled

log = logging.getLogger(__name__)

# ===== DEBUG: Control de logging de rate limiting =====
# Cambiar a True para activar los logs de comandos ignorados/enviados
DEBUG_RATE_LIMITING = False


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

    def __init__(self, port="/dev/ttyUSB0", enable_calibration=True, min_command_interval=0.015):
        """Inicializa el controlador RF.

        Args:
            port (str): Puerto serial donde está conectado el Arduino.
                Defaults to '/dev/ttyUSB0'.
            enable_calibration (bool): Si es True, aplica calibración individual por robot.
                Defaults to True.
            min_command_interval (float): Tiempo mínimo en segundos entre comandos al mismo robot.
                Defaults to 0.015 (15ms). Usar valores menores (ej: 0.005) para calibración
                de motores con movimiento ultra-fluido.
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

        # Contador de comandos ignorados seguidos (para logging agrupado)
        self.ignored_commands_count = {}  # {robot_id: count}

        # Rate limiter configurable: Mínimo entre comandos al mismo robot
        # (firmware tiene delay(10), más overhead de procesamiento)
        # Para calibración, puede reducirse a 5ms para movimiento más fluido
        self.last_send_time = {}  # {robot_id: timestamp}
        self.MIN_COMMAND_INTERVAL = min_command_interval
        log.debug(f"Rate limiting: {self.MIN_COMMAND_INTERVAL*1000:.1f}ms entre comandos")

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

        Recibe valores PWM directos y envía el comando correspondiente.

        IMPORTANTE: El firmware acepta int8_t (-127 a 127), NO -255 a 255.
        Los valores se limitan automáticamente al rango correcto.

        Implementa rate limiting para evitar saturar el buffer serial:
        - Solo envía si han pasado MIN_COMMAND_INTERVAL (15ms) desde el último comando
        - O si es un comando de detención (prioridad alta)
        - O si el cambio es significativo (>= 5 en PWM)

        Args:
            robot_id (int): ID del robot (1-4).
            left_speed (int): Velocidad del motor izquierdo en PWM (-127 a 127).
            right_speed (int): Velocidad del motor derecho en PWM (-127 a 127).

        Returns:
            bool: True si el comando se envió correctamente, False en caso contrario.
        """
        # Los valores ya vienen en PWM (-127 a 127), asegurar que estén en rango
        left_val = int(max(-127, min(127, left_speed)))
        right_val = int(max(-127, min(127, right_speed)))

        # Aplicar calibración individual si está habilitada
        # NOTA: robot_id aquí es firmware ID (1-4), pero la calibración JSON
        # usa Python IDs (0-3) como claves. Convertir antes de aplicar.
        if self.enable_calibration and self.calibration:
            python_id = robot_id - 1
            left_val_calibrated, right_val_calibrated = self.calibration.apply_calibration(
                python_id, left_val, right_val
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
        # Si se ignoraron comandos antes de este, mostrar cuántos se ignoraron
        ignored_count = self.ignored_commands_count.get(robot_id, 0)
        if DEBUG_RATE_LIMITING and ignored_count > 0:
            log.warning(
                f"Robot {robot_id}: IGNORADOS: {ignored_count} comandos | "
                f"Ahora enviando PWM:({left_val},{right_val})"
            )
            self.ignored_commands_count[robot_id] = 0  # Resetear contador

        # Solo loguear cuando se DETIENE el robot
        # NOTA: robot_id es firmware ID (1-4); restamos 1 para mostrar player ID (0-3).
        if is_stop_command and last_cmd != (0, 0):
            log.info("⏹️  Robot %i DETENIDO", robot_id - 1)

        # Actualizar comandos y tiempos
        self.last_commands[robot_id] = (left_val, right_val)
        self.last_send_time[robot_id] = current_time

        # Generar comando
        command = self.protocol.format_motor_command(robot_id, left_val, right_val)

        # ===== SISTEMA DE 3 PRIORIDADES =====
        # Calcular magnitud de velocidad (para detectar desaceleración)
        current_magnitude = math.sqrt(left_val**2 + right_val**2) / 255.0  # Normalizar a 0-1
        last_magnitude = self.last_speed_magnitude.get(robot_id, current_magnitude)

        # Detectar desaceleración significativa (entrada a rampa)
        is_decelerating = (last_magnitude - current_magnitude) > self.RAMP_DECELERATION_THRESHOLD

        # Actualizar magnitud para próxima iteración
        self.last_speed_magnitude[robot_id] = current_magnitude

        # PRIORIDAD ALTA: Detención (0, 0)
        if is_stop_command:
            success = self.serial_manager.send_priority_command(command, priority='high')
            # El stop se reenvía cada frame por seguridad (timeout firmware), pero la
            # transición a detenido ya se loguea una vez arriba (⏹️ DETENIDO con guard
            # last_cmd != (0,0)). A debug para no inundar INFO con un robot detenido/perdido.
            log.debug("🚨 ALTA prioridad (detención) para Robot %i", robot_id - 1)

        # PRIORIDAD MEDIA: Entrada a rampa (desaceleración >15%)
        elif is_decelerating and not is_stop_command:
            success = self.serial_manager.send_priority_command(command, priority='medium')
            log.info("🔶 MEDIA prioridad (rampa) para Robot %i: mag %.2f → %.2f (Δ=%.2f%%)",
                     robot_id - 1, last_magnitude, current_magnitude,
                     (last_magnitude - current_magnitude) * 100)

        # PRIORIDAD NORMAL: Movimiento regular
        else:
            success = self.serial_manager.send_command(command)

        # Logging de comando enviado exitosamente
        if DEBUG_RATE_LIMITING:
            log.info(f"Robot {robot_id}: >>> COMANDO ENVIADO | PWM:({left_val},{right_val})")

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

    def kick_priority(self, robot_id, power=1.0):
        """Activa el mecanismo de pateo usando cola de ALTA prioridad.

        Equivalente a kick() pero el comando se encola con prioridad alta para
        que no sea descartado por comandos de stop que lleguen en la misma iteración.

        Args:
            robot_id (int): ID del robot (1-4).
            power (float): Potencia del pateo (0.0-1.0). Defaults to 1.0.

        Returns:
            bool: True si el comando se envió correctamente, False en caso contrario.
        """
        power_val = int(power * 255)
        command = self.protocol.format_kick_command(robot_id, power_val)
        success = self.serial_manager.send_priority_command(command, priority='high')
        if success:
            log.debug("Robot %i: Pateo (alta prioridad) potencia %.2f", robot_id, power)
        return success

    def set_dribbler(self, robot_id, power=255):
        """Establece la potencia del dribbler de un robot.

        El firmware controla el motor DC del dribbler via SoftPWM,
        permitiendo potencia variable (no solo ON/OFF).

        Args:
            robot_id (int): ID del robot (1-4).
            power (int): Potencia del dribbler en PWM (0-255). El valor se recorta al
                cap blando DRIBBLER_MAX_PWM antes de enviarse (protección de corriente:
                el motor NO tiene sensor y a PWM alto en stall se quema).

        Returns:
            bool: True si el comando se envió correctamente, False en caso contrario.
        """
        # Gate de hardware: si el dribbler de este robot está marcado como AVERIADO, forzar 0
        # (nunca energizar el componente roto, venga el comando de donde venga). robot_id es el
        # id de firmware (1-4); el id de jugador es robot_id-1. Chokepoint único: todos los
        # paths de dribbler pasan por aquí.
        if not is_dribbler_enabled(robot_id - 1):
            power = 0
        # Clampar al cap blando (0..DRIBBLER_MAX_PWM): nunca acercarse a 255.
        power_val = max(0, min(DRIBBLER_MAX_PWM, int(power)))

        # Generar comando
        command = self.protocol.format_dribbler_command(robot_id, power_val)

        # Enviar comando
        success = self.serial_manager.send_command(command)
        if success:
            log.debug("Robot %i: Dribbler ajustado a %.2f", robot_id, power)

        return success

    def set_dribbler_config(self, robot_id, on_ms, off_ms, wdt_ms):
        """Envía la config de oscilación del dribbler (persiste en EEPROM del robot).

        Se manda al inicio de sesión (y cuando se quiera cambiar el duty/watchdog en runtime).
        NO es hot-path. El firmware oscila el rodillo con este duty; Python ya no oscila.
        robot_id es el id de firmware (1-4).

        Returns:
            bool: True si el comando se envió correctamente.
        """
        command = self.protocol.format_dribbler_config(robot_id, on_ms, off_ms, wdt_ms)
        success = self.serial_manager.send_command(command)
        if success:
            log.debug("Robot %i: dribbler config on=%d off=%d wdt=%d",
                      robot_id, on_ms, off_ms, wdt_ms)
        return success

    def set_dribbler_config_sync(self, robot_id, on_ms, off_ms, wdt_ms, timeout=1.5):
        """Envía la config y ESPERA la confirmación TELEM del firmware (round-trip EEPROM).

        El firmware reescribe EEPROM (solo si cambió), la RELEE y responde por ACK con la
        config que QUEDÓ guardada. Esto verifica que el robot va a oscilar con esos valores
        antes de empezar a jugar. Bloqueante; usar solo en arranque/setup, no en hot-path.

        Args:
            robot_id (int): ID de firmware (1-4).
            on_ms, off_ms, wdt_ms (int): config a setear.
            timeout (float): segundos máx. a esperar la confirmación.

        Returns:
            bool: True si el firmware confirmó cfg=on/off/wdt; False si no llegó o no coincide.
        """
        # Descartar telemetría vieja: solo queremos la confirmación de ESTE envío.
        self.serial_manager.drain_telemetry()
        if not self.set_dribbler_config(robot_id, on_ms, off_ms, wdt_ms):
            return False
        deadline = time.time() + timeout
        while time.time() < deadline:
            lines, _ = self.serial_manager.drain_telemetry()
            for _ts, line in lines:
                d = self._parse_telem(line)
                if (d is not None and d.get('robot') == robot_id
                        and d.get('on') == on_ms and d.get('off') == off_ms
                        and d.get('wdt') == wdt_ms):
                    log.info("Robot %i: config dribbler confirmada %d/%d/%d",
                             robot_id, on_ms, off_ms, wdt_ms)
                    return True
            time.sleep(0.05)
        log.warning("Robot %i: SIN confirmacion de config dribbler %d/%d/%d en %.1fs",
                    robot_id, on_ms, off_ms, wdt_ms, timeout)
        return False

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

    def poll_telemetry(self):
        """Lee la telemetría del firmware acumulada en el serial (D1).

        Drena las líneas TELEM que el SerialManager ya capturó (piggyback en los ACK del
        nRF24) y las parsea a dicts. Observación: NO envía nada, solo lee lo ya emitido.

        Returns:
            (list[dict], int): telemetrías parseadas y nº de ERROR de entrega desde el último
            poll. Cada dict: {robot, dbg, on, off, wdt, eng, pwr, ev, m, d, t}.
        """
        lines, err = self.serial_manager.drain_telemetry()
        out = []
        for ts, line in lines:
            d = self._parse_telem(line)
            if d is not None:
                d['t'] = ts
                out.append(d)
        return out, err

    @staticmethod
    def _parse_telem(line):
        """Parsea 'TELEM R2 dbg=1 cfg=65/15/150 eng=1 pwr=50 ev=1 m=153 d=2' a dict (o None)."""
        try:
            toks = line.split()
            if len(toks) < 2 or toks[0] != 'TELEM' or not toks[1].startswith('R'):
                return None
            d = {'robot': int(toks[1][1:])}
            for tok in toks[2:]:
                if '=' not in tok:
                    continue
                k, v = tok.split('=', 1)
                if k == 'cfg':
                    on, off, wdt = v.split('/')
                    d['on'], d['off'], d['wdt'] = int(on), int(off), int(wdt)
                else:
                    d[k] = int(v)
            return d
        except (ValueError, IndexError):
            return None

    def send_tablero(self, cmd_num):
        """Envía un comando al tablero de puntuación.

        Args:
            cmd_num (int): 1=toggle_pausa, 2=gol_eq1, 3=gol_eq2,
                           4=reset_goles, 5=reset_tiempo

        Returns:
            bool: True si el comando se encoló correctamente.
        """
        command = self.protocol.format_tablero_command(cmd_num)
        # Cola normal: los comandos de tablero son eventos puntuales,
        # no deben limpiar la cola de comandos de motores.
        success = self.serial_manager.send_command(command)
        if success:
            log.info("Tablero: cmd=%d enviado", cmd_num)
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

    def update_robot_calibration(self, robot_id, max_left, max_right, bias):
        # pylint: disable=unused-argument
        """OBSOLETO: Usar calibrate_robot_pwm_range.py y calibrate_robot_bias.py."""
        log.warning(
            "update_robot_calibration() está deprecado. "
            "Usar calibrate_robot_pwm_range.py (paso 1) y calibrate_robot_bias.py (paso 2)."
        )
