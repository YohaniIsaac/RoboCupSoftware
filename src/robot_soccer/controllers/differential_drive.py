import logging
import math
import time

from robot_soccer.utils.robot_logger import robot_status_logger
from robot_soccer.config import (
    ROBOT_MIN_ROTATION_SPEED,
    ROBOT_MAX_ROTATION_SPEED,
    ROBOT_ROTATION_ARRIVAL_ANGLE_DEG,
    ROBOT_ROTATION_NEAR_MIN,
    ROBOT_MIN_LINEAR_SPEED,
    ROBOT_MAX_LINEAR_SPEED,
    ROBOT_LINEAR_ARRIVAL_DISTANCE,
    ROBOT_LINEAR_NEAR_MIN,
    ROBOT_LINEAR_START_ANGLE_THRESHOLD_DEG,
    ROBOT_POSITION_THRESHOLD,
    ROBOT_ANGLE_THRESHOLD_DEG,
    MOTOR_MAX_PWM,
    PID_POSITION_KP,
    PID_POSITION_KI,
    PID_POSITION_KD,
    PID_ANGLE_KP,
    PID_ANGLE_KI,
    PID_ANGLE_KD,
    STUCK_MOVEMENT_THRESHOLD_PX,
    STUCK_DETECTION_WINDOW_S,
    STUCK_BOOST_INCREMENT,
    STUCK_BOOST_MAX,
    STUCK_BOOST_DECAY,
    STUCK_AUTO_KICK,
    CREEP_REGULATOR_ENABLED,
    CREEP_REGULATOR_WINDOW_S,
    CREEP_TARGET_DISPLACEMENT_MIN_PX,
    CREEP_TARGET_DISPLACEMENT_MAX_PX,
    CREEP_PWM_STEP,
    CREEP_PWM_MIN,
    CREEP_PWM_MAX,
)

log = logging.getLogger(__name__)


def _normalize_angle(angle):
    """Normaliza un ángulo entre -π y π.

    Args:
        angle (float): Ángulo en radianes.

    Returns:
        float: Ángulo normalizado entre -π y π.

    Example:
        >>> _normalize_angle(3.5)
        -2.7831853...
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class DifferentialDriveController:
    """Controlador para robots de tracción diferencial.

    Traduce comandos de movimiento de alto nivel a velocidades PWM de motores
    utilizando control PID para posición y orientación.

    Attributes:
        wheel_radius (float): Radio de las ruedas en metros.
        wheel_distance (float): Distancia entre las ruedas en metros.
        max_motor_speed (int): Velocidad máxima de los motores en PWM (0-255).
        rf_controller: Controlador RF para robots reales.
        position_threshold (int): Umbral de distancia en píxeles.
        angle_threshold (float): Umbral de ángulo en radianes.
    """

    def __init__(
        self,
        rf_controller=None,
        wheel_radius=0.025,
        wheel_distance=0.1,
        max_motor_speed=MOTOR_MAX_PWM,
        enable_debug_pid_logging=False,
    ):
        """Inicializa el controlador con parámetros físicos del robot.

        Args:
            rf_controller: Controlador RF compartido para robots reales.
            wheel_radius (float): Radio de las ruedas en metros. Por defecto 0.025.
            wheel_distance (float): Distancia entre ruedas en metros. Por defecto 0.1.
            max_motor_speed (int): Velocidad máxima PWM (0-255). Por defecto 255.
            enable_debug_pid_logging (bool): Habilita logging detallado de PID para calibración.

        Note:
            Los parámetros PID se inicializan con valores predeterminados que pueden
            requerir ajuste según el robot específico.
            Todas las velocidades se manejan internamente como PWM (0-255).
        """
        self.wheel_radius = wheel_radius
        self.wheel_distance = wheel_distance
        self.max_motor_speed = max_motor_speed
        self.rf_controller = rf_controller
        self.enable_debug_pid_logging = enable_debug_pid_logging

        # Estado PID por robot (evita corrupción con múltiples robots)
        self._pid_state = {}  # {robot_id: dict con estado PID}

        # Constantes para PID (configurables desde config.py)
        # Valores iniciales se cargan desde config, pero pueden ser sobrescritos
        # durante calibración usando el script calibrate_pid_controllers.py
        self.kp_pos = PID_POSITION_KP      # Ganancia proporcional de posición
        self.ki_pos = PID_POSITION_KI      # Ganancia integral de posición
        self.kd_pos = PID_POSITION_KD      # Ganancia derivativa de posición
        self.kp_angle = PID_ANGLE_KP       # Ganancia proporcional angular
        self.ki_angle = PID_ANGLE_KI       # Ganancia integral angular
        self.kd_angle = PID_ANGLE_KD       # Ganancia derivativa angular

        # Compensación de latencia para predictive stopping
        # Medición real: 25 FPS (40ms/frame) + procesamiento (15ms) + RF (10ms) = ~65ms
        self.LATENCY_COMPENSATION_MS = 70  # ms totales de latencia (conservador)

        # Umbrales de distancia (desde config.py)
        self.position_threshold = ROBOT_POSITION_THRESHOLD
        self.angle_threshold = math.radians(ROBOT_ANGLE_THRESHOLD_DEG)

        # Parámetros de perfil de velocidad lineal (desde config.py, en PWM)
        self.min_motor_speed = ROBOT_MIN_LINEAR_SPEED  # MIN cuando LEJOS (PWM)
        self.max_smooth_speed = ROBOT_MAX_LINEAR_SPEED  # MAX (PWM)
        self.distance_near = ROBOT_LINEAR_ARRIVAL_DISTANCE  # Donde empieza rampa (px)
        self.linear_near_min = ROBOT_LINEAR_NEAR_MIN  # MIN en la rampa (PWM)

        # Parámetros de perfil de velocidad angular (desde config.py, en PWM)
        self.min_rotation_speed = ROBOT_MIN_ROTATION_SPEED  # MIN cuando LEJOS (PWM)
        self.max_rotation_speed = ROBOT_MAX_ROTATION_SPEED  # MAX (PWM)
        self.angle_near = math.radians(ROBOT_ROTATION_ARRIVAL_ANGLE_DEG)  # Donde empieza rampa (rad)
        self.rotation_near_min = ROBOT_ROTATION_NEAR_MIN  # MIN en la rampa (PWM)

        # Override de velocidad lineal máxima (para creep/captura)
        # Cuando no es None, v se capea a este valor DESPUÉS del perfil de velocidad
        # pero ANTES de combinar con omega. Así la corrección angular sigue activa.
        self.max_linear_pwm_override = None
        self.detection_pwm_cap = None  # cap impuesto por pérdida de detección de cámara
        self.auto_kick_enabled = False  # solo True durante move_with_ball; gate del AUTO-KICK por stuck

        # Factor multiplicador de PWM con posesión de pelota (dribbler activo).
        # Se setea externamente por RobotCommandManager según has_ball().
        self.dribble_pwm_factor = 1.0

        # Límites de velocidad POR ROBOT (cargados de calibración JSON)
        # Se inicializan en _get_robot_speed_limits() la primera vez que se usa cada robot
        self._robot_speed_limits = {}  # {robot_id: (min_speed, max_speed)}

        # Factores de compensación para asimetría de calibración
        # Se calculan por robot cuando se obtiene su calibración
        self.rotation_asymmetry_factors = {}  # {robot_id: (cw_factor, ccw_factor)}

        # Para logging de cambios de velocidad
        self.last_logged_rotation_speed = {}  # {robot_id: speed}

        # Para logging periódico de velocidad (cada 250ms)
        self.last_periodic_log_time = {}  # {robot_id: timestamp}
        self.last_periodic_log_time_linear = {}  # {robot_id: timestamp} para movimiento lineal
        self.PERIODIC_LOG_INTERVAL = 0.25  # 250ms entre logs periódicos
        self.last_logged_linear_speed = {}  # {robot_id: speed} para movimiento lineal

        # Para logging detallado de PID (calibración) - cada 100ms
        self.last_pid_debug_log_time = {}  # {robot_id: timestamp}
        self.PID_DEBUG_LOG_INTERVAL = 0.1  # 100ms entre logs de PID

        # Parámetros de detección de atasco (calibrables externamente)
        self.stuck_movement_threshold_px = STUCK_MOVEMENT_THRESHOLD_PX
        self.stuck_detection_window_s    = STUCK_DETECTION_WINDOW_S
        self.stuck_boost_increment       = STUCK_BOOST_INCREMENT
        self.stuck_boost_max             = STUCK_BOOST_MAX
        self.stuck_boost_decay           = STUCK_BOOST_DECAY

        # Parámetros del regulador dinámico de velocidad del creep de captura
        self.creep_regulator_window_s          = CREEP_REGULATOR_WINDOW_S
        self.creep_target_displacement_min_px  = CREEP_TARGET_DISPLACEMENT_MIN_PX
        self.creep_target_displacement_max_px  = CREEP_TARGET_DISPLACEMENT_MAX_PX
        self.creep_pwm_step                    = CREEP_PWM_STEP
        self.creep_pwm_min                     = CREEP_PWM_MIN
        self.creep_pwm_max                     = CREEP_PWM_MAX

    def _get_robot_speed_limits(self, robot_id):
        """Obtiene límites de velocidad PWM desde la calibración JSON del robot.

        Carga pwm_min y pwm_max del archivo de calibración multipoint.
        Si no hay calibración, usa los valores por defecto de config.py.

        Args:
            robot_id: ID del robot (0-3, ID de ArUco marker).

        Returns:
            tuple: (min_speed, max_speed) en PWM.
        """
        if robot_id not in self._robot_speed_limits:
            if (self.rf_controller
                    and hasattr(self.rf_controller, 'calibration')
                    and self.rf_controller.calibration is not None):
                pwm_min, pwm_max = self.rf_controller.calibration.get_pwm_range(robot_id)
                log.info("Robot %d: velocidad desde calibración JSON [%d, %d] PWM",
                         robot_id, pwm_min, pwm_max)
            else:
                pwm_min = self.min_motor_speed
                pwm_max = self.max_smooth_speed
                log.info("Robot %d: velocidad desde config.py [%d, %d] PWM (sin calibración)",
                         robot_id, pwm_min, pwm_max)
            self._robot_speed_limits[robot_id] = (pwm_min, pwm_max)
        return self._robot_speed_limits[robot_id]

    def compute_reset_pwm(self, robot_id, factor):
        """Calcula el PWM de reset para un robot respetando su mínimo calibrado.

        Retorna max(pwm_min, round(pwm_max * factor)), garantizando que el
        robot pueda arrancar y que la velocidad esté por debajo de su máximo.
        """
        pwm_min, pwm_max = self._get_robot_speed_limits(robot_id)
        return max(pwm_min, round(pwm_max * factor))

    def _get_pid_state(self, robot_id):
        """Retorna el estado PID para un robot específico, creándolo si no existe.

        Args:
            robot_id: ID del robot.

        Returns:
            dict: Estado PID con last_error_pos, integral_pos, last_error_angle,
                  integral_angle, last_linear_speed, last_rotation_speed.
        """
        if robot_id not in self._pid_state:
            self._pid_state[robot_id] = {
                # PID lineal (escalar)
                'last_distance': 0,
                'integral_distance': 0,
                'last_derivative_distance': 0,
                'last_distance_measurement_time': 0,
                # PID angular (compartido entre rotación y corrección lineal)
                'last_error_angle': 0,
                'integral_angle': 0,
                'last_derivative_angle': 0,
                'last_angle_measurement_time': 0,
                # Tracking
                'last_linear_speed': 0,
                'last_rotation_speed': 0,
                'in_rotation_mode': False,
                'last_pid_time': 0,
                # Stuck detection
                'stuck_boost': 0,          # PWM adicional actualmente aplicado
                'stuck_ref_x': None,       # posición de referencia al inicio de la ventana
                'stuck_ref_y': None,
                'stuck_window_start': 0.0, # timestamp de inicio de la ventana actual
                'stuck_auto_kicked': False, # True cuando auto-kick fue disparado en esta ventana
                # Regulador dinámico de velocidad del creep de captura
                'creep_pwm': None,         # cap de velocidad regulado (None = usar override estático)
                'creep_ref_x': None,       # posición de referencia al inicio de la ventana
                'creep_ref_y': None,
                'creep_window_start': 0.0, # timestamp de inicio de la ventana del regulador
                # Histéresis de llegada: una vez declarado arrived, no reactivar movimiento
                # mientras dist < 2× position_threshold. Evita ping-pong por overshoot
                # inercial cuando la calibración del robot tiene rango PWM estrecho.
                'arrived': False,
                'last_target_pos': None,
            }
        return self._pid_state[robot_id]

    def _regulate_creep_speed(self, robot, _pid_st, _now):
        """Regulador bidireccional de velocidad para el creep de captura.

        Cierra el lazo sobre el desplazamiento REAL del robot (px medidos por cámara)
        en lugar de confiar en el PWM open-loop. Mide el desplazamiento por ventana
        temporal y ajusta el cap de velocidad lineal (``_pid_st['creep_pwm']``) para
        mantener una banda objetivo:

          - desplazamiento > MAX (rápido → empujaría la pelota): baja el cap (hasta 0,
            coasting permitido).
          - desplazamiento < MIN (lento o atascado): sube el cap para vencer la stiction.
          - dentro de banda: mantiene.

        Solo se invoca durante el creep (``max_linear_pwm_override`` activo, i.e.
        advance_to_contact). Reemplaza al STUCK durante esa fase, por lo que mantiene
        ``stuck_boost`` en 0 (sin boost aditivo). El cap se consume en move_to_position
        con fallback a ``max_linear_pwm_override`` (CAPTURE_CREEP_SPEED_PWM) en el
        primer tick.
        """
        # El regulador gobierna la velocidad durante el creep: sin boost aditivo STUCK.
        if _pid_st['stuck_boost'] != 0:
            _pid_st['stuck_boost'] = 0
            robot_status_logger.update(robot.id, stuck_boost=0)

        # Entrada al creep: baseline = override estático + ventana fresca.
        if _pid_st['creep_ref_x'] is None:
            _pid_st['creep_pwm'] = self.max_linear_pwm_override
            _pid_st['creep_ref_x'] = robot.x
            _pid_st['creep_ref_y'] = robot.y
            _pid_st['creep_window_start'] = _now
            robot_status_logger.update(robot.id, creep_pwm=_pid_st['creep_pwm'])
            return

        # Aún no se cierra la ventana de medición: mantener el cap actual.
        if _now - _pid_st['creep_window_start'] < self.creep_regulator_window_s:
            return

        _moved = math.sqrt(
            (robot.x - _pid_st['creep_ref_x'])**2 +
            (robot.y - _pid_st['creep_ref_y'])**2
        )
        if _moved > self.creep_target_displacement_max_px:
            _pid_st['creep_pwm'] = max(
                self.creep_pwm_min, _pid_st['creep_pwm'] - self.creep_pwm_step
            )
        elif _moved < self.creep_target_displacement_min_px:
            _pid_st['creep_pwm'] = min(
                self.creep_pwm_max, _pid_st['creep_pwm'] + self.creep_pwm_step
            )
        # else: dentro de banda → mantener.

        _pid_st['creep_ref_x'] = robot.x
        _pid_st['creep_ref_y'] = robot.y
        _pid_st['creep_window_start'] = _now
        robot_status_logger.update(robot.id, creep_pwm=_pid_st['creep_pwm'])

    def _calculate_rotation_compensation(self, robot_id):
        """Calcula factores de compensación para asimetría en rotación.

        Cuando un robot tiene calibración asimétrica (ej: motor_left=1.0, motor_right=0.79),
        gira más rápido en una dirección que en la otra. Esta función calcula factores
        de compensación para balancear las velocidades de rotación en ambas direcciones.

        Args:
            robot_id: ID del robot

        Returns:
            tuple: (cw_compensation, ccw_compensation)
                   Factores multiplicadores para giro horario y antihorario
        """
        if not self.rf_controller or not hasattr(self.rf_controller, 'calibration'):
            return (1.0, 1.0)  # Sin calibración = sin compensación

        if self.rf_controller.calibration is None:
            return (1.0, 1.0)

        # Obtener calibración del robot usando el max_speed del robot como referencia.
        # IMPORTANTE: usar el max_speed real del robot (no un valor fijo de 50 PWM)
        # para evitar que robots con rango estrecho (ej. Robot 0: [17-29]) queden
        # fuera del rango calibrado y retornen siempre (1.0, 1.0) → sin compensación.
        _, ref_speed = self._get_robot_speed_limits(robot_id)
        max_left, max_right, _ = self.rf_controller.calibration.get_calibration_at_speed(
            robot_id, ref_speed
        )

        # Si ambos motores son iguales, no hay asimetría
        if abs(max_left - max_right) < 0.01:
            return (1.0, 1.0)

        # Identificar cuál motor es más débil
        # El motor más débil limita la velocidad de giro cuando va "adelante"
        #
        # CCW (antihorario, angle_error > 0):
        #   Motor IZQ adelante, motor DER atrás
        #   → Limitado por max_left (motor que empuja adelante)
        #
        # CW (horario, angle_error < 0):
        #   Motor IZQ atrás, motor DER adelante
        #   → Limitado por max_right (motor que empuja adelante)
        #
        # Compensar el lado más débil inversamente
        weaker_motor = min(max_left, max_right)
        stronger_motor = max(max_left, max_right)

        if max_left < max_right:
            # Motor izquierdo es más débil → CCW (izq adelante) es más lento
            ccw_compensation = stronger_motor / weaker_motor  # Aumentar CCW
            cw_compensation = 1.0  # CW normal
        else:
            # Motor derecho es más débil → CW (der adelante) es más lento
            cw_compensation = stronger_motor / weaker_motor  # Aumentar CW
            ccw_compensation = 1.0  # CCW normal

        log.debug("Robot %d: Compensación rotación CW=%.3f, CCW=%.3f (cal: L=%.2f R=%.2f)",
                 robot_id, cw_compensation, ccw_compensation, max_left, max_right)

        return (cw_compensation, ccw_compensation)

    def move_to_position(self, robot, target_pos, target_angle=None,
                         arrival_threshold=None):
        """Mueve el robot a una posición usando control Dual PID (v, ω) estándar.

        Arquitectura estándar para tracción diferencial:
        - PID lineal: distancia → v (velocidad base, ambas ruedas)
        - PID angular: error_heading → ω (corrección diferencial)
        - Ambos corren SIMULTÁNEAMENTE en cada ciclo
        - Cinemática: left = v + ω, right = v - ω
        - Si error angular > umbral: v=0 (rotación pura, mismo PID angular)

        Args:
            robot: Objeto robot con atributos x, y, angle (radianes).
            target_pos (tuple): Posición objetivo como (x, y).
            target_angle (float, optional): Ángulo final en grados.
            arrival_threshold (float, optional): Umbral (px) de llegada para
                este movimiento. Si None se usa self.position_threshold
                (default global ROBOT_POSITION_THRESHOLD).

        Returns:
            bool: True si llegó al objetivo.
        """
        # === 1. CALCULAR ERRORES ===
        dx = target_pos[0] - robot.x
        dy = target_pos[1] - robot.y
        distance = math.sqrt(dx**2 + dy**2)

        threshold = arrival_threshold if arrival_threshold is not None \
                    else self.position_threshold

        state = self._get_pid_state(robot.id)
        robot_min_speed, robot_max_speed = self._get_robot_speed_limits(robot.id)

        # Detección por-robot de target nuevo: resetea histéresis de llegada para
        # que un re-comando al mismo punto no quede atrapado en estado arrived.
        if state['last_target_pos'] != target_pos:
            robot_status_logger.emit_event(
                robot.id,
                f"NUEVO TARGET: ({int(target_pos[0])},{int(target_pos[1])}) "
                f"desde ({int(robot.x)},{int(robot.y)}) d={distance:.0f}px"
            )
            state['last_target_pos'] = target_pos
            state['arrived']         = False

        # === 2. VERIFICACIÓN DE LLEGADA ===
        # Histéresis: si ya declaramos arrived, no reactivamos motores mientras
        # dist < 2× threshold. Evita ping-pong por overshoot inercial (R1 con
        # rango PWM estrecho oscilaba entre 2-7px del waypoint).
        if state['arrived']:
            if distance < 2.0 * threshold:
                if target_angle is not None:
                    return self.rotate_to_angle(robot, target_angle)
                self._send_motor_commands(robot, 0, 0)
                return True
            # Salimos del rango de histéresis: liberar y reactivar movimiento.
            state['arrived'] = False

        latency_s = self.LATENCY_COMPENSATION_MS / 1000.0
        speed_norm = abs(state['last_linear_speed']) / 255.0
        predicted_dist = distance - speed_norm * 200.0 * latency_s

        if predicted_dist < threshold and state['last_linear_speed'] > 0:
            robot_status_logger.emit_event(
                robot.id,
                f"STOP PREDICTIVO: dist={distance:.1f}px pred={predicted_dist:.1f}px"
            )
            state['last_linear_speed'] = 0
            state['arrived']           = True
            if target_angle is not None:
                # No enviar STOP: rotate_to_angle gestiona los motores directamente
                return self.rotate_to_angle(robot, target_angle)
            self._send_motor_commands(robot, 0, 0)
            return True

        if distance < threshold:
            robot_status_logger.emit_event(
                robot.id,
                f"WAYPOINT ALCANZADO: dist={distance:.1f}px"
            )
            state['last_linear_speed'] = 0
            state['arrived']           = True
            if target_angle is not None:
                # No enviar STOP: rotate_to_angle gestiona los motores directamente
                return self.rotate_to_angle(robot, target_angle)
            self._send_motor_commands(robot, 0, 0)
            return True

        # === 3. ERROR ANGULAR (heading hacia el objetivo) ===
        target_heading = math.atan2(dy, dx)
        angle_error = _normalize_angle(target_heading - robot.angle)

        # === 4. CALCULAR dt ===
        now = time.time()
        if state['last_pid_time'] == 0:
            dt = 0.033
        else:
            dt = now - state['last_pid_time']
        state['last_pid_time'] = now
        dt = max(0.001, min(0.1, dt))

        # === 5. PID ANGULAR → ω (corre SIEMPRE, en ambos modos) ===
        # Proporcional
        p_w = self.kp_angle * angle_error

        # Integral (con anti-windup y reset al cambiar dirección)
        if state['last_error_angle'] * angle_error < 0:
            state['integral_angle'] = 0
        else:
            state['integral_angle'] += angle_error * dt
        max_integral_angle = 3.0  # rad·s
        state['integral_angle'] = max(-max_integral_angle,
                                      min(max_integral_angle, state['integral_angle']))
        i_w = self.ki_angle * state['integral_angle']

        # Derivada (basada en medición, solo cuando hay nuevo frame)
        angle_changed = abs(angle_error - state['last_error_angle']) > 0.001
        if angle_changed:
            dt_meas = now - state.get('last_angle_measurement_time', now)
            if dt_meas > 0.001:
                state['last_derivative_angle'] = (
                    angle_error - state['last_error_angle']) / dt_meas
            state['last_angle_measurement_time'] = now
            state['last_error_angle'] = angle_error
        d_w = self.kd_angle * state.get('last_derivative_angle', 0)

        omega_raw = p_w + i_w + d_w

        # Guardar términos PID angulares para logging combinado
        p_w_stored = p_w
        i_w_stored = i_w
        d_w_stored = d_w

        # Inicializar términos PID lineales (para logging en modo rotación pura)
        p_v_stored = i_v_stored = d_v_stored = 0.0
        v_raw_stored = v_stored = 0.0
        distance_stored = angle_error_stored = 0.0

        # === 6. DECISIÓN DE MODO (histéresis) ===
        # Dos umbrales independientes para evitar switching rápido:
        #   enter_thresh: si error supera este umbral, entrar a rotación pura
        #   exit_thresh:  solo salir de rotación cuando error es MUY pequeño
        #
        # exit_thresh debe ser próximo al angle_threshold (umbral de llegada),
        # no al 50% del enter_thresh. Razón: con la corrección angular disponible
        # durante el movimiento lineal (~5-10 PWM diferencial para Robot 0), el
        # robot solo puede corregir errores pequeños (<10°). Salir a 15° con
        # corrección débil y target_heading dinámico causa zigzag.
        enter_thresh = math.radians(ROBOT_LINEAR_START_ANGLE_THRESHOLD_DEG)
        exit_thresh = self.angle_threshold + math.radians(3)  # ≈ 10° (angle_threshold + margen)

        if state['in_rotation_mode']:
            should_rotate = abs(angle_error) > exit_thresh
        else:
            should_rotate = abs(angle_error) > enter_thresh
        state['in_rotation_mode'] = should_rotate

        if should_rotate:
            # === MODO ROTACIÓN PURA (v=0, mismo PID angular) ===
            # Para rotación INLINE (dentro de move_to_position), escalamos el PID
            # directamente al rango del robot sin el perfil de rampa de rotate_to_angle.
            # Razón: el perfil de rampa tiene piso en min_speed (17 PWM para Robot 0),
            # y con kp típico el PID cae por debajo de ese piso desde ~35°, produciendo
            # velocidad constante mínima. La escala directa + clamp da el mismo resultado
            # para ángulos pequeños pero evita el artefacto de "rampa empieza a 25°".
            raw_pwm = abs(omega_raw) * robot_max_speed
            rotation_speed = int(max(robot_min_speed, min(robot_max_speed, raw_pwm)))

            # Compensación de asimetría de motores
            if robot.id not in self.rotation_asymmetry_factors:
                self.rotation_asymmetry_factors[robot.id] = (
                    self._calculate_rotation_compensation(robot.id))
            cw_comp, ccw_comp = self.rotation_asymmetry_factors[robot.id]
            if angle_error > 0:
                rotation_speed = int(rotation_speed * ccw_comp)
            else:
                rotation_speed = int(rotation_speed * cw_comp)

            if angle_error > 0:
                left_speed, right_speed = rotation_speed, -rotation_speed
            else:
                left_speed, right_speed = -rotation_speed, rotation_speed

            state['last_linear_speed'] = 0
            state['last_rotation_speed'] = rotation_speed

            # === DEBUG PID LOGGING COMBINADO (calibración) - throttled a 100ms ===
            if self.enable_debug_pid_logging:
                last_log = self.last_pid_debug_log_time.get(robot.id, 0)
                if now - last_log >= self.PID_DEBUG_LOG_INTERVAL:
                    self.last_pid_debug_log_time[robot.id] = now
                    err_deg = math.degrees(angle_error)
                    log.debug("🔧[PID] R%d | ANG: err∠=%+6.1f° P=%+.3f I=%+.3f D=%+.3f | ROT: speed=%d | PWM: L=%+3d R=%+3d | KP=%.3f/%.3f KI=%.3f/%.3f KD=%.3f/%.3f",
                              robot.id, err_deg, p_w_stored, i_w_stored, d_w_stored,
                              rotation_speed, left_speed, right_speed,
                              self.kp_angle, self.kp_pos, self.ki_angle, self.ki_pos,
                              self.kd_angle, self.kd_pos)

            # Logging rotación
            self._log_periodic(
                robot, now, 'rotation',
                angle_error=angle_error,
                speed=rotation_speed,
                min_spd=robot_min_speed, max_spd=robot_max_speed,
                left=left_speed, right=right_speed)
        else:
            # === MODO LINEAL + CORRECCIÓN ANGULAR (v>0, ω como diferencial) ===

            # --- PID LINEAL → v (escalar, basado en distancia) ---
            p_v = self.kp_pos * distance

            # Integral escalar (acumula distancia × tiempo)
            state['integral_distance'] += distance * dt
            max_integral_dist = 50.0  # px·s
            state['integral_distance'] = min(
                state['integral_distance'], max_integral_dist)
            i_v = self.ki_pos * state['integral_distance']

            # Derivada escalar CON SIGNO (negativa al acercarse = frena)
            last_dist = state['last_distance']
            dist_changed = abs(distance - last_dist) > 0.5
            if dist_changed:
                dt_meas_d = now - state.get('last_distance_measurement_time', now)
                if dt_meas_d > 0.001:
                    state['last_derivative_distance'] = (
                        distance - last_dist) / dt_meas_d
                state['last_distance_measurement_time'] = now
                state['last_distance'] = distance
            d_v = self.kd_pos * state.get('last_derivative_distance', 0)

            v_raw = p_v + i_v + d_v

            # Aplicar perfil de velocidad (rampa cerca del objetivo)
            v = self._apply_velocity_profile(
                v_raw, distance,
                min_speed=robot_min_speed, max_speed=robot_max_speed)

            # Override de velocidad lineal (creep mode para captura de balón).
            # Si el regulador dinámico ya ajustó un cap (creep_pwm), se usa ese; si
            # no (1er tick o regulador deshabilitado), cae al override estático.
            if self.max_linear_pwm_override is not None:
                _creep_cap = state.get('creep_pwm')
                if _creep_cap is None:
                    _creep_cap = self.max_linear_pwm_override
                v = min(v, _creep_cap)
            if self.detection_pwm_cap is not None:
                v = min(v, self.detection_pwm_cap)

            # Guardar términos PID lineales para logging combinado
            p_v_stored = p_v
            i_v_stored = i_v
            d_v_stored = d_v
            v_raw_stored = v_raw
            v_stored = v
            distance_stored = distance
            angle_error_stored = angle_error

            # --- Combinar v + ω (cinemática diferencial estándar) ---
            omega_pwm = int(omega_raw * robot_max_speed)

            left_speed = v + omega_pwm
            right_speed = v - omega_pwm

            # Clipear al rango calibrado del robot
            if self.max_linear_pwm_override is not None:
                # Creep mode: piso en 0 (no en robot_min_speed) para que
                # la corrección angular funcione a velocidades bajas
                left_speed = max(0, min(robot_max_speed, left_speed))
                right_speed = max(0, min(robot_max_speed, right_speed))
            else:
                # Desaturación con prioridad de giro: si una rueda supera el techo,
                # se baja el par COMPLETO en la misma cantidad para conservar el
                # diferencial L-R (el giro), en vez de recortar cada rueda por
                # separado, que descartaba la mitad de la corrección angular cuando
                # v ya estaba saturada. La rueda dominante queda en el techo; la
                # otra puede caer hasta 0 (pivote suave). v >= robot_min_speed
                # (garantizado por el perfil de velocidad), así que la rueda
                # dominante = v + |omega| siempre mantiene velocidad de movimiento.
                exceso = max(left_speed, right_speed) - robot_max_speed
                if exceso > 0:
                    left_speed -= exceso
                    right_speed -= exceso
                left_speed = max(0, min(robot_max_speed, left_speed))
                right_speed = max(0, min(robot_max_speed, right_speed))

            state['last_linear_speed'] = v
            state['last_rotation_speed'] = 0

            # === DEBUG PID LOGGING COMBINADO (calibración) - throttled a 100ms ===
            if self.enable_debug_pid_logging:
                last_log = self.last_pid_debug_log_time.get(robot.id, 0)
                if now - last_log >= self.PID_DEBUG_LOG_INTERVAL:
                    self.last_pid_debug_log_time[robot.id] = now
                    err_deg = math.degrees(angle_error_stored)
                    log.debug("🔧[PID] R%d | ANG: err∠=%+6.1f° P=%+.3f I=%+.3f D=%+.3f | LIN: dist=%5.0fpx P=%+.2f I=%+.2f D=%+.2f | PWM: L=%+3d R=%+3d | KP=%.3f/%.3f KI=%.3f/%.3f KD=%.3f/%.3f",
                              robot.id, err_deg, p_w_stored, i_w_stored, d_w_stored,
                              distance_stored, p_v_stored, i_v_stored, d_v_stored,
                              left_speed, right_speed,
                              self.kp_angle, self.kp_pos, self.ki_angle, self.ki_pos,
                              self.kd_angle, self.kd_pos)

            # Logging lineal
            self._log_periodic(
                robot, now, 'linear',
                target_pos=target_pos, distance=distance,
                angle_error=angle_error, speed=v, omega_pwm=omega_pwm,
                min_spd=robot_min_speed, max_spd=robot_max_speed,
                left=left_speed, right=right_speed)

        # === REGULACIÓN DEL CREEP / STUCK DETECTION ===
        # Congelada en rotación pura: al girar en el sitio la posición no cambia,
        # lo que se interpretaba como atasco y disparaba un boost de PWM espurio.
        _pid_st = self._get_pid_state(robot.id)
        _now = time.monotonic()
        # Durante el creep de captura (advance_to_contact setea max_linear_pwm_override)
        # el regulador dinámico gobierna la velocidad; el STUCK queda inhibido para no
        # solaparse. Fuera del creep se limpia su estado para re-inicializar en la
        # próxima captura, y el STUCK corre con su lógica intacta.
        _creep_mode = CREEP_REGULATOR_ENABLED and self.max_linear_pwm_override is not None
        if _creep_mode:
            self._regulate_creep_speed(robot, _pid_st, _now)
        else:
            _pid_st['creep_ref_x'] = None
        if not _creep_mode and distance > self.position_threshold and not _pid_st['in_rotation_mode']:
            if _pid_st['stuck_ref_x'] is None:
                _pid_st['stuck_ref_x'] = robot.x
                _pid_st['stuck_ref_y'] = robot.y
                _pid_st['stuck_window_start'] = _now
            elif _now - _pid_st['stuck_window_start'] >= self.stuck_detection_window_s:
                _moved = math.sqrt(
                    (robot.x - _pid_st['stuck_ref_x'])**2 +
                    (robot.y - _pid_st['stuck_ref_y'])**2
                )
                if _moved < self.stuck_movement_threshold_px:
                    _prev_boost = _pid_st['stuck_boost']
                    _pid_st['stuck_boost'] = min(
                        _pid_st['stuck_boost'] + self.stuck_boost_increment,
                        self.stuck_boost_max
                    )
                    robot_status_logger.update(robot.id, stuck_boost=_pid_st['stuck_boost'])
                    robot_status_logger.emit_event(
                        robot.id,
                        "STUCK: boost=%d/%d PWM (moved=%.1fpx)" % (
                            _pid_st['stuck_boost'], self.stuck_boost_max, _moved
                        )
                    )
                    # Auto-kick al alcanzar el boost máximo (solo durante move_with_ball)
                    if (STUCK_AUTO_KICK and self.auto_kick_enabled
                            and _pid_st['stuck_boost'] >= self.stuck_boost_max):
                        robot_status_logger.emit_event(
                            robot.id,
                            "AUTO-KICK: stuck_boost max=%d PWM — disparando" % self.stuck_boost_max
                        )
                        if self.rf_controller:
                            self.rf_controller.kick(robot.id + 1, 1.0)
                        _pid_st['stuck_auto_kicked'] = True
                        _pid_st['stuck_boost'] = 0
                        _pid_st['stuck_ref_x'] = robot.x
                        _pid_st['stuck_ref_y'] = robot.y
                        _pid_st['stuck_window_start'] = _now
                        robot_status_logger.update(robot.id, stuck_boost=0)
                        self._send_motor_commands(robot, left_speed, right_speed)
                        return False
                else:
                    _pid_st['stuck_boost'] = max(
                        0, _pid_st['stuck_boost'] - self.stuck_boost_decay
                    )
                    robot_status_logger.update(robot.id, stuck_boost=_pid_st['stuck_boost'])
                _pid_st['stuck_ref_x'] = robot.x
                _pid_st['stuck_ref_y'] = robot.y
                _pid_st['stuck_window_start'] = _now
        elif not _creep_mode:
            _pid_st['stuck_ref_x'] = None
            _pid_st['stuck_window_start'] = 0.0
            _pid_st['stuck_boost'] = max(0, _pid_st['stuck_boost'] - self.stuck_boost_decay)
            robot_status_logger.update(robot.id, stuck_boost=_pid_st['stuck_boost'])

        self._send_motor_commands(robot, left_speed, right_speed)
        return False

    def _log_periodic(self, robot, now, mode, **kwargs):
        """Actualiza el status logger con datos del controlador (rotacion o movimiento)."""
        last_time = self.last_periodic_log_time.get(robot.id, 0)
        if (now - last_time) < self.PERIODIC_LOG_INTERVAL:
            return
        self.last_periodic_log_time[robot.id] = now

        angle_err_deg = math.degrees(kwargs.get('angle_error', 0))

        if mode == 'rotation':
            robot_status_logger.update(
                robot.id,
                ang=math.degrees(robot.angle),
                err_ang=angle_err_deg,
            )
        else:
            tp = kwargs.get('target_pos', (0, 0))
            dist = kwargs.get('distance', 0)
            robot_status_logger.update(
                robot.id,
                ang=math.degrees(robot.angle),
                err_ang=angle_err_deg,
                pos=(int(robot.x), int(robot.y)),
                tgt_pos=(int(tp[0]), int(tp[1])),
                dist=dist,
            )

    def rotate_to_angle(self, robot, target_angle_deg):
        """Rota el robot hacia un ángulo específico usando control PID.

        Args:
            robot: Objeto robot con atributo angle.
            target_angle_deg (float): Ángulo objetivo en grados.

        Returns:
            bool: True si el robot ha alcanzado el ángulo objetivo, False en caso contrario.

        Note:
            El control PID mantiene la velocidad de rotación suave y previene
            oscilaciones alrededor del ángulo objetivo.
        """
        # Normalizar ángulos a rango -180 a 180 grados primero
        def normalize_degrees(angle_deg):
            """Normaliza ángulo a rango -180 a 180 grados."""
            while angle_deg > 180:
                angle_deg -= 360
            while angle_deg < -180:
                angle_deg += 360
            return angle_deg

        # Normalizar ambos ángulos (robot.angle está en radianes, convertir a grados)
        current_normalized = normalize_degrees(math.degrees(robot.angle))
        target_normalized = normalize_degrees(target_angle_deg)

        # Calcular error angular en el camino MÁS CORTO
        angle_error_deg = target_normalized - current_normalized
        # Normalizar el error para tomar el camino más corto
        angle_error_deg = normalize_degrees(angle_error_deg)

        # Convertir error a radianes
        angle_error = math.radians(angle_error_deg)

        # Obtener estado PID y límites de velocidad calibrados
        state = self._get_pid_state(robot.id)
        robot_min_speed, robot_max_speed = self._get_robot_speed_limits(robot.id)

        # ===== DETECCIÓN DE CRUCE DEL OBJETIVO =====
        # Si el error cambió de signo Y disminuyó en magnitud, CRUZAMOS el objetivo
        # IMPORTANTE: No considerar cruce si el error está cerca de ±180° (cambio de signo por normalización)
        if hasattr(robot, '_last_angle_error_deg'):
            current_sign = 1 if angle_error >= 0 else -1
            last_sign = 1 if state['last_error_angle'] >= 0 else -1

            # Verificar si cambió de signo Y el error absoluto disminuyó (verdadero cruce)
            # Evitar falsos positivos cerca de ±180° donde el signo puede cambiar sin cruzar
            sign_changed = (current_sign != last_sign)
            error_decreased = abs(angle_error_deg) < abs(robot._last_angle_error_deg)
            not_near_180 = abs(angle_error_deg) < 170  # Ignorar cruces cerca de ±180°

            if sign_changed and error_decreased and not_near_180:
                # CRUZAMOS el objetivo! Detener inmediatamente
                robot_status_logger.emit_event(
                    robot.id,
                    f"TARGET CROSS: ang={current_normalized:+.1f}° -> tgt={target_normalized:+.1f}°"
                )
                self._send_motor_commands(robot, 0, 0)
                state['last_rotation_speed'] = 0
                self.last_logged_rotation_speed[robot.id] = 0
                robot._last_angle_error_deg = 0
                return True

        # Guardar error anterior para próxima iteración
        robot._last_angle_error_deg = angle_error_deg

        # ===== PREDICTIVE STOPPING =====
        # Estimar dónde ESTARÁ el robot después de la latencia
        # Si el robot está girando a last_rotation_speed, en LATENCY_COMPENSATION_MS
        # habrá girado aproximadamente:
        latency_seconds = self.LATENCY_COMPENSATION_MS / 1000.0

        # Estimar rotación adicional por inercia (velocidad actual * tiempo de latencia)
        # Convertir PWM (0-255) a velocidad normalizada (0-1) para estimación
        # Asumimos que el robot gira a ~100 deg/s a velocidad máxima
        speed_normalized = abs(state['last_rotation_speed']) / 255.0
        estimated_rotation_rate_deg_per_sec = speed_normalized * 100.0
        predicted_additional_rotation_deg = estimated_rotation_rate_deg_per_sec * latency_seconds

        # Error PREDICHO (dónde estará el robot, no dónde está)
        if angle_error > 0:
            # Girando en sentido positivo
            predicted_error_deg = angle_error_deg - predicted_additional_rotation_deg
        else:
            # Girando en sentido negativo
            predicted_error_deg = angle_error_deg + predicted_additional_rotation_deg

        predicted_error = math.radians(predicted_error_deg)

        # Si el error PREDICHO está dentro del threshold, DETENER AHORA
        # (anticipando que llegará al objetivo después de la latencia)
        if abs(predicted_error) < self.angle_threshold:
            log.info("🎯 Robot %d STOP PREDICTIVO | Actual=%.1f° Target=%.1f° Error=%.1f° Predicho=%.1f° < Threshold=%.1f°",
                     robot.id, current_normalized, target_normalized, angle_error_deg,
                     predicted_error_deg, math.degrees(self.angle_threshold))
            self._send_motor_commands(robot, 0, 0)
            state['last_rotation_speed'] = 0
            self.last_logged_rotation_speed[robot.id] = 0
            return True

        # Si el error ACTUAL ya está dentro, también detener
        if abs(angle_error) < self.angle_threshold:
            log.info("🎯 Robot %d STOP DIRECTO | Actual=%.1f° Target=%.1f° Error=%.1f° < Threshold=%.1f°",
                     robot.id, current_normalized, target_normalized, angle_error_deg,
                     math.degrees(self.angle_threshold))
            self._send_motor_commands(robot, 0, 0)
            state['last_rotation_speed'] = 0
            self.last_logged_rotation_speed[robot.id] = 0
            return True

        # ===== PID ANGULAR BASADO EN TIEMPO =====
        now_angle = time.time()
        if state['last_pid_time'] == 0:
            dt_angle = 0.033
        else:
            dt_angle = now_angle - state['last_pid_time']
        state['last_pid_time'] = now_angle
        dt_angle = max(0.001, min(0.1, dt_angle))

        # Proporcional
        p_term = self.kp_angle * angle_error

        # Integral (con anti-windup, basado en tiempo)
        if (state['last_error_angle'] * angle_error) < 0:
            state['integral_angle'] = 0
            log.debug("Reset integral angular (cambio de direccion)")
        else:
            state['integral_angle'] += angle_error * dt_angle

        # Anti-windup: limitar integral (en radian·segundos)
        max_integral = 3.0
        state['integral_angle'] = max(-max_integral, min(max_integral, state['integral_angle']))

        i_term = self.ki_angle * state['integral_angle']

        # Derivada: solo recalcular cuando el ángulo CAMBIA (nuevo frame de cámara)
        angle_changed = abs(angle_error - state['last_error_angle']) > 0.001  # ~0.06°

        if angle_changed:
            dt_measurement = now_angle - state.get('last_angle_measurement_time', now_angle)
            if dt_measurement > 0.001:
                state['last_derivative_angle'] = (angle_error - state['last_error_angle']) / dt_measurement
            state['last_angle_measurement_time'] = now_angle
            state['last_error_angle'] = angle_error

        derivative = state.get('last_derivative_angle', 0)
        d_term = self.kd_angle * derivative

        # Calcular velocidad de rotación base del PID
        pid_rotation_speed = p_term + i_term + d_term

        # Aplicar perfil de velocidad suave para rotación
        rotation_speed = self._apply_rotation_profile(
            pid_rotation_speed, abs(angle_error),
            min_speed=robot_min_speed, max_speed=robot_max_speed
        )

        # Log cuando entra en rampa por primera vez
        angle_error_abs = abs(angle_error)
        in_ramp_zone = angle_error_abs <= self.angle_near and angle_error_abs > self.angle_threshold

        # Solo loguear una vez cuando entra a rampa (detectar transición)
        if not hasattr(robot, '_was_in_ramp'):
            robot._was_in_ramp = False

        if in_ramp_zone and not robot._was_in_ramp:
            # Primera vez que entra en rampa
            robot_status_logger.emit_event(
                robot.id,
                f"RAMP ENTRY: ang={current_normalized:+.1f}° tgt={target_normalized:+.1f}° "
                f"err={angle_error_deg:+.1f}°"
            )
            robot._was_in_ramp = True
        elif not in_ramp_zone:
            # Salió de rampa, resetear flag
            robot._was_in_ramp = False

        # ===== COMPENSACIÓN DE ASIMETRÍA POR CALIBRACIÓN =====
        # Si el robot tiene motores con diferente potencia, compensar la velocidad de giro
        if robot.id not in self.rotation_asymmetry_factors:
            self.rotation_asymmetry_factors[robot.id] = self._calculate_rotation_compensation(robot.id)

        cw_comp, ccw_comp = self.rotation_asymmetry_factors[robot.id]

        # Aplicar compensación según dirección
        if angle_error > 0:
            # CCW (antihorario)
            rotation_speed *= ccw_comp
        else:
            # CW (horario)
            rotation_speed *= cw_comp

        # Mantener el signo de la rotación
        if angle_error < 0:
            rotation_speed = -rotation_speed

        # Determinar velocidades de motores para girar
        # INTERCAMBIADOS: left_speed y right_speed
        if rotation_speed > 0:
            # Girar en sentido antihorario (positivo)
            left_speed = rotation_speed
            right_speed = -rotation_speed
        else:
            # Girar en sentido horario (negativo)
            left_speed = rotation_speed  # negativo (hacia atrás)
            right_speed = -rotation_speed  # positivo (hacia adelante)

        # Guardar velocidad de rotación para predictive stopping en la próxima iteración
        state['last_rotation_speed'] = rotation_speed

        # ===== ACTUALIZAR STATUS LOGGER =====
        current_time = time.time()
        last_logged_speed = self.last_logged_rotation_speed.get(robot.id, None)
        last_periodic_time = self.last_periodic_log_time.get(robot.id, 0)

        speed_changed = (last_logged_speed is None or
                        abs(abs(rotation_speed) - abs(last_logged_speed)) >= 1)
        periodic_log_due = (current_time - last_periodic_time) >= self.PERIODIC_LOG_INTERVAL

        if speed_changed or periodic_log_due:
            robot_status_logger.update(
                robot.id,
                ang=current_normalized,
                tgt_ang=target_normalized,
                err_ang=angle_error_deg,
            )
            self.last_logged_rotation_speed[robot.id] = rotation_speed
            self.last_periodic_log_time[robot.id] = current_time

        # Enviar comandos a los motores
        self._send_motor_commands(robot, left_speed, right_speed)

        # Aún no hemos alcanzado el ángulo objetivo
        return False

    def _apply_velocity_profile(self, pid_speed, distance, min_speed=None, max_speed=None):
        """Aplica perfil de velocidad con rampa de desaceleración.

        Zonas:
        - LEJOS (distance > distance_near): PID controla con piso en min_speed
        - RAMPA (distance_near >= distance > threshold): Techo baja de max → min
        - THRESHOLD (distance <= threshold): Detener

        Args:
            pid_speed (float): Velocidad calculada por el PID (normalizada, 0-1+).
            distance (float): Distancia al objetivo en píxeles.
            min_speed (int, optional): PWM mínimo del robot (de calibración JSON).
            max_speed (int, optional): PWM máximo del robot (de calibración JSON).

        Returns:
            int: Velocidad ajustada en PWM.
        """
        # Usar límites per-robot o fallback a config.py
        if min_speed is None:
            min_speed = self.min_motor_speed
        if max_speed is None:
            max_speed = self.max_smooth_speed

        # Si ya llegamos al threshold, detener
        if distance <= self.position_threshold:
            return 0

        # Convertir PID a PWM usando max_speed del robot como escala
        # pid_speed=1.0 → max_speed PWM (100% de la capacidad del robot)
        speed_pwm = int(pid_speed * max_speed)

        # Limitar al máximo del robot
        speed_pwm = min(speed_pwm, max_speed)

        # Aplicar mínimo según la zona
        if distance > self.distance_near:
            # ZONA LEJOS: Piso en min_speed (PWM mínimo donde el robot se mueve)
            speed_pwm = max(speed_pwm, min_speed)
        else:
            # ZONA RAMPA: Desaceleración de max_speed → min_speed
            ramp_factor = (distance - self.position_threshold) / \
                         (self.distance_near - self.position_threshold)

            # Interpolar entre max_speed y min_speed
            ceiling = int(min_speed + (max_speed - min_speed) * ramp_factor)

            # Techo: no superar el valor de rampa
            speed_pwm = min(speed_pwm, ceiling)
            # Piso: no bajar del mínimo del robot
            speed_pwm = max(speed_pwm, min_speed)

        return speed_pwm

    def _apply_rotation_profile(self, pid_rotation_speed, angle_error_abs,
                                min_speed=None, max_speed=None):
        """Aplica perfil de velocidad angular con rampa de desaceleración.

        Zonas:
        - LEJOS (angle_error > angle_near): PID controla con piso en min_speed
        - RAMPA (angle_near >= angle_error > threshold): Techo baja de max → min
        - THRESHOLD (angle_error <= threshold): Detener

        Args:
            pid_rotation_speed (float): Velocidad de rotación calculada por PID.
            angle_error_abs (float): Error angular absoluto en radianes.
            min_speed (int, optional): PWM mínimo del robot (de calibración).
            max_speed (int, optional): PWM máximo del robot (de calibración).

        Returns:
            int: Velocidad de rotación ajustada en PWM (positiva).
        """
        if min_speed is None:
            min_speed = self.min_rotation_speed
        if max_speed is None:
            max_speed = self.max_rotation_speed

        if angle_error_abs <= self.angle_threshold:
            return 0

        # Escalar PID con max_speed del robot (no max_motor_speed=127)
        rotation_speed_pwm = int(abs(pid_rotation_speed) * max_speed)
        rotation_speed_pwm = min(rotation_speed_pwm, max_speed)

        if angle_error_abs > self.angle_near:
            # ZONA LEJOS: Piso en min_speed
            rotation_speed_pwm = max(rotation_speed_pwm, min_speed)
        else:
            # ZONA RAMPA: Desaceleración de max_speed → min_speed
            ramp_factor = (angle_error_abs - self.angle_threshold) / \
                         (self.angle_near - self.angle_threshold)
            ceiling = int(min_speed + (max_speed - min_speed) * ramp_factor)
            rotation_speed_pwm = min(rotation_speed_pwm, ceiling)
            rotation_speed_pwm = max(rotation_speed_pwm, min_speed)

        return rotation_speed_pwm

    def _send_motor_commands(self, robot, left_speed_pwm, right_speed_pwm):
        """Envía comandos PWM a los motores del robot.

        Limita las velocidades al rango válido y actualiza tanto robots
        reales (vía RF) como simulados (actualizando velocidades directamente).

        Args:
            robot: Objeto robot con atributos id, angle, dx, dy, dw.
            left_speed_pwm (int): Velocidad PWM del motor izquierdo (-255 a 255).
            right_speed_pwm (int): Velocidad PWM del motor derecho (-255 a 255).

        Note:
            Para robots reales usa rf_controller, para simulación actualiza
            las velocidades dx, dy, dw del objeto robot directamente.
        """
        # Aplicar factor de dribble si el robot tiene posesión
        if self.dribble_pwm_factor != 1.0:
            left_speed_pwm = left_speed_pwm * self.dribble_pwm_factor
            right_speed_pwm = right_speed_pwm * self.dribble_pwm_factor

        # Aplicar boost anti-atasco (aditivo, preserva dirección del motor)
        _boost = self._pid_state.get(robot.id, {}).get('stuck_boost', 0)
        if _boost > 0:
            left_speed_pwm  += (1 if left_speed_pwm  > 0 else -1 if left_speed_pwm  < 0 else 0) * _boost
            right_speed_pwm += (1 if right_speed_pwm > 0 else -1 if right_speed_pwm < 0 else 0) * _boost

        # Limitar velocidades a rango válido PWM
        left_speed_pwm = int(max(-255, min(255, left_speed_pwm)))
        right_speed_pwm = int(max(-255, min(255, right_speed_pwm)))

        # Registrar PWM final en el status logger (incluye boost y dribble)
        robot_status_logger.update(robot.id, left_pwm=left_speed_pwm, right_pwm=right_speed_pwm)

        # Si tenemos controlador RF, enviar comandos PWM directamente
        if self.rf_controller:
            # Convertir robot_id de Python (0-3) a firmware (1-4)
            firmware_id = robot.id + 1
            self.rf_controller.set_motors(firmware_id, left_speed_pwm, right_speed_pwm)

        # Para simulación, actualizar directamente las velocidades del robot
        # Convertir PWM a velocidad normalizada para simulación
        left_speed_norm = left_speed_pwm / 255.0
        right_speed_norm = right_speed_pwm / 255.0

        linear_velocity = (right_speed_norm + left_speed_norm) / 2
        angular_velocity = (right_speed_norm - left_speed_norm) / self.wheel_distance

        # Calcular velocidad linear en componentes x, y
        current_angle_rad = math.radians(robot.angle)
        robot.dx = linear_velocity * math.cos(current_angle_rad)
        robot.dy = linear_velocity * math.sin(current_angle_rad)

        # Actualizar velocidad angular
        robot.dw = math.degrees(angular_velocity)
