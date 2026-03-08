import logging
import math
import time

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
    MAX_ANGULAR_CORRECTION_PWM,
    MOTOR_MAX_PWM,
    PID_POSITION_KP,
    PID_POSITION_KI,
    PID_POSITION_KD,
    PID_ANGLE_KP,
    PID_ANGLE_KI,
    PID_ANGLE_KD,
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
    ):
        """Inicializa el controlador con parámetros físicos del robot.

        Args:
            rf_controller: Controlador RF compartido para robots reales.
            wheel_radius (float): Radio de las ruedas en metros. Por defecto 0.025.
            wheel_distance (float): Distancia entre ruedas en metros. Por defecto 0.1.
            max_motor_speed (int): Velocidad máxima PWM (0-255). Por defecto 255.

        Note:
            Los parámetros PID se inicializan con valores predeterminados que pueden
            requerir ajuste según el robot específico.
            Todas las velocidades se manejan internamente como PWM (0-255).
        """
        self.wheel_radius = wheel_radius
        self.wheel_distance = wheel_distance
        self.max_motor_speed = max_motor_speed
        self.rf_controller = rf_controller

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

        # Corrección angular máxima durante movimiento lineal (en PWM)
        # Evita que la corrección haga que un motor vaya muy lento (causa oscilaciones)
        self.MAX_ANGULAR_CORRECTION = MAX_ANGULAR_CORRECTION_PWM  # Máximo ±10 PWM

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
                'last_error_pos': (0, 0),
                'integral_pos': (0, 0),
                'last_error_angle': 0,
                'integral_angle': 0,
                'last_linear_speed': 0,
                'last_rotation_speed': 0,
                'rotation_completed_at': 0,
            }
        return self._pid_state[robot_id]

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

        # Obtener calibración del robot usando velocidad de referencia
        # Con calibración multipoint, max_left/max_right varían según velocidad
        # Usamos 50 PWM como referencia (velocidad media típica en rango 20-80)
        REFERENCE_SPEED = 50
        max_left, max_right, _ = self.rf_controller.calibration.get_calibration_at_speed(
            robot_id, REFERENCE_SPEED
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

    def move_to_position(self, robot, target_pos, target_angle=None):
        """Calcula y envía comandos para mover el robot a una posición específica.

        Utiliza control PID para navegación suave hacia el objetivo. Si el error
        angular es grande, primero rota el robot antes de moverse.

        Args:
            robot: Objeto robot con atributos x, y, angle.
            target_pos (tuple): Posición objetivo como (x, y).
            target_angle (float, optional): Ángulo objetivo en grados.

        Returns:
            bool: True si el robot ha llegado a la posición objetivo, False en caso contrario.

        Example:
            >>> controller.move_to_position(robot, (100, 200), 45)
            False  # Robot aún en movimiento
        """
        # Calcular distancia y ángulo al objetivo
        current_pos = (robot.x, robot.y)
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]

        distance = math.sqrt(dx**2 + dy**2)

        # DEBUG: Log de vectores y distancias (solo cuando se inicia el movimiento)
        if not hasattr(self, '_last_target') or self._last_target != target_pos:
            log.info("📍 Robot(%d, %d) → Target(%d, %d) | dx=%d dy=%d dist=%.1f",
                     int(current_pos[0]), int(current_pos[1]),
                     int(target_pos[0]), int(target_pos[1]),
                     int(dx), int(dy), distance)
            self._last_target = target_pos

        # Obtener estado PID para este robot
        state = self._get_pid_state(robot.id)

        # ===== PREDICTIVE STOPPING LINEAL =====
        # Estimar dónde ESTARÁ el robot después de la latencia
        latency_seconds = self.LATENCY_COMPENSATION_MS / 1000.0

        # Estimar distancia adicional que recorrerá (velocidad actual * latencia)
        # Convertir PWM (0-255) a velocidad normalizada (0-1) para estimación
        # Asumimos ~200 px/s a velocidad máxima (depende del campo, ajustar si es necesario)
        speed_normalized = abs(state['last_linear_speed']) / 255.0
        estimated_speed_px_per_sec = speed_normalized * 200.0
        predicted_additional_distance_px = estimated_speed_px_per_sec * latency_seconds

        # Distancia PREDICHA (dónde estará el robot, no dónde está)
        predicted_distance = distance - predicted_additional_distance_px

        # Si la distancia PREDICHA está dentro del threshold, DETENER AHORA
        if predicted_distance < self.position_threshold and state['last_linear_speed'] > 0:
            log.info("🎯 Robot %d STOP PREDICTIVO LINEAL | Dist=%.1f px Predicha=%.1f px < Threshold=%d px",
                     robot.id, distance, predicted_distance, self.position_threshold)
            self._send_motor_commands(robot, 0, 0)
            state['last_linear_speed'] = 0
            if target_angle is not None:
                return self.rotate_to_angle(robot, target_angle)
            return True

        # Si estamos suficientemente cerca del objetivo (verificación directa)
        if distance < self.position_threshold:
            log.info("🎯 Waypoint alcanzado: dist=%.1f px < threshold=%d px",
                    distance, self.position_threshold)
            if target_angle is not None:
                # Alcanzamos la posición, ajustar orientación final
                return self.rotate_to_angle(robot, target_angle)
            else:
                # Detener el robot
                self._send_motor_commands(robot, 0, 0)
                return True

        # Calcular ángulo hacia el objetivo (en radianes)
        target_heading = math.atan2(dy, dx)

        # El ángulo del robot YA está en radianes (convertido en control_process)
        current_angle_rad = robot.angle

        # Calcular error de ángulo (normalizado entre -pi y pi)
        angle_error = _normalize_angle(target_heading - current_angle_rad)

        # ===== CONTROL HÍBRIDO: Girar en lugar vs Corregir mientras se mueve =====
        # Umbral configurable para decidir comportamiento (desde config.py)
        # 30° = conservador pero eficiente, 45° = más agresivo
        LARGE_ANGLE_ERROR_THRESHOLD = math.radians(ROBOT_LINEAR_START_ANGLE_THRESHOLD_DEG)

        # FASE 1: Error angular MUY grande (>umbral) → Girar en lugar
        # Evita movimientos inestables cuando el robot apunta en dirección muy incorrecta
        # Cooldown: tras completar rotación, esperar a que la cámara actualice antes
        # de re-evaluar, evitando oscilación stop-start con datos obsoletos
        rotation_cooldown_s = self.LATENCY_COMPENSATION_MS * 2 / 1000.0
        in_cooldown = (time.time() - state.get('rotation_completed_at', 0)) < rotation_cooldown_s

        if abs(angle_error) > LARGE_ANGLE_ERROR_THRESHOLD and not in_cooldown:
            # Girar en su lugar hasta estar razonablemente orientado
            angle_error_deg = math.degrees(angle_error)
            log.debug("🔄 Robot %d | Error angular grande: %.1f° > %.0f° → Girando en lugar",
                     robot.id, abs(angle_error_deg), ROBOT_LINEAR_START_ANGLE_THRESHOLD_DEG)
            reached = self.rotate_to_angle(robot, math.degrees(target_heading))
            if reached:
                state['rotation_completed_at'] = time.time()
            return False  # Todavía NO hemos llegado al waypoint

        # FASE 2: Error angular pequeño (≤umbral) → Moverse MIENTRAS corrige
        # La función _calculate_differential_speeds() ajusta velocidades L/R
        # para corregir la orientación durante el movimiento lineal

        # Implementar PID para la posición
        p_term = self.kp_pos * distance

        # Calcular integral y derivada
        state['integral_pos'] = (state['integral_pos'][0] + dx, state['integral_pos'][1] + dy)
        i_term = self.ki_pos * math.sqrt(
            state['integral_pos'][0] ** 2 + state['integral_pos'][1] ** 2
        )

        derivative_pos = (dx - state['last_error_pos'][0], dy - state['last_error_pos'][1])
        d_term = self.kd_pos * math.sqrt(
            derivative_pos[0] ** 2 + derivative_pos[1] ** 2
        )

        # Actualizar último error
        state['last_error_pos'] = (dx, dy)

        # Calcular velocidad base del PID
        pid_speed = p_term + i_term + d_term

        # Aplicar perfil de velocidad suave (Solución 1, 2 y 4 combinadas)
        speed = self._apply_velocity_profile(pid_speed, distance)

        # Ajustar velocidad basada en el error de ángulo
        left_speed, right_speed = self._calculate_differential_speeds(
            speed, angle_error
        )

        # ===== LOGGING PERIÓDICO MOVIMIENTO LINEAL =====
        current_time = time.time()
        last_logged_linear = self.last_logged_linear_speed.get(robot.id, None)
        last_periodic_time_linear = self.last_periodic_log_time_linear.get(robot.id, 0)

        speed_changed = (last_logged_linear is None or abs(speed - last_logged_linear) >= 1)
        periodic_log_due = (current_time - last_periodic_time_linear) >= self.PERIODIC_LOG_INTERVAL

        if speed_changed or periodic_log_due:
            # Determinar zona
            if distance <= self.position_threshold:
                zone_linear = "THRESHOLD"
            elif distance <= self.distance_near:
                zone_linear = "RAMPA"
            else:
                zone_linear = "LEJOS"

            # Determinar límites esperados (en PWM)
            if zone_linear == "LEJOS":
                expected_min = self.min_motor_speed
                expected_max = self.max_smooth_speed
                limits_info = f"Límites[{expected_min}-{expected_max}PWM]"
            elif zone_linear == "RAMPA":
                expected_min = self.linear_near_min
                expected_max = self.max_smooth_speed
                limits_info = f"Límites[{expected_min}-{expected_max}PWM]"
            else:  # THRESHOLD
                limits_info = "Límites[0-threshold]"

            # Log con corrección angular simultánea
            angle_error_deg = math.degrees(angle_error)
            if speed_changed:
                log.info("🚗 Robot %d | Pos(%.1f,%.1f)→(%.1f,%.1f) Dist=%.1fpx | Zona=%s | PWM=%d %s | Error∠=%.1f° | (L=%d R=%d)",
                        robot.id, robot.x, robot.y, target_pos[0], target_pos[1], distance,
                        zone_linear, speed, limits_info, angle_error_deg, left_speed, right_speed)
            else:
                log.info("⏱️  Robot %d | Pos(%.1f,%.1f)→(%.1f,%.1f) Dist=%.1fpx | Zona=%s | PWM=%d %s | Error∠=%.1f° | (L=%d R=%d)",
                        robot.id, robot.x, robot.y, target_pos[0], target_pos[1], distance,
                        zone_linear, speed, limits_info, angle_error_deg, left_speed, right_speed)

            self.last_logged_linear_speed[robot.id] = speed
            self.last_periodic_log_time_linear[robot.id] = current_time

        # Actualizar última velocidad lineal para predicción
        state['last_linear_speed'] = speed

        # Enviar comandos a los motores
        self._send_motor_commands(robot, left_speed, right_speed)

        # Aún no hemos llegado
        return False

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

        # Obtener estado PID para este robot
        state = self._get_pid_state(robot.id)

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
                log.info("🎯 Robot %d CRUCE DE OBJETIVO | Actual=%.1f° Target=%.1f° Error: %.1f° → %.1f°",
                         robot.id, current_normalized, target_normalized,
                         robot._last_angle_error_deg, angle_error_deg)
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

        # Implementar PID para el ángulo
        p_term = self.kp_angle * angle_error

        # Calcular integral (con anti-windup)
        # Si el error cambió de signo, resetear integral para evitar overshoot
        if (state['last_error_angle'] * angle_error) < 0:
            # Error cambió de signo → resetear integral
            state['integral_angle'] = 0
            log.debug("🔄 Reset integral angular (cambio de dirección)")
        else:
            state['integral_angle'] += angle_error

        # Anti-windup: limitar integral
        max_integral = 0.5  # Limitar contribución integral
        state['integral_angle'] = max(-max_integral, min(max_integral, state['integral_angle']))

        i_term = self.ki_angle * state['integral_angle']

        # Calcular derivada
        derivative = angle_error - state['last_error_angle']
        d_term = self.kd_angle * derivative

        # Actualizar último error
        state['last_error_angle'] = angle_error

        # Calcular velocidad de rotación base del PID
        pid_rotation_speed = p_term + i_term + d_term

        # Aplicar perfil de velocidad suave para rotación
        rotation_speed = self._apply_rotation_profile(pid_rotation_speed, abs(angle_error))

        # Log cuando entra en rampa por primera vez
        angle_error_abs = abs(angle_error)
        in_ramp_zone = angle_error_abs <= self.angle_near and angle_error_abs > self.angle_threshold

        # Solo loguear una vez cuando entra a rampa (detectar transición)
        if not hasattr(robot, '_was_in_ramp'):
            robot._was_in_ramp = False

        if in_ramp_zone and not robot._was_in_ramp:
            # Primera vez que entra en rampa
            log.info("📉 Robot %d ENTRA EN RAMPA | Actual=%.1f° Target=%.1f° Error=%.1f° | Speed antes=%.3f",
                    robot.id, current_normalized, target_normalized, angle_error_deg,
                    abs(rotation_speed))
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

        # ===== LOGGING DETALLADO =====
        current_time = time.time()
        last_logged_speed = self.last_logged_rotation_speed.get(robot.id, None)
        last_periodic_time = self.last_periodic_log_time.get(robot.id, 0)

        speed_changed = (last_logged_speed is None or
                        abs(abs(rotation_speed) - abs(last_logged_speed)) >= 1)
        periodic_log_due = (current_time - last_periodic_time) >= self.PERIODIC_LOG_INTERVAL

        # Determinar zona
        angle_error_abs = abs(angle_error)
        if angle_error_abs <= self.angle_threshold:
            zone = "THRESHOLD"
        elif angle_error_abs <= self.angle_near:
            zone = "RAMPA"
        else:
            zone = "LEJOS"

        # Log cuando cambia velocidad O cada 250ms
        if speed_changed or periodic_log_due:
            # Verificar límites
            speed_abs = abs(rotation_speed)
            limits_info = ""

            # Comparar con límites configurados
            if zone == "LEJOS":
                expected_min = self.min_rotation_speed
                expected_max = self.max_rotation_speed
                limits_info = f"Límites[{expected_min}-{expected_max}PWM]"
            elif zone == "RAMPA":
                expected_min = self.rotation_near_min
                expected_max = self.max_rotation_speed
                limits_info = f"Límites[{expected_min}-{expected_max}PWM]"
            else:  # THRESHOLD
                limits_info = "Límites[0-threshold]"

            # Log tipo según razón
            if speed_changed:
                log.info("🔄 Robot %d | Actual=%.1f° Target=%.1f° Error=%.1f° | Zona=%s | PWM=%d %s | (L=%d R=%d)",
                        robot.id, current_normalized, target_normalized, angle_error_deg,
                        zone, speed_abs, limits_info, left_speed, right_speed)
            else:
                log.info("⏱️  Robot %d | Actual=%.1f° Target=%.1f° Error=%.1f° | Zona=%s | PWM=%d %s | (L=%d R=%d)",
                        robot.id, current_normalized, target_normalized, angle_error_deg,
                        zone, speed_abs, limits_info, left_speed, right_speed)

            self.last_logged_rotation_speed[robot.id] = rotation_speed
            self.last_periodic_log_time[robot.id] = current_time

        # Enviar comandos a los motores
        self._send_motor_commands(robot, left_speed, right_speed)

        # Aún no hemos alcanzado el ángulo objetivo
        return False

    def _apply_velocity_profile(self, pid_speed, distance):
        """Aplica perfil de velocidad con rampa de desaceleración.

        Zonas:
        - LEJOS (distance > distance_near): MIN/MAX absolutos
        - RAMPA (distance_near >= distance > threshold): Rampa lineal MIN → NEAR_MIN
        - THRESHOLD (distance <= threshold): Detener

        Args:
            pid_speed (float): Velocidad calculada por el PID (normalizada).
            distance (float): Distancia al objetivo en píxeles.

        Returns:
            int: Velocidad ajustada en PWM (0-255).
        """
        # Si ya llegamos al threshold, detener
        if distance <= self.position_threshold:
            return 0

        # Convertir PID (normalizado) a PWM
        # PID trabaja con valores pequeños (kp=0.008), necesitamos escalar
        speed_pwm = int(pid_speed * self.max_motor_speed)

        # Limitar al máximo SIEMPRE
        speed_pwm = min(speed_pwm, self.max_smooth_speed)

        # Aplicar mínimo según la zona
        if distance > self.distance_near:
            # ZONA LEJOS: Aplicar MIN absoluto
            speed_pwm = max(speed_pwm, self.min_motor_speed)
        else:
            # ZONA RAMPA: Desaceleración lineal desde MIN hasta NEAR_MIN
            # ramp_factor va de 1.0 (en distance_near) a 0.0 (en position_threshold)
            ramp_factor = (distance - self.position_threshold) / \
                         (self.distance_near - self.position_threshold)

            # Interpolar entre max_smooth_speed y linear_near_min (ambos en PWM)
            # Usa max_smooth_speed como tope para transición suave desde LEJOS
            min_speed_in_ramp = int(self.linear_near_min + \
                               (self.max_smooth_speed - self.linear_near_min) * ramp_factor)

            # Forzar velocidad al valor de la rampa (techo, no piso)
            speed_pwm = min(speed_pwm, min_speed_in_ramp)

        return speed_pwm

    def _apply_rotation_profile(self, pid_rotation_speed, angle_error_abs):
        """Aplica perfil de velocidad angular con rampa de desaceleración.

        Zonas:
        - LEJOS (angle_error > angle_near): MIN/MAX absolutos
        - RAMPA (angle_near >= angle_error > threshold): Rampa lineal MIN → NEAR_MIN
        - THRESHOLD (angle_error <= threshold): Detener

        Args:
            pid_rotation_speed (float): Velocidad de rotación calculada por PID (normalizada).
            angle_error_abs (float): Error angular absoluto en radianes.

        Returns:
            int: Velocidad de rotación ajustada en PWM (0-255, positiva).
        """
        # Si ya llegamos al threshold, detener
        if angle_error_abs <= self.angle_threshold:
            return 0

        # Convertir PID (normalizado) a PWM y tomar valor absoluto
        rotation_speed_pwm = int(abs(pid_rotation_speed) * self.max_motor_speed)

        # Limitar al máximo SIEMPRE
        rotation_speed_pwm = min(rotation_speed_pwm, self.max_rotation_speed)

        # Aplicar mínimo según la zona
        if angle_error_abs > self.angle_near:
            # ZONA LEJOS: Aplicar MIN absoluto
            rotation_speed_pwm = max(rotation_speed_pwm, self.min_rotation_speed)
        else:
            # ZONA RAMPA: Desaceleración lineal desde MIN hasta NEAR_MIN
            # ramp_factor va de 1.0 (en angle_near) a 0.0 (en angle_threshold)
            ramp_factor = (angle_error_abs - self.angle_threshold) / \
                         (self.angle_near - self.angle_threshold)

            # Interpolar entre max_rotation_speed y rotation_near_min (ambos en PWM)
            # Usa max_rotation_speed como tope para transición suave desde LEJOS
            min_rotation_in_ramp = int(self.rotation_near_min + \
                                  (self.max_rotation_speed - self.rotation_near_min) * ramp_factor)

            # Forzar velocidad al valor de la rampa (techo, no piso)
            rotation_speed_pwm = min(rotation_speed_pwm, min_rotation_in_ramp)

        return rotation_speed_pwm

    def _calculate_differential_speeds(self, speed_pwm, angle_error):
        """Calcula las velocidades diferenciales para los motores.

        Ajusta las velocidades de los motores izquierdo y derecho para
        lograr el movimiento lineal deseado con corrección angular LIMITADA.

        IMPORTANTE: Diseñado para corrección suave durante movimiento,
        NO para rotación pura (usa rotate_to_angle para eso).

        Args:
            speed_pwm (int): Velocidad lineal deseada en PWM (0-255).
            angle_error (float): Error de ángulo en radianes.

        Returns:
            tuple: Tupla con velocidades PWM (izquierda, derecha).

        Note:
            - Limita corrección a MAX_ANGULAR_CORRECTION (PWM) para evitar oscilaciones
            - Mantiene velocidad lineal promedio cercana a 'speed_pwm'
            - NO normaliza agresivamente (evita robot muy lento)
        """
        # Factor de corrección basado SOLO en proporcional (sin i, sin d)
        # El kp_angle es bajo (0.08) para corrección suave
        # Convertir angle_error a corrección en PWM
        correction_normalized = self.kp_angle * angle_error
        correction_pwm = int(correction_normalized * self.max_motor_speed)

        # LIMITAR corrección para evitar un motor muy lento
        # Ejemplo: Si speed=90 PWM y MAX=10 PWM:
        #   correction máx = ±10 → left=100, right=80 (razonable)
        #   Sin límite con error 30°: correction=~26 → left=116, right=64 (asimétrico)
        correction_pwm = max(-self.MAX_ANGULAR_CORRECTION,
                        min(self.MAX_ANGULAR_CORRECTION, correction_pwm))

        # Calcular velocidades de los motores (en PWM)
        # INTERCAMBIADAS para coincidir con el sentido de rotación corregido
        left_speed_pwm = speed_pwm + correction_pwm
        right_speed_pwm = speed_pwm - correction_pwm

        # Clip individual (sin normalización que reduce velocidad total)
        # Esto permite que un motor vaya al máximo mientras el otro ajusta
        left_speed_pwm = max(-self.max_motor_speed, min(self.max_motor_speed, left_speed_pwm))
        right_speed_pwm = max(-self.max_motor_speed, min(self.max_motor_speed, right_speed_pwm))

        return left_speed_pwm, right_speed_pwm

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
        # Limitar velocidades a rango válido PWM
        left_speed_pwm = int(max(-255, min(255, left_speed_pwm)))
        right_speed_pwm = int(max(-255, min(255, right_speed_pwm)))

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

        log.debug(
            "Robot %i - PWM: L=%d, R=%d - Linear: (%.2f, %.2f) - Angular: %.2f",
            robot.id,
            left_speed_pwm,
            right_speed_pwm,
            robot.dx,
            robot.dy,
            robot.dw,
        )
