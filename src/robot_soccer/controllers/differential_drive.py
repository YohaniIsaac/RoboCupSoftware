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
    ROBOT_POSITION_THRESHOLD,
    ROBOT_ANGLE_THRESHOLD_DEG,
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

    Traduce comandos de movimiento de alto nivel a velocidades de motores
    utilizando control PID para posición y orientación.

    Attributes:
        wheel_radius (float): Radio de las ruedas en metros.
        wheel_distance (float): Distancia entre las ruedas en metros.
        max_motor_speed (float): Velocidad máxima de los motores (0 a 1).
        rf_controller: Controlador RF para robots reales.
        position_threshold (int): Umbral de distancia en píxeles.
        angle_threshold (float): Umbral de ángulo en radianes.
    """

    def __init__(
        self,
        rf_controller=None,
        wheel_radius=0.025,
        wheel_distance=0.1,
        max_motor_speed=1.0,
    ):
        """Inicializa el controlador con parámetros físicos del robot.

        Args:
            rf_controller: Controlador RF compartido para robots reales.
            wheel_radius (float): Radio de las ruedas en metros. Por defecto 0.025.
            wheel_distance (float): Distancia entre ruedas en metros. Por defecto 0.1.
            max_motor_speed (float): Velocidad máxima de motores (0 a 1). Por defecto 1.0.

        Note:
            Los parámetros PID se inicializan con valores predeterminados que pueden
            requerir ajuste según el robot específico.
        """
        self.wheel_radius = wheel_radius
        self.wheel_distance = wheel_distance
        self.max_motor_speed = max_motor_speed
        self.rf_controller = rf_controller

        # Parámetros para PID
        self.last_error_pos = (0, 0)
        self.integral_pos = (0, 0)
        self.last_error_angle = 0
        self.integral_angle = 0

        # Constantes para PID (ajustadas para respuesta suave pero precisa)
        self.kp_pos = 0.008          # Reducido de 0.5 a 0.008
        self.ki_pos = 0.0001         # Reducido de 0.01 a 0.0001
        self.kd_pos = 0.05           # Reducido de 0.1 a 0.05
        self.kp_angle = 0.25         # Aumentado de 0.15 a 0.25 para mejor tracking
        self.ki_angle = 0.001        # Reducido de 0.002 a 0.001 (menos integral windup)
        self.kd_angle = 0.08         # Aumentado de 0.05 a 0.08 para mejor damping

        # Compensación de latencia para predictive stopping
        self.LATENCY_COMPENSATION_MS = 55  # ms totales de latencia (percepción + comm + firmware)
        self.last_rotation_speed = 0.0  # Última velocidad angular enviada
        self.last_linear_speed = 0.0    # Última velocidad lineal enviada

        # Umbrales de distancia (desde config.py)
        self.position_threshold = ROBOT_POSITION_THRESHOLD
        self.angle_threshold = math.radians(ROBOT_ANGLE_THRESHOLD_DEG)

        # Parámetros de perfil de velocidad lineal (desde config.py)
        self.min_motor_speed = ROBOT_MIN_LINEAR_SPEED  # MIN cuando LEJOS
        self.max_smooth_speed = ROBOT_MAX_LINEAR_SPEED  # MAX
        self.distance_near = ROBOT_LINEAR_ARRIVAL_DISTANCE  # Donde empieza rampa
        self.linear_near_min = ROBOT_LINEAR_NEAR_MIN  # MIN en la rampa

        # Parámetros de perfil de velocidad angular (desde config.py)
        self.min_rotation_speed = ROBOT_MIN_ROTATION_SPEED  # MIN cuando LEJOS
        self.max_rotation_speed = ROBOT_MAX_ROTATION_SPEED  # MAX
        self.angle_near = math.radians(ROBOT_ROTATION_ARRIVAL_ANGLE_DEG)  # Donde empieza rampa
        self.rotation_near_min = ROBOT_ROTATION_NEAR_MIN  # MIN en la rampa

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

        # Obtener calibración del robot
        max_left, max_right, _ = self.rf_controller.calibration.get_calibration(robot_id)

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

        # Si estamos suficientemente cerca del objetivo
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

        # Convertir ángulo del robot a radianes
        current_angle_rad = math.radians(robot.angle)

        # Calcular error de ángulo (normalizado entre -pi y pi)
        angle_error = _normalize_angle(target_heading - current_angle_rad)

        # COMPORTAMIENTO SIMPLIFICADO: Primero orientar, luego mover
        # FASE 1: Si NO estamos bien orientados, SOLO girar (sin moverse)
        # Usar el MISMO threshold que rotate_to_angle para evitar loops infinitos
        if abs(angle_error) > self.angle_threshold:
            # Girar en su lugar hasta estar bien orientado
            # NO propagar el return True de rotate_to_angle, solo estamos orientando
            self.rotate_to_angle(robot, math.degrees(target_heading))
            return False  # Todavía NO hemos llegado al waypoint

        # FASE 2: Ya estamos bien orientados, ahora SÍ moverse

        # Implementar PID para la posición
        p_term = self.kp_pos * distance

        # Calcular integral y derivada
        self.integral_pos = (self.integral_pos[0] + dx, self.integral_pos[1] + dy)
        i_term = self.ki_pos * math.sqrt(
            self.integral_pos[0] ** 2 + self.integral_pos[1] ** 2
        )

        derivative_pos = (dx - self.last_error_pos[0], dy - self.last_error_pos[1])
        d_term = self.kd_pos * math.sqrt(
            derivative_pos[0] ** 2 + derivative_pos[1] ** 2
        )

        # Actualizar último error
        self.last_error_pos = (dx, dy)

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

        speed_changed = (last_logged_linear is None or abs(speed - last_logged_linear) >= 0.01)
        periodic_log_due = (current_time - last_periodic_time_linear) >= self.PERIODIC_LOG_INTERVAL

        if speed_changed or periodic_log_due:
            # Determinar zona
            if distance <= self.position_threshold:
                zone_linear = "THRESHOLD"
            elif distance <= self.distance_near:
                zone_linear = "RAMPA"
            else:
                zone_linear = "LEJOS"

            # Determinar límites esperados
            if zone_linear == "LEJOS":
                expected_min = self.min_motor_speed
                expected_max = self.max_smooth_speed
                limits_info = f"Límites[{expected_min:.3f}-{expected_max:.3f}]"
            elif zone_linear == "RAMPA":
                expected_min = self.linear_near_min
                expected_max = self.min_motor_speed
                limits_info = f"Límites[{expected_min:.3f}-{expected_max:.3f}]"
            else:  # THRESHOLD
                limits_info = "Límites[0.000-threshold]"

            # Log
            if speed_changed:
                log.info("🚗 Robot %d | Pos(%.1f,%.1f)→(%.1f,%.1f) Dist=%.1fpx | Zona=%s | Speed=%.3f %s | (L=%.3f R=%.3f)",
                        robot.id, robot.x, robot.y, target_pos[0], target_pos[1], distance,
                        zone_linear, speed, limits_info, left_speed, right_speed)
            else:
                log.info("⏱️  Robot %d | Pos(%.1f,%.1f)→(%.1f,%.1f) Dist=%.1fpx | Zona=%s | Speed=%.3f %s | (L=%.3f R=%.3f)",
                        robot.id, robot.x, robot.y, target_pos[0], target_pos[1], distance,
                        zone_linear, speed, limits_info, left_speed, right_speed)

            self.last_logged_linear_speed[robot.id] = speed
            self.last_periodic_log_time_linear[robot.id] = current_time

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

        # Normalizar ambos ángulos
        current_normalized = normalize_degrees(robot.angle)
        target_normalized = normalize_degrees(target_angle_deg)

        # Calcular error angular en el camino MÁS CORTO
        angle_error_deg = target_normalized - current_normalized
        # Normalizar el error para tomar el camino más corto
        angle_error_deg = normalize_degrees(angle_error_deg)

        # Convertir error a radianes
        angle_error = math.radians(angle_error_deg)

        # ===== DETECCIÓN DE CRUCE DEL OBJETIVO =====
        # Si el error cambió de signo, significa que CRUZAMOS el objetivo
        # Esto ocurre cuando el robot va demasiado rápido y salta por encima del threshold
        if hasattr(robot, '_last_angle_error_sign'):
            current_sign = 1 if angle_error >= 0 else -1
            if robot._last_angle_error_sign != current_sign and robot._last_angle_error_sign != 0:
                # CRUZAMOS el objetivo! Detener inmediatamente
                log.info("🎯 Robot %d CRUCE DE OBJETIVO | Actual=%.1f° Target=%.1f° Error cambió: %.1f° → %.1f°",
                         robot.id, current_normalized, target_normalized,
                         math.degrees(self.last_error_angle), angle_error_deg)
                self._send_motor_commands(robot, 0, 0)
                self.last_rotation_speed = 0.0
                self.last_logged_rotation_speed[robot.id] = 0.0
                robot._last_angle_error_sign = 0
                return True

        # Guardar signo del error para próxima iteración
        robot._last_angle_error_sign = 1 if angle_error >= 0 else -1

        # ===== PREDICTIVE STOPPING =====
        # Estimar dónde ESTARÁ el robot después de la latencia
        # Si el robot está girando a last_rotation_speed, en LATENCY_COMPENSATION_MS
        # habrá girado aproximadamente:
        latency_seconds = self.LATENCY_COMPENSATION_MS / 1000.0

        # Estimar rotación adicional por inercia (velocidad actual * tiempo de latencia)
        # Asumimos que la velocidad angular es proporcional a last_rotation_speed
        # y que el robot gira a ~100 deg/s a velocidad 1.0
        estimated_rotation_rate_deg_per_sec = abs(self.last_rotation_speed) * 100.0
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
            self.last_rotation_speed = 0.0
            self.last_logged_rotation_speed[robot.id] = 0.0
            return True

        # Si el error ACTUAL ya está dentro, también detener
        if abs(angle_error) < self.angle_threshold:
            log.info("🎯 Robot %d STOP DIRECTO | Actual=%.1f° Target=%.1f° Error=%.1f° < Threshold=%.1f°",
                     robot.id, current_normalized, target_normalized, angle_error_deg,
                     math.degrees(self.angle_threshold))
            self._send_motor_commands(robot, 0, 0)
            self.last_rotation_speed = 0.0
            self.last_logged_rotation_speed[robot.id] = 0.0
            return True

        # Implementar PID para el ángulo
        p_term = self.kp_angle * angle_error

        # Calcular integral (con anti-windup)
        # Si el error cambió de signo, resetear integral para evitar overshoot
        if (self.last_error_angle * angle_error) < 0:
            # Error cambió de signo → resetear integral
            self.integral_angle = 0
            log.debug("🔄 Reset integral angular (cambio de dirección)")
        else:
            self.integral_angle += angle_error

        # Anti-windup: limitar integral
        max_integral = 0.5  # Limitar contribución integral
        self.integral_angle = max(-max_integral, min(max_integral, self.integral_angle))

        i_term = self.ki_angle * self.integral_angle

        # Calcular derivada
        derivative = angle_error - self.last_error_angle
        d_term = self.kd_angle * derivative

        # Actualizar último error
        self.last_error_angle = angle_error

        # Calcular velocidad de rotación base del PID
        pid_rotation_speed = p_term + i_term + d_term

        # Aplicar perfil de velocidad suave para rotación
        rotation_speed_before_profile = abs(pid_rotation_speed)
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
        self.last_rotation_speed = rotation_speed

        # ===== LOGGING DETALLADO =====
        current_time = time.time()
        last_logged_speed = self.last_logged_rotation_speed.get(robot.id, None)
        last_periodic_time = self.last_periodic_log_time.get(robot.id, 0)

        speed_changed = (last_logged_speed is None or
                        abs(abs(rotation_speed) - abs(last_logged_speed)) >= 0.01)
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
                limits_info = f"Límites[{expected_min:.3f}-{expected_max:.3f}]"
            elif zone == "RAMPA":
                expected_min = self.rotation_near_min
                expected_max = self.min_rotation_speed
                limits_info = f"Límites[{expected_min:.3f}-{expected_max:.3f}]"
            else:  # THRESHOLD
                limits_info = "Límites[0.000-threshold]"

            # Log tipo según razón
            if speed_changed:
                log.info("🔄 Robot %d | Actual=%.1f° Target=%.1f° Error=%.1f° | Zona=%s | Speed=%.3f %s | (L=%.3f R=%.3f)",
                        robot.id, current_normalized, target_normalized, angle_error_deg,
                        zone, speed_abs, limits_info, left_speed, right_speed)
            else:
                log.info("⏱️  Robot %d | Actual=%.1f° Target=%.1f° Error=%.1f° | Zona=%s | Speed=%.3f %s | (L=%.3f R=%.3f)",
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
            pid_speed (float): Velocidad calculada por el PID.
            distance (float): Distancia al objetivo en píxeles.

        Returns:
            float: Velocidad ajustada.
        """
        # Si ya llegamos al threshold, detener
        if distance <= self.position_threshold:
            return 0.0

        # Calcular velocidad base del PID
        speed = pid_speed

        # Limitar al máximo SIEMPRE
        speed = min(speed, self.max_smooth_speed)

        # Aplicar mínimo según la zona
        if distance > self.distance_near:
            # ZONA LEJOS: Aplicar MIN absoluto
            speed = max(speed, self.min_motor_speed)
        else:
            # ZONA RAMPA: Desaceleración lineal desde MIN hasta NEAR_MIN
            # ramp_factor va de 1.0 (en distance_near) a 0.0 (en position_threshold)
            ramp_factor = (distance - self.position_threshold) / \
                         (self.distance_near - self.position_threshold)

            # Interpolar entre min_motor_speed y linear_near_min
            min_speed_in_ramp = self.linear_near_min + \
                               (self.min_motor_speed - self.linear_near_min) * ramp_factor

            # Aplicar este mínimo dinámico
            speed = max(speed, min_speed_in_ramp)

        return speed

    def _apply_rotation_profile(self, pid_rotation_speed, angle_error_abs):
        """Aplica perfil de velocidad angular con rampa de desaceleración.

        Zonas:
        - LEJOS (angle_error > angle_near): MIN/MAX absolutos
        - RAMPA (angle_near >= angle_error > threshold): Rampa lineal MIN → NEAR_MIN
        - THRESHOLD (angle_error <= threshold): Detener

        Args:
            pid_rotation_speed (float): Velocidad de rotación calculada por PID.
            angle_error_abs (float): Error angular absoluto en radianes.

        Returns:
            float: Velocidad de rotación ajustada (positiva).
        """
        # Si ya llegamos al threshold, detener
        if angle_error_abs <= self.angle_threshold:
            return 0.0

        # Calcular velocidad base del PID (tomar valor absoluto)
        rotation_speed = abs(pid_rotation_speed)

        # Limitar al máximo SIEMPRE
        rotation_speed = min(rotation_speed, self.max_rotation_speed)

        # Aplicar mínimo según la zona
        if angle_error_abs > self.angle_near:
            # ZONA LEJOS: Aplicar MIN absoluto
            rotation_speed = max(rotation_speed, self.min_rotation_speed)
        else:
            # ZONA RAMPA: Desaceleración lineal desde MIN hasta NEAR_MIN
            # ramp_factor va de 1.0 (en angle_near) a 0.0 (en angle_threshold)
            ramp_factor = (angle_error_abs - self.angle_threshold) / \
                         (self.angle_near - self.angle_threshold)

            # Interpolar entre min_rotation_speed y rotation_near_min
            min_rotation_in_ramp = self.rotation_near_min + \
                                  (self.min_rotation_speed - self.rotation_near_min) * ramp_factor

            # Aplicar este mínimo dinámico
            rotation_speed = max(rotation_speed, min_rotation_in_ramp)

        return rotation_speed

    def _calculate_differential_speeds(self, speed, angle_error):
        """Calcula las velocidades diferenciales para los motores.

        Ajusta las velocidades de los motores izquierdo y derecho para
        lograr el movimiento lineal deseado con corrección angular.

        Args:
            speed (float): Velocidad lineal deseada.
            angle_error (float): Error de ángulo en radianes.

        Returns:
            tuple: Tupla con velocidades (izquierda, derecha).

        Note:
            Las velocidades se normalizan para mantener el rango válido
            sin perder la proporción relativa entre motores.
        """
        # Factor de corrección basado en el error de ángulo
        correction = self.kp_angle * angle_error

        # Calcular velocidades de los motores
        # INTERCAMBIADAS para coincidir con el sentido de rotación corregido
        left_speed = speed + correction
        right_speed = speed - correction

        # Normalizar manteniendo la proporción
        max_speed = max(abs(left_speed), abs(right_speed))
        if max_speed > self.max_motor_speed:
            factor = self.max_motor_speed / max_speed
            left_speed *= factor
            right_speed *= factor

        return left_speed, right_speed

    def _send_motor_commands(self, robot, left_speed, right_speed):
        """Envía comandos a los motores del robot.

        Limita las velocidades al rango válido y actualiza tanto robots
        reales (vía RF) como simulados (actualizando velocidades directamente).

        Args:
            robot: Objeto robot con atributos id, angle, dx, dy, dw.
            left_speed (float): Velocidad del motor izquierdo (-1 a 1).
            right_speed (float): Velocidad del motor derecho (-1 a 1).

        Note:
            Para robots reales usa rf_controller, para simulación actualiza
            las velocidades dx, dy, dw del objeto robot directamente.
        """
        # Limitar velocidades a rango válido
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))

        # Si tenemos controlador RF, enviar comandos
        if self.rf_controller:
            # Convertir robot_id de Python (0-3) a firmware (1-4)
            firmware_id = robot.id + 1
            self.rf_controller.set_motors(firmware_id, left_speed, right_speed)

        # Para simulación, actualizar directamente las velocidades del robot
        linear_velocity = (right_speed + left_speed) / 2
        angular_velocity = (right_speed - left_speed) / self.wheel_distance

        # Calcular velocidad linear en componentes x, y
        current_angle_rad = math.radians(robot.angle)
        robot.dx = linear_velocity * math.cos(current_angle_rad)
        robot.dy = linear_velocity * math.sin(current_angle_rad)

        # Actualizar velocidad angular
        robot.dw = math.degrees(angular_velocity)

        log.debug(
            "Robot %i - Velocidades: L=%.2f, R=%.2f - Linear: (%.2f, %.2f) - Angular: %.2f",
            robot.id,
            left_speed,
            right_speed,
            robot.dx,
            robot.dy,
            robot.dw,
        )
