import logging
import math

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
            log.debug("🎯 Predictive stop: error actual=%.1f° predicho=%.1f° < threshold=%.1f°",
                     angle_error_deg, predicted_error_deg, math.degrees(self.angle_threshold))
            self._send_motor_commands(robot, 0, 0)
            self.last_rotation_speed = 0.0
            return True

        # Si el error ACTUAL ya está dentro, también detener
        if abs(angle_error) < self.angle_threshold:
            self._send_motor_commands(robot, 0, 0)
            self.last_rotation_speed = 0.0
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
        rotation_speed = self._apply_rotation_profile(pid_rotation_speed, abs(angle_error))

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
