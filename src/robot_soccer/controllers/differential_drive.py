import logging
import math

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

        # Constantes para PID
        self.kp_pos = 0.5
        self.ki_pos = 0.01
        self.kd_pos = 0.1
        self.kp_angle = 0.8
        self.ki_angle = 0.02
        self.kd_angle = 0.2

        # Umbrales
        self.position_threshold = 10  # distancia en píxeles
        self.angle_threshold = 0.05  # en radianes

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

        # Si estamos suficientemente cerca del objetivo
        if distance < self.position_threshold:
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

        # Si el error de ángulo es grande, girar primero
        if abs(angle_error) > 0.5:  # ~30 grados
            return self.rotate_to_angle(robot, math.degrees(target_heading))

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

        # Calcular velocidad deseada
        speed = min(p_term + i_term + d_term, self.max_motor_speed)

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
        # Convertir a radianes
        target_angle_rad = math.radians(target_angle_deg)
        current_angle_rad = math.radians(robot.angle)

        # Calcular error (normalizado entre -pi y pi)
        angle_error = _normalize_angle(target_angle_rad - current_angle_rad)

        # Si estamos suficientemente cerca del ángulo objetivo
        if abs(angle_error) < self.angle_threshold:
            # Detener el robot
            self._send_motor_commands(robot, 0, 0)
            return True

        # Implementar PID para el ángulo
        p_term = self.kp_angle * angle_error

        # Calcular integral
        self.integral_angle += angle_error
        i_term = self.ki_angle * self.integral_angle

        # Calcular derivada
        derivative = angle_error - self.last_error_angle
        d_term = self.kd_angle * derivative

        # Actualizar último error
        self.last_error_angle = angle_error

        # Calcular velocidad de rotación
        rotation_speed = p_term + i_term + d_term

        # Limitar velocidad de rotación
        rotation_speed = max(
            -self.max_motor_speed, min(rotation_speed, self.max_motor_speed)
        )

        # Determinar velocidades de motores para girar
        if rotation_speed > 0:
            # Girar a la izquierda
            left_speed = -rotation_speed
            right_speed = rotation_speed
        else:
            # Girar a la derecha
            left_speed = -rotation_speed
            right_speed = rotation_speed

        # Enviar comandos a los motores
        self._send_motor_commands(robot, left_speed, right_speed)

        # Aún no hemos alcanzado el ángulo objetivo
        return False

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
        left_speed = speed - correction
        right_speed = speed + correction

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
            # Invertir motores si es necesario según el comportamiento físico del robot
            # Esto depende de cómo estén configurados los motores en tu robot
            self.rf_controller.set_motors(robot.id, left_speed, right_speed)

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
