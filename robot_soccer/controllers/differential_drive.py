import math
# import numpy as np
from robot_soccer.utils.logger import get_logger
# from robot_soccer.communication.rf_controller import RFController


def _normalize_angle(angle):
    """
    Normaliza un ángulo entre -π y π.

    Args:
        angle: Ángulo en radianes

    Returns:
        float: Ángulo normalizado
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class DifferentialDriveController:
    """
    Controlador para robots de tracción diferencial.
    Traduce comandos de movimiento de alto nivel a velocidades de motores.
    """

    def __init__(self, rf_controller=None, wheel_radius=0.025, wheel_distance=0.1, max_motor_speed=1.0):
        """
        Inicializa el controlador con parámetros físicos del robot.

        Args:
            rf_controller: Controlador RF compartido
            wheel_radius: Radio de las ruedas en metros
            wheel_distance: Distancia entre las ruedas en metros
            max_motor_speed: Velocidad máxima de los motores (escalar de 0 a 1)
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

        # Inicializar logger
        self.logger = get_logger("controllers.differential_drive")

    def move_to_position(self, robot, target_pos, target_angle=None):
        """
        Calcula y envía comandos para mover el robot a una posición específica.

        Args:
            robot: Objeto robot con atributos x, y, angle
            target_pos: Posición objetivo (x, y)
            target_angle: Ángulo objetivo (en grados, opcional)

        Returns:
            bool: True si el robot ha llegado a la posición objetivo
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
        self.integral_pos = (
            self.integral_pos[0] + dx,
            self.integral_pos[1] + dy
        )
        i_term = self.ki_pos * math.sqrt(self.integral_pos[0]**2 + self.integral_pos[1]**2)

        derivative_pos = (
            dx - self.last_error_pos[0],
            dy - self.last_error_pos[1]
        )
        d_term = self.kd_pos * math.sqrt(derivative_pos[0]**2 + derivative_pos[1]**2)

        # Actualizar último error
        self.last_error_pos = (dx, dy)

        # Calcular velocidad deseada
        speed = min(p_term + i_term + d_term, self.max_motor_speed)

        # Ajustar velocidad basada en el error de ángulo
        left_speed, right_speed = self._calculate_differential_speeds(speed, angle_error)

        # Enviar comandos a los motores
        self._send_motor_commands(robot, left_speed, right_speed)

        # Aún no hemos llegado
        return False

    def rotate_to_angle(self, robot, target_angle_deg):
        """
        Rota el robot hacia un ángulo específico.

        Args:
            robot: Objeto robot
            target_angle_deg: Ángulo objetivo en grados

        Returns:
            bool: True si el robot ha alcanzado el ángulo objetivo
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
        rotation_speed = max(-self.max_motor_speed, min(rotation_speed, self.max_motor_speed))

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
        """
        Calcula las velocidades diferenciales para los motores.

        Args:
            speed: Velocidad lineal deseada
            angle_error: Error de ángulo en radianes

        Returns:
            tuple: Velocidades (izquierda, derecha)
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
        """
        Envía comandos a los motores del robot.

        Args:
            robot: Objeto robot
            left_speed: Velocidad del motor izquierdo (-1 a 1)
            right_speed: Velocidad del motor derecho (-1 a 1)
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

        self.logger.debug(f"Robot {robot.id} - Velocidades: L={left_speed:.2f}, R={right_speed:.2f} - " +
                          f"Linear: ({robot.dx:.2f}, {robot.dy:.2f}) - Angular: {robot.dw:.2f}")
