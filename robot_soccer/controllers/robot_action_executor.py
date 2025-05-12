import math
import numpy as np
from robot_soccer.utils.logger import get_logger


class RobotActionExecutor:
    """
    Ejecuta acciones complejas para robots de fútbol.
    Encapsula la lógica específica para comportamientos como capturar la pelota,
    patear, o moverse con la pelota, manteniendo estas responsabilidades
    separadas de la gestión de comandos.
    """

    def __init__(self, differential_controller, rf_controller=None):
        """
        Inicializa el ejecutor de acciones.

        Args:
            differential_controller: Controlador de movimiento diferencial asociado
            rf_controller: Controlador RF para comunicación con robots reales (opcional)
        """
        self.controller = differential_controller
        self.rf_controller = rf_controller
        self.logger = get_logger("controllers.action_executor")

    def execute_capture_ball(self, player, ball):
        """
        Ejecuta la acción de capturar la pelota.

        Args:
            player: Objeto jugador
            ball: Objeto pelota

        Returns:
            bool: True si la captura se completó
        """
        # Obtener posición de la pelota
        ball_pos = ball.get_position()

        # Calcular distancia a la pelota
        dist_to_ball = player.distance_to_ball(ball)

        if dist_to_ball < 30:
            # Estamos lo suficientemente cerca, activar mecanismo de captura
            if self.rf_controller:
                # Activar dribbler
                self.rf_controller.set_dribbler(player.id, 1.0)

            # Marcar como capturada en el modelo
            player.ball_hold = True
            return True

        # Calcular ángulo hacia la pelota
        dx = ball_pos[0] - player.x
        dy = ball_pos[1] - player.y
        angle_to_ball = np.degrees(np.arctan2(dy, dx))

        # Primero orientar el robot hacia la pelota
        current_angle = player.angle
        angle_diff = self._normalize_angle_deg(angle_to_ball - current_angle)

        if abs(angle_diff) > 10:
            # Primero girar hacia la pelota
            self.controller.rotate_to_angle(player, angle_to_ball)
            return False

        # Moverse hacia la pelota
        self.controller.move_to_position(player, ball_pos)
        return False

    def execute_kick_ball(self, player, target_pos, ball, power):
        """
        Ejecuta la acción de patear la pelota.

        Args:
            player: Objeto jugador
            target_pos: Posición objetivo
            ball: Objeto pelota
            power: Potencia del tiro (0-1)

        Returns:
            bool: True si el tiro se completó
        """
        if not player.ball_hold:
            # No tenemos la pelota, fallo
            return True

        # Calcular ángulo hacia el objetivo
        dx = target_pos[0] - player.x
        dy = target_pos[1] - player.y
        angle_to_target = np.degrees(np.arctan2(dy, dx))

        # Primero orientar el robot hacia el objetivo
        current_angle = player.angle
        angle_diff = self._normalize_angle_deg(angle_to_target - current_angle)

        if abs(angle_diff) > 5:
            # Primero girar hacia el objetivo
            self.controller.rotate_to_angle(player, angle_to_target)
            return False

        # Calcular velocidades para la pelota
        kick_speed = 15 * power  # Ajustar según necesidades
        kick_angle_rad = np.radians(angle_to_target)

        # Enviar comando de pateo si hay controlador RF
        if self.rf_controller:
            # Desactivar dribbler
            self.rf_controller.set_dribbler(player.id, 0)
            # Activar mecanismo de pateo
            self.rf_controller.kick(player.id, power)

        # Aplicar velocidad a la pelota en la simulación
        if hasattr(ball, 'dx') and hasattr(ball, 'dy'):
            ball.dx = kick_speed * np.cos(kick_angle_rad)
            ball.dy = kick_speed * np.sin(kick_angle_rad)

        # Desactivar la posesión
        player.ball_hold = False

        return True

    def execute_move_with_ball(self, player, target_pos, ball, speed_factor=0.7):
        """
        Ejecuta la acción de moverse con la pelota controlada.

        Args:
            player: Objeto jugador
            target_pos: Posición objetivo
            ball: Objeto pelota
            speed_factor: Factor de velocidad (0-1)

        Returns:
            bool: True si el movimiento se completó
        """
        if not player.ball_hold:
            # No tenemos la pelota, fallo
            return True

        # Obtener posición actual del jugador
        current_pos = (player.x, player.y)

        # Calcular distancia al objetivo
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        distance = math.sqrt(dx ** 2 + dy ** 2)

        # Si estamos suficientemente cerca del objetivo, completar
        if distance < 10:
            return True

        # Calcular ángulo hacia el objetivo
        target_angle = math.degrees(math.atan2(dy, dx))

        # Primero, asegurar que el robot está orientado correctamente
        current_angle = player.angle
        angle_diff = self._normalize_angle_deg(target_angle - current_angle)

        if abs(angle_diff) > 10:
            # Primero girar hacia el objetivo
            self.controller.rotate_to_angle(player, target_angle)
            return False

        # Moverse hacia el objetivo a velocidad controlada
        is_moving = not self.controller.move_to_position(
            player,
            target_pos,
            speed_factor=speed_factor
        )

        # Actualizar la posición de la pelota para que siga al robot
        if hasattr(ball, 'set_position'):
            # Calcular posición adelante del jugador
            offset = 20  # Distancia frente al robot
            angle_rad = math.radians(player.angle)
            ball_x = player.x + offset * math.cos(angle_rad)
            ball_y = player.y + offset * math.sin(angle_rad)

            # Actualizar posición de la pelota
            ball.set_position(ball_x, ball_y)

        # Si el controlador indica que hemos llegado, completar
        return not is_moving

    @staticmethod
    def _normalize_angle_deg(angle):
        """
        Normaliza un ángulo en grados entre -180 y 180.

        Args:
            angle: Ángulo en grados

        Returns:
            float: Ángulo normalizado
        """
        angle = angle % 360
        if angle > 180:
            angle -= 360
        return angle
