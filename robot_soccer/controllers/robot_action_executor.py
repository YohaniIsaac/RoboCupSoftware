import math
import numpy as np
from robot_soccer.utils.logger import get_logger
from robot_soccer.config import *

from robot_soccer.ai.behavior_tree.utils import (
    calculate_ball_approach_position,
    calculate_shooting_position
)


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
        Ejecuta la acción de capturar la pelota ACTIVANDO FÍSICAMENTE EL MOTOR.

        Args:
            player: Objeto jugador
            ball: Objeto pelota

        Returns:
            bool: True si la captura se completó
        """
        player_pos = player.get_position()
        ball_pos = ball.get_position()

        # Obtener arco enemigo para cálculo individualizado
        if player.team == 'red':
            opponent_goal_pos = (ANCHO_CAMPO, ALTO_CAMPO / 2)
        else:
            opponent_goal_pos = (0, ALTO_CAMPO / 2)

        # Calcular distancia a la pelota
        dist_to_ball = player.distance_to_ball(ball)

        if dist_to_ball < 45:
            # PASO 1: Verificar orientación hacia la pelota
            angle_to_ball = np.degrees(np.arctan2(
                ball_pos[1] - player_pos[1],
                ball_pos[0] - player_pos[0]
            ))

            current_angle = player.angle
            angle_diff = abs((angle_to_ball - current_angle + 180) % 360 - 180)

            if angle_diff > 12:
                # Orientarse hacia la pelota
                self.controller.rotate_to_angle(player, angle_to_ball)
                return False

            # PASO 2: ACTIVAR MOTOR DE CAPTURA FÍSICAMENTE
            if self.rf_controller:
                # ACTIVAR DRIBBLER/MOTOR EN ROBOTS REALES
                self.rf_controller.set_dribbler(player.id, 1.0)  # Potencia máxima
                self.logger.info(f"Robot {player.id}: Motor de captura ACTIVADO")

            # PASO 3: Acercarse un poco más con motor activo
            if dist_to_ball > 25:
                # Moverse lentamente hacia la pelota con motor activo
                self.controller.move_to_position(player, ball_pos, speed_factor=0.3)
                return False

            # PASO 4: Confirmar captura
            # Marcar como capturada en el modelo
            player._has_ball = True

            # En simulación, "pegar" la pelota al robot
            if hasattr(ball, 'set_position'):
                front_offset = 25
                angle_rad = math.radians(player.angle)
                ball_x = player.x + front_offset * math.cos(angle_rad)
                ball_y = player.y + front_offset * math.sin(angle_rad)
                ball.set_position(ball_x, ball_y)

            self.logger.info(f"Robot {player.id}: Pelota capturada exitosamente")
            return True

        # Aún lejos, moverse a posición estratégica INDIVIDUALIZADA
        target_pos = calculate_ball_approach_position(
            player_pos,
            ball_pos,
            opponent_goal_pos,  # CLAVE: Considerar arco enemigo
            35,  # Distancia más cercana para captura
            player.team
        )

        # Moverse a la posición estratégica individual
        self.controller.move_to_position(player, target_pos)
        return False

    def execute_kick_ball(self, player, target_pos, ball, power, use_strategic_positioning=True):
        """
        Ejecuta la acción de patear la pelota usando posicionamiento estratégico.

        Args:
            player: Objeto jugador
            target_pos: Posición objetivo
            ball: Objeto pelota
            power: Potencia del tiro (0-1)
            use_strategic_positioning: Si usar posicionamiento estratégico

        Returns:
            bool: True si el tiro se completó
        """
        if not player._has_ball:
            # No tenemos la pelota, fallar
            return True

        player_pos = player.get_position()
        ball_pos = ball.get_position()

        if use_strategic_positioning:
            # Calcular posición óptima para el disparo
            optimal_shooting_pos = calculate_shooting_position(
                player_pos,
                ball_pos,
                target_pos,
                approach_distance=55
            )

            # Verificar si estamos en buena posición para disparar
            distance_to_optimal = np.linalg.norm(
                np.array(player_pos) - np.array(optimal_shooting_pos)
            )

            if distance_to_optimal > 30:
                # Moverse a mejor posición primero
                self.controller.move_to_position(player, optimal_shooting_pos)
                self.logger.debug(f"Robot {player.id}: Posicionándose para disparo óptimo")
                return False

        # Calcular ángulo hacia el objetivo
        dx = target_pos[0] - player_pos[0]
        dy = target_pos[1] - player_pos[1]
        angle_to_target = np.degrees(np.arctan2(dy, dx))

        # Verificar orientación
        current_angle = player.angle
        angle_diff = abs((angle_to_target - current_angle + 180) % 360 - 180)

        if angle_diff > 8:
            # Orientarse hacia el objetivo
            self.controller.rotate_to_angle(player, angle_to_target)
            return False

        # Calcular velocidades para la pelota
        kick_speed = 15 * power
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
        player._has_ball = False

        self.logger.info(f"Robot {player.id}: Pelota pateada hacia {target_pos} con potencia {power}")

        return True

    def execute_move_with_ball(self, player, target_pos, ball, maintain_ball_control=True):
        """
        Ejecuta la acción de moverse con la pelota controlada usando mejor control.

        Args:
            player: Objeto jugador
            target_pos: Posición objetivo
            ball: Objeto pelota
            maintain_ball_control: Si mantener control estricto de la pelota

        Returns:
            bool: True si el movimiento se completó
        """
        if not player._has_ball:
            # No tenemos la pelota, fallar
            return True

        player_pos = player.get_position()

        # Calcular distancia al objetivo
        dx = target_pos[0] - player_pos[0]
        dy = target_pos[1] - player_pos[1]
        distance = math.sqrt(dx ** 2 + dy ** 2)

        # Si estamos suficientemente cerca del objetivo, completar
        if distance < 15:
            return True

        # Calcular ángulo hacia el objetivo
        target_angle = math.degrees(math.atan2(dy, dx))

        # Verificar orientación actual
        current_angle = player.angle
        angle_diff = abs((target_angle - current_angle + 180) % 360 - 180)

        if angle_diff > 15:
            # Girar hacia el objetivo (con la pelota)
            self.controller.rotate_to_angle(player, target_angle)

            # Mantener pelota en posición durante la rotación
            if maintain_ball_control and hasattr(ball, 'set_position'):
                offset = 25
                angle_rad = math.radians(player.angle)
                ball_x = player.x + offset * math.cos(angle_rad)
                ball_y = player.y + offset * math.sin(angle_rad)
                ball.set_position(ball_x, ball_y)

            return False

        # Moverse hacia el objetivo con velocidad controlada
        is_moving = not self.controller.move_to_position(
            player,
            target_pos
        )

        # Actualizar la posición de la pelota para que siga al robot
        if maintain_ball_control and hasattr(ball, 'set_position'):
            # Calcular posición adelante del jugador
            offset = 22  # Distancia frente al robot
            angle_rad = math.radians(player.angle)
            ball_x = player.x + offset * math.cos(angle_rad)
            ball_y = player.y + offset * math.sin(angle_rad)

            # Actualizar posición de la pelota
            ball.set_position(ball_x, ball_y)

        return not is_moving

    def execute_strategic_positioning(self, player, ball, position_type='support'):
        """
        Ejecuta posicionamiento estratégico sin pelota.

        Args:
            player: Objeto jugador
            ball: Objeto pelota
            position_type: Tipo de posicionamiento ('support', 'defensive', 'press')

        Returns:
            bool: True si el posicionamiento se completó
        """
        player_pos = player.get_position()
        ball_pos = ball.get_position()

        if position_type == 'support':
            # Posicionamiento de apoyo
            # Calcular posición a 90 grados de la línea pelota-portería
            goal_pos = [1500, 450]  # Portería rival por defecto
            ball_to_goal = np.array(goal_pos) - np.array(ball_pos)

            if np.linalg.norm(ball_to_goal) > 0:
                ball_to_goal = ball_to_goal / np.linalg.norm(ball_to_goal)
                # Vector perpendicular
                perp_vector = np.array([-ball_to_goal[1], ball_to_goal[0]])
                support_pos = np.array(ball_pos) + perp_vector * 150
            else:
                support_pos = np.array([ball_pos[0] + 100, ball_pos[1]])

        elif position_type == 'defensive':
            # Posicionamiento defensivo
            own_goal = [0, 450]  # Portería propia por defecto
            ball_to_own_goal = np.array(own_goal) - np.array(ball_pos)

            if np.linalg.norm(ball_to_own_goal) > 0:
                ball_to_own_goal = ball_to_own_goal / np.linalg.norm(ball_to_own_goal)
                # Posicionarse entre pelota y portería
                support_pos = np.array(ball_pos) + ball_to_own_goal * 100
            else:
                support_pos = np.array([ball_pos[0] - 100, ball_pos[1]])

        elif position_type == 'press':
            # Posicionamiento de presión
            # Aproximarse a la pelota pero no tanto como para capturarla
            support_pos = calculate_ball_approach_position(
                player_pos,
                ball_pos,
                approach_distance=80,
                strategy='direct'
            )

        else:
            # Posicionamiento por defecto
            support_pos = ball_pos

        # Asegurar que la posición está dentro del campo
        support_pos[0] = max(30, min(1470, support_pos[0]))
        support_pos[1] = max(30, min(870, support_pos[1]))

        # Moverse a la posición
        target_pos = tuple(support_pos)
        is_completed = self.controller.move_to_position(player, target_pos)

        if is_completed:
            self.logger.debug(f"Robot {player.id}: Posicionamiento {position_type} completado")

        return is_completed

    @staticmethod
    def _normalize_angle_deg(angle):
        """
        Normaliza un ángulo en grados entre -180 y 180.
        """
        angle = angle % 360
        if angle > 180:
            angle -= 360
        return angle
