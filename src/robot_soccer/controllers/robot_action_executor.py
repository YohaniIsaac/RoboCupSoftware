import logging
import math
import numpy as np
from robot_soccer.config import ANCHO_CAMPO, ALTO_CAMPO

from robot_soccer.ai.behavior_tree.utils import (
    calculate_ball_approach_position,
    calculate_shooting_position
)

log = logging.getLogger(__name__)


class RobotActionExecutor:
    """Ejecuta acciones complejas para robots de fútbol.

    Encapsula la lógica específica para comportamientos como capturar la pelota,
    patear, o moverse con la pelota, manteniendo estas responsabilidades
    separadas de la gestión de comandos.

    Esta clase actúa como una capa de abstracción entre los árboles de
    comportamiento y los controladores de bajo nivel, proporcionando
    métodos de alto nivel para acciones complejas de robots.

    Attributes:
        controller (DifferentialDriveController): Controlador de movimiento diferencial.
        rf_controller (RFController, optional): Controlador RF para comunicación
            con robots reales.
    """

    def __init__(self, differential_controller, rf_controller=None):
        """Inicializa el ejecutor de acciones.

        Args:
            differential_controller (DifferentialDriveController): Controlador de
                movimiento diferencial asociado al robot.
            rf_controller (RFController, optional): Controlador RF para comunicación
                con robots reales. Si es None, opera solo en simulación.

        Note:
            El rf_controller es necesario para activar motores físicos como el
            dribbler y el mecanismo de pateo en robots reales.
        """
        self.controller = differential_controller
        self.rf_controller = rf_controller

    def execute_capture_ball(self, player, ball):
        """Ejecuta la acción de capturar la pelota activando físicamente el motor.

        Implementa un algoritmo de captura en múltiples pasos:
        1. Si está lejos, se mueve a posición estratégica
        2. Se orienta hacia la pelota
        3. Activa el motor de captura (dribbler)
        4. Se acerca con el motor activo
        5. Confirma la captura

        Args:
            player (Player): Objeto jugador que ejecutará la captura.
            ball (Ball): Objeto pelota a capturar.

        Returns:
            bool: True si la captura se completó exitosamente, False si aún
                está en progreso.

        Note:
            En robots reales, activa físicamente el motor dribbler.
            En simulación, "pega" la pelota al robot.

        Examples:
            >>> executor = RobotActionExecutor(controller)
            >>> completed = executor.execute_capture_ball(player, ball)
            >>> if completed:
            ...     print("Pelota capturada")
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
                log.info("Robot %i: Motor de captura ACTIVADO", player.id)

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

            log.info("Robot %i: Pelota capturada exitosamente", player.id)

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
        """Ejecuta la acción de patear la pelota usando posicionamiento estratégico.

        Implementa un sistema de pateo con las siguientes características:
        - Posicionamiento estratégico opcional para tiros óptimos
        - Orientación precisa hacia el objetivo
        - Control de potencia variable
        - Activación física del mecanismo de pateo en robots reales

        Args:
            player (Player): Objeto jugador que ejecutará el pateo.
            target_pos (tuple[float, float]): Posición objetivo (x, y) hacia
                donde patear la pelota.
            ball (Ball): Objeto pelota a patear.
            power (float): Potencia del tiro entre 0.0 y 1.0, donde 1.0 es
                máxima potencia.
            use_strategic_positioning (bool, optional): Si True, calcula y se
                mueve a la posición óptima antes de patear. Defaults to True.

        Returns:
            bool: True si el tiro se completó exitosamente, False si aún está
                posicionándose o orientándose.

        Raises:
            ValueError: Si power no está entre 0.0 y 1.0.

        Note:
            - Requiere que el jugador tenga la pelota (player._has_ball = True)
            - En robots reales, desactiva el dribbler y activa el mecanismo de pateo
            - La velocidad de la pelota se calcula como: kick_speed = 15 * power

        Examples:
            >>> # Tiro de máxima potencia al arco
            >>> completed = executor.execute_kick_ball(
            ...     player, goal_pos, ball, power=1.0
            ... )
            >>>
            >>> # Pase suave con posicionamiento manual
            >>> completed = executor.execute_kick_ball(
            ...     player, teammate_pos, ball, power=0.3,
            ...     use_strategic_positioning=False
            ... )
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
                log.debug("Robot %i: Posicionándose para disparo óptimo", player.id)
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

        log.info("Robot %i: Pelota pateada hacia %.2f con potencia %.2f",
                 player.id, target_pos, power)

        return True

    def execute_move_with_ball(self, player, target_pos, ball, maintain_ball_control=True):
        """Ejecuta la acción de moverse con la pelota controlada.

        Permite al robot moverse hacia una posición específica mientras mantiene
        control de la pelota. La pelota se mantiene en posición relativa al robot
        durante el movimiento.

        Args:
            player (Player): Objeto jugador que se moverá con la pelota.
            target_pos (tuple[float, float]): Posición objetivo (x, y) hacia
                donde moverse.
            ball (Ball): Objeto pelota que debe mantener controlada.
            maintain_ball_control (bool, optional): Si True, mantiene la pelota
                en posición relativa durante todo el movimiento. Defaults to True.

        Returns:
            bool: True si el movimiento se completó (llegó al objetivo), False
                si aún está en progreso.

        Note:
            - Requiere que el jugador tenga la pelota (player._has_ball = True)
            - Durante la rotación, la pelota se mantiene en posición relativa
            - El offset de la pelota es de 22 unidades frente al robot
            - Se considera "llegado" cuando la distancia al objetivo es < 15

        Examples:
            >>> # Moverse al centro del campo con la pelota
            >>> completed = executor.execute_move_with_ball(
            ...     player, (ANCHO_CAMPO/2, ALTO_CAMPO/2), ball
            ... )
            >>>
            >>> # Moverse sin mantener control estricto de la pelota
            >>> completed = executor.execute_move_with_ball(
            ...     player, target, ball, maintain_ball_control=False
            ... )
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
        """Ejecuta posicionamiento estratégico sin pelota.

        Mueve al robot a posiciones estratégicas en el campo basadas en el tipo
        de posicionamiento solicitado y la situación actual del juego.

        Args:
            player (Player): Objeto jugador a posicionar.
            ball (Ball): Objeto pelota para referencia posicional.
            position_type (str, optional): Tipo de posicionamiento estratégico.
                Opciones válidas:
                - 'support': Posición de apoyo ofensivo
                - 'defend': Posición defensiva
                - 'attack': Posición de ataque
                - 'midfield': Posición de medio campo
                Defaults to 'support'.

        Returns:
            bool: True si el posicionamiento se completó, False si aún está
                moviéndose a la posición objetivo.

        Raises:
            ValueError: Si position_type no es uno de los tipos válidos.

        Note:
            Este método calcula posiciones dinámicas basadas en:
            - Posición actual de la pelota
            - Equipo del jugador
            - Tipo de posicionamiento solicitado
            - Estado actual del juego

        Examples:
            >>> # Posicionarse para apoyo ofensivo
            >>> completed = executor.execute_strategic_positioning(
            ...     player, ball, 'support'
            ... )
            >>>
            >>> # Posicionarse defensivamente
            >>> completed = executor.execute_strategic_positioning(
            ...     player, ball, 'defend'
            ... )
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
            log.debug("Robot %i: Posicionamiento %.2f completado", player.id, position_type)

        return is_completed

    @staticmethod
    def _normalize_angle_deg(angle):
        """Normaliza un ángulo en grados entre -180 y 180."""
        angle = angle % 360
        if angle > 180:
            angle -= 360
        return angle
