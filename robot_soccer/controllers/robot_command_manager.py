from robot_soccer.utils.logger import get_logger
from robot_soccer.controllers.differential_drive import DifferentialDriveController
from robot_soccer.communication.rf_controller import RFController
import numpy as np


class RobotCommandManager:
    """
    Gestor de comandos para robots. Traduce acciones de alto nivel definidas
    en los árboles de comportamiento a comandos específicos de motores.
    """

    def __init__(self, team_players, ball, use_real_robots=False, port='/dev/ttyUSB0'):
        """
        Inicializa el gestor de comandos.

        Args:
            team_players: Lista de jugadores del equipo
            ball: Objeto pelota
            use_real_robots: Si True, utiliza comunicación real con robots
            port: Puerto serial para comunicación con Arduino
        """
        # Inicializar logger
        self.logger = get_logger("controllers.command_manager")
        self.team_players = team_players
        self.ball = ball
        self.use_real_robots = use_real_robots

        # Inicializar controlador RF si se utilizan robots reales
        self.rf_controller = None
        if use_real_robots:
            from robot_soccer.communication.rf_controller import RFController
            self.rf_controller = RFController(port=port)
            success = self.rf_controller.initialize()
            if not success:
                self.logger.error("No se pudo inicializar el controlador RF. Usando modo simulación.")
                self.use_real_robots = False
                self.rf_controller = None

        # Crear un controlador para cada robot
        self.controllers = {}
        for player in team_players:
            from robot_soccer.controllers.differential_drive import DifferentialDriveController
            self.controllers[player.id] = DifferentialDriveController(rf_controller=self.rf_controller)

        # Memoria de acciones en curso
        self.actions_in_progress = {}

    def shutdown(self):
        """
        Cierra las conexiones y detiene los robots.
        """
        if self.rf_controller:
            # Detener todos los robots
            for player in self.team_players:
                self.rf_controller.stop_robot(player.id)

            # Cerrar comunicación
            self.rf_controller.shutdown()

    def execute_commands(self):
        """
        Ejecuta los comandos pendientes para todos los robots del equipo.
        Debe llamarse en cada iteración del bucle principal.
        """
        for player in self.team_players:
            if player.id in self.actions_in_progress:
                action = self.actions_in_progress[player.id]

                if action['type'] == 'move':
                    # Movimiento a una posición
                    is_completed = self.controllers[player.id].move_to_position(
                        player,
                        action['target_pos'],
                        action.get('target_angle')
                    )

                    if is_completed:
                        # Acción completada, eliminar de la lista
                        del self.actions_in_progress[player.id]
                        self.logger.info(f"Robot {player.id}: Completado movimiento a {action['target_pos']}")

                elif action['type'] == 'rotate':
                    # Rotación a un ángulo
                    is_completed = self.controllers[player.id].rotate_to_angle(
                        player,
                        action['target_angle']
                    )

                    if is_completed:
                        # Acción completada, eliminar de la lista
                        del self.actions_in_progress[player.id]
                        self.logger.info(f"Robot {player.id}: Completado giro a {action['target_angle']} grados")

                elif action['type'] == 'capture_ball':
                    # Movimiento para capturar la pelota
                    is_completed = self._execute_capture_ball(player)

                    if is_completed:
                        del self.actions_in_progress[player.id]
                        self.logger.info(f"Robot {player.id}: Pelota capturada")

                elif action['type'] == 'kick_ball':
                    # Patear la pelota
                    is_completed = self._execute_kick_ball(
                        player,
                        action['target_pos'],
                        action['ball'],
                        action['power']
                    )

                    if is_completed:
                        del self.actions_in_progress[player.id]
                        self.logger.info(f"Robot {player.id}: Pelota pateada hacia {action['target_pos']}")

                elif action['type'] == 'move_with_ball':
                    # Mover con la pelota
                    is_completed = self._execute_move_with_ball(
                        player,
                        action['target_pos'],
                        action['ball'],
                        action['speed_factor']
                    )

                    if is_completed:
                        del self.actions_in_progress[player.id]
                        self.logger.info(
                            f"Robot {player.id}: Movimiento con pelota completado a {action['target_pos']}")

    def move_robot_to(self, player_id, target_pos, target_angle=None, speed_factor=1.0):
        """
        Ordena a un robot moverse a una posición específica.

        Args:
            player_id: ID del jugador
            target_pos: Posición objetivo (x, y)
            target_angle: Ángulo objetivo en grados (opcional)
            speed_factor: Factor de velocidad (0-2.0)
        """
        self.actions_in_progress[player_id] = {
            'type': 'move',
            'target_pos': target_pos,
            'target_angle': target_angle,
            'speed_factor': speed_factor
        }
        self.logger.info(f"Robot {player_id}: Ordenado movimiento a {target_pos}")

    def rotate_robot_to(self, player_id, target_angle):
        """
        Ordena a un robot girar a un ángulo específico.

        Args:
            player_id: ID del jugador
            target_angle: Ángulo objetivo en grados
        """
        self.actions_in_progress[player_id] = {
            'type': 'rotate',
            'target_angle': target_angle
        }
        self.logger.info(f"Robot {player_id}: Ordenado giro a {target_angle} grados")

    def capture_ball(self, player_id):
        """
        Ordena a un robot capturar la pelota.

        Args:
            player_id: ID del jugador
        """
        self.actions_in_progress[player_id] = {
            'type': 'capture_ball'
        }
        self.logger.info(f"Robot {player_id}: Ordenado capturar pelota")

    def kick_ball(self, player_id, target_pos, ball, power=1.0):
        """
        Ordena a un robot patear la pelota hacia una posición.

        Args:
            player_id: ID del jugador
            target_pos: Posición objetivo para la pelota
            ball: Objeto pelota
            power: Potencia del tiro (0-1)
        """
        self.actions_in_progress[player_id] = {
            'type': 'kick_ball',
            'target_pos': target_pos,
            'power': power,
            'ball': ball
        }
        self.logger.info(f"Robot {player_id}: Ordenado patear pelota hacia {target_pos} con potencia {power}")

    def move_with_ball(self, player_id, target_pos, ball, speed_factor=0.7):
        """
        Ordena a un robot moverse con la pelota hacia una posición.

        Args:
            player_id: ID del jugador
            target_pos: Posición objetivo
            ball: Objeto pelota
            speed_factor: Factor de velocidad (0-1)
        """
        self.actions_in_progress[player_id] = {
            'type': 'move_with_ball',
            'target_pos': target_pos,
            'speed_factor': speed_factor,
            'ball': ball
        }
        self.logger.info(f"Robot {player_id}: Ordenado moverse con pelota hacia {target_pos}")

    def _execute_capture_ball(self, player):
        """
        Ejecuta la acción de capturar la pelota.

        Args:
            player: Objeto jugador

        Returns:
            bool: True si la captura se completó
        """
        # Obtener posición de la pelota
        ball_pos = self.ball.get_position()

        # Calcular distancia a la pelota
        dist_to_ball = player.distance_to_ball(self.ball)

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
            self.controllers[player.id].rotate_to_angle(player, angle_to_ball)
            return False

        # Moverse hacia la pelota
        self.controllers[player.id].move_to_position(player, ball_pos)
        return False

    def _execute_kick_ball(self, player, target_pos, ball, power):
        """
        Ejecuta la acción de patear la pelota.

        Args:
            player: Objeto jugador
            target_pos: Posición objetivo
            ball: Objeto pelota
            power: Potencia del tiro

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
            self.controllers[player.id].rotate_to_angle(player, angle_to_target)
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

    def _execute_kick_ball(self, player, target_pos, ball, power):
        """
        Ejecuta la acción de patear la pelota.

        Args:
            player: Objeto jugador
            target_pos: Posición objetivo
            ball: Objeto pelota
            power: Potencia del tiro

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
            self.controllers[player.id].rotate_to_angle(player, angle_to_target)
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

    def _normalize_angle_deg(self, angle):
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