import time
from robot_soccer.utils.logger import get_logger
from robot_soccer.controllers.differential_drive import DifferentialDriveController
from robot_soccer.controllers.robot_action_executor import RobotActionExecutor


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
        self.action_executors = {}
        for player in team_players:
            controller = DifferentialDriveController(rf_controller=self.rf_controller)
            self.controllers[player.id] = controller
            self.action_executors[player.id] = RobotActionExecutor(controller, self.rf_controller)

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
                    is_completed = self.action_executors[player.id].execute_capture_ball(
                        player, self.ball
                    )

                    if is_completed:
                        del self.actions_in_progress[player.id]
                        self.logger.info(f"Robot {player.id}: Pelota capturada")

                elif action['type'] == 'kick_ball':
                    # Patear la pelota
                    is_completed = self.action_executors[player.id].execute_kick_ball(
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
                    is_completed = self.action_executors[player.id].execute_move_with_ball(
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
        Ordena a un robot capturar la pelota ACTIVANDO EL MOTOR físicamente.

        Args:
            player_id: ID del jugador
        """
        # ACTIVAR MOTOR DE CAPTURA EN ROBOTS REALES
        if self.rf_controller:
            # Activar dribbler con potencia alta para capturar
            success = self.rf_controller.set_dribbler(player_id, 1.0)
            if success:
                self.logger.info(f"Robot {player_id}: Motor de captura ACTIVADO vía RF")
            else:
                self.logger.error(f"Robot {player_id}: Error al activar motor de captura")

        # Registrar acción en progreso
        self.actions_in_progress[player_id] = {
            'type': 'capture_ball',
            'motor_activated': True,
            'start_time': time.time()
        }

        self.logger.info(f"Robot {player_id}: Ordenado capturar pelota con motor")

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
