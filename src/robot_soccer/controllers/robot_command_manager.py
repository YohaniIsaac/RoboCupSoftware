import time
import logging
from robot_soccer.config import FIELD_SIM
from robot_soccer.controllers.differential_drive import DifferentialDriveController
from robot_soccer.controllers.robot_action_executor import RobotActionExecutor
from robot_soccer.communication.rf_controller import RFController

log = logging.getLogger(__name__)


def _normalize_angle_deg(angle):
    """Normaliza un ángulo en grados entre -180 y 180.

    Args:
        angle (float): Ángulo en grados a normalizar.

    Returns:
        float: Ángulo normalizado entre -180 y 180 grados.
    """
    angle = angle % 360
    if angle > 180:
        angle -= 360
    return angle


class RobotCommandManager:
    """Gestor de comandos para robots de fútbol.

    Esta clase actúa como intermediario entre los árboles de comportamiento
    de alto nivel y los controladores específicos de motores. Traduce acciones
    como "mover a posición" o "patear pelota" en comandos específicos para
    los motores de los robots físicos o simulados.

    Attributes:
        team_players (list): Lista de jugadores del equipo.
        ball (Ball): Objeto pelota del juego.
        use_real_robots (bool): Indica si se usan robots físicos.
        rf_controller (RFController): Controlador de comunicación RF.
        controllers (dict): Controladores de movimiento por robot.
        action_executors (dict): Ejecutores de acciones por robot.
        actions_in_progress (dict): Acciones actuales en progreso.
    """

    def __init__(self, team_players, ball, use_real_robots=False, port='/dev/ttyUSB0',
                 field=None):
        """Inicializa el gestor de comandos para robots.

        Args:
            team_players (list): Lista de objetos Player del equipo.
            ball (Ball): Objeto pelota del juego.
            use_real_robots (bool, optional): Si True, utiliza comunicación
                real con robots físicos. Defaults to False.
            port (str, optional): Puerto serial para comunicación con Arduino.
                Defaults to '/dev/ttyUSB0'.
            field: FieldGeometry con geometría del campo. Defaults to FIELD_SIM.

        Note:
            Si falla la inicialización del controlador RF, automáticamente
            cambia a modo simulación sin lanzar excepciones.
        """
        # Inicializar logger
        self.team_players = team_players
        self.ball = ball
        self.use_real_robots = use_real_robots
        self.field = field if field is not None else FIELD_SIM

        # Inicializar controlador RF si se utilizan robots reales
        self.rf_controller = None
        if use_real_robots:
            self.rf_controller = RFController(port=port)
            success = self.rf_controller.initialize()
            if not success:
                log.error("No se pudo inicializar el controlador RF. Usando modo simulación.")
                self.use_real_robots = False
                self.rf_controller = None

        # Crear un controlador para cada robot
        self.controllers = {}
        self.action_executors = {}
        for player in team_players:
            controller = DifferentialDriveController(rf_controller=self.rf_controller)
            self.controllers[player.id] = controller
            self.action_executors[player.id] = RobotActionExecutor(
                controller, self.rf_controller, field=self.field
            )

        # Memoria de acciones en curso
        self.actions_in_progress = {}

    def shutdown(self):
        """Cierra las conexiones y detiene todos los robots.

        Detiene todos los robots del equipo y cierra la comunicación RF
        si está activa. Debe llamarse antes de terminar la aplicación.

        Note:
            Es seguro llamar este método múltiples veces.
        """
        if self.rf_controller:
            # Detener todos los robots
            for player in self.team_players:
                self.rf_controller.stop_robot(player.id)

            # Cerrar comunicación
            self.rf_controller.shutdown()

    def execute_commands(self):
        """Ejecuta los comandos pendientes para todos los robots del equipo.

        Procesa todas las acciones en progreso y actualiza el estado
        de cada robot según su acción asignada. Debe llamarse en cada
        iteración del bucle principal del juego.

        Note:
            Las acciones completadas se eliminan automáticamente de
            la cola de acciones en progreso.
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
                        log.info("Robot %i: Completado movimiento a %s",
                                 player.id, action['target_pos'])

                elif action['type'] == 'rotate':
                    # Rotación a un ángulo
                    is_completed = self.controllers[player.id].rotate_to_angle(
                        player,
                        action['target_angle']
                    )

                    if is_completed:
                        # Acción completada, eliminar de la lista
                        del self.actions_in_progress[player.id]
                        log.info("Robot %i: Completado giro a %s grados",
                                         player.id, action['target_angle'])

                elif action['type'] == 'capture_ball':
                    # Movimiento para capturar la pelota
                    is_completed = self.action_executors[player.id].execute_capture_ball(
                        player, self.ball
                    )

                    if is_completed:
                        del self.actions_in_progress[player.id]
                        log.info("Robot %i: Pelota capturada", player.id)

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
                        log.info("Robot %i: Pelota pateada hacia %s",
                                 player.id, action['target_pos'])

                elif action['type'] == 'move_with_ball':
                    # Mover con la pelota
                    is_completed = self.action_executors[player.id].execute_move_with_ball(
                        player,
                        action['target_pos'],
                        action['ball']
                    )

                    if is_completed:
                        del self.actions_in_progress[player.id]
                        log.info("Robot %i: Movimiento con pelota completado a %s",
                                 player.id, action['target_pos'])

    def move_robot_to(self, player_id, target_pos, target_angle=None, speed_factor=1.0):
        """Ordena a un robot moverse a una posición específica.

        Args:
            player_id (int): ID único del jugador.
            target_pos (tuple): Posición objetivo como tupla (x, y).
            target_angle (float, optional): Ángulo objetivo en grados.
                Defaults to None.
            speed_factor (float, optional): Factor de velocidad entre 0 y 2.0.
                Defaults to 1.0.

        Note:
            La acción se ejecuta de forma asíncrona. Use execute_commands()
            para procesar el movimiento en cada frame.

        Example:
            >>> manager.move_robot_to(1, (500, 300), target_angle=90)
        """
        self.actions_in_progress[player_id] = {
            'type': 'move',
            'target_pos': target_pos,
            'target_angle': target_angle,
            'speed_factor': speed_factor
        }
        log.info("Robot %i: Ordenado movimiento a %s", player_id, target_pos)

    def rotate_robot_to(self, player_id, target_angle):
        """Ordena a un robot girar a un ángulo específico.

        Args:
            player_id (int): ID único del jugador.
            target_angle (float): Ángulo objetivo en grados (0-360).

        Note:
            El ángulo se normaliza automáticamente al rango -180 a 180.
            La rotación se ejecuta de forma asíncrona.

        Example:
            >>> manager.rotate_robot_to(1, 90)  # Girar hacia arriba
        """
        self.actions_in_progress[player_id] = {
            'type': 'rotate',
            'target_angle': target_angle
        }
        log.info("Robot %i: Ordenado giro a %.2f grados", player_id, target_angle)

    def capture_ball(self, player_id):
        """Ordena a un robot capturar la pelota activando el motor físico.

        Activa el motor dribbler del robot para capturar y mantener
        la pelota. En robots reales, envía comando RF para activar
        el motor con potencia alta.

        Args:
            player_id (int): ID único del jugador.

        Note:
            Para robots reales, activa físicamente el motor dribbler.
            La acción permanece activa hasta completarse o cancelarse.

        Example:
            >>> manager.capture_ball(1)
        """
        # ACTIVAR MOTOR DE CAPTURA EN ROBOTS REALES
        if self.rf_controller:
            # Activar dribbler con potencia alta para capturar
            success = self.rf_controller.set_dribbler(player_id, 1.0)
            if success:
                log.info("Robot %i: Motor de captura ACTIVADO vía RF", player_id)
            else:
                log.error("Robot %i: Error al activar motor de captura", player_id)

        # Registrar acción en progreso
        self.actions_in_progress[player_id] = {
            'type': 'capture_ball',
            'motor_activated': True,
            'start_time': time.time()
        }

        log.info("Robot %i: Ordenado capturar pelota con motor", player_id)

    def kick_ball(self, player_id, target_pos, ball, power=1.0):
        """Ordena a un robot patear la pelota hacia una posición objetivo.

        Args:
            player_id (int): ID único del jugador.
            target_pos (tuple): Posición objetivo como tupla (x, y).
            ball (Ball): Objeto pelota del juego.
            power (float, optional): Potencia del tiro entre 0 y 1.
                Defaults to 1.0.

        Note:
            El robot debe tener la pelota antes de ejecutar esta acción.
            La potencia afecta tanto la velocidad como la distancia del tiro.

        Example:
            >>> manager.kick_ball(1, (750, 450), ball, power=0.8)
        """
        self.actions_in_progress[player_id] = {
            'type': 'kick_ball',
            'target_pos': target_pos,
            'power': power,
            'ball': ball
        }
        log.info("Robot %i: Ordenado patear pelota hacia %s con potencia %.2f",
                 player_id, target_pos, power)

    def move_with_ball(self, player_id, target_pos, ball, speed_factor=0.7):
        """Ordena a un robot moverse con la pelota hacia una posición.

        El robot mantiene el control de la pelota mientras se mueve,
        típicamente usado para driblar o posicionarse estratégicamente.

        Args:
            player_id (int): ID único del jugador.
            target_pos (tuple): Posición objetivo como tupla (x, y).
            ball (Ball): Objeto pelota del juego.
            speed_factor (float, optional): Factor de velocidad reducido
                para mantener control. Defaults to 0.7.

        Note:
            La velocidad se reduce automáticamente para mantener el control
            de la pelota durante el movimiento.

        Example:
            >>> manager.move_with_ball(1, (400, 300), ball, speed_factor=0.5)
        """
        self.actions_in_progress[player_id] = {
            'type': 'move_with_ball',
            'target_pos': target_pos,
            'speed_factor': speed_factor,
            'ball': ball
        }
        log.info("Robot %i: Ordenado moverse con pelota hacia %s",
                 player_id, target_pos)
