import math
import time
import logging
import numpy as np
from robot_soccer.config import (
    FIELD_SIM,
    DRIBBLE_PWM_FACTOR,
    DRIBBLER_CAPTURE_POWER,
    DRIBBLER_HOLD_POWER,
    CAPTURE_CREEP_SPEED_PWM,
    PATH_PLANNING_ROBOT_OBSTACLE_RADIUS,
    RRT_WAYPOINT_ARRIVAL_PX,
    RRT_REPLAN_POSITION_PX,
    RRT_REPLAN_COOLDOWN_S,
    RRT_OBSTACLE_MOVE_PX,
)
from robot_soccer.ai.path_planning.tools_for_path_planing import (
    path_closest_waypoint_idx,
    obstacles_moved,
)
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

        # Path planning: canales de comunicación con planning_worker subprocesses
        self._plan_pipes:  dict = {}   # player_id → Pipe
        self._path_queues: dict = {}   # player_id → Queue
        # Estado de seguimiento de path por robot
        self._current_paths:  dict = {}  # player_id → [(x,y), ...]
        self._current_wp_idx: dict = {}  # player_id → int
        self._last_sent_pos:  dict = {}  # player_id → (x,y)
        self._last_sent_goal: dict = {}  # player_id → (x,y)
        self._last_replan_t:  dict = {}  # player_id → float timestamp
        self._last_obs_pos:   dict = {}  # player_id → {robot_id: (x,y)}
        self._all_robot_data: list = []  # [{id,x,y}, ...] actualizado cada frame

    def shutdown(self):
        """Cierra las conexiones y detiene todos los robots.

        Detiene todos los robots del equipo y cierra la comunicación RF
        si está activa. Debe llamarse antes de terminar la aplicación.

        Note:
            Es seguro llamar este método múltiples veces.
        """
        if self.rf_controller:
            for player in self.team_players:
                firmware_id = player.id + 1
                self.rf_controller.set_dribbler(firmware_id, 0)  # apagar dribbler
                self.rf_controller.stop_robot(firmware_id)

            self.rf_controller.shutdown()

    def set_planning_channels(self, plan_pipes: dict, path_queues: dict):
        """Conecta los pipes/queues de los subprocesos planning_worker."""
        self._plan_pipes  = plan_pipes
        self._path_queues = path_queues

    def update_robot_data(self, all_robot_data: list):
        """Actualiza posiciones de todos los robots para construir obstacles del planner."""
        self._all_robot_data = all_robot_data

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
            # Activar factor de dribble cuando el robot tiene posesión (post-captura)
            if player.id in self.controllers:
                controller = self.controllers[player.id]
                controller.dribble_pwm_factor = DRIBBLE_PWM_FACTOR if player.has_ball() else 1.0

            if player.id in self.actions_in_progress:
                action = self.actions_in_progress[player.id]

                if action['type'] == 'move':
                    target_pos   = action['target_pos']
                    target_angle = action.get('target_angle')
                    player_id    = player.id

                    if player_id in self._plan_pipes:
                        # ── PATH PLANNING MODE ─────────────────────────────────────────
                        now     = time.time()
                        cur_pos = (int(player.x), int(player.y))
                        goal    = (int(target_pos[0]), int(target_pos[1]))

                        # Construir lista de obstáculos: todos los robots excepto yo
                        obs_dicts = [r for r in self._all_robot_data if r['id'] != player_id]
                        obstacles = [
                            [r['x'], r['y'], PATH_PLANNING_ROBOT_OBSTACLE_RADIUS]
                            for r in obs_dicts
                        ]

                        # 1. Consumir path nuevo si llegó del planner
                        try:
                            result   = self._path_queues[player_id].get_nowait()
                            new_path = result.get('path', [])
                            if new_path and result.get('goal') == goal:
                                wp_idx = path_closest_waypoint_idx(new_path, player.x, player.y)
                                self._current_paths[player_id]  = new_path
                                self._current_wp_idx[player_id] = wp_idx
                                self.controllers[player_id]._pid_state.pop(player_id, None)
                                log.info("R%d: path adoptado %d wp (desde idx %d)",
                                         player_id, len(new_path), wp_idx)
                        except Exception:
                            pass

                        # 2. Decidir si hay que pedir replan
                        last_goal = self._last_sent_goal.get(player_id)
                        last_pos  = self._last_sent_pos.get(player_id)
                        need_replan = False
                        if last_goal != goal:
                            need_replan = True   # goal cambió
                        elif (last_pos is None or
                              math.hypot(cur_pos[0] - last_pos[0],
                                         cur_pos[1] - last_pos[1]) > RRT_REPLAN_POSITION_PX):
                            if now - self._last_replan_t.get(player_id, 0) >= RRT_REPLAN_COOLDOWN_S:
                                need_replan = True   # robot se alejó del punto enviado
                        elif obstacles_moved(self._last_obs_pos.get(player_id, {}),
                                             obs_dicts, player_id, RRT_OBSTACLE_MOVE_PX):
                            need_replan = True   # un obstáculo se movió

                        if need_replan:
                            pipe = self._plan_pipes[player_id]
                            try:
                                while pipe.poll():
                                    pipe.recv()
                                pipe.send({
                                    'robot_pos': cur_pos,
                                    'goal_pos':  goal,
                                    'obstacles': obstacles,
                                })
                                self._last_sent_pos[player_id]  = cur_pos
                                self._last_sent_goal[player_id] = goal
                                self._last_replan_t[player_id]  = now
                                self._last_obs_pos[player_id]   = {
                                    r['id']: (r['x'], r['y']) for r in obs_dicts
                                }
                                log.debug("R%d: replan solicitado → %s", player_id, goal)
                            except Exception as e:
                                log.warning("R%d: error enviando replan: %s", player_id, e)

                        # 3. Ejecutar movimiento
                        path   = self._current_paths.get(player_id, [])
                        wp_idx = self._current_wp_idx.get(player_id, 0)

                        if path and wp_idx < len(path):
                            # Seguir waypoint activo del path planificado
                            wp      = path[wp_idx]
                            is_last = (wp_idx == len(path) - 1)
                            arrived = self.controllers[player_id].move_to_position(
                                player, wp, target_angle
                            )
                            # Umbral relajado en waypoints intermedios
                            if not is_last:
                                dist = math.hypot(player.x - wp[0], player.y - wp[1])
                                if dist < RRT_WAYPOINT_ARRIVAL_PX:
                                    arrived = True
                            if arrived:
                                if is_last:
                                    del self.actions_in_progress[player_id]
                                    self._current_paths.pop(player_id, None)
                                    self._current_wp_idx.pop(player_id, None)
                                    log.info("R%d: llegó al objetivo (último wp)", player_id)
                                else:
                                    self._current_wp_idx[player_id] = wp_idx + 1
                                    self.controllers[player_id]._pid_state.pop(player_id, None)
                                    log.debug("R%d: wp %d/%d alcanzado",
                                              player_id, wp_idx + 1, len(path))
                        else:
                            # Fallback: PID directo mientras llega el primer path
                            is_completed = self.controllers[player_id].move_to_position(
                                player, target_pos, target_angle
                            )
                            if is_completed:
                                del self.actions_in_progress[player_id]
                                log.info("Robot %i: Completado movimiento a %s (directo)",
                                         player_id, target_pos)

                    else:
                        # Sin planner configurado: comportamiento original
                        is_completed = self.controllers[player.id].move_to_position(
                            player,
                            action['target_pos'],
                            action.get('target_angle')
                        )
                        if is_completed:
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
                    # El dribbler ya fue activado en command_manager.capture_ball().
                    # La lógica de orientación, acercamiento y confirmación la gestiona
                    # el nodo capture_ball del árbol de comportamiento.
                    # execute_capture_ball() tiene un bug de unidades de ángulo cuando
                    # se usa junto con el BT (player.angle en radianes vs grados).
                    # Marcar como completado inmediatamente para liberar el slot de acción.
                    del self.actions_in_progress[player.id]
                    log.debug("Robot %i: Accion capture_ball liberada (dribbler activo)", player.id)

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

                elif action['type'] == 'creep_forward':
                    # Avance lento directo (sin PID) para acercamiento a la pelota.
                    # Permanece activo hasta que el BT lo cancele explícitamente.
                    if self.rf_controller:
                        fw_id = player.id + 1
                        speed = action.get('speed', CAPTURE_CREEP_SPEED_PWM)
                        self.rf_controller.set_motors(fw_id, speed, speed)

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
            Si ya hay un movimiento en curso hacia una posición similar (< 20px),
            el comando se ignora para no interrumpir al PID mid-ejecución.

        Example:
            >>> manager.move_robot_to(1, (500, 300), target_angle=90)
        """
        # Guard: no sobreescribir si ya hay un movimiento en curso hacia el mismo target.
        # Esto evita que el BT (que tickea cada 100ms) interrumpa al PID
        # antes de que complete una rotación o movimiento.
        existing = self.actions_in_progress.get(player_id)
        if existing and existing['type'] == 'move':
            delta = np.linalg.norm(
                np.array(existing['target_pos'][:2]) - np.array(target_pos[:2])
            )
            if delta < 20:
                log.debug("[Guard:move] Robot %i: comando ignorado (mismo target, delta=%.1fpx)", player_id, delta)
                return  # Mismo destino, dejar que el PID continúe
        log.debug("[Guard:move] Robot %i: nuevo target aceptado → %s", player_id, target_pos)

        # Nuevo target → invalidar path anterior para forzar replanificación
        self._current_paths.pop(player_id, None)
        self._current_wp_idx.pop(player_id, None)
        self._last_sent_goal.pop(player_id, None)

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
            Si ya hay una rotación en curso hacia el mismo ángulo (< 5°),
            el comando se ignora para no interrumpir al PID.

        Example:
            >>> manager.rotate_robot_to(1, 90)  # Girar hacia arriba
        """
        # Guard: no sobreescribir rotación en curso si el ángulo es similar
        existing = self.actions_in_progress.get(player_id)
        if existing and existing['type'] == 'rotate':
            angle_diff = abs((target_angle - existing['target_angle'] + 180) % 360 - 180)
            if angle_diff < 5:
                return  # Misma rotación, dejar que el PID continúe

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
            # Firmware usa IDs 1-based (player_id es 0-based)
            firmware_id = player_id + 1
            success = self.rf_controller.set_dribbler(firmware_id, DRIBBLER_CAPTURE_POWER)
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

    def creep_forward(self, player_id, speed=None):
        """Avance lento directo hacia adelante, sin PID.

        Envía PWM constante a ambos motores a través de execute_commands(),
        permaneciendo activo hasta que el BT cancele la acción explícitamente.
        Usado para acercarse a la pelota suavemente sin empujarla.

        Args:
            player_id (int): ID del jugador.
            speed (int, optional): PWM de avance. Defaults a CAPTURE_CREEP_SPEED_PWM.
        """
        if speed is None:
            speed = CAPTURE_CREEP_SPEED_PWM
        self.actions_in_progress[player_id] = {
            'type': 'creep_forward',
            'speed': int(speed),
        }
        log.info("Robot %i: Creep forward activado | PWM=%d", player_id, speed)

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
