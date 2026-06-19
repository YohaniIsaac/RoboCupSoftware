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
    PATH_PLANNING_BALL_OBSTACLE_RADIUS,
    PATH_PLANNING_OBSTACLE_CLEARANCE,
    PATH_PLANNING_CONTEST_RADIUS_PX,
    PATH_PLANNING_CONTEST_CLEARANCE,
    CAPTURE_ACTIVATE_DISTANCE_PX,
    RRT_WAYPOINT_ARRIVAL_PX,
    RRT_REPLAN_POSITION_PX,
    RRT_REPLAN_COOLDOWN_S,
    RRT_OBSTACLE_MOVE_PX,
    RRT_HOLD_REPLAN_PERIOD_S,
    ROTATE_RECOMMAND_MIN_DEG,
    ROBOT_DETECTION_LOST_RAMPDOWN_S,
    ROBOT_MAX_LINEAR_SPEED,
    OBSTACLE_PROXIMITY_NEAR_PX,
    OBSTACLE_PROXIMITY_FAR_PX,
    OBSTACLE_TIGHT_THRESHOLD_PX,
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
                 field=None, rf_controller=None):
        """Inicializa el gestor de comandos para robots.

        Args:
            team_players (list): Lista de objetos Player del equipo.
            ball (Ball): Objeto pelota del juego.
            use_real_robots (bool, optional): Si True, utiliza comunicación
                real con robots físicos. Defaults to False.
            port (str, optional): Puerto serial para comunicación con Arduino.
                Defaults to '/dev/ttyUSB0'.
            field: FieldGeometry con geometría del campo. Defaults to FIELD_SIM.
            rf_controller: RFController externo compartido. Si se pasa, se usa
                directamente sin abrir un nuevo puerto serial.

        Note:
            Si falla la inicialización del controlador RF, automáticamente
            cambia a modo simulación sin lanzar excepciones.
        """
        # Inicializar logger
        self.team_players = team_players
        self.ball = ball
        self.use_real_robots = use_real_robots
        self.field = field if field is not None else FIELD_SIM

        # Inicializar controlador RF
        self.rf_controller = None
        if rf_controller is not None:
            # RFController compartido externamente (ej. decisión 2v2)
            self.rf_controller = rf_controller
            self.use_real_robots = True
        elif use_real_robots:
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
        self._path_projected: dict = {}  # player_id → bool: path termina antes del goal real
        self._waiting_goal_clear: dict = {}  # player_id → True: goal bloqueado, sosteniendo posición
        self._last_sent_pos:  dict = {}  # player_id → (x,y)
        self._last_sent_goal: dict = {}  # player_id → (x,y)
        self._last_replan_t:  dict = {}  # player_id → float timestamp
        self._last_obs_pos:   dict = {}  # player_id → {robot_id: (x,y)}
        self._all_robot_data: list = []  # [{id,x,y}, ...] actualizado cada frame
        self._ball_pos: tuple | None = None  # (x, y) de la pelota, actualizado cada frame
        self.replan_cooldown_s_override: float | None = None  # None = usar RRT_REPLAN_COOLDOWN_S

    def get_path_state(self, player_id):
        """Retorna (path, wp_idx_activo) para visualización.

        Returns:
            tuple: (list[(x,y), ...], int) — lista vacía y -1 si el robot no
            tiene path activo en este momento.
        """
        return (
            list(self._current_paths.get(player_id, [])),
            self._current_wp_idx.get(player_id, -1),
        )

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

    def update_ball_data(self, ball_x: float, ball_y: float):
        """Actualiza posición de la pelota para usarla como obstáculo en el planner."""
        self._ball_pos = (int(ball_x), int(ball_y))

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
            # Ramp-down de velocidad cuando el robot no es detectado por la cámara
            _lost_elapsed = (time.time() - player.last_seen_t
                             if player.last_seen_t > 0 else 0.0)
            if _lost_elapsed >= ROBOT_DETECTION_LOST_RAMPDOWN_S:
                if self.rf_controller:
                    self.rf_controller.set_motors(player.id + 1, 0, 0)
                if player.id in self.controllers:
                    self.controllers[player.id].detection_pwm_cap = 0
                continue
            _detection_factor = 1.0 - _lost_elapsed / ROBOT_DETECTION_LOST_RAMPDOWN_S
            if player.id in self.controllers:
                self.controllers[player.id].detection_pwm_cap = (
                    None if _detection_factor >= 1.0
                    else int(_detection_factor * ROBOT_MAX_LINEAR_SPEED)
                )

            # Activar factor de dribble cuando el robot tiene posesión (post-captura)
            if player.id in self.controllers:
                controller = self.controllers[player.id]
                controller.dribble_pwm_factor = DRIBBLE_PWM_FACTOR if player.has_ball() else 1.0

            # Gate del AUTO-KICK por stuck: solo válido cuando el robot está empujando la pelota
            # (move_with_ball). Evita kicks falsos durante circle_ball, move_to_intercept, etc.
            if player.id in self.controllers:
                _action_type = (
                    self.actions_in_progress[player.id].get('type')
                    if player.id in self.actions_in_progress else None
                )
                self.controllers[player.id].auto_kick_enabled = (
                    _action_type == 'move_with_ball'
                )

            if player.id in self.actions_in_progress:
                action = self.actions_in_progress[player.id]

                if action['type'] == 'move':
                    target_pos   = action['target_pos']
                    target_angle = action.get('target_angle')
                    nominal_arrival = action.get('arrival_threshold')
                    player_id    = player.id

                    if action.get('direct', False):
                        # ── MODO DIRECTO (sin planner) ──────────────────────────────────
                        # PID con steering directo al target, sin RRT*. Lo usa
                        # advance_to_contact: el corredor ya fue validado por el BT y el
                        # planner es contraproducente (proyectaría el target de contacto
                        # fuera del radio inflado del rival/pelota y congelaría el avance,
                        # impidiendo el kick). El estado de path/hold se limpia en move_robot_to.
                        is_completed = self.controllers[player.id].move_to_position(
                            player, target_pos, target_angle,
                            arrival_threshold=nominal_arrival,
                        )
                        if is_completed:
                            del self.actions_in_progress[player.id]
                            log.info("Robot %i: Completado movimiento a %s (directo, sin planner)",
                                     player.id, target_pos)
                    elif player_id in self._plan_pipes:
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

                        # Umbral de llegada efectivo: contraído si hay obstáculo cerca
                        # del target. Se aplica al sanity check del planner y al PID.
                        effective_arrival = self._effective_threshold(
                            target_pos, nominal_arrival, obs_dicts, player_id
                        )

                        # Inflación dependiente del contexto + pelota como obstáculo.
                        # Si el goal cae en la zona de disputa de la pelota se usa un
                        # clearance reducido (varios robots convergen ahí y el contacto
                        # es obligatorio; el global los infla a ~3x y bloquea el área).
                        # En navegación normal se mantiene el clearance completo.
                        # La pelota se incluye como obstáculo solo si el goal está lejos
                        # de ella (en captura/contacto el robot DEBE llegar a la pelota).
                        req_clearance = PATH_PLANNING_OBSTACLE_CLEARANCE
                        if self._ball_pos is not None:
                            dist_goal_ball = math.hypot(
                                goal[0] - self._ball_pos[0],
                                goal[1] - self._ball_pos[1],
                            )
                            if dist_goal_ball <= PATH_PLANNING_CONTEST_RADIUS_PX:
                                req_clearance = PATH_PLANNING_CONTEST_CLEARANCE
                            if dist_goal_ball > CAPTURE_ACTIVATE_DISTANCE_PX:
                                obstacles.append([self._ball_pos[0], self._ball_pos[1],
                                                  PATH_PLANNING_BALL_OBSTACLE_RADIUS])

                        # 1. Consumir path nuevo si llegó del planner
                        try:
                            result      = self._path_queues[player_id].get_nowait()
                            new_path    = result.get('path', [])
                            result_goal = result.get('goal') or (0, 0)
                            goal_drift  = math.hypot(result_goal[0] - goal[0],
                                                     result_goal[1] - goal[1])
                            # Bug 3: comparación fuzzy (tolerancia 2×ARRIVAL) en vez de igualdad exacta.
                            # Permite adoptar paths planificados para un goal cercano al actual.
                            if new_path and goal_drift < RRT_WAYPOINT_ARRIVAL_PX * 2:
                                wp_idx = path_closest_waypoint_idx(new_path, player.x, player.y)
                                # Bug 1: saltar wps triviales al inicio para no enviar PWM=0.
                                # move_to_position para wp[0]=robot_pos causaba "WAYPOINT ALCANZADO
                                # dist=0.0px" + stop de motor en cada adopción de path.
                                while wp_idx < len(new_path) - 1:
                                    d = math.hypot(player.x - new_path[wp_idx][0],
                                                   player.y - new_path[wp_idx][1])
                                    if d < RRT_WAYPOINT_ARRIVAL_PX:
                                        wp_idx += 1
                                    else:
                                        break
                                # Descartar path si el único wp restante está en la posición actual
                                # (cubre el caso len=1 y cualquier path donde todos los wps son triviales).
                                # Sin este check, move_to_position con dist=0 enviaba PWM=0 → motor stop.
                                last_d = math.hypot(player.x - new_path[wp_idx][0],
                                                    player.y - new_path[wp_idx][1])
                                if wp_idx == len(new_path) - 1 and last_d < RRT_WAYPOINT_ARRIVAL_PX:
                                    log.debug("R%d: path trivial descartado (único wp restante "
                                              "a %.0fpx)", player_id, last_d)
                                else:
                                    self._current_paths[player_id]  = new_path
                                    self._current_wp_idx[player_id] = wp_idx
                                    self._path_projected[player_id] = result.get('projected', False)
                                    self._waiting_goal_clear.pop(player_id, None)
                                    self.controllers[player_id]._pid_state.pop(player_id, None)
                                    log.info("R%d: path adoptado %d wp (desde idx %d)%s",
                                             player_id, len(new_path), wp_idx,
                                             " [goal proyectado]"
                                             if self._path_projected[player_id] else "")
                        except Exception:
                            pass

                        # 2. Decidir si hay que pedir replan
                        last_goal   = self._last_sent_goal.get(player_id)
                        last_pos    = self._last_sent_pos.get(player_id)
                        _replan_cd  = (self.replan_cooldown_s_override
                                       if self.replan_cooldown_s_override is not None
                                       else RRT_REPLAN_COOLDOWN_S)
                        need_replan = False
                        # Bug 2b: comparación por distancia en vez de igualdad exacta.
                        # Evita replans continuos por jitter mínimo del goal (pelota que tiembla <20px).
                        if last_goal is None or math.hypot(
                                last_goal[0] - goal[0],
                                last_goal[1] - goal[1]) > RRT_WAYPOINT_ARRIVAL_PX:
                            need_replan = True   # goal cambió significativamente
                        elif (last_pos is None or
                              math.hypot(cur_pos[0] - last_pos[0],
                                         cur_pos[1] - last_pos[1]) > RRT_REPLAN_POSITION_PX):
                            if now - self._last_replan_t.get(player_id, 0) >= _replan_cd:
                                need_replan = True   # robot se alejó del punto enviado
                        # Bug 5: cooldown también para obstacles_moved (antes no lo tenía).
                        elif obstacles_moved(self._last_obs_pos.get(player_id, {}),
                                             obs_dicts, player_id, RRT_OBSTACLE_MOVE_PX):
                            if now - self._last_replan_t.get(player_id, 0) >= _replan_cd:
                                need_replan = True   # un obstáculo robot se movió
                        # Bug 4: pelota no estaba en obs_dicts → no se detectaba su movimiento.
                        elif self._ball_pos is not None:
                            last_ball = self._last_obs_pos.get(player_id, {}).get('_ball')
                            if last_ball is not None and math.hypot(
                                    self._ball_pos[0] - last_ball[0],
                                    self._ball_pos[1] - last_ball[1]) > RRT_OBSTACLE_MOVE_PX:
                                if now - self._last_replan_t.get(player_id, 0) >= _replan_cd:
                                    need_replan = True   # pelota se movió significativamente

                        # Goal bloqueado por obstáculo: replan periódico aunque nada
                        # se mueva, para detectar cuándo el bloqueador despeja el goal.
                        if (not need_replan
                                and self._waiting_goal_clear.get(player_id)
                                and now - self._last_replan_t.get(player_id, 0)
                                >= RRT_HOLD_REPLAN_PERIOD_S):
                            need_replan = True

                        if need_replan:
                            pipe = self._plan_pipes[player_id]
                            try:
                                while pipe.poll():
                                    pipe.recv()
                                pipe.send({
                                    'robot_pos': cur_pos,
                                    'goal_pos':  goal,
                                    'obstacles': obstacles,
                                    'clearance': req_clearance,
                                    'arrival_px': (effective_arrival
                                                   if effective_arrival is not None
                                                   else RRT_WAYPOINT_ARRIVAL_PX),
                                })
                                self._last_sent_pos[player_id]  = cur_pos
                                self._last_sent_goal[player_id] = goal
                                self._last_replan_t[player_id]  = now
                                self._last_obs_pos[player_id] = {
                                    r['id']: (r['x'], r['y']) for r in obs_dicts
                                }
                                # Bug 4: guardar pelota para detectar si se mueve hacia el camino
                                if self._ball_pos is not None:
                                    self._last_obs_pos[player_id]['_ball'] = self._ball_pos
                                log.debug("R%d: replan solicitado → %s", player_id, goal)
                            except Exception as e:
                                log.warning("R%d: error enviando replan: %s", player_id, e)

                        # 3. Ejecutar movimiento
                        path   = self._current_paths.get(player_id, [])
                        wp_idx = self._current_wp_idx.get(player_id, 0)

                        if path and wp_idx < len(path):
                            # Seguir waypoint activo del path planificado.
                            # En el último wp se aplica el umbral efectivo (arrival_threshold);
                            # en los intermedios se mantiene RRT_WAYPOINT_ARRIVAL_PX.
                            wp      = path[wp_idx]
                            is_last = (wp_idx == len(path) - 1)
                            wp_arrival = effective_arrival if is_last else None
                            arrived = self.controllers[player_id].move_to_position(
                                player, wp, target_angle, arrival_threshold=wp_arrival
                            )
                            # Umbral relajado en waypoints intermedios
                            if not is_last:
                                dist = math.hypot(player.x - wp[0], player.y - wp[1])
                                if dist < RRT_WAYPOINT_ARRIVAL_PX:
                                    arrived = True
                            if arrived:
                                if is_last:
                                    # Limpiar path siempre al terminar el último waypoint
                                    self._current_paths.pop(player_id, None)
                                    self._current_wp_idx.pop(player_id, None)
                                    was_projected = self._path_projected.pop(player_id, False)
                                    # Solo completar la acción si este path apuntaba al goal actual.
                                    # Si el goal cambió (path obsoleto), mantener la acción activa
                                    # y dejar que el PID directo lleve al robot mientras llega el nuevo plan.
                                    dist_wp_to_goal = math.hypot(wp[0] - goal[0], wp[1] - goal[1])
                                    if dist_wp_to_goal < RRT_WAYPOINT_ARRIVAL_PX:
                                        del self.actions_in_progress[player_id]
                                        log.info("R%d: llegó al objetivo (último wp)", player_id)
                                    elif was_projected:
                                        # El goal real sigue bloqueado por un obstáculo:
                                        # sostener posición aquí (sin PID directo, que
                                        # embestiría al bloqueador) y replanificar
                                        # periódicamente hasta que se despeje.
                                        self._waiting_goal_clear[player_id] = True
                                        if self.rf_controller:
                                            self.rf_controller.set_motors(player.id + 1, 0, 0)
                                        log.info("R%d: goal %s bloqueado — esperando a que "
                                                 "se despeje a %.0fpx", player_id, goal,
                                                 dist_wp_to_goal)
                                    else:
                                        log.debug("R%d: path obsoleto (last wp a %.0fpx del goal actual), "
                                                  "esperando nuevo plan", player_id, dist_wp_to_goal)
                                else:
                                    self._current_wp_idx[player_id] = wp_idx + 1
                                    self.controllers[player_id]._pid_state.pop(player_id, None)
                                    log.debug("R%d: wp %d/%d alcanzado",
                                              player_id, wp_idx + 1, len(path))
                        elif self._waiting_goal_clear.get(player_id):
                            # Goal bloqueado: sostener posición hasta adoptar un path
                            # nuevo. El PID directo embestiría al robot bloqueador.
                            # Se reenvía stop cada frame (mismo patrón que detección
                            # perdida) para que el robot no avance ante un comando suelto.
                            if self.rf_controller:
                                self.rf_controller.set_motors(player.id + 1, 0, 0)
                        else:
                            # Fallback: PID directo mientras llega el primer path
                            is_completed = self.controllers[player_id].move_to_position(
                                player, target_pos, target_angle,
                                arrival_threshold=effective_arrival,
                            )
                            if is_completed:
                                del self.actions_in_progress[player_id]
                                log.info("Robot %i: Completado movimiento a %s (directo)",
                                         player_id, target_pos)

                    else:
                        # Sin planner configurado: comportamiento original.
                        # Sin obs_dicts disponibles aquí; se usa el umbral nominal directo.
                        is_completed = self.controllers[player.id].move_to_position(
                            player,
                            action['target_pos'],
                            action.get('target_angle'),
                            arrival_threshold=nominal_arrival,
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
                        speed = int(speed * _detection_factor)
                        self.rf_controller.set_motors(fw_id, speed, speed)

    def move_robot_to(self, player_id, target_pos, target_angle=None, speed_factor=1.0,
                      arrival_threshold=None, direct=False):
        """Ordena a un robot moverse a una posición específica.

        Args:
            player_id (int): ID único del jugador.
            target_pos (tuple): Posición objetivo como tupla (x, y).
            target_angle (float, optional): Ángulo objetivo en grados.
                Defaults to None.
            speed_factor (float, optional): Factor de velocidad entre 0 y 2.0.
                Defaults to 1.0.
            arrival_threshold (float, optional): Umbral nominal (px) para
                declarar el target alcanzado. Si None, se usa el del PID
                (ROBOT_POSITION_THRESHOLD). El umbral efectivo se contrae
                automáticamente si hay otros robots cerca del target.
            direct (bool, optional): Si True, ignora el planner (RRT*) y va al
                target con PID directo + steering. Para maniobras de corto alcance
                ya validadas por el caller (p.ej. el creep de advance_to_contact),
                donde el planner proyectaría/congelaría el avance. Defaults to False.

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
        if (existing and existing['type'] == 'move'
                and existing.get('direct', False) == direct):
            delta = np.linalg.norm(
                np.array(existing['target_pos'][:2]) - np.array(target_pos[:2])
            )
            if delta < 20:
                log.debug("[Guard:move] Robot %i: comando ignorado (mismo target, delta=%.1fpx)", player_id, delta)
                return  # Mismo destino y mismo modo, dejar que el PID continúe
        log.debug("[Guard:move] Robot %i: nuevo target aceptado → %s", player_id, target_pos)

        # Bug 2a: solo limpiar _last_sent_goal si el nuevo goal difiere significativamente del
        # último planificado. Evita replans innecesarios cuando behind_pos jitters <20px entre ticks.
        last_planned = self._last_sent_goal.get(player_id)
        if last_planned is None or math.hypot(
            target_pos[0] - last_planned[0], target_pos[1] - last_planned[1]
        ) > RRT_WAYPOINT_ARRIVAL_PX:
            self._last_sent_goal.pop(player_id, None)  # forzar replan solo si cambio significativo
            # La espera y el flag de proyección pertenecen al goal anterior
            self._waiting_goal_clear.pop(player_id, None)
            self._path_projected.pop(player_id, None)

        if direct:
            # Modo directo: descartar cualquier path/hold del planner para que el
            # robot no siga un waypoint obsoleto ni sostenga posición durante el creep.
            self._current_paths.pop(player_id, None)
            self._current_wp_idx.pop(player_id, None)
            self._waiting_goal_clear.pop(player_id, None)
            self._path_projected.pop(player_id, None)

        self.actions_in_progress[player_id] = {
            'type': 'move',
            'target_pos': target_pos,
            'target_angle': target_angle,
            'speed_factor': speed_factor,
            'arrival_threshold': arrival_threshold,
            'direct': direct,
        }
        log.info("Robot %i: Ordenado movimiento a %s%s", player_id, target_pos,
                 " (directo)" if direct else "")

    def _effective_threshold(self, target_pos, nominal_threshold, obstacles, exclude_id):
        """Contrae el umbral nominal de llegada cuando hay obstáculos cerca del target.

        Si nominal es None, retorna None (PID usa su default). Si el obstáculo
        más cercano está a >= OBSTACLE_PROXIMITY_FAR_PX → umbral nominal sin
        cambios. Si está a <= OBSTACLE_PROXIMITY_NEAR_PX → umbral mínimo.
        Entre ambos rangos se interpola linealmente.

        Esto evita colisiones cuando dos robots tienen targets cercanos y
        ambos usan thresholds laxos (sus circunferencias de aceptación se
        solapan).
        """
        if nominal_threshold is None:
            return None
        nearest = float('inf')
        for r in obstacles:
            if r.get('id') == exclude_id:
                continue
            d = math.hypot(r['x'] - target_pos[0], r['y'] - target_pos[1])
            if d < nearest:
                nearest = d
        if nearest >= OBSTACLE_PROXIMITY_FAR_PX:
            return nominal_threshold
        if nearest <= OBSTACLE_PROXIMITY_NEAR_PX:
            return min(nominal_threshold, float(OBSTACLE_TIGHT_THRESHOLD_PX))
        ratio = (nearest - OBSTACLE_PROXIMITY_NEAR_PX) / (
            OBSTACLE_PROXIMITY_FAR_PX - OBSTACLE_PROXIMITY_NEAR_PX
        )
        return OBSTACLE_TIGHT_THRESHOLD_PX + ratio * (
            nominal_threshold - OBSTACLE_TIGHT_THRESHOLD_PX
        )

    def rotate_robot_to(self, player_id, target_angle):
        """Ordena a un robot girar a un ángulo específico.

        Args:
            player_id (int): ID único del jugador.
            target_angle (float): Ángulo objetivo en grados (0-360).

        Note:
            El ángulo se normaliza automáticamente al rango -180 a 180.
            La rotación se ejecuta de forma asíncrona.
            Si ya hay una rotación en curso hacia un ángulo similar
            (< ROTATE_RECOMMAND_MIN_DEG), el comando se ignora para no
            interrumpir al PID con jitter de detección.

        Example:
            >>> manager.rotate_robot_to(1, 90)  # Girar hacia arriba
        """
        # Guard: no sobreescribir rotación en curso si el ángulo es similar.
        # El umbral está calibrado para absorber el jitter de la detección de
        # pelota (4-8°/frame) sin perder responsividad ante cambios reales.
        existing = self.actions_in_progress.get(player_id)
        if existing and existing['type'] == 'rotate':
            angle_diff = abs((target_angle - existing['target_angle'] + 180) % 360 - 180)
            if angle_diff < ROTATE_RECOMMAND_MIN_DEG:
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
