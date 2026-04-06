"""Comportamientos específicos de fútbol para el sistema de árboles de comportamiento.

Este módulo implementa nodos específicos y árboles predefinidos para las
estrategias y tácticas de los robots en el juego de fútbol.
"""

import logging
import time

import numpy as np

log = logging.getLogger(__name__)
from robot_soccer.ai.behavior_tree.utils import calculate_ball_approach_position
from robot_soccer.config import (
    ROL_ATACANTE, ROL_DEFENSIVO, FIELD_SIM,
    BT_SHOT_DISTANCE_RATIO, BT_PASS_MIN_RATIO, BT_PASS_MAX_RATIO,
    BT_CAPTURE_RANGE_RATIO, BT_APPROACH_RATIO, BT_CAPTURE_ACTIVATE_RATIO,
    BT_CAPTURE_CONFIRM_RATIO, BT_INTERCEPT_RATIO, BT_SUPPORT_DISTANCE_RATIO,
    BT_DEFENDER_WAIT_RATIO, BT_DRIBBLE_GOAL_RATIO, BT_DRIBBLE_SPACING_RATIO,
    BT_DEFENSIVE_ARRIVAL_RATIO, BT_ATTACKER_PRIORITY_MARGIN_RATIO,
    ROBOT_POSITION_THRESHOLD,
    CAPTURE_ACTIVATE_DISTANCE_PX,
    CAPTURE_OVERSHOOT_PX,
    CAPTURE_CONFIRM_DISTANCE_PX,
    CAPTURE_CREEP_SPEED_PWM,
    DRIBBLER_HOLD_POWER,
    BEHIND_BALL_APPROACH_PX,
    BEHIND_BALL_LATERAL_OFFSET_PX,
    BEHIND_BALL_ALIGN_TOLERANCE_DEG,
    PUSH_BURST_PWM,
    CONTACT_SETTLE_TIME_S,
    ADVANCE_ESCAPE_FACTOR,
    SETTLE_ESCAPE_FACTOR,
)

from .base import (
    NodeStatus,
    SequenceNode,
    SelectorNode,
    ConditionNode,
    ActionNode,
    StatefulActionNode,
    InverterNode,
)
from .utils import (
    calculate_dribbling_path_positions,
    calculate_shooting_position,
    calculate_pass_position,
    calculate_interception_position,
)

class Blackboard:
    """Pizarra (blackboard) para compartir datos entre nodos del árbol de comportamiento.

    Contiene el estado del juego y los robots.
    """

    def __init__(self, player, ball, team_players, opponents, team="red", field=None):
        """Inicializa la pizarra con el estado del juego.

        Args:
            player: Jugador que ejecuta este árbol
            ball: Objeto pelota
            team_players: Lista de jugadores del equipo
            opponents: Lista de jugadores rivales
            team: Equipo ('red' o 'blue'). Defaults to "red".
            field: FieldGeometry con geometría del campo. Defaults to FIELD_SIM.
        """
        self.logger = logging.getLogger(__name__)
        self.player = player  # Jugador que ejecuta este árbol
        self.ball = ball  # Pelota
        self.team_players = team_players  # Lista de jugadores del equipo
        self.opponents = opponents  # Lista de jugadores rivales
        self.team = team  # Equipo ('red' o 'blue')
        self.field = field if field is not None else FIELD_SIM

        # Datos contextuales del juego
        self.posesion_pelota = 0.5  # 0: posesión aliada, 0.5: libre, 1: posesión rival
        self.proximidad_equipo = 1.0  # 0: aliado, 1: neutral, 2: rival
        self.zona_pelota = 1.0  # 0: defensiva, 1: neutral, 2: ofensiva

        # Memoria del jugador
        self.last_action = None
        self.target_position = None
        self.action_start_time = None
        self.current_path = None

        # Variables para determinar orientación del campo según equipo
        self.own_goal_pos = None
        self.opponent_goal_pos = None
        self.defensive_zone_center = None
        self.neutral_zone_center = None
        self.offensive_zone_center = None

        # Variables de posicion
        self.defensive_positions = None
        self.offensive_positions = None

        # Posiciones estratégicas
        self.update_strategic_positions()

    def update_game_context(self, posesion, proximidad, zona):
        """Actualiza el contexto del juego con los valores de lógica difusa.

        Args:
            posesion: Valor de posesión de la pelota (0-1)
            proximidad: Valor de proximidad del equipo a la pelota (0-2)
            zona: Valor de zona donde está la pelota (0-2)
        """
        self.posesion_pelota = posesion
        self.proximidad_equipo = proximidad
        self.zona_pelota = zona

    def update_strategic_positions(self):
        """Calcula posiciones estratégicas en el campo según la orientación del equipo."""
        f = self.field
        # Determinar orientación del campo según equipo
        if self.team == "red":
            # Equipo rojo ataca hacia la derecha
            self.own_goal_pos = list(f.goal_left_center)
            self.opponent_goal_pos = list(f.goal_right_center)
            self.defensive_zone_center = [f.zone_x(0.2), f.zone_y(0.5)]
            self.neutral_zone_center = [f.zone_x(0.5), f.zone_y(0.5)]
            self.offensive_zone_center = [f.zone_x(0.8), f.zone_y(0.5)]
        else:
            # Equipo azul ataca hacia la izquierda
            self.own_goal_pos = list(f.goal_right_center)
            self.opponent_goal_pos = list(f.goal_left_center)
            self.defensive_zone_center = [f.zone_x(0.8), f.zone_y(0.5)]
            self.neutral_zone_center = [f.zone_x(0.5), f.zone_y(0.5)]
            self.offensive_zone_center = [f.zone_x(0.2), f.zone_y(0.5)]

        # Posiciones estratégicas defensivas
        self.defensive_positions = [
            [self.defensive_zone_center[0], f.zone_y(0.3)],  # Defensa arriba
            [self.defensive_zone_center[0], f.zone_y(0.7)],  # Defensa abajo
        ]

        # Posiciones estratégicas ofensivas
        self.offensive_positions = [
            [self.offensive_zone_center[0], f.zone_y(0.3)],  # Ataque arriba
            [self.offensive_zone_center[0], f.zone_y(0.7)],  # Ataque abajo
        ]


# CONDICIONES ESPECÍFICAS PARA FÚTBOL


def is_ball_free(blackboard):
    """Comprueba si la pelota está libre.

    Args:
        blackboard: Pizarra con el estado del juego

    Returns:
        bool: True si la pelota está libre
    """
    return 0.25 <= blackboard.posesion_pelota <= 0.75


def is_ball_in_team_possession(blackboard):
    """Comprueba si la pelota está en posesión del equipo.

    Args:
        blackboard: Pizarra con el estado del juego

    Returns:
        bool: True si el equipo tiene posesión de la pelota
    """
    return blackboard.posesion_pelota < 0.25


def is_ball_in_opponent_possession(blackboard):
    """Comprueba si la pelota está en posesión del equipo rival.

    Args:
        blackboard: Pizarra con el estado del juego

    Returns:
        bool: True si el equipo rival tiene posesión de la pelota
    """
    return blackboard.posesion_pelota > 0.75


def is_ball_close_to_team(blackboard):
    """Comprueba si la pelota está más cerca del equipo que del rival.

    Args:
        blackboard: Pizarra con el estado del juego

    Returns:
        bool: True si la pelota está más cerca del equipo
    """
    return blackboard.proximidad_equipo < 0.8


def is_ball_close_to_opponent(blackboard):
    """Comprueba si la pelota está más cerca del rival que del equipo.

    Args:
        blackboard: Pizarra con el estado del juego

    Returns:
        bool: True si la pelota está más cerca del equipo rival
    """
    return blackboard.proximidad_equipo > 1.2


def is_ball_in_defensive_zone(blackboard):
    """Comprueba si la pelota está en zona defensiva.

    Args:
        blackboard: Pizarra con el estado del juego

    Returns:
        bool: True si la pelota está en zona defensiva
    """
    return blackboard.zona_pelota < 0.4


def is_ball_in_neutral_zone(blackboard):
    """Comprueba si la pelota está en zona neutral.

    Args:
        blackboard: Pizarra con el estado del juego

    Returns:
        bool: True si la pelota está en zona neutral
    """
    return 0.4 <= blackboard.zona_pelota <= 1.6


def is_ball_in_offensive_zone(blackboard):
    """Comprueba si la pelota está en zona ofensiva.

    Args:
        blackboard: Pizarra con el estado del juego

    Returns:
        bool: True si la pelota está en zona ofensiva
    """
    return blackboard.zona_pelota > 1.6


def is_player_with_ball(blackboard):
    """Comprueba si este jugador tiene la pelota.

    Args:
        blackboard: Pizarra con el estado del juego

    Returns:
        bool: True si el jugador tiene la pelota
    """
    return blackboard.player.has_ball()


def is_player_attacker(blackboard):
    """Comprueba si este jugador tiene rol de atacante.

    Args:
        blackboard: Pizarra con el estado del juego

    Returns:
        bool: True si el jugador es atacante
    """
    return blackboard.player.rol == ROL_ATACANTE


def is_player_defender(blackboard):
    """Comprueba si este jugador tiene rol de defensor.

    Args:
        blackboard: Pizarra con el estado del juego

    Returns:
        bool: True si el jugador es defensor
    """
    return blackboard.player.rol == ROL_DEFENSIVO


def is_shot_possible(blackboard):
    """Comprueba si es posible realizar un tiro a portería.

    Args:
        blackboard: Pizarra con el estado del juego

    Returns:
        bool: True si es posible realizar un tiro
    """
    # Verificar distancia a portería
    player_pos = blackboard.player.get_position()
    goal_pos = blackboard.opponent_goal_pos

    # Distancia euclidiana a la portería
    distance_to_goal = np.linalg.norm(np.array(player_pos) - np.array(goal_pos))

    # Ángulo hacia la portería
    angle_to_goal = np.arctan2(goal_pos[1] - player_pos[1], goal_pos[0] - player_pos[0])
    player_angle = np.radians(blackboard.player.angle)
    angle_diff = abs((angle_to_goal - player_angle + np.pi) % (2 * np.pi) - np.pi)

    # Verificar si hay oponentes en la trayectoria
    shot_blocked = False
    for opponent in blackboard.opponents:
        opp_pos = opponent.get_position()

        # Vector desde el jugador hasta la portería
        to_goal = np.array(goal_pos) - np.array(player_pos)
        to_goal_normalized = to_goal / np.linalg.norm(to_goal)

        # Vector desde el jugador hasta el oponente
        to_opponent = np.array(opp_pos) - np.array(player_pos)

        # Proyectar 'to_opponent' sobre 'to_goal_normalized'
        projection = np.dot(to_opponent, to_goal_normalized)

        # Solo considerar oponentes que están entre el jugador y la portería
        if 0 < projection < np.linalg.norm(to_goal):
            # Calcular la distancia perpendicular del oponente a la línea de tiro
            perp_distance = np.linalg.norm(
                to_opponent - projection * to_goal_normalized
            )

            # Si el oponente está cerca de la línea de tiro, está bloqueando
            if perp_distance < 50:  # 50 es un umbral razonable
                shot_blocked = True
                break

    # Condiciones para un tiro posible
    # No hay restricción de distancia: shoot_to_goal maneja orientación internamente.
    # Esto evita caer en dribble_forward (movimiento lineal con pelota).
    return (
        not shot_blocked  # Sin oponentes bloqueando
    )


def is_pass_possible(blackboard):
    """Comprueba si es posible realizar un pase a un compañero.

    Args:
        blackboard: Pizarra con el estado del juego

    Returns:
        bool: True si es posible realizar un pase
    """
    # Este jugador debe tener la pelota
    if not blackboard.player.has_ball():
        return False

    # Buscar compañeros en posición de recibir pase
    for teammate in blackboard.team_players:
        # No pasar a uno mismo
        if teammate.id == blackboard.player.id:
            continue

        player_pos = blackboard.player.get_position()
        teammate_pos = teammate.get_position()

        # Verificar distancia (ni muy cerca ni muy lejos)
        pass_distance = np.linalg.norm(np.array(teammate_pos) - np.array(player_pos))
        if (pass_distance < blackboard.field.ratio_to_px(BT_PASS_MIN_RATIO) or
                pass_distance > blackboard.field.ratio_to_px(BT_PASS_MAX_RATIO)):
            continue

        # Verificar si hay oponentes que puedan interceptar
        pass_blocked = False
        for opponent in blackboard.opponents:
            opp_pos = opponent.get_position()

            # Vector desde el jugador hasta el compañero
            to_teammate = np.array(teammate_pos) - np.array(player_pos)
            to_teammate_normalized = to_teammate / np.linalg.norm(to_teammate)

            # Vector desde el jugador hasta el oponente
            to_opponent = np.array(opp_pos) - np.array(player_pos)

            # Proyectar 'to_opponent' sobre 'to_teammate_normalized'
            projection = np.dot(to_opponent, to_teammate_normalized)

            # Solo considerar oponentes que están entre el jugador y el compañero
            if 0 < projection < np.linalg.norm(to_teammate):
                # Calcular la distancia perpendicular del oponente a la línea de pase
                perp_distance = np.linalg.norm(
                    to_opponent - projection * to_teammate_normalized
                )

                # Si el oponente está cerca de la línea de pase, está bloqueando
                if perp_distance < 50:  # 50 es un umbral razonable
                    pass_blocked = True
                    break

        # Si encontramos al menos un compañero válido, podemos pasar
        if not pass_blocked:
            blackboard.pass_target = teammate  # Guardar el objetivo del pase
            return True

    return False


# ACCIONES ESPECÍFICAS PARA FÚTBOL


def _move_to_ball_start(blackboard):
    """Fase on_start de move_to_ball: emite comando de ir directo a la pelota UNA VEZ.

    Estrategia "capture first": ir directamente a la pelota sin posición de
    aproximación. La orientación al arco se hace DESPUÉS de capturar.
    """
    if not hasattr(blackboard, "command_manager"):
        return NodeStatus.FAILURE

    ball_pos = blackboard.ball.get_position()
    target_pos = (int(ball_pos[0]), int(ball_pos[1]))

    # Guardar target en blackboard para que on_running pueda consultarlo
    blackboard._move_to_ball_target = target_pos

    # Emitir comando: el guard en move_robot_to evitará sobreescritura si target es igual
    blackboard.command_manager.move_robot_to(blackboard.player.id, target_pos)
    return NodeStatus.RUNNING


def _move_to_ball_running(blackboard):
    """Fase on_running de move_to_ball: monitorea completación sin reemitir comandos.

    Llamada en cada tick mientras el nodo está RUNNING.
    Solo comprueba si el PID terminó el movimiento o si la pelota se alejó.
    """
    player_id = blackboard.player.id
    player_pos = blackboard.player.get_position()
    ball_pos = blackboard.ball.get_position()
    dist_to_ball = np.linalg.norm(ball_pos - player_pos)

    # Si el PID completó el movimiento (acción eliminada de actions_in_progress)
    if player_id not in blackboard.command_manager.actions_in_progress:
        # Umbral: max del ratio calibrado para simulación y el threshold real del PID + margen.
        # En cámara (640px): ratio→25px < ROBOT_POSITION_THRESHOLD(32px) → usa 38px.
        # En simulación (1500px): ratio→60px > 38px → usa 60px. Funciona en ambos.
        capture_range = max(
            blackboard.field.ratio_to_px(BT_CAPTURE_RANGE_RATIO),
            ROBOT_POSITION_THRESHOLD + 6
        )

        if dist_to_ball < capture_range:
            # Robot llegó tan cerca como el PID puede → pasar a capture_ball
            return NodeStatus.SUCCESS

        # La pelota se alejó del target → recalcular
        return _move_to_ball_start(blackboard)

    # Si la pelota se alejó mucho del target actual (pelota en movimiento), recalcular
    if hasattr(blackboard, '_move_to_ball_target'):
        target = np.array(blackboard._move_to_ball_target)
        dist_target_to_ball = np.linalg.norm(ball_pos - target)
        if dist_target_to_ball > 80:
            # La pelota se movió >80px del target → recalcular
            return _move_to_ball_start(blackboard)

    # Movimiento todavía en progreso → esperar
    return NodeStatus.RUNNING


def move_to_ball(blackboard):
    """Compatibilidad: wrapper stateless de move_to_ball para uso directo.

    Preferir create_move_to_ball_node() para uso en árboles de comportamiento,
    ya que StatefulActionNode evita reemitir comandos en cada tick.
    """
    return _move_to_ball_start(blackboard)


def create_move_to_ball_node():
    """Crea un nodo StatefulActionNode para move_to_ball.

    Usar este factory en lugar de ActionNode(move_to_ball, ...) para evitar
    que el BT reemita comandos RF en cada tick (problema de oscilación PID).
    """
    return StatefulActionNode(
        _move_to_ball_start,
        _move_to_ball_running,
        name="MoverHaciaPelota"
    )


# === ORIENT TO GOAL (capture first, orient second) ===

def _orient_to_goal_start(blackboard):
    """on_start: ordena rotación hacia el arco rival. Se llama UNA VEZ."""
    if not hasattr(blackboard, "command_manager"):
        return NodeStatus.FAILURE

    # Solo orientar si tenemos la pelota
    if not blackboard.player.has_ball():
        return NodeStatus.FAILURE

    player_pos = blackboard.player.get_position()
    goal_pos = blackboard.opponent_goal_pos

    # Calcular ángulo al arco rival (en grados, convención del BT)
    angle_to_goal = float(np.degrees(
        np.arctan2(goal_pos[1] - player_pos[1], goal_pos[0] - player_pos[0])
    ))

    current_angle = blackboard.player.angle
    angle_diff = abs((angle_to_goal - current_angle + 180) % 360 - 180)

    log.info("[orient_goal] Robot %d | actual=%.1f° | objetivo=%.1f° | diff=%.1f°",
             blackboard.player.id, current_angle, angle_to_goal, angle_diff)

    # Ya orientado
    if angle_diff <= 15:
        blackboard.last_action = "oriented_to_goal"
        log.info("[orient_goal] Ya orientado → SUCCESS")
        return NodeStatus.SUCCESS

    # Rotación > 60°: rotar en el lugar empujaría la pelota lejos del dribbler.
    # Mejor pasar directamente a dribble_forward/shoot que manejan la orientación
    # con movimiento (la corrección angular del PID se aplica mientras avanza).
    if angle_diff > 60:
        blackboard.last_action = "skip_orient_large_angle"
        log.info("[orient_goal] Rotación %.1f° > 60° — saltando orient, dribble manejará corrección", angle_diff)
        return NodeStatus.SUCCESS

    # Rotación pequeña (15°-60°): orientar en el lugar es seguro
    blackboard._orient_start_time = time.time()
    blackboard.command_manager.rotate_robot_to(blackboard.player.id, angle_to_goal)
    blackboard.last_action = "orienting_to_goal"
    log.info("[orient_goal] Rotando %.1f° hacia arco", angle_diff)
    return NodeStatus.RUNNING


def _orient_to_goal_running(blackboard):
    """on_running: monitorea rotación. Verifica timeout (2s) y pérdida de pelota."""
    ORIENT_TIMEOUT_S = 2.0

    # Si perdimos la pelota durante la rotación, abortar
    if not blackboard.player.has_ball():
        return NodeStatus.FAILURE

    # Verificar timeout
    start_time = getattr(blackboard, '_orient_start_time', None)
    if start_time is not None and (time.time() - start_time) > ORIENT_TIMEOUT_S:
        return NodeStatus.FAILURE

    # Verificar si ya estamos orientados
    player_pos = blackboard.player.get_position()
    goal_pos = blackboard.opponent_goal_pos
    angle_to_goal = float(np.degrees(
        np.arctan2(goal_pos[1] - player_pos[1], goal_pos[0] - player_pos[0])
    ))

    current_angle = blackboard.player.angle
    angle_diff = abs((angle_to_goal - current_angle + 180) % 360 - 180)

    if angle_diff <= 15:
        blackboard.last_action = "oriented_to_goal"
        return NodeStatus.SUCCESS

    # Si el PID completó la rotación pero no estamos alineados, reemitir
    player_id = blackboard.player.id
    if player_id not in blackboard.command_manager.actions_in_progress:
        blackboard.command_manager.rotate_robot_to(player_id, angle_to_goal)

    return NodeStatus.RUNNING


def create_orient_to_goal_node():
    """Crea un nodo StatefulActionNode para orientarse al arco rival.

    Se usa después de capture_ball: el robot rota en sitio para encarar
    el arco contrario antes de disparar o avanzar.
    """
    return StatefulActionNode(
        _orient_to_goal_start,
        _orient_to_goal_running,
        name="OrientarAlArco"
    )


def capture_ball(blackboard):
    """Capturar la pelota activando el motor de captura.

    Versión mejorada que activa físicamente el dribbler/motor.

    Args:
        blackboard: Pizarra con el estado del juego

    Returns:
        NodeStatus: Estado de la ejecución (SUCCESS, RUNNING, FAILURE)
    """
    # Verificar que existe el gestor de comandos
    if not hasattr(blackboard, "command_manager"):
        blackboard.logger.warning(
            "No command manager found in blackboard. Action may not work properly."
        )
        return NodeStatus.FAILURE

    player_pos = blackboard.player.get_position()
    ball_pos = blackboard.ball.get_position()

    # Verificar si ya tenemos la pelota
    if blackboard.player.has_ball():
        return NodeStatus.SUCCESS

    # Calcular distancia a la pelota
    distance_to_ball = blackboard.player.distance_to_ball(blackboard.ball)

    log.info("[capture_ball] Robot %d | dist=%.1fpx | activa=%.0fpx confirma=%.0fpx | has_ball=%s",
             blackboard.player.id, distance_to_ball,
             CAPTURE_ACTIVATE_DISTANCE_PX, CAPTURE_CONFIRM_DISTANCE_PX,
             blackboard.player.has_ball())

    if distance_to_ball < CAPTURE_ACTIVATE_DISTANCE_PX:
        # PASO 1: Orientarse hacia la pelota (solo ajustes pequeños ≤10°)
        angle_to_ball = np.degrees(
            np.arctan2(ball_pos[1] - player_pos[1], ball_pos[0] - player_pos[0])
        )
        current_angle = blackboard.player.angle
        angle_diff = abs((angle_to_ball - current_angle + 180) % 360 - 180)

        if angle_diff > 10:
            log.info("[capture_ball] Orientando | diff=%.1f°", angle_diff)
            blackboard.command_manager.rotate_robot_to(
                blackboard.player.id, angle_to_ball
            )
            return NodeStatus.RUNNING

        # PASO 2: ACTIVAR DRIBBLER (pre-spin antes del contacto)
        log.info("[capture_ball] DRIBBLER ON | dist=%.1fpx", distance_to_ball)
        blackboard.command_manager.capture_ball(blackboard.player.id)

        # PASO 3: CREEP FORWARD — target OVERSHOOT_PX más allá de la pelota.
        # El PID para a ROBOT_POSITION_THRESHOLD antes del target, colocando el
        # dribbler físicamente encima de la pelota.
        # Geometría: robot para a (OVERSHOOT - ROBOT_THRESHOLD) px del centro pelota.
        robot_to_ball = ball_pos - player_pos
        dist = np.linalg.norm(robot_to_ball)
        if dist > 0:
            forward = robot_to_ball / dist
            overshoot = ball_pos + forward * CAPTURE_OVERSHOOT_PX
            overshoot_target = (int(overshoot[0]), int(overshoot[1]))
            blackboard.command_manager.move_robot_to(
                blackboard.player.id, overshoot_target
            )
            log.info("[capture_ball] CREEP → overshoot(%d,%d) | distancia_final≈%dpx",
                     overshoot_target[0], overshoot_target[1],
                     max(0, CAPTURE_OVERSHOOT_PX - ROBOT_POSITION_THRESHOLD))

        # PASO 4: Confirmar captura (robot dentro del dribbler)
        if distance_to_ball < CAPTURE_CONFIRM_DISTANCE_PX:
            blackboard.player._has_ball = True
            blackboard.last_action = "ball_captured_with_motor"
            log.info("[capture_ball] CAPTURA CONFIRMADA | dist=%.1fpx", distance_to_ball)
            return NodeStatus.SUCCESS

        return NodeStatus.RUNNING

    log.info("[capture_ball] Fuera de rango (%.1fpx > %.0fpx)",
             distance_to_ball, CAPTURE_ACTIVATE_DISTANCE_PX)
    return NodeStatus.RUNNING


def dribble_forward(blackboard):
    """Avanzar con la pelota hacia la portería rival usando waypoints estratégicos.

    Args:
        blackboard: Pizarra con el estado del juego

    Returns:
        NodeStatus: Estado de la ejecución (SUCCESS, RUNNING, FAILURE)
    """
    if not blackboard.player.has_ball():
        return NodeStatus.FAILURE

    # Verificar que existe el gestor de comandos
    if not hasattr(blackboard, "command_manager"):
        blackboard.logger.warning(
            "No command manager found in blackboard. Action may not work properly."
        )
        return NodeStatus.FAILURE

    player_pos = blackboard.player.get_position()
    ball_pos = blackboard.ball.get_position()
    goal_pos = blackboard.opponent_goal_pos

    # Calcular waypoints para el dribbling
    waypoints = calculate_dribbling_path_positions(
        ball_pos, goal_pos, num_waypoints=2,
        spacing=blackboard.field.ratio_to_px(BT_DRIBBLE_SPACING_RATIO),
        field=blackboard.field,
    )

    if waypoints:
        # Usar el primer waypoint como objetivo inmediato
        target_pos = waypoints[0]

        # Registrar para debugging
        if hasattr(blackboard, "tracer") and hasattr(
            blackboard.tracer, "set_planned_movement"
        ):
            blackboard.tracer.set_planned_movement(
                blackboard.player.id,
                target_pos,
                "dribble_forward",
                {"waypoints": waypoints, "goal_direction": goal_pos},
            )

        # Moverse con la pelota
        blackboard.command_manager.move_with_ball(
            blackboard.player.id, target_pos, blackboard.ball, speed_factor=0.7
        )

        # Verificar progreso
        distance_to_goal = np.linalg.norm(np.array(player_pos) - np.array(goal_pos))
        if distance_to_goal < blackboard.field.ratio_to_px(BT_DRIBBLE_GOAL_RATIO):
            return NodeStatus.SUCCESS

        return NodeStatus.RUNNING

    return NodeStatus.FAILURE


def shoot_to_goal(blackboard):
    """Disparar a portería desde la posición actual.

    Estrategia sin dribble: el robot captura, orienta y dispara sin movimiento
    lineal con la pelota. Solo ajusta orientación (rotación en sitio) y patea.

    Args:
        blackboard: Pizarra con el estado del juego

    Returns:
        NodeStatus: Estado de la ejecución (SUCCESS, RUNNING, FAILURE)
    """
    if not blackboard.player.has_ball():
        return NodeStatus.FAILURE

    if not hasattr(blackboard, "command_manager"):
        return NodeStatus.FAILURE

    player_pos = blackboard.player.get_position()
    goal_pos = blackboard.opponent_goal_pos

    # Apuntar al centro del arco (sin offset aleatorio para evitar oscilación)
    target = [goal_pos[0], goal_pos[1]]

    # Verificar orientación hacia la portería
    angle_to_goal = np.degrees(
        np.arctan2(target[1] - player_pos[1], target[0] - player_pos[0])
    )

    current_angle = blackboard.player.angle
    angle_diff = abs((angle_to_goal - current_angle + 180) % 360 - 180)

    if angle_diff > 10:
        # Reducir potencia dribbler durante rotación → protege regulador de tensión
        rf = blackboard.command_manager.rf_controller
        if rf:
            fw_id = blackboard.player.id + 1
            rf.set_dribbler(fw_id, DRIBBLER_HOLD_POWER)
        # Orientarse antes de disparar (rotación en sitio, sin movimiento lineal)
        blackboard.command_manager.rotate_robot_to(blackboard.player.id, angle_to_goal)
        log.info("[shoot] Robot %d | orientando | actual=%.1f° objetivo=%.1f° diff=%.1f°",
                 blackboard.player.id, current_angle, angle_to_goal, angle_diff)
        return NodeStatus.RUNNING

    # Apagar dribbler antes de disparar
    rf = blackboard.command_manager.rf_controller
    if rf:
        fw_id = blackboard.player.id + 1
        rf.set_dribbler(fw_id, 0)

    # Disparar
    blackboard.command_manager.kick_ball(
        blackboard.player.id, target, blackboard.ball, power=0.9
    )

    blackboard.last_action = "shoot_to_goal"
    log.info("[shoot] Robot %d | DISPARO al arco (%.0f, %.0f)",
             blackboard.player.id, target[0], target[1])

    return NodeStatus.SUCCESS


def pass_to_teammate(blackboard):
    """Realizar un pase a un compañero desde una posición estratégica.

    Args:
        blackboard: Pizarra con el estado del juego

    Returns:
        NodeStatus: Estado de la ejecución (SUCCESS, RUNNING, FAILURE)
    """
    if not blackboard.player.has_ball() or not hasattr(blackboard, "pass_target"):
        return NodeStatus.FAILURE

    # Verificar que existe el gestor de comandos
    if not hasattr(blackboard, "command_manager"):
        blackboard.logger.warning(
            "No command manager found in blackboard. Action may not work properly."
        )
        return NodeStatus.FAILURE

    player_pos = blackboard.player.get_position()
    ball_pos = blackboard.ball.get_position()
    teammate_pos = blackboard.pass_target.get_position()

    # Calcular posición óptima para el pase
    pass_pos = calculate_pass_position(
        player_pos, ball_pos, teammate_pos, approach_distance=40,
        field=blackboard.field,
    )

    # Verificar si estamos en buena posición para pasar
    distance_to_pass_pos = np.linalg.norm(np.array(player_pos) - np.array(pass_pos))

    if distance_to_pass_pos > blackboard.field.ratio_to_px(BT_APPROACH_RATIO):
        # Moverse a mejor posición de pase
        blackboard.command_manager.move_with_ball(
            blackboard.player.id, pass_pos, blackboard.ball, speed_factor=0.6
        )
        return NodeStatus.RUNNING

    # Calcular punto de pase (liderar al compañero)
    teammate_angle = np.radians(blackboard.pass_target.angle)
    lead_distance = 80
    lead_pos = [
        teammate_pos[0] + lead_distance * np.cos(teammate_angle),
        teammate_pos[1] + lead_distance * np.sin(teammate_angle),
    ]

    # Verificar orientación hacia el pase
    angle_to_pass = np.degrees(
        np.arctan2(lead_pos[1] - player_pos[1], lead_pos[0] - player_pos[0])
    )

    current_angle = blackboard.player.angle
    angle_diff = abs((angle_to_pass - current_angle + 180) % 360 - 180)

    if angle_diff > 15:
        # Orientarse mejor antes de pasar
        blackboard.command_manager.rotate_robot_to(blackboard.player.id, angle_to_pass)
        return NodeStatus.RUNNING

    # Realizar el pase
    blackboard.command_manager.kick_ball(
        blackboard.player.id, lead_pos, blackboard.ball, power=0.7
    )

    # Registrar la acción
    blackboard.last_action = "pass_to_teammate"

    return NodeStatus.SUCCESS


def intercept_ball(blackboard):
    """Interceptar la trayectoria de la pelota usando predicción mejorada.

    Args:
        blackboard: Pizarra con el estado del juego

    Returns:
        NodeStatus: Estado de la ejecución (SUCCESS, RUNNING, FAILURE)
    """
    # Verificar que existe el gestor de comandos
    if not hasattr(blackboard, "command_manager"):
        blackboard.logger.warning(
            "No command manager found in blackboard. Action may not work properly."
        )
        return NodeStatus.FAILURE

    player_pos = blackboard.player.get_position()
    ball_pos = blackboard.ball.get_position()

    # Obtener velocidad de la pelota si está disponible
    ball_velocity = [0, 0]
    if hasattr(blackboard.ball, "dx") and hasattr(blackboard.ball, "dy"):
        ball_velocity = [blackboard.ball.dx, blackboard.ball.dy]
        ball_speed = np.linalg.norm(ball_velocity)

        # Ajustar tiempo de predicción basado en la velocidad
        if ball_speed > 5:
            prediction_time = 1.5  # Más tiempo para pelotas rápidas
        elif ball_speed > 2:
            prediction_time = 1.0
        else:
            prediction_time = 0.5  # Menos tiempo para pelotas lentas
    else:
        prediction_time = 0.8

    # Calcular posición de intercepción
    target_pos = calculate_interception_position(
        player_pos, ball_pos, ball_velocity,
        blackboard.opponent_goal_pos, blackboard.team,
        prediction_time, approach_distance=40,
        field=blackboard.field,
    )

    # Registrar para debugging
    if hasattr(blackboard, "tracer") and hasattr(
        blackboard.tracer, "set_planned_movement"
    ):
        blackboard.tracer.set_planned_movement(
            blackboard.player.id,
            target_pos,
            "intercept_ball",
            {
                "ball_velocity": ball_velocity,
                "prediction_time": prediction_time,
                "predicted_ball_pos": tuple(
                    np.array(ball_pos) + np.array(ball_velocity) * prediction_time
                ),
            },
        )

    # Ordenar movimiento con velocidad alta
    blackboard.command_manager.move_robot_to(
        blackboard.player.id, target_pos, speed_factor=1.5
    )

    # Verificar si estamos cerca de la pelota
    distance = blackboard.player.distance_to_ball(blackboard.ball)
    if distance < blackboard.field.ratio_to_px(BT_INTERCEPT_RATIO):
        return NodeStatus.SUCCESS

    return NodeStatus.RUNNING


def block_opponent(blackboard):
    """Bloquear al oponente más cercano a nuestra portería.

    Args:
        blackboard: Pizarra con el estado del juego

    Returns:
        NodeStatus: Estado de la ejecución (SUCCESS, RUNNING, FAILURE)
    """
    # Verificar que existe el gestor de comandos
    if not hasattr(blackboard, "command_manager"):
        blackboard.logger.warning(
            "No command manager found in blackboard. Action may not work properly."
        )
        return NodeStatus.FAILURE

    # Encontrar el oponente más peligroso (más cercano a nuestra portería)
    most_dangerous = None
    min_distance = float("inf")

    for opponent in blackboard.opponents:
        opp_pos = opponent.get_position()
        distance_to_goal = np.linalg.norm(
            np.array(opp_pos) - np.array(blackboard.own_goal_pos)
        )

        if distance_to_goal < min_distance:
            min_distance = distance_to_goal
            most_dangerous = opponent

    if not most_dangerous:
        return NodeStatus.FAILURE

    # Calcular posición para bloquear (entre el oponente y nuestra portería)
    opp_pos = most_dangerous.get_position()
    goal_pos = blackboard.own_goal_pos

    # Vector desde oponente a portería
    to_goal = np.array(goal_pos) - np.array(opp_pos)
    distance = np.linalg.norm(to_goal)

    if distance > 0:
        # Normalizar y escalar
        to_goal = to_goal / distance * min(100, distance / 2)

    # Posición de bloqueo
    block_pos = tuple(opp_pos + to_goal)

    # Ordenar movimiento
    blackboard.command_manager.move_robot_to(
        blackboard.player.id, block_pos, speed_factor=1.2
    )

    # Registrar la acción
    blackboard.last_action = "block_opponent"

    # Este es un comportamiento continuo, siempre está en ejecución
    return NodeStatus.RUNNING


def move_to_defensive_position(blackboard):
    """Mover el jugador a una posición defensiva estratégica.

    Elige entre las posiciones defensivas predefinidas la más cercana a la
    posición ACTUAL del defensor (no a la pelota), para minimizar movimiento
    y evitar cruzar el camino del atacante.

    Si el atacante está activamente yendo a la pelota, el defensor espera
    en su posición actual para no interferir.

    Args:
        blackboard: Pizarra con el estado del juego

    Returns:
        NodeStatus: Estado de la ejecución (SUCCESS, RUNNING, FAILURE)
    """
    # Verificar que existe el gestor de comandos
    if not hasattr(blackboard, "command_manager"):
        blackboard.logger.warning(
            "No command manager found in blackboard. Action may not work properly."
        )
        return NodeStatus.FAILURE

    ball_pos = blackboard.ball.get_position()
    player_pos = blackboard.player.get_position()

    # PRIORIDAD 1: Verificar si el atacante está yendo activamente a la pelota
    # Si es así, el defensor NO se mueve para no obstruir
    teammates = blackboard.team_players
    for teammate in teammates:
        if teammate.id != blackboard.player.id and teammate.rol == ROL_ATACANTE:
            teammate_pos = teammate.get_position()

            # Calcular distancia del atacante a la pelota
            attacker_distance_to_ball = np.linalg.norm(
                np.array(teammate_pos) - np.array(ball_pos)
            )

            # Si el atacante está cerca de la pelota, esperar
            if attacker_distance_to_ball < blackboard.field.ratio_to_px(BT_DEFENDER_WAIT_RATIO) * 2:
                # No moverse, quedarse en posición actual
                # El defensor esperará hasta que el atacante termine
                return NodeStatus.RUNNING

    # PRIORIDAD 2: Si el atacante NO está activo, elegir posición defensiva
    # Elegir la posición defensiva MÁS CERCANA a la posición ACTUAL del defensor
    # (NO a la pelota, para minimizar movimiento)
    defensive_pos = None
    min_distance = float("inf")

    for pos in blackboard.defensive_positions:
        distance = np.linalg.norm(np.array(pos) - np.array(player_pos))
        if distance < min_distance:
            min_distance = distance
            defensive_pos = pos

    if not defensive_pos:
        defensive_pos = blackboard.defensive_positions[0]

    # Ordenar movimiento
    blackboard.command_manager.move_robot_to(blackboard.player.id, defensive_pos)

    # Verificar si hemos llegado
    distance = np.linalg.norm(np.array(player_pos) - np.array(defensive_pos))

    if distance < blackboard.field.ratio_to_px(BT_DEFENSIVE_ARRIVAL_RATIO):
        return NodeStatus.SUCCESS

    # Continuar ejecutando la acción
    return NodeStatus.RUNNING


def move_to_support_position(blackboard):
    """Mover el jugador a una posición de apoyo ofensivo.

    Args:
        blackboard: Pizarra con el estado del juego

    Returns:
        NodeStatus: Estado de la ejecución (SUCCESS, RUNNING, FAILURE)
    """
    # Verificar que existe el gestor de comandos
    if not hasattr(blackboard, "command_manager"):
        blackboard.logger.warning(
            "No command manager found in blackboard. Action may not work properly."
        )
        return NodeStatus.FAILURE

    # Encontrar posición de apoyo estratégica
    ball_pos = blackboard.ball.get_position()
    goal_pos = blackboard.opponent_goal_pos

    # Vector desde la pelota hacia la portería
    to_goal = np.array(goal_pos) - np.array(ball_pos)
    distance = np.linalg.norm(to_goal)

    if distance > 0:
        # Normalizar y rotar 45 grados
        to_goal = to_goal / distance
        angle = np.arctan2(to_goal[1], to_goal[0])
        angle += np.pi / 4  # Rotar 45 grados

        # Vector rotado
        rotated = [np.cos(angle), np.sin(angle)]

        # Posición de apoyo (proporción del campo)
        support_pos = ball_pos + np.array(rotated) * blackboard.field.ratio_to_px(BT_SUPPORT_DISTANCE_RATIO)
    else:
        # Fallback si la pelota está en la portería
        support_pos = [(ball_pos[0] + goal_pos[0]) / 2, (ball_pos[1] + goal_pos[1]) / 2]

    # Asegurar que la posición está dentro del campo
    support_pos = list(blackboard.field.clamp(support_pos))

    # Ordenar movimiento
    blackboard.command_manager.move_robot_to(blackboard.player.id, tuple(support_pos))

    # Registrar la acción
    blackboard.last_action = "move_to_support_position"

    # Este es un comportamiento continuo, siempre está en ejecución
    return NodeStatus.RUNNING


def position_to_defend_goal(blackboard):
    """Posicionarse para defender la portería.

    Args:
        blackboard: Pizarra con el estado del juego

    Returns:
        NodeStatus: Estado de la ejecución (SUCCESS, RUNNING, FAILURE)
    """
    # Verificar que existe el gestor de comandos
    if not hasattr(blackboard, "command_manager"):
        blackboard.logger.warning(
            "No command manager found in blackboard. Action may not work properly."
        )
        return NodeStatus.FAILURE

    ball_pos = blackboard.ball.get_position()
    goal_pos = blackboard.own_goal_pos

    # Vector desde la pelota hacia nuestra portería
    to_ball = np.array(ball_pos) - np.array(goal_pos)
    distance = np.linalg.norm(to_ball)

    if distance > 0:
        # Normalizar y escalar para posicionarse entre la pelota y la portería
        to_ball = to_ball / distance
        defend_distance = min(
            distance * 0.6, 200
        )  # No alejarse demasiado de la portería

        # Posición de defensa
        defend_pos = tuple(np.array(goal_pos) + to_ball * defend_distance)
    else:
        # Fallback
        defend_pos = tuple(goal_pos)

    # Ordenar movimiento
    blackboard.command_manager.move_robot_to(blackboard.player.id, defend_pos)

    # Registrar la acción
    blackboard.last_action = "position_to_defend_goal"

    # Este es un comportamiento continuo, siempre está en ejecución
    return NodeStatus.RUNNING

def is_closest_attacker_to_ball(blackboard):
    """Comprueba si este jugador es el atacante del equipo.

    Nota: Como solo hay 1 atacante por equipo, esta condición simplemente
    verifica que el jugador tenga el rol de atacante.
    """
    return blackboard.player.rol == ROL_ATACANTE


def is_closest_defender_to_ball(blackboard):
    """Comprueba si este jugador es el defensor del equipo.

    Nota: Como solo hay 1 defensor por equipo, esta condición simplemente
    verifica que el jugador tenga el rol de defensor.
    """
    return blackboard.player.rol == ROL_DEFENSIVO


def is_no_attacker_closer_to_ball(blackboard):
    """Comprueba si no hay ningún atacante del equipo más cerca de la pelota.

    El defensor solo irá a la pelota si está al menos 200px más cerca que
    todos los atacantes del equipo, dando prioridad clara al rol atacante.
    """
    my_distance = blackboard.player.distance_to_ball(blackboard.ball)

    # Verificar atacantes del equipo
    for teammate in blackboard.team_players:
        if teammate.rol == ROL_ATACANTE:
            teammate_distance = teammate.distance_to_ball(blackboard.ball)
            # Si el atacante está dentro del margen de prioridad, darle prioridad
            if teammate_distance < my_distance + blackboard.field.ratio_to_px(BT_ATTACKER_PRIORITY_MARGIN_RATIO):
                return False

    return True


# =============================================================================
# ATAQUE SIN DRIBBLER: posicionar detrás → contactar → disparar solenoide
# =============================================================================

def _path_near_ball(p1, p2, ball, threshold=35):
    """Retorna True si el segmento p1→p2 pasa a menos de threshold px de ball."""
    p1 = np.array(p1, dtype=float)
    p2 = np.array(p2, dtype=float)
    b  = np.array(ball, dtype=float)
    d  = p2 - p1
    len_sq = float(np.dot(d, d))
    if len_sq < 1.0:
        return False
    t = float(np.dot(b - p1, d) / len_sq)
    t = max(0.0, min(1.0, t))
    closest = p1 + t * d
    return bool(np.linalg.norm(closest - b) < threshold)


def _clear_advance_state(blackboard, player_id):
    """Limpia estado del avance al contacto: para motores, desactiva creep PID."""
    blackboard.command_manager.actions_in_progress.pop(player_id, None)
    controller = blackboard.command_manager.controllers.get(player_id)
    if controller:
        controller.max_linear_pwm_override = None
    rf = blackboard.command_manager.rf_controller
    if rf:
        rf.set_motors(player_id + 1, 0, 0)


def _move_behind_ball_start(blackboard):
    """on_start: calcula la posición detrás de la pelota y emite el movimiento.

    Posición objetivo: BEHIND_BALL_APPROACH_PX detrás del centro de la pelota,
    en la línea arco-rival → pelota (lado opuesto al arco).
    Si el camino directo pasa por la pelota, calcula un desvío lateral primero.
    """
    if not hasattr(blackboard, "command_manager"):
        return NodeStatus.FAILURE

    # Limpiar override de velocidad residual de Phase 2 previa
    controller = blackboard.command_manager.controllers.get(blackboard.player.id)
    if controller and controller.max_linear_pwm_override is not None:
        controller.max_linear_pwm_override = None

    ball_pos   = np.array(blackboard.ball.get_position(), dtype=float)
    goal_pos   = np.array(blackboard.opponent_goal_pos, dtype=float)
    player_pos = np.array(blackboard.player.get_position(), dtype=float)

    # Fast path: si estamos cerca de la pelota Y la pelota está entre el robot
    # y el arco Y el robot está mirando hacia la pelota → saltar Phase 1.
    # Caso típico: post-kick fallido, la pelota sigue al frente del robot.
    # Se requieren 3 condiciones:
    #   1. Proximidad: dist < BEHIND_BALL_APPROACH_PX
    #   2. Geometría: pelota en dirección al arco (alignment < 30°)
    #   3. Heading:   robot mirando hacia la pelota (err < 2x tolerancia)
    dist_to_ball = float(np.linalg.norm(ball_pos - player_pos))
    if dist_to_ball <= BEHIND_BALL_APPROACH_PX:
        ang_to_ball = float(np.degrees(np.arctan2(
            ball_pos[1] - player_pos[1], ball_pos[0] - player_pos[0])))
        ang_to_goal = float(np.degrees(np.arctan2(
            goal_pos[1] - player_pos[1], goal_pos[0] - player_pos[0])))
        alignment = abs((ang_to_ball - ang_to_goal + 180) % 360 - 180)
        err_heading = abs((ang_to_ball - blackboard.player.angle + 180) % 360 - 180)
        if alignment < 30 and err_heading < BEHIND_BALL_ALIGN_TOLERANCE_DEG * 2:
            blackboard.last_action = "behind_ball_ready"
            log.info("[behind_ball] Robot %d | pelota al frente (%.0fpx, alin=%.1f°, "
                     "heading=%.1f°) → avance directo",
                     blackboard.player.id, dist_to_ball, alignment, err_heading)
            return NodeStatus.SUCCESS

    # Vector del arco hacia la pelota (dirección "detrás de la pelota")
    goal_to_ball = ball_pos - goal_pos
    dist_gtb = np.linalg.norm(goal_to_ball)
    if dist_gtb < 1.0:
        return NodeStatus.FAILURE

    unit_behind = goal_to_ball / dist_gtb  # apunta desde arco → pelota (y más allá)

    # Posición ideal: BEHIND_BALL_APPROACH_PX más allá de la pelota, lejos del arco
    behind_pos = ball_pos + unit_behind * BEHIND_BALL_APPROACH_PX
    behind_pos = np.array(blackboard.field.clamp(behind_pos.astype(int)), dtype=float)

    # Guardar estado para on_running
    blackboard._behind_ball_pos = behind_pos.copy()
    blackboard._behind_ball_ref = ball_pos.copy()

    # ¿El camino directo choca con la pelota?
    if _path_near_ball(player_pos, behind_pos, ball_pos):
        # Calcular desvío lateral: perpendicular al eje arco-pelota
        ball_to_goal_unit = -unit_behind  # apunta de pelota → arco
        perp = np.array([-ball_to_goal_unit[1], ball_to_goal_unit[0]])

        # Elegir el lado donde ya está el robot (minimiza movimiento)
        side = float(np.sign(np.dot(player_pos - ball_pos, perp)))
        if side == 0.0:
            side = 1.0

        lateral = ball_pos + perp * side * BEHIND_BALL_LATERAL_OFFSET_PX
        lateral = np.array(blackboard.field.clamp(lateral.astype(int)), dtype=float)

        blackboard._behind_ball_detour = lateral.copy()
        blackboard._behind_ball_phase  = 'detour'
        target = (int(lateral[0]), int(lateral[1]))
        log.info("[behind_ball] Robot %d | desvío lateral → (%d,%d)",
                 blackboard.player.id, *target)
    else:
        blackboard._behind_ball_detour = None
        blackboard._behind_ball_phase  = 'direct'
        target = (int(behind_pos[0]), int(behind_pos[1]))
        log.info("[behind_ball] Robot %d | camino directo → (%d,%d)",
                 blackboard.player.id, *target)

    blackboard.command_manager.move_robot_to(blackboard.player.id, target)
    blackboard.last_action = "move_behind_ball"
    return NodeStatus.RUNNING


def _move_behind_ball_running(blackboard):
    """on_running: monitorea el avance hacia la posición detrás de la pelota.

    Fases:
      'detour'  → espera llegar al punto lateral, luego cambia a 'direct'.
      'direct'  → espera que el PID complete, luego verifica alineación angular.
    Recalcula si la pelota se movió > 60 px del punto de referencia.
    """
    player_id  = blackboard.player.id
    player_pos = np.array(blackboard.player.get_position(), dtype=float)
    ball_pos   = np.array(blackboard.ball.get_position(), dtype=float)
    phase      = getattr(blackboard, '_behind_ball_phase', 'direct')

    # Recalcular si la pelota se movió demasiado
    if np.linalg.norm(ball_pos - blackboard._behind_ball_ref) > 60:
        log.info("[behind_ball] Robot %d | pelota movida, recalculando", player_id)
        return _move_behind_ball_start(blackboard)

    in_progress = player_id in blackboard.command_manager.actions_in_progress

    if phase == 'detour':
        detour = blackboard._behind_ball_detour
        dist_detour = np.linalg.norm(player_pos - detour)
        if not in_progress or dist_detour < ROBOT_POSITION_THRESHOLD:
            # Llegamos al punto lateral → continuar hacia behind_pos
            blackboard._behind_ball_phase = 'direct'
            bp = blackboard._behind_ball_pos
            target = (int(bp[0]), int(bp[1]))
            blackboard.command_manager.move_robot_to(player_id, target)
            log.info("[behind_ball] Robot %d | desvío OK, yendo a behind_pos (%d,%d)",
                     player_id, *target)
        return NodeStatus.RUNNING

    # phase == 'direct'
    if in_progress:
        return NodeStatus.RUNNING

    # PID completado → verificar alineación con la pelota
    angle_to_ball = float(np.degrees(
        np.arctan2(ball_pos[1] - player_pos[1], ball_pos[0] - player_pos[0])
    ))
    current_angle = blackboard.player.angle
    angle_diff = abs((angle_to_ball - current_angle + 180) % 360 - 180)

    if angle_diff <= BEHIND_BALL_ALIGN_TOLERANCE_DEG:
        blackboard.last_action = "behind_ball_ready"
        log.info("[behind_ball] Robot %d | listo | dist_ball=%.0fpx | align=%.1f°",
                 player_id,
                 np.linalg.norm(ball_pos - player_pos),
                 angle_diff)
        return NodeStatus.SUCCESS

    # Rotar para alinearse con la pelota
    log.info("[behind_ball] Robot %d | ajustando ángulo | diff=%.1f°", player_id, angle_diff)
    blackboard.command_manager.rotate_robot_to(player_id, angle_to_ball)
    return NodeStatus.RUNNING


def create_move_behind_ball_node():
    """Crea el nodo stateful que posiciona al robot detrás de la pelota."""
    return StatefulActionNode(
        _move_behind_ball_start,
        _move_behind_ball_running,
        name="PosicionarDetrásPelota"
    )


def _compute_overshoot_target(ball_pos, goal_pos, overshoot_px):
    """Calcula target de overshoot: punto más allá de la pelota hacia el arco.

    El PID persigue este punto, que está PAST la pelota en dirección al arco.
    El robot físicamente se detiene al hacer contacto con la pelota antes de
    alcanzar el overshoot, garantizando que el PID no frene prematuramente.
    """
    ball_to_goal = goal_pos - ball_pos
    dist_bg = float(np.linalg.norm(ball_to_goal))
    if dist_bg > 0:
        unit_to_goal = ball_to_goal / dist_bg
    else:
        unit_to_goal = np.array([1, 0], dtype=float)
    return ball_pos + unit_to_goal * overshoot_px


def _advance_to_contact_start(blackboard):
    """on_start: inicia acercamiento con PID lento hacia la pelota.

    El robot ya está alineado (llegó desde detrás de la pelota).
    Usa move_robot_to con max_linear_pwm_override para mantener corrección
    angular activa a baja velocidad, en vez de PWM fijo sin steering.
    El target es un punto de overshoot (más allá de la pelota hacia el arco)
    para evitar que el PID se detenga a ROBOT_POSITION_THRESHOLD del target
    antes de hacer contacto.
    """
    if not hasattr(blackboard, "command_manager"):
        return NodeStatus.FAILURE

    player_id  = blackboard.player.id
    player_pos = np.array(blackboard.player.get_position(), dtype=float)
    ball_pos   = np.array(blackboard.ball.get_position(), dtype=float)
    goal_pos   = np.array(blackboard.opponent_goal_pos, dtype=float)
    dist       = float(np.linalg.norm(ball_pos - player_pos))

    # Reiniciar estado de contacto
    blackboard._contact_made         = False
    blackboard._contact_settle_start = None

    # ¿Ya en contacto desde el inicio?
    # Requiere proximidad Y alineación: si el robot está cerca pero no mira
    # la pelota, no es contacto real (ej: post-kick con robot girado).
    if dist < CAPTURE_CONFIRM_DISTANCE_PX:
        ang_to_ball = float(np.degrees(np.arctan2(
            ball_pos[1] - player_pos[1], ball_pos[0] - player_pos[0])))
        err_heading = abs((ang_to_ball - blackboard.player.angle + 180) % 360 - 180)
        if err_heading <= BEHIND_BALL_ALIGN_TOLERANCE_DEG * 2:
            blackboard._contact_made         = True
            blackboard._contact_settle_start = time.time()
            blackboard.player._has_ball      = True
            blackboard.last_action           = "settling_contact"
            log.info("[advance_contact] Robot %d | contacto inmediato | dist=%.1fpx | heading=%.1f°",
                     player_id, dist, err_heading)
            return NodeStatus.RUNNING
        else:
            log.info("[advance_contact] Robot %d | cerca (%.1fpx) pero desalineado "
                     "(heading=%.1f° > %.1f°) → FAILURE",
                     player_id, dist, err_heading,
                     BEHIND_BALL_ALIGN_TOLERANCE_DEG * 2)
            return NodeStatus.FAILURE

    # Calcular target de overshoot (más allá de la pelota hacia el arco)
    overshoot = _compute_overshoot_target(ball_pos, goal_pos, CAPTURE_OVERSHOOT_PX)
    target = (int(overshoot[0]), int(overshoot[1]))

    # Activar PID con velocidad lineal limitada (creep mode con steering)
    controller = blackboard.command_manager.controllers.get(player_id)
    if controller:
        controller.max_linear_pwm_override = CAPTURE_CREEP_SPEED_PWM

    blackboard.command_manager.move_robot_to(player_id, target)
    blackboard.last_action = "advancing_to_contact"
    log.info("[advance_contact] Robot %d | PID creep iniciado | dist=%.1fpx "
             "| overshoot=(%d,%d) | creep_pwm=%d",
             player_id, dist, *target, CAPTURE_CREEP_SPEED_PWM)
    return NodeStatus.RUNNING


def _advance_to_contact_running(blackboard):
    """on_running: monitorea contacto, asentamiento y posible escape de la pelota.

    Usa PID con velocidad lineal limitada (max_linear_pwm_override) en vez de
    PWM fijo, manteniendo corrección angular activa durante el avance.
    Recalcula el overshoot target cada tick para seguir la pelota si se mueve.

    Fases:
      Acercamiento: PID activo hasta que dist < CAPTURE_CONFIRM_DISTANCE_PX.
        - Si la pelota escapa (dist > BEHIND_BALL_APPROACH_PX * ADVANCE_ESCAPE_FACTOR) → FAILURE.
      Asentamiento: robot detenido, espera CONTACT_SETTLE_TIME_S.
        - Si la pelota escapa (dist > CAPTURE_CONFIRM_DISTANCE_PX * SETTLE_ESCAPE_FACTOR) → FAILURE.
        - Tras el tiempo de espera → SUCCESS y se dispara el solenoide.
    """
    player_id  = blackboard.player.id
    player_pos = np.array(blackboard.player.get_position(), dtype=float)
    ball_pos   = np.array(blackboard.ball.get_position(), dtype=float)
    goal_pos   = np.array(blackboard.opponent_goal_pos, dtype=float)
    dist       = float(np.linalg.norm(ball_pos - player_pos))

    contact_made = getattr(blackboard, '_contact_made', False)

    # ──────────────────────────────
    # FASE 1: Acercamiento (PID con velocidad limitada)
    # ──────────────────────────────
    if not contact_made:
        # Pelota empujada demasiado lejos
        if dist > BEHIND_BALL_APPROACH_PX * ADVANCE_ESCAPE_FACTOR:
            _clear_advance_state(blackboard, player_id)
            log.info("[advance_contact] Robot %d | pelota escapó (%.0fpx) → FAILURE",
                     player_id, dist)
            return NodeStatus.FAILURE

        # Contacto alcanzado: detener PID y empezar asentamiento
        if dist < CAPTURE_CONFIRM_DISTANCE_PX:
            _clear_advance_state(blackboard, player_id)
            blackboard._contact_made         = True
            blackboard._contact_settle_start = time.time()
            blackboard.player._has_ball      = True
            blackboard.last_action           = "settling_contact"
            log.info("[advance_contact] Robot %d | contacto! dist=%.1fpx | Asentando %.2fs...",
                     player_id, dist, CONTACT_SETTLE_TIME_S)
            return NodeStatus.RUNNING

        # Recalcular overshoot target (la pelota pudo moverse ligeramente).
        # El guard en move_robot_to ignora si delta < 20px (mismo target).
        overshoot = _compute_overshoot_target(ball_pos, goal_pos, CAPTURE_OVERSHOOT_PX)
        target = (int(overshoot[0]), int(overshoot[1]))
        blackboard.command_manager.move_robot_to(player_id, target)
        return NodeStatus.RUNNING

    # ──────────────────────────────
    # FASE 2: Asentamiento (robot quieto)
    # ──────────────────────────────
    # Pelota escapó durante el asentamiento
    if dist > CAPTURE_CONFIRM_DISTANCE_PX * SETTLE_ESCAPE_FACTOR:
        blackboard.player._has_ball = False
        log.info("[advance_contact] Robot %d | pelota escapó en asentamiento (%.0fpx) → FAILURE",
                 player_id, dist)
        return NodeStatus.FAILURE

    # ¿Terminó el tiempo de asentamiento?
    elapsed = time.time() - blackboard._contact_settle_start
    if elapsed >= CONTACT_SETTLE_TIME_S:
        blackboard.player._has_ball = True
        blackboard.last_action      = "contact_confirmed"
        log.info("[advance_contact] Robot %d | asentamiento OK (%.2fs) → SUCCESS",
                 player_id, elapsed)
        return NodeStatus.SUCCESS

    return NodeStatus.RUNNING


def create_advance_to_contact_node():
    """Crea el nodo stateful que avanza al contacto con la pelota."""
    return StatefulActionNode(
        _advance_to_contact_start,
        _advance_to_contact_running,
        name="AvanzarAlContacto"
    )


def kick_immediately(blackboard):
    """Dispara el solenoide inmediatamente desde la posición actual.

    El robot ya está alineado con el arco (posicionado por move_behind_ball).
    No necesita rotar ni moverse: dispara directo.

    Returns:
        NodeStatus.SUCCESS siempre (el disparo es instantáneo).
    """
    if not hasattr(blackboard, "command_manager"):
        return NodeStatus.FAILURE

    player_pos = blackboard.player.get_position()
    goal_pos   = blackboard.opponent_goal_pos

    # Disparar solenoide en robots reales
    rf = blackboard.command_manager.rf_controller
    if rf:
        fw_id = blackboard.player.id + 1
        rf.kick(fw_id, 1.0)

    # En simulación: aplicar velocidad a la pelota hacia el arco
    ball = blackboard.ball
    if hasattr(ball, 'dx') and hasattr(ball, 'dy'):
        direction = np.array(goal_pos, dtype=float) - np.array(player_pos, dtype=float)
        dist = np.linalg.norm(direction)
        if dist > 0:
            direction /= dist
        ball.dx = 15.0 * direction[0]
        ball.dy = 15.0 * direction[1]

    blackboard.player._has_ball = False
    blackboard.last_action = "kick_immediately"
    log.info("[kick] Robot %d | solenoide disparado → arco (%.0f, %.0f)",
             blackboard.player.id, goal_pos[0], goal_pos[1])
    return NodeStatus.SUCCESS


# CREACIÓN DE ÁRBOLES DE COMPORTAMIENTO COMPLETOS


def create_attacker_tree():
    """Árbol de comportamiento para atacante sin dribbler activo.

    Estrategia: posicionarse detrás de la pelota (alineado con el arco rival)
    → avanzar hasta hacer contacto → disparar el solenoide inmediatamente.
    El robot nunca necesita rotar con la pelota ni activar el dribbler.

    Ciclo continuo:
        PosicionarDetrásPelota → AvanzarAlContacto → DisparoInmediato → (reinicio)

    Returns:
        SelectorNode: Árbol de comportamiento del atacante.
    """
    # Ciclo de ataque directo: posicionar → contactar → disparar
    attack_cycle = SequenceNode("CicloAtaqueDirecto")
    attack_cycle.add_children(
        create_move_behind_ball_node(),
        create_advance_to_contact_node(),
        ActionNode(kick_immediately, "DisparoInmediato"),
    )

    attacker_tree = SelectorNode("ComportamientoAtacante")
    attacker_tree.add_children(attack_cycle)

    return attacker_tree


def create_defender_tree():
    """Crea el árbol de comportamiento para un jugador defensor.

    Returns:
        SelectorNode: Árbol de comportamiento completo para defensor
    """
    # COMPORTAMIENTO DEFENSIVO (pelota en zona defensiva)
    defensive_behavior = SequenceNode("ComportamientoDefensivo")
    defensive_behavior.add_children(
        ConditionNode(is_ball_in_defensive_zone, "PelotaEnZonaDefensiva"),
        SelectorNode("AccionDefensiva").add_children(
            # Capturar la pelota si está libre
            SequenceNode("CapturarPelotaDefensiva").add_children(
                ConditionNode(is_ball_free, "PelotaLibre"),
                create_move_to_ball_node(),
                ActionNode(capture_ball, "CapturarPelota"),
            ),
            # Bloquear rival si tiene la pelota
            SequenceNode("BloquearRival").add_children(
                ConditionNode(is_ball_in_opponent_possession, "PelotaEnPosesionRival"),
                ActionNode(block_opponent, "BloquearOponente"),
            ),
            # Posicionarse para defender la portería
            ActionNode(position_to_defend_goal, "DefenderPorteria"),
        ),
    )

    # COMPORTAMIENTO DE APOYO (pelota en zona neutral u ofensiva)
    support_behavior = SequenceNode("ComportamientoApoyo")
    support_behavior.add_children(
        InverterNode(
            ConditionNode(is_ball_in_defensive_zone, "PelotaNoEnZonaDefensiva")
        ),
        SelectorNode("AccionApoyo").add_children(
            # Si tiene la pelota, avanzar o pasar
            SequenceNode("ConPelotaEnApoyo").add_children(
                ConditionNode(is_player_with_ball, "TienePelota"),
                SelectorNode("AccionConPelota").add_children(
                    # Intentar pasar a un compañero
                    SequenceNode("IntentarPase").add_children(
                        ConditionNode(is_pass_possible, "PasePosible"),
                        ActionNode(pass_to_teammate, "PasarACompanero"),
                    ),
                    # Avanzar con la pelota
                    ActionNode(dribble_forward, "AvanzarConPelota"),
                ),
            ),
            # Si la pelota está libre en zona neutral, intentar capturarla
            SequenceNode("CapturarPelotaNeutral").add_children(
                ConditionNode(is_ball_in_neutral_zone, "PelotaEnZonaNeutral"),
                ConditionNode(is_ball_free, "PelotaLibre"),
                ConditionNode(is_closest_defender_to_ball, "DefensorMasCercano"),
                ConditionNode(is_no_attacker_closer_to_ball, "NoHayAtacanteMasCerca"),
                create_move_to_ball_node(),
                ActionNode(capture_ball, "CapturarPelota"),
            ),
            # Mantener posición defensiva
            ActionNode(move_to_defensive_position, "MantenerPosicionDefensiva"),
        ),
    )

    # ÁRBOL PRINCIPAL DEL DEFENSOR
    defender_tree = SelectorNode("ComportamientoDefensor")
    defender_tree.add_children(defensive_behavior, support_behavior)

    return defender_tree
