"""Comportamientos específicos de fútbol para el sistema de árboles de comportamiento.

Este módulo implementa nodos específicos y árboles predefinidos para las
estrategias y tácticas de los robots en el juego de fútbol.
"""

import logging

import numpy as np
from robot_soccer.ai.behavior_tree.utils import calculate_ball_approach_position
from robot_soccer.config import (
    ROL_ATACANTE, ROL_DEFENSIVO, FIELD_SIM,
    BT_SHOT_DISTANCE_RATIO, BT_PASS_MIN_RATIO, BT_PASS_MAX_RATIO,
    BT_CAPTURE_RANGE_RATIO, BT_APPROACH_RATIO, BT_CAPTURE_ACTIVATE_RATIO,
    BT_CAPTURE_CONFIRM_RATIO, BT_INTERCEPT_RATIO, BT_SUPPORT_DISTANCE_RATIO,
    BT_DEFENDER_WAIT_RATIO, BT_DRIBBLE_GOAL_RATIO,
    BT_DEFENSIVE_ARRIVAL_RATIO, BT_ATTACKER_PRIORITY_MARGIN_RATIO,
)

from .base import (
    NodeStatus,
    SequenceNode,
    SelectorNode,
    ConditionNode,
    ActionNode,
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
    return (
        distance_to_goal < blackboard.field.ratio_to_px(BT_SHOT_DISTANCE_RATIO)
        and angle_diff < np.pi / 4  # Bien orientado hacia la portería
        and not shot_blocked  # Sin oponentes bloqueando
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


def move_to_ball(blackboard):
    """Mover el jugador hacia una posición estratégica individualizada cerca de la pelota.

    Cada robot calcula su punto óptimo considerando el arco enemigo y su posición actual.

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

    # Obtener posiciones
    player_pos = blackboard.player.get_position()
    ball_pos = blackboard.ball.get_position()

    # CLAVE: Obtener arco enemigo según el equipo del robot
    opponent_goal_pos = blackboard.opponent_goal_pos

    # CÁLCULO INDIVIDUALIZADO: Cada robot calcula SU punto óptimo
    approach_distance = 45
    target_pos = calculate_ball_approach_position(
        player_pos,  # Posición ACTUAL del robot
        ball_pos,
        opponent_goal_pos,  # Arco enemigo para alineación
        approach_distance,
        blackboard.team,  # Equipo del robot
        field=blackboard.field,
    )

    # Registrar movimiento planificado para debugging
    if hasattr(blackboard, "tracer") and hasattr(
        blackboard.tracer, "set_planned_movement"
    ):
        blackboard.tracer.set_planned_movement(
            blackboard.player.id,
            target_pos,
            "move_to_ball_individualized",
            {
                "player_pos": player_pos,
                "opponent_goal": opponent_goal_pos,
                "approach_distance": approach_distance,
                "ball_position": ball_pos,
                "team": blackboard.team,
            },
        )

    # Ordenar movimiento al gestor de comandos
    blackboard.command_manager.move_robot_to(blackboard.player.id, target_pos)

    # Verificar si hemos llegado lo suficientemente cerca del objetivo
    distance_to_target = np.linalg.norm(np.array(player_pos) - np.array(target_pos))
    if distance_to_target < blackboard.field.ratio_to_px(BT_APPROACH_RATIO):
        # Si llegamos al punto objetivo, verificar distancia a la pelota
        distance_to_ball = blackboard.player.distance_to_ball(blackboard.ball)
        if distance_to_ball < blackboard.field.ratio_to_px(BT_CAPTURE_RANGE_RATIO):
            return NodeStatus.SUCCESS

    # Continuar ejecutando la acción
    return NodeStatus.RUNNING


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

    if distance_to_ball < blackboard.field.ratio_to_px(BT_CAPTURE_ACTIVATE_RATIO):
        # PASO 1: Orientarse hacia la pelota
        angle_to_ball = np.degrees(
            np.arctan2(ball_pos[1] - player_pos[1], ball_pos[0] - player_pos[0])
        )

        current_angle = blackboard.player.angle
        angle_diff = abs((angle_to_ball - current_angle + 180) % 360 - 180)

        if angle_diff > 10:  # Necesita orientarse mejor
            blackboard.command_manager.rotate_robot_to(
                blackboard.player.id, angle_to_ball
            )
            return NodeStatus.RUNNING

        # PASO 2: ACTIVAR MOTOR DE CAPTURA
        # Activar dribbler/motor para "agarrar" la pelota
        blackboard.command_manager.capture_ball(blackboard.player.id)

        # PASO 3: Verificar captura exitosa
        if distance_to_ball < blackboard.field.ratio_to_px(BT_CAPTURE_CONFIRM_RATIO):
            # Marcar como capturada en el modelo
            blackboard.player._has_ball = True

            # Registrar acción exitosa
            blackboard.last_action = "ball_captured_with_motor"

            return NodeStatus.SUCCESS

    # Aún no estamos en rango, continuar acercándose
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
    """Disparar a portería desde una posición estratégica óptima.

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

    # Calcular posición óptima para disparar
    shooting_pos = calculate_shooting_position(
        player_pos, ball_pos, goal_pos, approach_distance=50,
        field=blackboard.field,
    )

    # Verificar si estamos en buena posición para disparar
    distance_to_shooting_pos = np.linalg.norm(
        np.array(player_pos) - np.array(shooting_pos)
    )

    if distance_to_shooting_pos > blackboard.field.ratio_to_px(BT_APPROACH_RATIO):
        # Moverse a mejor posición de disparo
        blackboard.command_manager.move_with_ball(
            blackboard.player.id, shooting_pos, blackboard.ball, speed_factor=0.8
        )
        return NodeStatus.RUNNING

    # Calcular mejor punto de la portería para disparar
    goal_width = blackboard.field.goal_right_size
    # Añadir variación para no disparar siempre al centro
    offset = goal_width * 0.3 * (1 if np.random.random() > 0.5 else -1)
    target = [goal_pos[0], goal_pos[1] + offset]

    # Verificar orientación hacia la portería
    angle_to_goal = np.degrees(
        np.arctan2(target[1] - player_pos[1], target[0] - player_pos[0])
    )

    current_angle = blackboard.player.angle
    angle_diff = abs((angle_to_goal - current_angle + 180) % 360 - 180)

    if angle_diff > 10:
        # Orientarse mejor antes de disparar
        blackboard.command_manager.rotate_robot_to(blackboard.player.id, angle_to_goal)
        return NodeStatus.RUNNING

    # Disparar
    blackboard.command_manager.kick_ball(
        blackboard.player.id, target, blackboard.ball, power=0.9
    )

    # Registrar la acción
    blackboard.last_action = "shoot_to_goal"

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

# CREACIÓN DE ÁRBOLES DE COMPORTAMIENTO COMPLETOS


def create_attacker_tree():
    """Crea el árbol de comportamiento para un jugador atacante.

    Returns:
        SelectorNode: Árbol de comportamiento completo para atacante
    """
    # COMPORTAMIENTO OFENSIVO (con pelota)
    offensive_with_ball = SequenceNode("OfensivaConPelota")
    offensive_with_ball.add_children(
        ConditionNode(is_player_with_ball, "TienePelota"),
        SelectorNode("AccionOfensiva").add_children(
            # Intentar tirar si es posible
            SequenceNode("IntentarTiro").add_children(
                ConditionNode(is_shot_possible, "TiroPosible"),
                ActionNode(shoot_to_goal, "TirarAPorteria"),
            ),
            # Intentar pasar si es posible
            SequenceNode("IntentarPase").add_children(
                ConditionNode(is_pass_possible, "PasePosible"),
                ActionNode(pass_to_teammate, "PasarACompanero"),
            ),
            # Avanzar con la pelota
            ActionNode(dribble_forward, "AvanzarConPelota"),
        ),
    )

    # COMPORTAMIENTO OFENSIVO (sin pelota)
    offensive_without_ball = SequenceNode("OfensivaSinPelota")
    offensive_without_ball.add_children(
        InverterNode(ConditionNode(is_player_with_ball, "NoTienePelota")),
        SelectorNode("AccionSinPelota").add_children(
            # Capturar la pelota si está libre y cerca del equipo
            SequenceNode("CapturarPelotaLibre").add_children(
                ConditionNode(is_ball_free, "PelotaLibre"),
                ConditionNode(is_ball_close_to_team, "PelotaCercaDelEquipo"),
                ActionNode(move_to_ball, "MoverHaciaPelota"),
                ActionNode(capture_ball, "CapturarPelota"),
            ),
            # NUEVA: Capturar pelota libre si soy el atacante más cercano (sin restricción de proximidad)
            SequenceNode("CapturarPelotaLibreSiMasCercano").add_children(
                ConditionNode(is_ball_free, "PelotaLibre"),
                ConditionNode(is_closest_attacker_to_ball, "AtacanteMasCercano"),
                ActionNode(move_to_ball, "MoverHaciaPelota"),
                ActionNode(capture_ball, "CapturarPelota"),
            ),
            # Interceptar si el rival tiene la pelota
            SequenceNode("InterceptarRival").add_children(
                ConditionNode(is_ball_in_opponent_possession, "PelotaEnPosesionRival"),
                ActionNode(intercept_ball, "InterceptarPelota"),
            ),
            # Buscar posición de apoyo si un compañero tiene la pelota
            SequenceNode("ApoyoOfensivo").add_children(
                ConditionNode(is_ball_in_team_possession, "PelotaEnPosesionEquipo"),
                ActionNode(move_to_support_position, "MoverAPosicionDeApoyo"),
            ),
        ),
    )

    # ÁRBOL PRINCIPAL DEL ATACANTE
    attacker_tree = SelectorNode("ComportamientoAtacante")
    attacker_tree.add_children(offensive_with_ball, offensive_without_ball)

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
                ActionNode(move_to_ball, "MoverHaciaPelota"),
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
                ActionNode(move_to_ball, "MoverHaciaPelota"),
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
