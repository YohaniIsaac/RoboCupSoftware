"""
Comportamientos específicos de fútbol para el sistema de árboles de comportamiento.

Este módulo implementa nodos específicos y árboles predefinidos para las
estrategias y tácticas de los robots en el juego de fútbol.
"""

import numpy as np
from .base import (
    BehaviorNode, NodeStatus, SequenceNode, SelectorNode,
    ParallelNode, ConditionNode, ActionNode, InverterNode
)
from robot_soccer.config import *
from robot_soccer.controllers.robot_command_manager import RobotCommandManager


class Blackboard:
    """
    Pizarra (blackboard) para compartir datos entre nodos del árbol de comportamiento.
    Contiene el estado del juego y los robots.
    """

    def __init__(self, player, ball, team_players, opponents, team='red'):
        self.player = player  # Jugador que ejecuta este árbol
        self.ball = ball  # Pelota
        self.team_players = team_players  # Lista de jugadores del equipo
        self.opponents = opponents  # Lista de jugadores rivales
        self.team = team  # Equipo ('red' o 'blue')

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
        """Actualiza el contexto del juego con los valores de lógica difusa."""
        self.posesion_pelota = posesion
        self.proximidad_equipo = proximidad
        self.zona_pelota = zona

    def update_strategic_positions(self):
        """Calcula posiciones estratégicas en el campo."""
        # Determinar orientación del campo según equipo
        if self.team == 'red':
            # Equipo rojo ataca hacia la derecha
            self.own_goal_pos = [0, ALTO_CAMPO / 2]
            self.opponent_goal_pos = [ANCHO_CAMPO, ALTO_CAMPO / 2]
            self.defensive_zone_center = [ANCHO_CAMPO * 0.2, ALTO_CAMPO / 2]
            self.neutral_zone_center = [ANCHO_CAMPO * 0.5, ALTO_CAMPO / 2]
            self.offensive_zone_center = [ANCHO_CAMPO * 0.8, ALTO_CAMPO / 2]
        else:
            # Equipo azul ataca hacia la izquierda
            self.own_goal_pos = [ANCHO_CAMPO, ALTO_CAMPO / 2]
            self.opponent_goal_pos = [0, ALTO_CAMPO / 2]
            self.defensive_zone_center = [ANCHO_CAMPO * 0.8, ALTO_CAMPO / 2]
            self.neutral_zone_center = [ANCHO_CAMPO * 0.5, ALTO_CAMPO / 2]
            self.offensive_zone_center = [ANCHO_CAMPO * 0.2, ALTO_CAMPO / 2]

        # Posiciones estratégicas defensivas
        self.defensive_positions = [
            [self.defensive_zone_center[0], ALTO_CAMPO * 0.3],  # Defensa arriba
            [self.defensive_zone_center[0], ALTO_CAMPO * 0.7]  # Defensa abajo
        ]

        # Posiciones estratégicas ofensivas
        self.offensive_positions = [
            [self.offensive_zone_center[0], ALTO_CAMPO * 0.3],  # Ataque arriba
            [self.offensive_zone_center[0], ALTO_CAMPO * 0.7]  # Ataque abajo
        ]


# CONDICIONES ESPECÍFICAS PARA FÚTBOL

def is_ball_free(blackboard):
    """Comprueba si la pelota está libre."""
    return 0.3 <= blackboard.posesion_pelota <= 0.7


def is_ball_in_team_possession(blackboard):
    """Comprueba si la pelota está en posesión del equipo."""
    return blackboard.posesion_pelota < 0.3


def is_ball_in_opponent_possession(blackboard):
    """Comprueba si la pelota está en posesión del equipo rival."""
    return blackboard.posesion_pelota > 0.7


def is_ball_close_to_team(blackboard):
    """Comprueba si la pelota está más cerca del equipo que del rival."""
    return blackboard.proximidad_equipo < 0.8


def is_ball_close_to_opponent(blackboard):
    """Comprueba si la pelota está más cerca del rival que del equipo."""
    return blackboard.proximidad_equipo > 1.2


def is_ball_in_defensive_zone(blackboard):
    """Comprueba si la pelota está en zona defensiva."""
    return blackboard.zona_pelota < 0.4


def is_ball_in_neutral_zone(blackboard):
    """Comprueba si la pelota está en zona neutral."""
    return 0.4 <= blackboard.zona_pelota <= 1.6


def is_ball_in_offensive_zone(blackboard):
    """Comprueba si la pelota está en zona ofensiva."""
    return blackboard.zona_pelota > 1.6


def is_player_with_ball(blackboard):
    """Comprueba si este jugador tiene la pelota."""
    return blackboard.player.has_ball()


def is_player_attacker(blackboard):
    """Comprueba si este jugador tiene rol de atacante."""
    return blackboard.player.rol == ROL_ATACANTE


def is_player_defender(blackboard):
    """Comprueba si este jugador tiene rol de defensor."""
    return blackboard.player.rol == ROL_DEFENSIVO


def is_shot_possible(blackboard):
    """Comprueba si es posible realizar un tiro a portería."""
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
            perp_distance = np.linalg.norm(to_opponent - projection * to_goal_normalized)

            # Si el oponente está cerca de la línea de tiro, está bloqueando
            if perp_distance < 50:  # 50 es un umbral razonable
                shot_blocked = True
                break

    # Condiciones para un tiro posible
    return (
            distance_to_goal < 400 and  # Distancia razonable para tirar
            angle_diff < np.pi / 4 and  # Bien orientado hacia la portería
            not shot_blocked  # Sin oponentes bloqueando
    )


def is_pass_possible(blackboard):
    """Comprueba si es posible realizar un pase a un compañero."""
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
        if pass_distance < 100 or pass_distance > 600:
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
                perp_distance = np.linalg.norm(to_opponent - projection * to_teammate_normalized)

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
    """Mover el jugador hacia la pelota."""
    # Obtener posición de la pelota
    ball_pos = blackboard.ball.get_position()

    # Verificar que existe el gestor de comandos
    if not hasattr(blackboard, 'command_manager'):
        blackboard.logger.warning("No command manager found in blackboard. Action may not work properly.")
        return NodeStatus.FAILURE

    # Ordenar movimiento al gestor de comandos
    blackboard.command_manager.move_robot_to(blackboard.player.id, ball_pos)

    # Verificar si hemos llegado lo suficientemente cerca
    distance = blackboard.player.distance_to_ball(blackboard.ball)
    if distance < 50:
        return NodeStatus.SUCCESS

    # Continuar ejecutando la acción
    return NodeStatus.RUNNING


def capture_ball(blackboard):
    """Capturar la pelota."""
    # Verificar que existe el gestor de comandos
    if not hasattr(blackboard, 'command_manager'):
        blackboard.logger.warning("No command manager found in blackboard. Action may not work properly.")
        return NodeStatus.FAILURE

    # Ordenar captura al gestor de comandos
    blackboard.command_manager.capture_ball(blackboard.player.id)

    # Verificar si se tiene control de la pelota
    if blackboard.player.has_ball():
        return NodeStatus.SUCCESS

    # Continuar ejecutando la acción
    return NodeStatus.RUNNING


def dribble_forward(blackboard):
    """Avanzar con la pelota hacia la portería rival."""
    if not blackboard.player.has_ball():
        return NodeStatus.FAILURE

    # Verificar que existe el gestor de comandos
    if not hasattr(blackboard, 'command_manager'):
        blackboard.logger.warning("No command manager found in blackboard. Action may not work properly.")
        return NodeStatus.FAILURE

    # Calcular punto objetivo (hacia la portería rival)
    player_pos = blackboard.player.get_position()
    goal_pos = blackboard.opponent_goal_pos

    # Vector hacia la portería
    to_goal = np.array(goal_pos) - np.array(player_pos)
    distance_to_goal = np.linalg.norm(to_goal)

    # Normalizar y escalar (no ir directamente a la portería, sino avanzar en esa dirección)
    if distance_to_goal > 0:
        to_goal = to_goal / distance_to_goal * min(distance_to_goal, 200)

    # Punto objetivo
    target_pos = tuple(player_pos + to_goal)

    # Ordenar movimiento con la pelota
    blackboard.command_manager.move_with_ball(
        blackboard.player.id,
        target_pos,
        blackboard.ball
    )

    # Si estamos cerca de la portería, consideramos exitoso el dribbling
    if distance_to_goal < 200:
        return NodeStatus.SUCCESS

    # Continuar ejecutando la acción
    return NodeStatus.RUNNING


def shoot_to_goal(blackboard):
    """Disparar a portería."""
    if not blackboard.player.has_ball():
        return NodeStatus.FAILURE

    # Verificar que existe el gestor de comandos
    if not hasattr(blackboard, 'command_manager'):
        blackboard.logger.warning("No command manager found in blackboard. Action may not work properly.")
        return NodeStatus.FAILURE

    # Calcular el mejor punto para disparar (no siempre al centro)
    goal_pos = blackboard.opponent_goal_pos
    goal_width = 200  # Ancho de la portería

    # Añadir algo de variación para no disparar siempre al centro
    offset = goal_width * 0.4 * (1 if np.random.random() > 0.5 else -1)
    target = [goal_pos[0], goal_pos[1] + offset]

    # Ordenar pateo
    blackboard.command_manager.kick_ball(
        blackboard.player.id,
        target,
        blackboard.ball,
        power=0.9
    )

    # Registrar la acción
    blackboard.last_action = "shoot_to_goal"

    # Esta acción se completa inmediatamente
    return NodeStatus.SUCCESS


def pass_to_teammate(blackboard):
    """Realizar un pase a un compañero."""
    if not blackboard.player.has_ball() or not hasattr(blackboard, 'pass_target'):
        return NodeStatus.FAILURE

    # Verificar que existe el gestor de comandos
    if not hasattr(blackboard, 'command_manager'):
        blackboard.logger.warning("No command manager found in blackboard. Action may not work properly.")
        return NodeStatus.FAILURE

    # Obtener posición del compañero
    teammate_pos = blackboard.pass_target.get_position()

    # Ajustar punto de pase (liderar al compañero)
    teammate_angle = np.radians(blackboard.pass_target.angle)
    lead_distance = 100  # Distancia de adelanto
    lead_pos = [
        teammate_pos[0] + lead_distance * np.cos(teammate_angle),
        teammate_pos[1] + lead_distance * np.sin(teammate_angle)
    ]

    # Ordenar pateo
    blackboard.command_manager.kick_ball(
        blackboard.player.id,
        lead_pos,
        blackboard.ball,
        power=0.7
    )

    # Registrar la acción
    blackboard.last_action = "pass_to_teammate"

    # Esta acción se completa inmediatamente
    return NodeStatus.SUCCESS


def intercept_ball(blackboard):
    """Interceptar la trayectoria de la pelota."""
    # Verificar que existe el gestor de comandos
    if not hasattr(blackboard, 'command_manager'):
        blackboard.logger.warning("No command manager found in blackboard. Action may not work properly.")
        return NodeStatus.FAILURE

    # Determinar tiempo de predicción según velocidad de la pelota
    prediction_time = 1.0  # segundos
    if hasattr(blackboard.ball, 'dx') and hasattr(blackboard.ball, 'dy'):
        ball_speed = np.linalg.norm([blackboard.ball.dx, blackboard.ball.dy])
        prediction_time = max(0.3, min(1.5, 0.3 + ball_speed * 0.05))

    # Calcular posición futura de la pelota
    ball_pos = blackboard.ball.get_position()
    future_pos = ball_pos
    if hasattr(blackboard.ball, 'dx') and hasattr(blackboard.ball, 'dy'):
        future_pos = (
            ball_pos[0] + blackboard.ball.dx * prediction_time,
            ball_pos[1] + blackboard.ball.dy * prediction_time
        )

    # Ordenar movimiento al punto de intercepción
    blackboard.command_manager.move_robot_to(
        blackboard.player.id,
        future_pos,
        speed_factor=1.5
    )

    # Verificar si estamos cerca de la pelota
    distance = blackboard.player.distance_to_ball(blackboard.ball)
    if distance < 30:
        return NodeStatus.SUCCESS

    # Continuar ejecutando la acción
    return NodeStatus.RUNNING


def block_opponent(blackboard):
    """Bloquear al oponente más cercano a nuestra portería."""
    # Verificar que existe el gestor de comandos
    if not hasattr(blackboard, 'command_manager'):
        blackboard.logger.warning("No command manager found in blackboard. Action may not work properly.")
        return NodeStatus.FAILURE

    # Encontrar el oponente más peligroso (más cercano a nuestra portería)
    most_dangerous = None
    min_distance = float('inf')

    for opponent in blackboard.opponents:
        opp_pos = opponent.get_position()
        distance_to_goal = np.linalg.norm(np.array(opp_pos) - np.array(blackboard.own_goal_pos))

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
        blackboard.player.id,
        block_pos,
        speed_factor=1.2
    )

    # Registrar la acción
    blackboard.last_action = "block_opponent"

    # Este es un comportamiento continuo, siempre está en ejecución
    return NodeStatus.RUNNING


def move_to_defensive_position(blackboard):
    """Mover el jugador a una posición defensiva estratégica."""
    # Verificar que existe el gestor de comandos
    if not hasattr(blackboard, 'command_manager'):
        blackboard.logger.warning("No command manager found in blackboard. Action may not work properly.")
        return NodeStatus.FAILURE

    # Elegir posición defensiva
    ball_pos = blackboard.ball.get_position()
    defensive_pos = None

    # Elegir la posición defensiva más cercana a la pelota
    min_distance = float('inf')
    for pos in blackboard.defensive_positions:
        distance = np.linalg.norm(np.array(pos) - np.array(ball_pos))
        if distance < min_distance:
            min_distance = distance
            defensive_pos = pos

    if not defensive_pos:
        defensive_pos = blackboard.defensive_positions[0]

    # Ordenar movimiento
    blackboard.command_manager.move_robot_to(
        blackboard.player.id,
        defensive_pos
    )

    # Verificar si hemos llegado
    player_pos = blackboard.player.get_position()
    distance = np.linalg.norm(np.array(player_pos) - np.array(defensive_pos))

    if distance < 30:
        return NodeStatus.SUCCESS

    # Continuar ejecutando la acción
    return NodeStatus.RUNNING


def move_to_support_position(blackboard):
    """Mover el jugador a una posición de apoyo ofensivo."""
    # Verificar que existe el gestor de comandos
    if not hasattr(blackboard, 'command_manager'):
        blackboard.logger.warning("No command manager found in blackboard. Action may not work properly.")
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

        # Posición de apoyo (a 200 unidades de la pelota)
        support_pos = ball_pos + np.array(rotated) * 200
    else:
        # Fallback si la pelota está en la portería
        support_pos = [
            (ball_pos[0] + goal_pos[0]) / 2,
            (ball_pos[1] + goal_pos[1]) / 2
        ]

    # Asegurar que la posición está dentro del campo
    support_pos[0] = max(50, min(ANCHO_CAMPO - 50, support_pos[0]))
    support_pos[1] = max(50, min(ALTO_CAMPO - 50, support_pos[1]))

    # Ordenar movimiento
    blackboard.command_manager.move_robot_to(
        blackboard.player.id,
        tuple(support_pos)
    )

    # Registrar la acción
    blackboard.last_action = "move_to_support_position"

    # Este es un comportamiento continuo, siempre está en ejecución
    return NodeStatus.RUNNING


def position_to_defend_goal(blackboard):
    """Posicionarse para defender la portería."""
    # Verificar que existe el gestor de comandos
    if not hasattr(blackboard, 'command_manager'):
        blackboard.logger.warning("No command manager found in blackboard. Action may not work properly.")
        return NodeStatus.FAILURE

    ball_pos = blackboard.ball.get_position()
    goal_pos = blackboard.own_goal_pos

    # Vector desde la pelota hacia nuestra portería
    to_ball = np.array(ball_pos) - np.array(goal_pos)
    distance = np.linalg.norm(to_ball)

    if distance > 0:
        # Normalizar y escalar para posicionarse entre la pelota y la portería
        to_ball = to_ball / distance
        defend_distance = min(distance * 0.6, 200)  # No alejarse demasiado de la portería

        # Posición de defensa
        defend_pos = tuple(np.array(goal_pos) + to_ball * defend_distance)
    else:
        # Fallback
        defend_pos = tuple(goal_pos)

    # Ordenar movimiento
    blackboard.command_manager.move_robot_to(
        blackboard.player.id,
        defend_pos
    )

    # Registrar la acción
    blackboard.last_action = "position_to_defend_goal"

    # Este es un comportamiento continuo, siempre está en ejecución
    return NodeStatus.RUNNING


# CREACIÓN DE ÁRBOLES DE COMPORTAMIENTO COMPLETOS

def create_attacker_tree():
    """Crea el árbol de comportamiento para un jugador atacante."""

    # COMPORTAMIENTO OFENSIVO (con pelota)
    offensive_with_ball = SequenceNode("OfensivaConPelota")
    offensive_with_ball.add_children(
        ConditionNode(is_player_with_ball, "TienePelota"),
        SelectorNode("AccionOfensiva").add_children(
            # Intentar tirar si es posible
            SequenceNode("IntentarTiro").add_children(
                ConditionNode(is_shot_possible, "TiroPosible"),
                ActionNode(shoot_to_goal, "TirarAPorteria")
            ),
            # Intentar pasar si es posible
            SequenceNode("IntentarPase").add_children(
                ConditionNode(is_pass_possible, "PasePosible"),
                ActionNode(pass_to_teammate, "PasarACompanero")
            ),
            # Avanzar con la pelota
            ActionNode(dribble_forward, "AvanzarConPelota")
        )
    )

    # COMPORTAMIENTO OFENSIVO (sin pelota)
    offensive_without_ball = SequenceNode("OfensivaSinPelota")
    offensive_without_ball.add_children(
        InverterNode(ConditionNode(is_player_with_ball, "NoTienePelota")),
        SelectorNode("AccionSinPelota").add_children(
            # Capturar la pelota si está libre y cerca
            SequenceNode("CapturarPelotaLibre").add_children(
                ConditionNode(is_ball_free, "PelotaLibre"),
                ConditionNode(is_ball_close_to_team, "PelotaCercaDelEquipo"),
                ActionNode(move_to_ball, "MoverHaciaPelota"),
                ActionNode(capture_ball, "CapturarPelota")
            ),
            # Interceptar si el rival tiene la pelota
            SequenceNode("InterceptarRival").add_children(
                ConditionNode(is_ball_in_opponent_possession, "PelotaEnPosesionRival"),
                ActionNode(intercept_ball, "InterceptarPelota")
            ),
            # Buscar posición de apoyo si un compañero tiene la pelota
            SequenceNode("ApoyoOfensivo").add_children(
                ConditionNode(is_ball_in_team_possession, "PelotaEnPosesionEquipo"),
                ActionNode(move_to_support_position, "MoverAPosicionDeApoyo")
            )
        )
    )

    # ÁRBOL PRINCIPAL DEL ATACANTE
    attacker_tree = SelectorNode("ComportamientoAtacante")
    attacker_tree.add_children(
        offensive_with_ball,
        offensive_without_ball
    )

    return attacker_tree


def create_defender_tree():
    """Crea el árbol de comportamiento para un jugador defensor."""

    # COMPORTAMIENTO DEFENSIVO (pelota en zona defensiva)
    defensive_behavior = SequenceNode("ComportamientoDefensivo")
    defensive_behavior.add_children(
        ConditionNode(is_ball_in_defensive_zone, "PelotaEnZonaDefensiva"),
        SelectorNode("AccionDefensiva").add_children(
            # Capturar la pelota si está libre
            SequenceNode("CapturarPelotaDefensiva").add_children(
                ConditionNode(is_ball_free, "PelotaLibre"),
                ActionNode(move_to_ball, "MoverHaciaPelota"),
                ActionNode(capture_ball, "CapturarPelota")
            ),
            # Bloquear rival si tiene la pelota
            SequenceNode("BloquearRival").add_children(
                ConditionNode(is_ball_in_opponent_possession, "PelotaEnPosesionRival"),
                ActionNode(block_opponent, "BloquearOponente")
            ),
            # Posicionarse para defender la portería
            ActionNode(position_to_defend_goal, "DefenderPorteria")
        )
    )

    # COMPORTAMIENTO DE APOYO (pelota en zona neutral u ofensiva)
    support_behavior = SequenceNode("ComportamientoApoyo")
    support_behavior.add_children(
        InverterNode(ConditionNode(is_ball_in_defensive_zone, "PelotaNoEnZonaDefensiva")),
        SelectorNode("AccionApoyo").add_children(
            # Si tiene la pelota, avanzar o pasar
            SequenceNode("ConPelotaEnApoyo").add_children(
                ConditionNode(is_player_with_ball, "TienePelota"),
                SelectorNode("AccionConPelota").add_children(
                    # Intentar pasar a un compañero
                    SequenceNode("IntentarPase").add_children(
                        ConditionNode(is_pass_possible, "PasePosible"),
                        ActionNode(pass_to_teammate, "PasarACompanero")
                    ),
                    # Avanzar con la pelota
                    ActionNode(dribble_forward, "AvanzarConPelota")
                )
            ),
            # Si la pelota está libre en zona neutral, intentar capturarla
            SequenceNode("CapturarPelotaNeutral").add_children(
                ConditionNode(is_ball_in_neutral_zone, "PelotaEnZonaNeutral"),
                ConditionNode(is_ball_free, "PelotaLibre"),
                ActionNode(move_to_ball, "MoverHaciaPelota"),
                ActionNode(capture_ball, "CapturarPelota")
            ),
            # Mantener posición defensiva
            ActionNode(move_to_defensive_position, "MantenerPosicionDefensiva")
        )
    )

    # ÁRBOL PRINCIPAL DEL DEFENSOR
    defender_tree = SelectorNode("ComportamientoDefensor")
    defender_tree.add_children(
        defensive_behavior,
        support_behavior
    )

    return defender_tree
