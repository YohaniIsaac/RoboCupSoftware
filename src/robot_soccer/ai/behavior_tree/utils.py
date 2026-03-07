"""Utilitario para calcular posiciones estratégicas cerca de la pelota."""

import numpy as np
from robot_soccer.config import ROL_ATACANTE, ROL_DEFENSIVO


def calculate_ball_approach_position(
    player_pos, ball_pos, opponent_goal_pos, approach_distance=50, team="red",
    field=None
):
    """Calcula la posición óptima INDIVIDUALIZADA para aproximarse a la pelota.

    Cada robot calcula su punto óptimo considerando:
    - Su posición actual
    - La posición del arco enemigo
    - Alineación pelota-robot-arco
    - Minimización de rotación necesaria

    Args:
        player_pos: Posición actual del jugador (x, y)
        ball_pos: Posición de la pelota (x, y)
        opponent_goal_pos: Posición del arco enemigo (x, y)
        approach_distance: Distancia de parada antes de la pelota
        team: Equipo del robot ('red' o 'blue')
        field: FieldGeometry para clamping de posiciones

    Returns:
        tuple: Posición objetivo INDIVIDUAL (x, y) para el robot
    """
    player_pos = np.array(player_pos)
    ball_pos = np.array(ball_pos)
    opponent_goal_pos = np.array(opponent_goal_pos)

    # 1. CÁLCULO DEL PUNTO IDEAL: Alineación pelota-robot-arco
    # Vector desde arco enemigo hacia pelota
    goal_to_ball = ball_pos - opponent_goal_pos
    goal_to_ball_distance = np.linalg.norm(goal_to_ball)

    if goal_to_ball_distance < 1:
        # Pelota muy cerca del arco, usar posición actual del robot
        direction_vector = player_pos - ball_pos
        direction_distance = np.linalg.norm(direction_vector)
        if direction_distance < 1:
            direction_vector = np.array([1, 0])  # Vector por defecto
        else:
            direction_vector = direction_vector / direction_distance
    else:
        # Normalizar: dirección ideal para empujar pelota al arco
        direction_vector = goal_to_ball / goal_to_ball_distance

    # 2. PUNTO IDEAL: Detrás de la pelota, alineado con el arco
    ideal_position = ball_pos + direction_vector * approach_distance

    # 3. OPTIMIZACIÓN POR EFICIENCIA ANGULAR
    # Calcular cuánto tendría que rotar el robot para llegar al punto ideal
    current_to_ideal = ideal_position - player_pos
    ideal_distance = np.linalg.norm(current_to_ideal)

    if ideal_distance > 1:
        pass

    # 4. CONSIDERAR POSICIÓN ACTUAL DEL ROBOT
    # Si el robot está muy lejos del punto ideal, buscar compromiso
    if ideal_distance > 200:  # Robot muy lejos del punto ideal
        # Calcular punto de compromiso entre posición actual y punto ideal
        # Peso 70% punto ideal, 30% minimizar movimiento
        compromise_factor = 0.7

        # Vector desde robot hacia pelota
        player_to_ball = ball_pos - player_pos
        player_to_ball_distance = np.linalg.norm(player_to_ball)

        if player_to_ball_distance > 1:
            player_to_ball = player_to_ball / player_to_ball_distance

            # Punto de compromiso: robot se acerca por su lado pero considerando el arco
            direct_approach = ball_pos + player_to_ball * approach_distance

            # Mezclar punto ideal con aproximación directa
            target_pos = (
                compromise_factor * ideal_position
                + (1 - compromise_factor) * direct_approach
            )
        else:
            target_pos = ideal_position
    else:
        # Robot relativamente cerca del punto ideal, usar punto ideal
        target_pos = ideal_position

    # 5. ASEGURAR QUE LA POSICIÓN ESTÁ DENTRO DEL CAMPO
    if field is not None:
        target_pos = field.clamp(target_pos)
    else:
        target_pos = (int(target_pos[0]), int(target_pos[1]))

    return tuple(target_pos)


def calculate_interception_position(
    player_pos,
    ball_pos,
    ball_velocity,
    opponent_goal_pos,
    team="red",
    prediction_time=1.0,
    approach_distance=40,
    field=None,
):
    """Calcula la posición óptima para interceptar la pelota en movimiento.

    Args:
        player_pos: Posición actual del jugador (x, y)
        ball_pos: Posición actual de la pelota (x, y)
        ball_velocity: Velocidad de la pelota (dx, dy)
        opponent_goal_pos: Posición del arco enemigo (x, y)
        team: Equipo del robot ('red' o 'blue')
        prediction_time: Tiempo de predicción en segundos
        approach_distance: Distancia de aproximación a la pelota
        field: FieldGeometry para clamping de posiciones

    Returns:
        tuple: Posición objetivo para interceptar (x, y)
    """
    # Predecir posición futura de la pelota
    ball_pos = np.array(ball_pos)
    ball_velocity = np.array(ball_velocity)

    # Posición predicha de la pelota
    future_ball_pos = ball_pos + ball_velocity * prediction_time

    # Asegurar que la posición predicha está dentro del campo
    if field is not None:
        future_ball_pos[0] = max(0, min(field.width, int(future_ball_pos[0])))
        future_ball_pos[1] = max(0, min(field.height, int(future_ball_pos[1])))
    else:
        future_ball_pos = np.array([int(future_ball_pos[0]), int(future_ball_pos[1])])

    # Calcular posición de aproximación a la pelota predicha
    return calculate_ball_approach_position(
        player_pos,
        tuple(future_ball_pos),
        opponent_goal_pos,
        approach_distance,
        team,
        field=field,
    )


def calculate_shooting_position(
    player_pos, ball_pos, target_goal, approach_distance=60, field=None
):
    """Calcula la posición óptima para disparar hacia la portería.

    Args:
        player_pos: Posición actual del jugador (x, y)
        ball_pos: Posición de la pelota (x, y)
        target_goal: Posición de la portería objetivo (x, y)
        approach_distance: Distancia óptima para el disparo
        field: FieldGeometry para clamping de posiciones

    Returns:
        tuple: Posición objetivo para el disparo (x, y)
    """
    ball_pos = np.array(ball_pos)
    target_goal = np.array(target_goal)

    # Vector desde portería hacia pelota
    goal_to_ball = ball_pos - target_goal
    distance = np.linalg.norm(goal_to_ball)

    if distance > 1:
        goal_to_ball = goal_to_ball / distance
        # Posicionarse detrás de la pelota en línea con la portería
        target_pos = ball_pos + goal_to_ball * approach_distance
    else:
        # Fallback: usar posición del jugador actual
        return calculate_ball_approach_position(
            player_pos, ball_pos, target_goal, approach_distance, field=field
        )

    # Asegurar que la posición está dentro del campo
    if field is not None:
        target_pos = field.clamp(target_pos)
    else:
        target_pos = (int(target_pos[0]), int(target_pos[1]))

    return tuple(target_pos)


def calculate_pass_position(player_pos, ball_pos, teammate_pos, approach_distance=50,
                            field=None):
    """Calcula la posición óptima para hacer un pase a un compañero.

    Args:
        player_pos: Posición actual del jugador (x, y)
        ball_pos: Posición de la pelota (x, y)
        teammate_pos: Posición del compañero (x, y)
        approach_distance: Distancia de aproximación
        field: FieldGeometry para clamping de posiciones

    Returns:
        tuple: Posición objetivo para el pase (x, y)
    """
    ball_pos = np.array(ball_pos)
    teammate_pos = np.array(teammate_pos)

    # Vector desde compañero hacia pelota
    teammate_to_ball = ball_pos - teammate_pos
    distance = np.linalg.norm(teammate_to_ball)

    if distance > 1:
        teammate_to_ball = teammate_to_ball / distance
        # Posicionarse para hacer el pase en línea con el compañero
        target_pos = ball_pos + teammate_to_ball * approach_distance
    else:
        # Fallback a aproximación directa
        return calculate_ball_approach_position(
            player_pos, ball_pos, teammate_pos, approach_distance, field=field
        )

    # Asegurar que la posición está dentro del campo
    if field is not None:
        target_pos = field.clamp(target_pos)
    else:
        target_pos = (int(target_pos[0]), int(target_pos[1]))

    return tuple(target_pos)


def get_optimal_ball_approach_strategy(blackboard):
    """Determina la estrategia óptima de aproximación a la pelota basada en el contexto del juego.

    Args:
        blackboard: Contexto del juego

    Returns:
        str: Estrategia recomendada ('direct', 'defensive', 'offensive')
    """
    # Determinar estrategia basada en la zona de la pelota y el contexto
    if blackboard.zona_pelota < 0.4:  # Zona defensiva
        return "defensive"
    if blackboard.zona_pelota > 1.6:  # Zona ofensiva
        return "offensive"
    if 0.4 < blackboard.zona_pelota < 1.6:  # Zona neutral
        if blackboard.player.rol == ROL_ATACANTE:
            return "offensive"
        if blackboard.player.rol == ROL_DEFENSIVO:
            return "defensive"
    return None


def calculate_dribbling_path_positions(
    ball_pos, target_goal, num_waypoints=3, spacing=100, field=None
):
    """Calcula una secuencia de posiciones para driblar hacia la portería.

    Args:
        ball_pos: Posición de la pelota
        target_goal: Posición de la portería objetivo
        num_waypoints: Número de puntos intermedios
        spacing: Espaciado entre puntos
        field: FieldGeometry para clamping de posiciones

    Returns:
        list: Lista de posiciones [(x, y), ...] para el dribbling
    """
    ball_pos = np.array(ball_pos)
    target_goal = np.array(target_goal)

    # Vector desde pelota hacia portería
    direction = target_goal - ball_pos
    distance = np.linalg.norm(direction)

    if distance < 1:
        return [tuple(ball_pos)]

    direction = direction / distance

    # Crear waypoints
    waypoints = []
    for i in range(1, num_waypoints + 1):
        waypoint = ball_pos + direction * (spacing * i)

        # Asegurar que está dentro del campo
        if field is not None:
            waypoint = field.clamp(waypoint)
        else:
            waypoint = (int(waypoint[0]), int(waypoint[1]))

        waypoints.append(tuple(waypoint))

    return waypoints
