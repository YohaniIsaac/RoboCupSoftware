import numpy as np


def calcular_posicion_estrategica(player_pos, ball_pos, goal_pos):
    """
    Calcula qué tan estratégica es la posición de un jugador para atacar.
    Considera la posición relativa entre el jugador, la pelota y la portería rival.

    Returns:
        float: Valor entre 0 y 1, donde 1 es la posición más estratégica.
    """

    # Distancia jugador-pelota
    dist_to_ball = np.linalg.norm(player_pos - ball_pos)

    # Distancia pelota-portería
    ball_to_goal = np.linalg.norm(ball_pos - np.array(goal_pos))

    # Distancia jugador-portería
    dist_to_goal = np.linalg.norm(player_pos - np.array(goal_pos))

    # Calcular ángulo entre jugador-pelota-portería
    # Un ángulo pequeño significa que el jugador está bien posicionado
    vec1 = player_pos - ball_pos
    vec2 = np.array(goal_pos) - ball_pos

    if np.linalg.norm(vec1) > 0 and np.linalg.norm(vec2) > 0:
        cos_ang = np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))
        angle = np.arccos(np.clip(cos_ang, -1.0, 1.0))
    else:
        angle = np.pi  # ángulo máximo si algún vector es cero

    # Normalizar valores
    norm_dist_ball = 1.0 - min(1.0, dist_to_ball / 500)  # 500 es una distancia máxima de referencia
    norm_angle = 1.0 - min(1.0, angle / np.pi)
    norm_dist_goal = 1.0 - min(1.0, dist_to_goal / 1500)  # Considerando el tamaño del campo

    # Ponderar factores (puedes ajustar estos pesos)
    score = 0.5 * norm_dist_ball + 0.3 * norm_angle + 0.2 * norm_dist_goal

    return score
