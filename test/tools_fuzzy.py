"""
Herramientas y utilidades para el sistema de lógica difusa.
Funciones auxiliares para extraer y procesar variables del sistema difuso.
"""

import numpy as np


def get_fuzzy_variables(fuzzy_manager, robot_id, players, ball):
    """
    Extrae todas las variables de entrada y salida del sistema de lógica difusa.

    Args:
        fuzzy_manager: Instancia de FuzzyRobotTeamManager
        robot_id: ID del robot enfocado
        players: Diccionario de jugadores {id: Player}
        ball: Instancia de la pelota

    Returns:
        dict: Diccionario con variables de entrada y salida
    """
    # Forzar recálculo del contexto para obtener valores actuales
    context_output = fuzzy_manager.evaluar_ms_logic_difusse()

    # Obtener las distancias a la pelota de todos los robots
    team_players = []
    rival_players = []

    # Separar jugadores por equipo
    for player_id, player in players.items():
        if player.team == fuzzy_manager.team:
            team_players.append(player)
        else:
            rival_players.append(player)

    # Calcular distancias manualmente para tener acceso a los valores
    ball_pos = (ball.x, ball.y)

    # Distancias de aliados
    dist_aliado1 = 0
    dist_aliado2 = 0
    if len(team_players) >= 1:
        dist_aliado1 = np.sqrt((team_players[0].x - ball_pos[0])**2 +
                               (team_players[0].y - ball_pos[1])**2)
    if len(team_players) >= 2:
        dist_aliado2 = np.sqrt((team_players[1].x - ball_pos[0])**2 +
                               (team_players[1].y - ball_pos[1])**2)

    # Distancias de rivales
    dist_rival1 = 0
    dist_rival2 = 0
    if len(rival_players) >= 1:
        dist_rival1 = np.sqrt((rival_players[0].x - ball_pos[0])**2 +
                              (rival_players[0].y - ball_pos[1])**2)
    if len(rival_players) >= 2:
        dist_rival2 = np.sqrt((rival_players[1].x - ball_pos[0])**2 +
                              (rival_players[1].y - ball_pos[1])**2)

    # Calcular orientaciones hacia la pelota
    def calc_orientation_to_ball(player):
        """Calcula el ángulo entre la orientación del robot y la dirección hacia la pelota"""
        dx = ball_pos[0] - player.x
        dy = ball_pos[1] - player.y
        angle_to_ball = np.arctan2(dy, dx)
        player_angle_rad = np.radians(player.angle)

        # Diferencia angular (normalizada entre 0 y pi)
        diff = abs(angle_to_ball - player_angle_rad)
        if diff > np.pi:
            diff = 2 * np.pi - diff

        return min(diff, np.pi)  # Limitado a pi radianes máximo

    # Orientaciones de aliados
    orient_aliado1 = 0
    orient_aliado2 = 0
    if len(team_players) >= 1:
        orient_aliado1 = calc_orientation_to_ball(team_players[0])
    if len(team_players) >= 2:
        orient_aliado2 = calc_orientation_to_ball(team_players[1])

    # Orientaciones de rivales
    orient_rival1 = 0
    orient_rival2 = 0
    if len(rival_players) >= 1:
        orient_rival1 = calc_orientation_to_ball(rival_players[0])
    if len(rival_players) >= 2:
        orient_rival2 = calc_orientation_to_ball(rival_players[1])

    # Obtener velocidad y dirección de la pelota
    velocity_data = ball.get_velocity() if hasattr(ball, 'get_velocity') else [0, 0, 0]
    v_x, v_y, speed = velocity_data

    # Calcular dirección según la lógica del FuzzyRobotTeamManager
    if abs(v_x) < 1 and abs(v_y) < 1:  # Pelota quieta
        direccion = 1.0  # neutral
    else:
        # Determinar dirección según lado del equipo y velocidad X
        if fuzzy_manager.side == "LEFT":
            if v_x < -2:  # Se mueve hacia izquierda (zona aliada)
                direccion = 0.0  # hacia_zona_aliada
            elif v_x > 2:  # Se mueve hacia derecha (zona rival)
                direccion = 2.0  # hacia_zona_rival
            else:
                direccion = 1.0  # neutral
        else:  # RIGHT
            if v_x > 2:  # Se mueve hacia derecha (zona aliada)
                direccion = 0.0  # hacia_zona_aliada
            elif v_x < -2:  # Se mueve hacia izquierda (zona rival)
                direccion = 2.0  # hacia_zona_rival
            else:
                direccion = 1.0  # neutral

    # Calcular ventaja de proximidad (simplificado)
    min_dist_aliados = min(dist_aliado1, dist_aliado2) if dist_aliado2 > 0 else dist_aliado1
    min_dist_rivales = min(dist_rival1, dist_rival2) if dist_rival2 > 0 else dist_rival1
    ventaja_proximidad = min_dist_rivales - min_dist_aliados

    return {
        'inputs': {
            'distancia_aliado1': dist_aliado1,
            'distancia_aliado2': dist_aliado2,
            'distancia_rival1': dist_rival1,
            'distancia_rival2': dist_rival2,
            'orientacion_aliado1': orient_aliado1,
            'orientacion_aliado2': orient_aliado2,
            'orientacion_rival1': orient_rival1,
            'orientacion_rival2': orient_rival2,
            'velocidad_pelota': speed,
            'direccion_movimiento': direccion,
            'posicion_x': ball_pos[0],
            'ventaja_proximidad': ventaja_proximidad,
            'posesion_pelota_result': context_output.get('estado_pelota', 0.0) if isinstance(context_output, dict) else context_output[0]
        },
        'outputs': {
            'posesion_pelota': context_output.get('estado_pelota', 0.0) if isinstance(context_output, dict) else context_output[0],
            'proximidad_equipo': context_output.get('equipo_cercano', 0.0) if isinstance(context_output, dict) else context_output[1],
            'zona_pelota': context_output.get('zona_pelota', 0.0) if isinstance(context_output, dict) else context_output[2]
        }
    }


def format_fuzzy_value(value, decimal_places=2):
    """
    Formatea un valor difuso con el número de decimales especificado.

    Args:
        value: Valor a formatear
        decimal_places: Número de decimales (default: 2)

    Returns:
        str: Valor formateado
    """
    return f"{value:.{decimal_places}f}"


def get_fuzzy_system_summary(fuzzy_vars):
    """
    Genera un resumen de los valores del sistema difuso.

    Args:
        fuzzy_vars: Diccionario de variables difusas

    Returns:
        dict: Resumen organizado por sistema
    """
    return {
        'posesion_system': {
            'inputs': {
                'distancias': [
                    fuzzy_vars['inputs']['distancia_aliado1'],
                    fuzzy_vars['inputs']['distancia_aliado2'],
                    fuzzy_vars['inputs']['distancia_rival1'],
                    fuzzy_vars['inputs']['distancia_rival2']
                ],
                'orientaciones': [
                    fuzzy_vars['inputs']['orientacion_aliado1'],
                    fuzzy_vars['inputs']['orientacion_aliado2'],
                    fuzzy_vars['inputs']['orientacion_rival1'],
                    fuzzy_vars['inputs']['orientacion_rival2']
                ],
                'pelota': {
                    'velocidad': fuzzy_vars['inputs']['velocidad_pelota'],
                    'direccion': fuzzy_vars['inputs']['direccion_movimiento']
                }
            },
            'output': fuzzy_vars['outputs']['posesion_pelota']
        },
        'proximidad_system': {
            'inputs': {
                'posesion_result': fuzzy_vars['inputs']['posesion_pelota_result'],
                'ventaja_proximidad': fuzzy_vars['inputs']['ventaja_proximidad'],
                'velocidad_pelota': fuzzy_vars['inputs']['velocidad_pelota']
            },
            'output': fuzzy_vars['outputs']['proximidad_equipo']
        },
        'zona_system': {
            'inputs': {
                'posicion_x': fuzzy_vars['inputs']['posicion_x'],
                'direccion_movimiento': fuzzy_vars['inputs']['direccion_movimiento']
            },
            'output': fuzzy_vars['outputs']['zona_pelota']
        }
    }
