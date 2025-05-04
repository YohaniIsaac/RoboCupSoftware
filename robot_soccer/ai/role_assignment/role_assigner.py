"""
Sistema de asignación de roles para robots en fútbol.

Este módulo implementa estrategias para asignar dinámicamente roles
a los robots basándose en múltiples factores como posición, orientación,
distancia a la pelota y situación del juego.
"""

import numpy as np
from robot_soccer.config import *


class RoleAssigner:
    """
    Asignador de roles que determina dinámicamente qué robot
    debe ser atacante y cuál defensor.
    """

    def __init__(self, team_players, ball):
        """
        Inicializa el asignador de roles.

        Args:
            team_players: Lista de jugadores del equipo
            ball: Objeto pelota
        """
        self.team_players = team_players
        self.ball = ball

        # Historial para mantener estabilidad en los cambios de rol
        self.last_assignment = None
        self.role_change_cooldown = 0
        self.min_time_between_changes = 10  # frames/ticks

    def assign_roles(self):
        """
        Asigna roles a los jugadores basándose en la situación actual.
        Retorna información sobre la asignación para debugging.
        """
        # Decrementar cooldown
        if self.role_change_cooldown > 0:
            self.role_change_cooldown -= 1
            # Si estamos en cooldown y ya hay roles asignados, mantenerlos
            if self.last_assignment:
                return self.last_assignment

        # Calcular puntuación para cada jugador
        scores = []
        for player in self.team_players:
            score = self._calculate_player_score(player)
            scores.append((player, score))

        # Ordenar por puntuación (mayor primero)
        scores.sort(key=lambda x: x[1], reverse=True)

        # Determinar nueva asignación
        new_assignment = {}
        attacker_id = scores[0][0].id  # El mejor puntuado será atacante

        # Asignar roles
        for player in self.team_players:
            if player.id == attacker_id:
                player.set_rol(ROL_ATACANTE)
                new_assignment[player.id] = ROL_ATACANTE
            else:
                player.set_rol(ROL_DEFENSIVO)
                new_assignment[player.id] = ROL_DEFENSIVO

        # Verificar si hubo cambio en la asignación
        if self.last_assignment and self.last_assignment != new_assignment:
            self.role_change_cooldown = self.min_time_between_changes

        # Guardar la asignación actual
        self.last_assignment = new_assignment

        return new_assignment

    def _calculate_player_score(self, player):
        """
        Calcula una puntuación para determinar qué tan adecuado es un jugador
        para el rol de atacante.

        Args:
            player: Objeto jugador a evaluar

        Returns:
            float: Puntuación para el rol de atacante (mayor es mejor)
        """
        # Obtener factores clave
        distance = player.distance_to_ball(self.ball)
        orientation = player.angle_difference_ball(self.ball)

        # Normalización (0-1, donde 1 es mejor)
        dist_norm = max(0, 1.0 - (distance / 500))  # 500 como distancia máxima
        ori_norm = max(0, 1.0 - (orientation / np.pi))  # π como desorientación máxima

        # Factor de posesión actual
        possession_factor = 1.0 if player.has_ball() else 0.0

        # Factor de inercia (para mantener estabilidad)
        inertia_factor = 0.0
        if self.last_assignment and self.last_assignment.get(player.id) == ROL_ATACANTE:
            inertia_factor = 0.2

        # Calcular posición estratégica
        strategic_position = self._calculate_strategic_position(player)

        # Combinación ponderada
        score = (
                0.35 * dist_norm +  # Distancia a la pelota
                0.25 * ori_norm +  # Orientación hacia la pelota
                0.20 * possession_factor +  # Posesión actual
                0.10 * strategic_position +  # Posición estratégica
                0.10 * inertia_factor  # Estabilidad
        )

        return score

    def _calculate_strategic_position(self, player):
        """
        Evalúa la posición estratégica del jugador en relación a la pelota y la meta.

        Args:
            player: Objeto jugador

        Returns:
            float: Factor de posición estratégica (0-1, donde 1 es mejor)
        """
        # Implementación básica - puede expandirse con geometría más avanzada
        player_pos = player.get_position()
        ball_pos = self.ball.get_position()

        # Vector desde la pelota hacia el jugador
        to_player = player_pos - ball_pos

        # Normalizar si es posible
        if np.linalg.norm(to_player) > 0:
            to_player = to_player / np.linalg.norm(to_player)

        # Consideramos mejor estar delante de la pelota (en dirección a la meta)
        # Esto es muy simplificado y deberías adaptarlo según la geometría de tu campo
        forward_factor = to_player[0]  # Componente x

        # Normalizar a rango 0-1
        return (forward_factor + 1) / 2
