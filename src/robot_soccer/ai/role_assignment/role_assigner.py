"""Sistema de asignación de roles para robots en fútbol.

Este módulo implementa estrategias para asignar dinámicamente roles
a los robots basándose en múltiples factores como posición, orientación,
distancia a la pelota y situación del juego.

NOTA: Este módulo usa los mismos métodos de cálculo de distancia y orientación
que proximity_calculator.py para garantizar consistencia, pero mantiene
factores específicos para asignación de roles (posesión, posición estratégica).
"""

import numpy as np
from robot_soccer.config import ROL_ATACANTE, ROL_DEFENSIVO

# Constantes para mejor legibilidad (alineadas con proximity_calculator.py)
DISTANCIA_MAX_NORMALIZACION = 1200  # Distancia máxima para normalización (igual que proximity_calculator)


class RoleAssigner:
    """Asignador de roles que determina dinámicamente qué robot debe ser atacante y cuál defensor.

    Esta clase evalúa múltiples factores para asignar roles óptimos a los robots,
    incluyendo distancia a la pelota, orientación, posesión actual y posición estratégica.
    Incluye mecanismos de estabilidad para evitar cambios de rol demasiado frecuentes.

    Attributes:
        team_players (list): Lista de jugadores del equipo.
        ball: Objeto pelota del juego.
        last_assignment (dict): Última asignación de roles por jugador.
        role_change_cooldown (int): Tiempo restante antes de permitir cambios de rol.
        min_time_between_changes (int): Tiempo mínimo entre cambios de rol en frames.
    """

    def __init__(self, team_players, ball):
        """Inicializa el asignador de roles.

        Args:
            team_players (list): Lista de jugadores del equipo.
            ball: Objeto pelota del juego.
        """
        self.team_players = team_players
        self.ball = ball

        # Historial para mantener estabilidad en los cambios de rol
        self.last_assignment = None
        self.role_change_cooldown = 0
        self.min_time_between_changes = 10  # frames/ticks

    def assign_roles(self):
        """Asigna roles a los jugadores basándose en la situación actual.

        Evalúa cada jugador usando múltiples criterios y asigna el rol de atacante
        al jugador con mayor puntuación. Incluye mecanismos de estabilidad para
        evitar cambios demasiado frecuentes.

        Returns:
            dict: Diccionario con asignación de roles por ID de jugador.
                 Formato: {player_id: rol}
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
        """Calcula una puntuación para determinar qué tan adecuado es un jugador para el rol de atacante.

        Usa los mismos métodos de cálculo de distancia y orientación que proximity_calculator.py,
        pero mantiene factores específicos para asignación de roles como posesión actual y
        posición estratégica.

        Args:
            player: Objeto jugador a evaluar.

        Returns:
            float: Puntuación para el rol de atacante (mayor es mejor).
                  Rango típico: 0.0 - 1.0
        """
        # Obtener factores clave
        distance = player.distance_to_ball(self.ball)
        orientation = player.angle_difference_ball(self.ball)

        # 1. Factor de distancia (0-1, donde 1 es mejor)
        # MÉTODO de proximity_calculator: normalización con 1200px
        dist_norm = max(0, 1.0 - (distance / DISTANCIA_MAX_NORMALIZACION))

        # 2. Factor de orientación (0-1, donde 1 es mejor)
        # MÉTODO de proximity_calculator: usar coseno con exponente para amplificar diferencias
        cos_orientacion = np.cos(orientation)
        if cos_orientacion > 0:
            ori_norm = cos_orientacion ** 1.5  # Amplifica diferencias
        else:
            ori_norm = 0.05  # Ángulos > 90° son muy malos

        # 3. Factor de posesión actual (específico para asignación de roles)
        possession_factor = 1.0 if player.has_ball() else 0.0

        # 4. Factor de posición estratégica (específico para asignación de roles)
        strategic_position = self._calculate_strategic_position(player)

        # 5. Factor de inercia (para mantener estabilidad)
        inertia_factor = 0.0
        if self.last_assignment and self.last_assignment.get(player.id) == ROL_ATACANTE:
            inertia_factor = 0.2

        # Combinación ponderada (manteniendo estructura original)
        score = (
            0.35 * dist_norm           # 35% - Distancia a la pelota
            + 0.25 * ori_norm          # 25% - Orientación hacia la pelota
            + 0.20 * possession_factor # 20% - Posesión actual
            + 0.10 * strategic_position# 10% - Posición estratégica
            + 0.10 * inertia_factor    # 10% - Estabilidad
        )

        return score

    def _calculate_strategic_position(self, player):
        """Evalúa la posición estratégica del jugador en relación a la pelota y la meta.

        Determina qué tan ventajosa es la posición actual del jugador para
        realizar acciones ofensivas considerando la geometría del campo.

        Args:
            player: Objeto jugador a evaluar.

        Returns:
            float: Factor de posición estratégica (0-1, donde 1 es mejor).
                  Valores más altos indican mejor posición para atacar.
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
