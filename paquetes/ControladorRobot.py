# import paquetes.rrt_star_smart as rrt
# import logging
import numpy as np
from config import *
import time


class RobotController:
    """
    Controlador que orquesta las estrategias de los robots en el campo.
    Interpreta decisiones del AdministradorEstados y define acciones estratégicas.
    """

    def __init__(self, team_players, opponents, ball):
        self.team_players = team_players  # Lista de objetos Player del equipo
        self.opponents = opponents        # Lista de objetos Player del equipo rival
        self.ball = ball                  # Referencia a la pelota
        # ===========================================================
        # Tiempo mínimo entre cambios de rol para evitar oscilaciones
        self.min_time_between_role_changes = 0.5  # segundos
        self.last_role_change_time = time.time()

        # Factor de ponderación para la orientación vs. distancia
        self.orientation_weight = 0.3

        # Umbral de distancia para considerar un cambio de rol
        self.role_change_threshold = 100  # unidades de distancia

    def _assign_dynamic_roles(self):
        """
        Asigna roles de atacante y defensor basados en la situación actual.

        Returns:
            tuple: (jugador_atacante, jugador_defensor)
        """
        # Verificar si ha pasado suficiente tiempo desde el último cambio
        current_time = time.time()
        if current_time - self.last_role_change_time < self.min_time_between_role_changes:
            # Mantener roles actuales
            attacker = next((p for p in self.team_players if p.rol == 1), self.team_players[0])
            defender = next((p for p in self.team_players if p.rol == 0), self.team_players[1])
            return attacker, defender

        # Calcular puntuación para cada jugador
        scores = []
        for player in self.team_players:
            # Factores principales: distancia y orientación hacia la pelota
            distance = player.distance_to_ball(self.ball)
            orientation = player.angle_difference_ball(self.ball)

            # Normalizar la orientación (0-1, donde 0 es perfectamente orientado)
            orientation_normalized = min(1.0, orientation / np.pi)

            # Calcular puntuación (mayor es mejor)
            # Fórmula: 1000 - distancia - (peso_orientación * orientación_normalizada * 100)
            # Esto favorece al jugador más cercano con mejor orientación
            score = 1000 - distance - (self.orientation_weight * orientation_normalized * 100)
            scores.append((player, score))

        # Ordenar por puntuación descendente
        scores.sort(key=lambda x: x[1], reverse=True)

        # Verificar si la diferencia justifica un cambio de roles
        current_attacker = next((p for p in self.team_players if p.rol == 1), None)

        # Si ya hay un atacante, verificar si debería cambiarse
        if current_attacker and current_attacker != scores[0][0]:
            # Obtener puntuación del atacante actual
            current_score = next(score for player, score in scores if player == current_attacker)
            best_score = scores[0][1]

            # Solo cambiar si hay una diferencia significativa
            if best_score - current_score < self.role_change_threshold:
                # No cambiar roles, mantener al atacante actual
                attacker = current_attacker
                defender = next(player for player in self.team_players if player != attacker)

        # Asignar nuevos roles
        attacker = scores[0][0]
        defender = scores[1][0]

        # Actualizar roles en los objetos jugador
        attacker.set_rol(ROL_ATACANTE)
        defender.set_rol(ROL_DEFENSIVO)

        # Registrar el tiempo del cambio
        self.last_role_change_time = current_time

    def execute_team_strategy(self, attacker_state, defender_state):
        """
        Método principal que asigna acciones a los jugadores basándose
        en los estados determinados por el AdministradorEstados.
        """
        self._assign_dynamic_roles()
        # Identificar jugadores por rol
        attacker = next(p for p in self.team_players if p.rol == 1)
        defender = next(p for p in self.team_players if p.rol == 0)

        # Asignar acciones según estados
        self._execute_attacker_action(attacker, attacker_state)
        self._execute_defender_action(defender, defender_state)

    def _execute_attacker_action(self, attacker, state):
        """Determina y ejecuta la acción del atacante según su estado"""
        if state <= 0.2:
            self.presionar(attacker)
        elif 0.2 < state <= 0.5:
            self.interceptar(attacker)
        elif 0.5 < state <= 0.8:
            self.capturar_pelota(attacker)
        elif 0.8 < state <= 1.1:
            self.adelantar_lanzar(attacker)

    def _execute_defender_action(self, defender, state):
        """Determina y ejecuta la acción del defensor según su estado"""
        if state <= 0.2:
            self.preparar_pase(defender)
        elif 0.2 < state <= 0.5:
            self.marcar(defender)
        elif 0.5 < state <= 0.8:
            self.posicion_defensiva(defender)
        elif 0.8 < state <= 1.1:
            self.bloquear_tiro(defender)

    # ======================================================
    # ================= ACCIONES OFENSIVAS =================
    # ======================================================

    def presionar(self, player):
        """Presiona al jugador rival con la pelota"""
        # Identificar rival más cercano a la pelota
        closest_opponent = min(self.opponents,
                               key=lambda op: op.distance_to_ball(self.ball))

        # Calcular posición estratégica para presionar
        # (entre el rival y nuestra portería)
        our_goal_pos = [0, ALTO_CAMPO / 2] if player.team == 'red' else [ANCHO_CAMPO, ALTO_CAMPO / 2]
        opponent_pos = closest_opponent.get_position()

        # Vector desde rival a portería (normalizado y escalado)
        to_goal_vector = np.array(our_goal_pos) - opponent_pos
        if np.linalg.norm(to_goal_vector) > 0:
            to_goal_vector = to_goal_vector / np.linalg.norm(to_goal_vector) * 60

        # Posición final de presión
        pressing_position = opponent_pos + to_goal_vector

        # Ordenar al jugador moverse a esa posición
        player.move_to_position(pressing_position, speed_factor=1.3)

    def interceptar(self, player):
        """Intercepta la trayectoria de la pelota"""
        # Determinar tiempo de predicción según velocidad de la pelota
        ball_speed = np.linalg.norm([getattr(self.ball, 'dx', 0),
                                     getattr(self.ball, 'dy', 0)])
        prediction_time = min(1.0, 0.3 + ball_speed * 0.05)

        # Delegar la intercepción al jugador
        player.intercept_ball(self.ball, prediction_time)

    def capturar_pelota(self, player):
        """Captura la pelota de forma controlada"""
        player.capture_ball(self.ball)

    def adelantar_lanzar(self, player):
        """Avanza con la pelota y busca oportunidad de lanzamiento"""
        # Ubicación de la portería rival
        opponent_goal_pos = [ANCHO_CAMPO,
                             ALTO_CAMPO / 2] if player.team == 'red' else [0, ALTO_CAMPO / 2]

        if player.has_ball():
            # Determinar si estamos en posición de tiro
            dist_to_goal = np.linalg.norm(player.get_position() - np.array(opponent_goal_pos))

            if dist_to_goal < 300:  # Distancia adecuada para disparar
                # Calcular punto óptimo para el tiro
                shooting_target = self._calculate_shooting_target(player, opponent_goal_pos)
                player.kick_ball(shooting_target, power=0.9)
            else:
                # Avanzar hacia la portería evitando obstáculos
                advancing_path = self._calculate_advancing_path(player, opponent_goal_pos)
                player.move_with_ball(advancing_path)
        else:
            # Si no tiene la pelota, intentar capturarla
            player.capture_ball(self.ball)

    # ======================================================
    # ================= ACCIONES DEFENSIVAS ================
    # ======================================================

    def preparar_pase(self, player):
        """Busca posición para recibir un pase del compañero"""
        # Identificar compañero
        teammate = next(p for p in self.team_players if p != player)

        # Determinar si el compañero tiene la pelota
        if teammate.has_ball():
            # Calcular posición óptima para recibir pase
            ball_pos = self.ball.get_position()
            opponent_goal_pos = [ANCHO_CAMPO,
                                 ALTO_CAMPO / 2] if player.team == 'red' else [0, ALTO_CAMPO / 2]

            # Vector hacia la portería y perpendicular
            forward_vector = np.array(opponent_goal_pos) - ball_pos
            if np.linalg.norm(forward_vector) > 0:
                forward_vector = forward_vector / np.linalg.norm(forward_vector) * 200

            # Vector perpendicular para dar opción de pase lateral
            perp_vector = np.array([-forward_vector[1], forward_vector[0]]) * 0.7

            # Posición para recibir pase
            passing_position = ball_pos + forward_vector + perp_vector

            # Moverse a esa posición
            player.move_to_position(passing_position)
        else:
            # Si el compañero no tiene la pelota, posición defensiva
            self.posicion_defensiva(player)

    def marcar(self, player):
        """Marca al jugador rival más peligroso"""
        # Identificar rival más peligroso
        dangerous_opponent = self._identify_most_dangerous_opponent()

        # Calcular posición para marcar (entre el rival y nuestra portería)
        our_goal_pos = [0, ALTO_CAMPO / 2] if player.team == 'red' else [ANCHO_CAMPO, ALTO_CAMPO / 2]
        opponent_pos = dangerous_opponent.get_position()

        # Vector desde rival a portería
        to_goal_vector = np.array(our_goal_pos) - opponent_pos
        if np.linalg.norm(to_goal_vector) > 0:
            to_goal_vector = to_goal_vector / np.linalg.norm(to_goal_vector) * 50

        # Posición de marcaje
        marking_position = opponent_pos + to_goal_vector

        # Moverse a esa posición
        player.move_to_position(marking_position, speed_factor=1.2)

    def posicion_defensiva(self, player):
        """Mantiene una posición defensiva óptima"""
        # Entre la pelota y nuestra portería
        our_goal_pos = [0, ALTO_CAMPO / 2] if player.team == 'red' else [ANCHO_CAMPO, ALTO_CAMPO / 2]
        ball_pos = self.ball.get_position()

        # Vector desde pelota a portería
        to_goal_vector = np.array(our_goal_pos) - ball_pos

        if np.linalg.norm(to_goal_vector) > 0:
            to_goal_vector = to_goal_vector / np.linalg.norm(to_goal_vector)

        # Distancia variable según la posición de la pelota
        distance_factor = min(np.linalg.norm(ball_pos - np.array(our_goal_pos)) * 0.6, 300)

        # Posición defensiva
        defensive_position = ball_pos + to_goal_vector * distance_factor

        # Moverse a esa posición
        player.move_to_position(defensive_position)

    def bloquear_tiro(self, player):
        """Bloquea posible tiro a portería"""
        # Identificar rival con posesión o más cercano a la pelota
        ball_possessor = min(self.opponents, key=lambda op: op.distance_to_ball(self.ball))

        # Calcular línea de tiro probable
        our_goal_pos = [0, ALTO_CAMPO / 2] if player.team == 'red' else [ANCHO_CAMPO, ALTO_CAMPO / 2]

        # Vector de tiro
        shoot_vector = np.array(our_goal_pos) - ball_possessor.get_position()
        if np.linalg.norm(shoot_vector) > 0:
            shoot_vector = shoot_vector / np.linalg.norm(shoot_vector)

        # Punto de intercepción
        intercept_point = ball_possessor.get_position() + shoot_vector * 70

        # Moverse a posición de bloqueo con alta prioridad
        player.move_to_position(intercept_point, speed_factor=1.5)

    # =====================================================
    # ================= MÉTODOS AUXILIARES ================
    # =====================================================

    def _calculate_shooting_target(self, player, goal_pos):
        """Calcula el mejor punto para un tiro a portería"""
        # Versión básica: apuntar cerca de las esquinas
        goal_width = 200
        side_offset = goal_width * 0.4 * (1 if np.random.random() > 0.5 else -1)
        return [goal_pos[0], goal_pos[1] + side_offset]

    def _calculate_advancing_path(self, player, goal_pos):
        """Calcula la ruta óptima para avanzar hacia la portería rival"""
        # Usando el planificador del jugador para generar la ruta
        player.plan_path_to(goal_pos, self.opponents)
        return player.current_path or [player.get_position(), goal_pos]

    def _identify_most_dangerous_opponent(self):
        """Identifica al rival más peligroso"""
        # Rival más cercano a nuestra portería es el más peligroso
        our_goal = [0, ALTO_CAMPO / 2] if self.team_players[0].team == 'red' else [ANCHO_CAMPO, ALTO_CAMPO / 2]
        return min(self.opponents, key=lambda op: np.linalg.norm(op.get_position() - np.array(our_goal)))
