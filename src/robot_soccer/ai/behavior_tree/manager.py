"""Gestor de comportamientos para los robots de fútbol.

Este módulo implementa el gestor principal que integra los árboles de comportamiento
con el sistema de lógica difusa para proporcionar un control completo de los robots.
"""

import logging
# from .base import NodeStatus
from robot_soccer.config import LADO_IZQUIERDO, LADO_DERECHO, EQUIPO_ROJO, ROL_ATACANTE, FIELD_SIM
from robot_soccer.controllers.robot_command_manager import RobotCommandManager
from .soccer_behaviors import (Blackboard, create_attacker_tree, create_defender_tree)

log = logging.getLogger(__name__)


class BehaviorManager:
    """Gestor de comportamientos que coordina la toma de decisiones para los robots.

    Integra el sistema de lógica difusa con los árboles de comportamiento.
    """

    def __init__(
            self, players, ball, team='red', use_real_robots=False, serial_port='/dev/ttyUSB0',
            field=None
    ):
        """Inicializa el gestor de equipos de forma dinámica.

        Args:
            players (list): Lista de todos los jugadores (objetos Player)
            ball: Objeto pelota
            team (str): Equipo que gestionará este manager ('red' o 'blue')
            use_real_robots: Si True, utiliza comunicación real con robots
            serial_port: Puerto serial para comunicación con Arduino
            field: FieldGeometry con geometría del campo. Defaults to FIELD_SIM.
        """
        self.team = team
        self.field = field if field is not None else FIELD_SIM
        self.side = LADO_IZQUIERDO if team == EQUIPO_ROJO else LADO_DERECHO
        self.ball = ball
        # Filtrar jugadores por equipo
        self.team_players = [p for p in players if p.team == team]
        self.opponents = [p for p in players if p.team != team]

        # Crear gestor de comandos compartido
        self.command_manager = RobotCommandManager(
            self.team_players,
            self.ball,
            use_real_robots=use_real_robots,
            port=serial_port
        )

        # Crear blackboards para cada jugador
        self.blackboards = {}
        for player in self.team_players:
            self.blackboards[player.id] = Blackboard(
                player, ball, self.team_players, self.opponents, team,
                field=self.field
            )
            # Añadir el gestor de comandos a cada blackboard
            self.blackboards[player.id].command_manager = self.command_manager

        # Crear árboles de comportamiento
        self.attacker_tree = create_attacker_tree()
        self.defender_tree = create_defender_tree()

    def shutdown(self):
        """Libera recursos y cierra comunicaciones."""
        self.command_manager.shutdown()

    def update_game_context(self, context_info):
        """Actualiza el contexto del juego en todos los blackboards.

        Args:
            context_info: Tupla (posesion, proximidad, zona) con los valores de lógica difusa
        """
        posesion, proximidad, zona = context_info

        for blackboard in self.blackboards.values():
            blackboard.update_game_context(posesion, proximidad, zona)

    def update(self):
        """Ejecuta una iteración del sistema de comportamientos para todos los jugadores."""
        for player in self.team_players:
            blackboard = self.blackboards[player.id]

            # Ejecutar el árbol correspondiente según el rol del jugador
            if player.rol == ROL_ATACANTE:
                status = self.attacker_tree.tick(blackboard)
                log.debug("Árbol atacante para jugador %s completado con estado %s",
                          player.id, status
                )
            else:  # ROL_DEFENSIVO
                status = self.defender_tree.tick(blackboard)
                log.debug(
                    "Árbol defensor para jugador %s completado con estado %s", player.id, status
                )

    def get_current_state(self, player_id):
        """Obtiene el estado actual del comportamiento para un jugador específico.

        Args:
            player_id: ID del jugador

        Returns:
            dict: Diccionario con información sobre el estado actual
        """
        # Buscar el jugador
        player = next((p for p in self.team_players if p.id == player_id), None)
        if not player:
            return {"error": "Jugador no encontrado"}

        # Obtener el blackboard del jugador
        blackboard = self.blackboards.get(player_id)
        if not blackboard:
            return {"error": "Blackboard no encontrado"}

        # Determinar el árbol activo
        # active_tree = self.attacker_tree if player.rol == ROL_ATACANTE else self.defender_tree

        # Construir información de estado
        state_info = {
            "rol": "Atacante" if player.rol == ROL_ATACANTE else "Defensor",
            "posesion_pelota": self._translate_possession(blackboard.posesion_pelota),
            "proximidad_equipo": self._translate_proximity(blackboard.proximidad_equipo),
            "zona_pelota": self._translate_zone(blackboard.zona_pelota),
            "tiene_pelota": player.has_ball(),
            "distancia_pelota": player.distance_to_ball(self.ball),
            "angulo_pelota": player.angle_difference_ball(self.ball),
            "last_action": blackboard.last_action
        }

        return state_info

    def _translate_possession(self, value):
        """Traduce el valor numérico de posesión a texto descriptivo."""
        if value < 0.3:
            return "Posesión aliada"
        if value > 0.7:
            return "Posesión rival"
        return "Pelota libre"

    def _translate_proximity(self, value):
        """Traduce el valor numérico de proximidad a texto descriptivo."""
        if value < 0.8:
            return "Cerca de aliados"
        if value > 1.2:
            return "Cerca de rivales"
        return "Neutral"

    def _translate_zone(self, value):
        """Traduce el valor numérico de zona a texto descriptivo."""
        if value < 0.4:
            return "Zona defensiva"
        if value > 1.6:
            return "Zona ofensiva"
        return "Zona neutral"
