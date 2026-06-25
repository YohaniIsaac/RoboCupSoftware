"""Gestor de comportamientos para los robots de fútbol.

Este módulo implementa el gestor principal que integra los árboles de comportamiento
con el sistema de lógica difusa para proporcionar un control completo de los robots.
"""

import time
import logging
from robot_soccer.config import (
    LADO_IZQUIERDO, LADO_DERECHO, EQUIPO_ROJO,
    ROL_ATACANTE, FIELD_SIM,
    BT_ROLE_SWITCH_HYSTERESIS,
    BT_ROLE_COMMITMENT_RATIO,
    BT_ROLE_SWITCH_COOLDOWN_S,
)
from robot_soccer.controllers.robot_command_manager import RobotCommandManager
from .soccer_behaviors import (Blackboard, create_attacker_tree, create_defender_tree)

log = logging.getLogger(__name__)


class BehaviorManager:
    """Gestor de comportamientos que coordina la toma de decisiones para los robots.

    Integra el sistema de lógica difusa con los árboles de comportamiento.
    """

    def __init__(
            self, players, ball, team='red', use_real_robots=False, serial_port='/dev/ttyUSB0',
            field=None, rf_controller=None
    ):
        """Inicializa el gestor de equipos de forma dinámica.

        Args:
            players (list): Lista de todos los jugadores (objetos Player)
            ball: Objeto pelota
            team (str): Equipo que gestionará este manager ('red' o 'blue')
            use_real_robots: Si True, utiliza comunicación real con robots
            serial_port: Puerto serial para comunicación con Arduino
            field: FieldGeometry con geometría del campo. Defaults to FIELD_SIM.
            rf_controller: RFController externo compartido para modo 2v2.
        """
        self.team = team
        self.field = field if field is not None else FIELD_SIM
        self.side = LADO_IZQUIERDO if team == EQUIPO_ROJO else LADO_DERECHO
        self.ball = ball
        self.team_players = [p for p in players if p.team == team]
        self.opponents = [p for p in players if p.team != team]

        self.command_manager = RobotCommandManager(
            self.team_players,
            self.ball,
            use_real_robots=use_real_robots,
            port=serial_port,
            rf_controller=rf_controller,
        )

        self.blackboards = {}
        for player in self.team_players:
            self.blackboards[player.id] = Blackboard(
                player, ball, self.team_players, self.opponents, team,
                field=self.field
            )
            self.blackboards[player.id].command_manager = self.command_manager

        # Árbol por jugador: cada robot tiene su propia instancia para que
        # StatefulActionNode (_started / status en el nodo) no comparta estado
        # entre robots que cambian de rol. Al hacer switch, se recrea el árbol.
        self.trees = {
            player.id: (create_attacker_tree() if player.rol == ROL_ATACANTE
                        else create_defender_tree())
            for player in self.team_players
        }

        # Arbitraje dinámico de roles
        self._role_last_switch_t: float = 0.0

    def shutdown(self):
        """Libera recursos y cierra comunicaciones."""
        self.command_manager.shutdown()

    def update_game_context(self, context_info):
        """Actualiza el contexto del juego en todos los blackboards."""
        posesion, proximidad, zona = context_info
        for blackboard in self.blackboards.values():
            blackboard.update_game_context(posesion, proximidad, zona)

    def update(self):
        """Ejecuta una iteración del sistema de comportamientos para todos los jugadores."""
        self._update_roles()
        for player in self.team_players:
            blackboard = self.blackboards[player.id]
            status = self.trees[player.id].tick(blackboard)
            log.debug("Árbol %s para jugador %s → %s",
                      "atacante" if player.rol == ROL_ATACANTE else "defensor",
                      player.id, status)

    def _update_roles(self):
        """Reasigna roles con histéresis. Llamado cada tick del BT (~10Hz).

        Reglas en orden de prioridad:
        1. Posesión: si un robot tiene la pelota, es ATACANTE inmediato (sin cooldown).
        2. Cooldown: si pasó menos de BT_ROLE_SWITCH_COOLDOWN_S desde el último switch, no cambiar.
        3. Commitment: si el atacante está a <BT_ROLE_COMMITMENT_RATIO del campo de la pelota,
           está en su secuencia de aproximación — no interrumpir.
        4. Histéresis: solo cambiar si el defensor es ×BT_ROLE_SWITCH_HYSTERESIS más cercano
           que el atacante actual.
        """
        if len(self.team_players) < 2:
            return

        now = time.time()

        # Regla 1: posesión — override inmediato sin esperar cooldown
        for p in self.team_players:
            if p.has_ball():
                if p.rol != ROL_ATACANTE:
                    log.info("ROL SWITCH (posesion): R%d tiene pelota → ATACANTE", p.id)
                    self._do_role_switch(p.id, now)
                return

        # Regla 2: cooldown general
        if now - self._role_last_switch_t < BT_ROLE_SWITCH_COOLDOWN_S:
            return

        current_attacker = next(
            (p for p in self.team_players if p.rol == ROL_ATACANTE), None
        )
        if current_attacker is None:
            self._do_role_switch(self.team_players[0].id, now)
            return

        attacker_dist = float(current_attacker.distance_to_ball(self.ball))

        # Regla 3: commitment zone — atacante en secuencia de aproximación
        commitment_px = self.field.ratio_to_px(BT_ROLE_COMMITMENT_RATIO)
        if attacker_dist < commitment_px:
            return

        # Regla 4: histéresis — buscar defensor significativamente más cercano
        for p in self.team_players:
            if p.rol == ROL_ATACANTE:
                continue
            other_dist = float(p.distance_to_ball(self.ball))
            if other_dist * BT_ROLE_SWITCH_HYSTERESIS < attacker_dist:
                log.info(
                    "ROL SWITCH: R%d → ATACANTE (%.0fpx) | R%d → DEFENSOR (%.0fpx) "
                    "| ratio=%.2f > histeresis=%.1f",
                    p.id, other_dist, current_attacker.id, attacker_dist,
                    attacker_dist / max(other_dist, 1.0), BT_ROLE_SWITCH_HYSTERESIS,
                )
                self._do_role_switch(p.id, now)
                return

    def _do_role_switch(self, new_attacker_id: int, now: float):
        """Aplica el cambio de rol y recrea los árboles afectados con estado limpio."""
        for p in self.team_players:
            if p.id == new_attacker_id:
                p.set_rol(ROL_ATACANTE)
                self.trees[p.id] = create_attacker_tree()
            else:
                from robot_soccer.config import ROL_DEFENSIVO
                p.set_rol(ROL_DEFENSIVO)
                self.trees[p.id] = create_defender_tree()
            # Reiniciar el cronómetro de posesión: un atacante recién promovido empieza
            # de cero (evita que un valor obsoleto dispare el tope al instante).
            bb = self.blackboards.get(p.id)
            if bb is not None:
                bb._possession_start_time = None
        self._role_last_switch_t = now

    def get_current_state(self, player_id):
        """Obtiene el estado actual del comportamiento para un jugador específico."""
        player = next((p for p in self.team_players if p.id == player_id), None)
        if not player:
            return {"error": "Jugador no encontrado"}

        blackboard = self.blackboards.get(player_id)
        if not blackboard:
            return {"error": "Blackboard no encontrado"}

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
        if value < 0.3:
            return "Posesión aliada"
        if value > 0.7:
            return "Posesión rival"
        return "Pelota libre"

    def _translate_proximity(self, value):
        if value < 0.8:
            return "Cerca de aliados"
        if value > 1.2:
            return "Cerca de rivales"
        return "Neutral"

    def _translate_zone(self, value):
        if value < 0.4:
            return "Zona defensiva"
        if value > 1.6:
            return "Zona ofensiva"
        return "Zona neutral"
