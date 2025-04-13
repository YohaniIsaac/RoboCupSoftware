import paquetes.tools as tools
import numpy as np
import time
from config import *


class RobotStateMachine:
    """
    Gestiona el estado de un equipo de robots en un partido.

    La clase controla el estado de la pelota (si está libre o en posesión),
    determina la proximidad de la pelota (aliado o rival), y define la zona
    del campo donde se encuentra la pelota en relación con el equipo (defensa, centro, ataque).

    Args:
        one         (int): Identificador del primer robot del equipo
        two         (int): Identificador del segundo robot del equipo
        team        (str): Equipo del robot, 'red' (por defecto) para la izquierda o 'blue' para la derecha
        umbral_cambio_roles     (float):    Umbral de distancia (en píxeles) que debe existir para que haya un cambio
                                            en los roles de los jugadores

    Attributes:
        state_ball              (str): Estado de la pelota ('LIBRE' o 'POSESION')
        state_ball_proximity    (str): Proximidad de la pelota ('ALIADA' o 'RIVAL')
        field_area              (str): Zona del campo donde está la pelota ('DEFENSA', 'CENTRO', 'ATAQUE')

        player_close_to_ball    (int):  ID del robot del equipo más cercano a la pelota
        information_robots      (list): Información de los robots, ordenados según la distancia que los separa con la
                                        pelota. (distancia con la pelota, id, centro del robot, ángulo del robot)

        state_robot_defense     (None): Estado del robot de defensa
        state_robot_attack      (None): Estado del robot de ataque

        team                    (str):  Zona asociada al equipo ('LEFT' o 'RIGHT')
        ids_teams               (list): Identificadores de los robots del equipo

        last_role_change_time       (float): Alamacena el tiempo del último cambio de roles que hubo
        UMBRAL_CAMBIO_ROLES         (float): Umbral de distancia para que haya un cambio de roles

        last_zone_change_time   (float): Almacena el tiempo del último cambio de zona de la pelota que hubo
        last_field_area         (str):   Almacena el último cambio de zona de la pelota que hubo, esto para compararlo
                                         con el actual
    """

    def __init__(self, one, two, team='red', umbral_cambio_roles=10):
        self.state_ball = ESTADO_PELOTA_LIBRE   # estado de la pelota LIBRE, POSESION
        self.state_ball_proximity = None        # ALIADO, RIVAL
        self.field_area = None                  # DEFENSIVA, NEUTRAL, OFENSIVA

        self.player_close_to_ball = None        # ID del robot más cercano a la pelota
        self.information_robots = None          # Información de los robots ordenados segun las distancia con la pelota
                                                # (disntancia con la pelota, id, centro del robot, angulo del robot)

        # Roles de los robots
        self.state_robot_defense = None
        self.state_robot_attack = None

        # Configuración del equipo
        self.team = LADO_IZQUIERDO if team == EQUIPO_ROJO else LADO_DERECHO
        self.ids_teams = [one, two]

        # Tiempo mínimo para que ocurra un cambio en los roles de los jugadores
        self.last_role_change_time = time.time()
        self.UMBRAL_CAMBIO_ROLES = umbral_cambio_roles  # Umbral de distancia para cambiar roles

        # Filtro para cambios de zona del campo con un tiempo mínim, antes de volver a cambiar de zona la pelota
        self.last_zone_change_time = time.time()
        self.last_field_area = None

    def update_state(self, ball, coords_players):
        """
        Actualiza el estado de la máquina basado en la posición de la pelota y los jugadores.
        Determina:
        1. Si la pelota está libre o en posesión.
        2. La proximidad de la pelota (aliado o rival).
        3. La zona del campo donde se encuentra la pelota (defensa, centro, ataque).

        Args:
            ball (class): Coordenadas [x, y] de la pelota
            coords_players (dict{class}): Lista de diccionarios con los datos de los jugadores.
                Cada diccionario debe tener las claves:
                    - 'id' (int): Identificador del jugador.
                    - 'x' (float): Coordenada x del jugador.
                    - 'y' (float): Coordenada y del jugador.
                    - 'angulo' (float): Ángulo de orientación del jugador en grados.
        """

        # Calcula si alguien tiene posesión de la pelota y entrega la información de todos los robots.
        possession, information_all_robots = tools.position_ball(coords_players, ball, self.ids_teams)
        if len(information_all_robots) < 2:
            return print("Error! no hay suficientes robots para asignar roles")

        # Determinar si la pelota la posee alguno de los dos equipos
        self.state_ball = ESTADO_PELOTA_POSESION if possession else ESTADO_PELOTA_LIBRE

        # ID del jugador que tiene la pelota más cercana (idependiente del equipo)
        id_player_closest2ball = information_all_robots[0][1]

        # Determinar si la pelota está más cerca de un rival o un aliado
        self.state_ball_proximity = PROXIMIDAD_ALIADA if id_player_closest2ball in self.ids_teams else PROXIMIDAD_RIVAL

        # Filtrar ID's del equipo
        self.information_robots = [d for d in information_all_robots if d[1] in self.ids_teams]

        # Determinar la zona del campo
        new_field_area = tools.ball_zone(ball, self.team)

        # Solo cambiar la zona si ha pasado el tiempo mínimo
        if new_field_area != self.last_field_area:
            if time.time() - self.last_zone_change_time >= MIN_TIME_IN_ZONE:
                self.field_area = new_field_area
                self.last_field_area = new_field_area
                self.last_zone_change_time = time.time()
        else:
            self.last_zone_change_time = time.time()

        # Validar si la pelota está fuera de los límites
        if self.field_area == ZONA_FUERA:
            pass
        else:
            # Asigna roles a cada robot dependiendo de la cercanía con la pelota
            self._update_roles()

    def _update_roles(self):
        """
        Asigna rorles dinámicamente a los robots, dependiendo de la cercanía con la pelota (ataque o defensa)
        """

        # Verificar si ha pasado el tiempo mínimo desde el último cambio
        if time.time() - self.last_role_change_time < MIN_TIME_BETWEEN_CHANGES:
            return

        dist_robot1_with_ball, id_robot1, _, _ = self.information_robots[0]
        dist_robot2_with_ball, id_robot2, _, _ = self.information_robots[1]

        # Determinar el atacante y defensor basado en la distancia
        if dist_robot1_with_ball < dist_robot2_with_ball - self.UMBRAL_CAMBIO_ROLES:
            attacker = id_robot1
            defender = id_robot2
        elif dist_robot2_with_ball < dist_robot1_with_ball - self.UMBRAL_CAMBIO_ROLES:
            attacker = id_robot2
            defender = id_robot1
        else:
            # No hay cambio de roles si la diferencia es menor al umbral
            return

        # Asignar roles según el estado de la pelota
        if self.state_ball == ESTADO_PELOTA_LIBRE:
            self._assign_free_ball_roles(attacker, defender)
        elif self.state_ball == ESTADO_PELOTA_POSESION:
            self._assign_possession_roles(attacker, defender)

        # Actualizar el tiempo del último cambio de roles
        self.last_role_change_time = time.time()

    def _assign_free_ball_roles(self, attacker, defender):
        """
        Define roles cuando la pelota no está en posesión de ningún equipo.

        Args:
            attacker (int): ID del robot atacante
            defender (int): ID del robot defensor.
        """

        if self.state_ball_proximity == PROXIMIDAD_ALIADA:
            if self.field_area in [ZONA_DEFENSIVA, ZONA_NEUTRAL]:
                self.state_robot_attack = f"Robot {attacker}: {ESTADO_CAPTURAR}"
                self.state_robot_defense = f"Robot {defender}: {ESTADO_DEFENSIVO}"

            elif self.field_area == ZONA_OFENSIVA:
                self.state_robot_attack = f"Robot {attacker}: {ESTADO_CAPTURAR}"
                self.state_robot_defense = f"Robot {defender}: {ESTADO_PREPARAR_PASE}"

        elif self.state_ball_proximity == PROXIMIDAD_RIVAL:
            if self.field_area in [ZONA_DEFENSIVA, ZONA_NEUTRAL]:
                self.state_robot_attack = f"Robot {attacker}: {ESTADO_INTERCEPTAR}"
                self.state_robot_defense = f"Robot {defender}: {ESTADO_BLOQUEAR}"

            elif self.field_area == ZONA_OFENSIVA:
                self.state_robot_attack = f"Robot {attacker}: {ESTADO_CAPTURAR}"
                self.state_robot_defense = f"Robot {defender}: {ESTADO_PRESIONAR}"

    def _assign_possession_roles(self, attacker, defender):
        """
        Define roles cuando la pelota está en de algún equipo.

        Args:
            attacker (int): ID del robot atacante
            defender (int): ID del robot defensor.
        """

        if self.state_ball_proximity == PROXIMIDAD_ALIADA:
            if self.field_area in [ZONA_DEFENSIVA, ZONA_NEUTRAL]:
                self.state_robot_attack = f"Robot {attacker}: {ESTADO_AVANZAR}"
                self.state_robot_defense = f"Robot {defender}: {ESTADO_ADELANTAR}"
            elif self.field_area == ZONA_OFENSIVA:
                self.state_robot_attack = f"Robot {attacker}: {ESTADO_LANZAR}"
                self.state_robot_defense = f"Robot {defender}: {ESTADO_APOYAR}"
        elif self.state_ball_proximity == PROXIMIDAD_RIVAL:
            if self.field_area in [ZONA_DEFENSIVA, ZONA_NEUTRAL]:
                self.state_robot_attack = f"Robot {attacker}: {ESTADO_PRESIONAR}"
                self.state_robot_defense = f"Robot {defender}: {ESTADO_BLOQUEAR}"
            elif self.field_area == ZONA_OFENSIVA:
                self.state_robot_attack = f"Robot {attacker}: {ESTADO_PRESIONAR}"
                self.state_robot_defense = f"Robot {defender}: {ESTADO_RETROCEDER}"


# Simulación de la Máquina de Estados
if __name__ == "__main__":
    # x_ball, y_ball = ballReceived.recv()
    # coords_players = playerReceived.recv()

    # Ejemplo de uso
    team_red = RobotStateMachine(1, 2, team=EQUIPO_ROJO)
    team_red.update_state(ball=np.array([5, 5]), coords_players=[
        {'id': 1, 'x': 3, 'y': 3, 'angulo': 90},
        {'id': 2, 'x': 7, 'y': 7, 'angulo': 270},
        {'id': 3, 'x': 6, 'y': 6, 'angulo': 0},
        {'id': 3, 'x': 10, 'y': 30, 'angulo': 0}
    ])

    team_blue = RobotStateMachine(3, 4, team=EQUIPO_AZUL)
    team_blue.update_state(ball=np.array([5, 5]), coords_players=[
        {'id': 1, 'x': 3, 'y': 3, 'angulo': 90},
        {'id': 2, 'x': 7, 'y': 7, 'angulo': 270},
        {'id': 3, 'x': 6, 'y': 6, 'angulo': 0},
        {'id': 3, 'x': 10, 'y': 30, 'angulo': 0}
    ])
