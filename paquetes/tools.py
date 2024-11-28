import math
import numpy as np
from paquetes.rrt_star_smart import RrtStarSmart
import multiprocessing
import threading
import time


class Player:
    def __init__(self, player_id, x, y, angle, team='red'):
        """
        Inicializa un jugador con sus características principales.

        :param player_id: Identificador único del jugador
        :param x: Coordenada x de la posición
        :param y: Coordenada y de la posición
        :param angle: Ángulo de orientación del jugador
        :param team: Equipo al que pertenece el jugador (opcional)
        """
        self.id = player_id
        self.x = int(x)
        self.y = int(y)
        self.angle = angle
        self.team = team

        self.distance2ball = None
        self.anglerobot2ball = None
        self.rol = None # 0: defensivo, 1: atacante

        # Componentes de planificación
        self.planner = RrtStarSmart(
            step_len=20,
            goal_sample_rate=0.05,
            search_radius=50,
            iter_max=500  # Reducido para planificación rápida
        )
        self.current_path = None
        self.goal = None
        self.path_lock = threading.Lock()  # Para acceso seguro desde múltiples hilos
        self.needs_replanning = False
        self.last_planning_time = 0
        self.min_replanning_interval = 0.2  # Segundos entre replanificaciones

    def update(self, all_players):
        """Método principal a llamar en cada ciclo del hilo"""
        current_time = time.time()

        # Verificar si necesitamos replanificar
        replanning_needed = self.needs_replanning or self._check_path_blocked(all_players)

        # Limitar la frecuencia de replanificación
        if (replanning_needed and
                current_time - self.last_planning_time > self.min_replanning_interval):
            self._plan_path(all_players)
            self.last_planning_time = current_time
            self.needs_replanning = False

        # Seguir el camino actual (implementar lógica de movimiento aquí)
        if self.current_path:
            self._follow_path()

    def _plan_path(self, all_players):
        """Planifica o replanifica la ruta"""
        if not self.goal:
            return

        # Obtener obstáculos (otros jugadores)
        obstacles = self._get_player_obstacles(all_players)

        # Configurar y ejecutar el planificador
        with self.path_lock:  # Proteger acceso concurrente
            self.planner.setup([self.x, self.y], self.goal, obstacles)
            self.planner.planning()
            self.current_path = self.planner.path
    def set_goal(self, goal_position):
        """Establece el objetivo del jugador"""
        self.goal = goal_position
        self.needs_replanning = True

    def _check_path_blocked(self, all_players):
        """Verifica si la ruta actual está bloqueada"""
        if not self.current_path:
            return False

        with self.path_lock:
            for player_id, player in all_players.items():
                if player_id == self.id:
                    continue

                player_pos = np.array([player.x, player.y])

                # Verificar si otro jugador bloquea algún segmento del camino
                for i in range(len(self.current_path) - 1):
                    segment_start = np.array(self.current_path[i])
                    segment_end = np.array(self.current_path[i + 1])

                    # Calcular la distancia del jugador al segmento
                    dist = self._point_to_segment_distance(player_pos, segment_start, segment_end)

                    if dist < 50:  # Radio de seguridad
                        return True

            return False

    def _get_player_obstacles(self, all_players):
        """Convierte otros jugadores en obstáculos para el planificador"""
        obstacles = []
        for player_id, player in all_players.items():
            if player_id != self.id:
                # Crear obstáculo circular
                obstacles.append([player.x, player.y, 40])  # Radio de jugador + margen
        return obstacles

    def _follow_path(self):
        """Lógica para seguir el camino planificado"""
        if not self.current_path or len(self.current_path) < 2:
            return

        # Obtener el siguiente punto en la ruta
        next_point = self.current_path[-2]  # El último punto es la posición inicial

        # Calcular vector hacia el siguiente punto
        direction = np.array(next_point) - np.array([self.x, self.y])
        distance = np.linalg.norm(direction)

        if distance < 20:  # Si estamos cerca, avanzar al siguiente punto
            self.current_path.pop()
            return

        # Normalizar y escalar por velocidad deseada
        if distance > 0:
            direction = direction / distance * min(distance, 10)  # Velocidad máxima

        # Actualizar posición (implementación básica, ajustar según tu sistema)
        self.x += int(direction[0])
        self.y += int(direction[1])

        # Actualizar ángulo hacia la dirección del movimiento
        self.angle = np.degrees(np.arctan2(direction[1], direction[0]))

    @staticmethod
    def _point_to_segment_distance(p, v, w):
        """Calcula la distancia de un punto p al segmento entre v y w"""
        l2 = np.sum((v - w) ** 2)
        if l2 == 0:
            return np.linalg.norm(p - v)

        t = max(0, min(1, np.dot(p - v, w - v) / l2))
        projection = v + t * (w - v)
        return np.linalg.norm(p - projection)

    def get_position(self):
        """ Obtiene la posición actual como un array de NumPy. """
        return np.array([self.x, self.y])

    def get_angle(self):
        """Obtiene el ángulo actual del jugador."""
        return self.angle

    def set_position(self, x, y):
        """Actualiza la posición del jugador."""
        self.x = int(x)
        self.y = int(y)

    def set_angle(self, angle):
        """Actualiza el ángulo del jugador."""
        self.angle = angle

    def set_rol(self, rol_str):
        """Actualiza el rol del jugador."""
        if rol_str == "atacante" or "ATACANTE":
            self.rol = 1
        elif rol_str == "defensa" or "DEFENSA":
            self.rol = 0
        else:
            self.rol = None

    def distance_to_ball(self, ball):
        """Calcula la distancia euclidiana entre este jugador y otro."""

        self.distance2ball = np.linalg.norm(self.get_position() - ball.get_position())
        return self.distance2ball

    def angle_difference_ball(self, ball):
        """Calcula la diferencia angular entre este jugador y otro.

        :param ball: Instancia de pelota con el que se calcula la diferencia angular
        :return: Diferencia angular en radianes
        """
        robot_angle_radians = np.radians(self.angle)
        # 1. Calcular el vector de diferencia entre la pelota y el robot
        delta_pos = ball.get_position() - self.get_position()

        # 2. Calcular el ángulo hacia la pelota
        theta_pelota = np.arctan2(delta_pos[1], delta_pos[0])

        # Asegurar que theta_pelota esté en el rango de 0 a 2π
        theta_pelota = theta_pelota % (2 * np.pi)

        # 3. Calcular la diferencia angular
        delta_theta = theta_pelota - robot_angle_radians

        # 4. Normalizar la diferencia angular entre 0 y 2π
        delta_theta = delta_theta % (2 * np.pi)

        # 5. Asegurar que la diferencia angular sea la más corta (por ejemplo, 350° y 10° tienen una diferencia de 20°,
        # no 340°)
        if delta_theta > np.pi:
            delta_theta = 2 * np.pi - delta_theta

        self.anglerobot2ball = abs(delta_theta)

        return self.anglerobot2ball


class Ball:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def get_position(self):
        return np.array([self.x, self.y])

    def set_position(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        return f"Ball(x={self.x}, y={self.y})"


def separate_coords(datos, _id):
    """
    Separa los datos de una lista de diccionarios, dejando una lista
    Args:
        datos (list): Lista de diccionarios con claves
        _id (int): identificador del robot que se quiere conocer los datos
    Return:
        array: Coordenadaas 'x', 'y' del robot
        float: Ángulo 'angle' del robot
    """

    for info in datos:
        if _id == info["id"]:
            return np.array([info["x"], info["y"]]), math.radians(info["angulo"])
    raise ValueError(f"ID {_id} no encontrado en los datos")  # Opción con excepción


def angle_robot2ball(robot_pos, robot_angle_grados, pelota_pos, umbral_grados=5):
    """
    Determina si el robot está apuntando hacia la pelota.

    Args:
        robot_pos (np.array): Coordenadas [x, y] del robot
        robot_angle_grados (float): Ángulo de orientación del robot en grados
        pelota_pos (np.array): Coordenadas [x, y] de la pelota
        umbral_grados (float): Umbral en grados para considerar que el robot apunta hacia la pelota

    Returns:
        bool: True si el robot apunta hacia la pelota, False en caso contrario.
    """
    robot_angle_radians = np.radians(robot_angle_grados)
    umbral_radianes = np.radians(umbral_grados)
    # 1. Calcular el vector de diferencia entre la pelota y el robot
    delta_pos = pelota_pos - robot_pos

    # 2. Calcular el ángulo hacia la pelota
    theta_pelota = np.arctan2(delta_pos[1], delta_pos[0])

    # Asegurar que theta_pelota esté en el rango de 0 a 2π
    theta_pelota = theta_pelota % (2 * np.pi)

    # 3. Calcular la diferencia angular
    delta_theta = theta_pelota - robot_angle_radians

    # 4. Normalizar la diferencia angular entre 0 y 2π
    delta_theta = delta_theta % (2 * np.pi)

    # 5. Asegurar que la diferencia angular sea la más corta (por ejemplo, 350° y 10° tienen una diferencia de 20°,
    # no 340°)
    if delta_theta > np.pi:
        delta_theta = 2 * np.pi - delta_theta

    # 6. Comparar la diferencia angular con el umbral
    return abs(delta_theta)


def position_ball(datos, ball):
    """
    Calcula si alguien tiene posesión de la pelota idependiente del equipo al que pertenezca, también entrega la
    información de los robots del equipo.

    Args:
        datos       (dict):         Datos de cada jugador, con claves 'x', 'y', 'angulo', 'id'
        ball        (list[float]):  Coordenadas [x, y] de la pelota

    Returns:
        tuple:
            bool: True si alguien posee la pelota, idependiente del equipo
            list: Información de los robots (distancia, id, centro, angulo)
    """
    distancias = []
    center_ball = ball.get_position()
    for i in datos:
        center = datos[i].get_position()
        angle = datos[i].get_angle()

        distancia = np.linalg.norm(center - center_ball)
        angulo_apuntado = angle_robot2ball(center, angle, center_ball)
        distancias.append((i, distancia, center, angulo_apuntado))

    # Ordenar por distancia
    distancias.sort(key=lambda x: x[0])

    # Verificar si el robot más cercano está en posesión
    _, distancia_previa, center, angle = distancias[0]
    if distancia_previa < 40 and angle_robot2ball(center, angle, center_ball):
        return True, distancias
    return False, distancias


def distancias_players2ball(datos, ball):
    """
    Calcula la distancia de cada robot a la pelota
    """
    distancias = []
    center_ball = ball.get_position()
    for i in datos:
        center = datos[i].get_position()
        angle = datos[i].get_angle()

        distancia = np.linalg.norm(center - center_ball)
        angulo_apuntado = angle_robot2ball(center, angle, center_ball)
        distancias.append((i, distancia, center, angulo_apuntado))

    # Ordenar por distancia
    distancias.sort(key=lambda x: x[0])

    return distancias


def ball_zone2(ball, zone_team, ancho=1500, zonas=(0.3, 0.7)):
    """
    Determina la zona del campo en la que se encuentra la pelota, dividiendo
    el campo en tres zonas: 'DEFENSIVA', 'NEUTRAL' y 'OFENSIVA'.

    Args:
        ball (class):   Coordenadas [x, y] del punto
        zone_team(str): Indica la posición en el campo que tiene el jugador del equipo
        ancho (int):    Ancho del mapa (por defecto 1500)
        zonas (tuple):  Porcentajes de corte para dividir las zonas

    Returns:
        str: 'DEFENSIVA', 'NEUTRAL', 'OFENSIVA', o 'FUERA' si está fuera del rango.
    """
    center_ball = ball.get_position()
    x, _ = center_ball
    izquierda, derecha = [ancho * z for z in zonas]

    if zone_team == "LEFT":
        if x < izquierda:
            return "DEFENSIVA"
        elif izquierda <= x < derecha:
            return "NEUTRAL"
        elif derecha <= x <= ancho:
            return "OFENSIVA"
        else:
            return "FUERA"
    else:
        if x < izquierda:
            return "OFENSIVA"
        elif izquierda <= x < derecha:
            return "NEUTRAL"
        elif derecha <= x <= ancho:
            return "DEFENSIVA"
        else:
            return "FUERA"


def ball_zone(ball, zone_team, ancho=1500, zonas=(0.3, 0.7)):
    x, _ = ball.get_position()
    izquierda, derecha = [ancho * z for z in zonas]

    zonas_mapa = {
        "LEFT": {
            (0, izquierda): "DEFENSIVA",
            (izquierda, derecha): "NEUTRAL",
            (derecha, ancho): "OFENSIVA"
        },
        "RIGHT": {
            (0, izquierda): "OFENSIVA",
            (izquierda, derecha): "NEUTRAL",
            (derecha, ancho): "DEFENSIVA"
        }
    }

    for (inicio, fin), zona in zonas_mapa[zone_team].items():
        if inicio <= x < fin:
            return zona
    return "FUERA"
