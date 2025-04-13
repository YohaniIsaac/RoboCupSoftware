import math
import numpy as np
from paquetes.rrt_star_smart import RrtStarSmart
# import multiprocessing
import threading
import time
from config import *


class Player:
    """
    Representa un jugador con capacidades para ejecutar acciones físicas.
    Implementa la lógica de bajo nivel para movimiento, control de pelota y navegación.
    """

    def __init__(self, player_id, x, y, angle, ball, team='red'):
        # Mantener atributos existentes
        self.id = player_id
        self.x = int(x)
        self.y = int(y)
        self.angle = angle
        self.team = team
        self.ball = ball

        self.distance2ball = None
        self.anglerobot2ball = None
        self.rol = None  # 0: defensivo, 1: atacante

        # Estado de posesión
        self._has_ball = False
        self.ball_capture_distance = 30

        # Parámetros de movimiento
        self.max_speed = 10
        self.max_rotation_speed = 5

        # Mantener componentes de planificación existentes...
        self.planner = RrtStarSmart(step_len=20, goal_sample_rate=0.05,
                                    search_radius=50, iter_max=500)
        self.current_path = None
        self.goal = None
        self.path_lock = threading.Lock()
        self.needs_replanning = False
        self.last_planning_time = 0
        self.min_replanning_interval = 0.2
        # === MÉTODOS DE MOVIMIENTO BÁSICO ===

    def move_to_position(self, target_position, speed_factor=1.0):
        """Mueve al jugador hacia una posición objetivo"""
        # Planificar ruta si es necesario
        target_position = np.array(target_position)
        if (self.goal is None or
                np.linalg.norm(target_position - np.array(self.goal)) > 20):
            self.set_goal(target_position)

        # Ajustar velocidad máxima
        self.max_speed = 10 * speed_factor

        # Ejecutar un paso de movimiento
        if self.current_path and len(self.current_path) > 1:
            self._follow_path()
        else:
            # Movimiento directo si no hay ruta planificada
            self._move_directly_to(target_position)

    def _move_directly_to(self, target_position):
        """Movimiento directo hacia un punto sin planificación de ruta"""
        direction = np.array(target_position) - np.array([self.x, self.y])
        distance = np.linalg.norm(direction)

        if distance > 0:
            # Normalizar y aplicar velocidad
            direction = direction / distance * min(distance, self.max_speed)

            # Actualizar posición
            self.x += int(direction[0])
            self.y += int(direction[1])

            # Actualizar orientación
            self._orient_towards(target_position)

    def _orient_towards(self, target_position):
        """Orienta al jugador hacia una posición"""
        # Calcular ángulo objetivo
        dx = target_position[0] - self.x
        dy = target_position[1] - self.y
        target_angle = math.degrees(math.atan2(dy, dx))

        # Calcular diferencia angular
        angle_diff = (target_angle - self.angle) % 360
        if angle_diff > 180:
            angle_diff -= 360

        # Aplicar rotación con límite de velocidad
        rotation = min(abs(angle_diff), self.max_rotation_speed) * (1 if angle_diff > 0 else -1)
        self.angle = (self.angle + rotation) % 360

    def _follow_path(self):
        """Sigue el camino planificado actual"""
        if not self.current_path or len(self.current_path) < 2:
            return

        # Obtener el siguiente punto en la ruta
        next_point = self.current_path[-2]  # El último punto es la posición inicial

        # Moverse hacia ese punto
        self._move_directly_to(next_point)

        # Si llegamos al punto, avanzar al siguiente
        current_pos = np.array([self.x, self.y])
        if np.linalg.norm(current_pos - np.array(next_point)) < 20:
            self.current_path.pop()

        # === MÉTODOS DE INTERACCIÓN CON LA PELOTA ===

    def has_ball(self):
        """Verifica si el jugador tiene control de la pelota"""
        return self._has_ball

    def intercept_ball(self, ball, prediction_time=0.5):
        """Intercepta la pelota en su trayectoria futura"""
        # Predicción de posición: posición actual + velocidad * tiempo
        if hasattr(ball, 'dx') and hasattr(ball, 'dy'):
            ball_velocity = np.array([ball.dx, ball.dy])
            predicted_pos = ball.get_position() + ball_velocity * prediction_time
        else:
            # Sin información de velocidad, ir a la posición actual
            predicted_pos = ball.get_position()

        # Moverse a alta velocidad hacia el punto de intercepción
        self.move_to_position(predicted_pos, speed_factor=1.5)

    def capture_ball(self, ball):
        """Captura la pelota controlando la velocidad de aproximación"""
        ball_pos = ball.get_position()
        dist_to_ball = self.distance_to_ball(ball)

        if dist_to_ball < self.ball_capture_distance:
            # Cerca de la pelota: movimiento preciso y lento
            approach_speed = max(0.2, min(0.6, dist_to_ball / self.ball_capture_distance))

            # Orientarse hacia la pelota con precisión
            self._orient_towards(ball_pos)

            # Verificar si podemos capturar
            angle_diff = self.angle_difference_ball(ball)
            if dist_to_ball < 10 and angle_diff < 0.5:
                self._has_ball = True
                # Actualizar posición de la pelota (física)
                if hasattr(ball, 'set_position'):
                    front_offset = 20  # Distancia desde el centro del robot
                    angle_rad = math.radians(self.angle)
                    ball_x = self.x + front_offset * math.cos(angle_rad)
                    ball_y = self.y + front_offset * math.sin(angle_rad)
                    ball.set_position(ball_x, ball_y)
        else:
            # Lejos de la pelota: aproximación rápida
            approach_speed = 1.0

        # Moverse hacia la pelota
        self.move_to_position(ball_pos, speed_factor=approach_speed)

    def kick_ball(self, target_position, power=1.0):
        """Patea la pelota hacia un objetivo"""
        if not self._has_ball:
            return False

        # Calcular dirección del tiro
        kick_direction = np.array(target_position) - self.get_position()
        if np.linalg.norm(kick_direction) > 0:
            kick_direction = kick_direction / np.linalg.norm(kick_direction)

        # Determinar velocidad según potencia (escala de 0 a máx_velocidad)
        max_kick_speed = 15  # Velocidad máxima de disparo
        kick_speed = power * max_kick_speed

        # Aplicar velocidad a la pelota (comunicación con el sistema físico)
        # [Implementación según tu motor físico]
        if hasattr(self.ball, 'dx') and hasattr(self.ball, 'dy'):
            self.ball.dx = kick_direction[0] * kick_speed
            self.ball.dy = kick_direction[1] * kick_speed

        self._has_ball = False
        return True

    def move_with_ball(self, path_points, speed_factor=0.7):
        """Avanza con la pelota en control"""
        if not self._has_ball:
            return False

        # Establecer camino
        self.current_path = path_points

        # Avanzar a velocidad controlada
        self._follow_path()

        # Actualizar posición de la pelota
        if hasattr(self.ball, 'set_position'):
            # Calcular posición adelante del jugador
            front_offset = 20
            angle_rad = math.radians(self.angle)
            ball_x = self.x + front_offset * math.cos(angle_rad)
            ball_y = self.y + front_offset * math.sin(angle_rad)

            # Actualizar pelota
            self.ball.set_position(ball_x, ball_y)

        return True

        # === MÉTODOS DE PLANIFICACIÓN ===

    def plan_path_to(self, goal_position, obstacles=None):
        """Planifica una ruta hacia un objetivo evitando obstáculos"""
        obstacles_list = []

        # Convertir obstáculos a formato compatible con RRT
        if obstacles:
            for obstacle in obstacles:
                # Para jugadores, crear obstáculos circulares
                if hasattr(obstacle, 'get_position'):
                    pos = obstacle.get_position()
                    obstacles_list.append([pos[0], pos[1], 40])  # Radio 40

        # Configurar y ejecutar planificador
        with self.path_lock:
            self.planner.setup([self.x, self.y], goal_position, obstacles_list)
            self.planner.planning()
            self.current_path = self.planner.path
            self.goal = goal_position

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

    def set_rol(self, rol):
        """Actualiza el rol del jugador."""
        self.rol = rol

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


def angle_robot2ball(robot_pos, robot_angle_grados, pelota_pos):
    """
    Determina si el robot está apuntando hacia la pelota.

    Args:
        robot_pos (np.array): Coordenadas [x, y] del robot
        robot_angle_grados (float): Ángulo de orientación del robot en grados
        pelota_pos (np.array): Coordenadas [x, y] de la pelota

    Returns:
        bool: True si el robot apunta hacia la pelota, False en caso contrario.
    """
    robot_angle_radians = np.radians(robot_angle_grados)
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


def ball_zone2(ball, zone_team, zonas=(0.3, 0.7)):
    """
    Determina la zona del campo en la que se encuentra la pelota, dividiendo
    el campo en tres zonas: 'DEFENSIVA', 'NEUTRAL' y 'OFENSIVA'.

    Args:
        ball (class):   Coordenadas [x, y] del punto
        zone_team(str): Indica la posición en el campo que tiene el jugador del equipo
        zonas (tuple):  Porcentajes de corte para dividir las zonas

    Returns:
        str: 'DEFENSIVA', 'NEUTRAL', 'OFENSIVA', o 'FUERA' si está fuera del rango.
    """
    center_ball = ball.get_position()
    x, _ = center_ball
    izquierda, derecha = [ANCHO_CAMPO * z for z in zonas]

    if zone_team == LADO_IZQUIERDO:
        if x < izquierda:
            return ZONA_DEFENSIVA
        elif izquierda <= x < derecha:
            return ZONA_NEUTRAL
        elif derecha <= x <= ANCHO_CAMPO:
            return ZONA_OFENSIVA
        else:
            return ZONA_FUERA
    else:
        if x < izquierda:
            return ZONA_OFENSIVA
        elif izquierda <= x < derecha:
            return ZONA_NEUTRAL
        elif derecha <= x <= ANCHO_CAMPO:
            return ZONA_DEFENSIVA
        else:
            return ZONA_FUERA


def ball_zone(ball, zone_team, zonas=(0.3, 0.7)):
    x, _ = ball.get_position()
    izquierda, derecha = [ANCHO_CAMPO * z for z in zonas]

    zonas_mapa = {
        LADO_IZQUIERDO: {
            (0, izquierda): ZONA_DEFENSIVA,
            (izquierda, derecha): ZONA_NEUTRAL,
            (derecha, ANCHO_CAMPO): ZONA_OFENSIVA
        },
        LADO_DERECHO: {
            (0, izquierda): ZONA_OFENSIVA,
            (izquierda, derecha): ZONA_NEUTRAL,
            (derecha, ANCHO_CAMPO): ZONA_DEFENSIVA
        }
    }

    for (inicio, fin), zona in zonas_mapa[zone_team].items():
        if inicio <= x < fin:
            return zona
    return ZONA_FUERA
