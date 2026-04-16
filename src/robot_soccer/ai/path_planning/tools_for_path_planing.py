"""Environment for rrt_2D.

Módulo que define el entorno y utilidades para la planificación de rutas RRT.
Incluye definición de obstáculos, límites del área y verificación de colisiones.

@author: huiming zhou
"""
import math
import numpy as np
from robot_soccer.config import FIELD_SIM


class Node:
    """Representa un nodo en el espacio 2D para algoritmos de planificación.

    Attributes:
        x (float): Coordenada x del nodo.
        y (float): Coordenada y del nodo.
        parent (Node): Nodo padre en el árbol.
        flag (str): Estado del nodo ('VALID' por defecto).
    """

    def __init__(self, n):
        """Inicializa un nodo con coordenadas dadas.

        Args:
            n (tuple): Tupla con coordenadas (x, y) del nodo.
        """
        self.x = n[0]
        self.y = n[1]
        self.parent = None
        self.flag = "VALID"

    def __eq__(self, other):
        """Comprueba igualdad entre nodos basada en coordenadas.

        Args:
            other (Node): Otro nodo para comparar.

        Returns:
            bool: True si los nodos tienen las mismas coordenadas.
        """
        # Comprobar si 'other' es una instancia de Node
        if isinstance(other, Node):
            return self.x == other.x and self.y == other.y
        return False


class Env:
    """Define el entorno de planificación con obstáculos y límites.

    Gestiona la configuración del espacio de trabajo incluyendo límites del área,
    obstáculos circulares y rectangulares para algoritmos de planificación de rutas.

    Attributes:
        x_range (tuple): Límites del área en x (min, max).
        y_range (tuple): Límites del área en y (min, max).
        obs_boundary (list): Lista de obstáculos que forman los límites externos.
        obs_circle (list): Lista de obstáculos circulares.
        obs_rectangle (list): Lista de obstáculos rectangulares.
    """
    def __init__(self, list_obs, field=None):
        """Inicializa el entorno con obstáculos dados.

        Args:
            list_obs (list): Lista de obstáculos en formato específico.
            field: FieldGeometry con geometría del campo. Defaults to FIELD_SIM.
        """
        self.field = field if field is not None else FIELD_SIM
        self.x_range = (0, self.field.width)   # Límite del área
        self.y_range = (0, self.field.height)   # Límite del área
        self.obs_boundary = self._obs_boundary_met()             # Límite externos del campo como obstáculos
        self.obs_circle = self.obs_circle_met(list_obs)         # Obstáculos circulares
        self.obs_rectangle = self.obs_rectangle_met(list_obs)   # Obstáculos rectangulares

    def update_obtacle(self, n_list_obs):
        """Actualiza la lista de obstáculos del entorno.

        Args:
            n_list_obs (list): Nueva lista de obstáculos.
        """
        self.obs_circle = self.obs_circle_met(n_list_obs)         # Obstáculos circulares
        self.obs_rectangle = self.obs_rectangle_met(n_list_obs)   # Obstáculos rectangulares

    def _obs_boundary_met(self):
        """Genera límites del entorno como obstáculos rectangulares delgados.

        Returns:
            list: Lista de obstáculos rectangulares que forman el perímetro del campo.
        """
        w, h = self.field.width, self.field.height
        obs_boundary = [
            [0, 0, 1, h],
            [0, h, w, 1],
            [1, 0, w, 1],
            [w, 1, 1, h]
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle_met(list_obs):
        """Convierte obstáculos rectangulares a coordenadas de vértices.

        Args:
            list_obs (list): Lista de obstáculos donde cada elemento rectangular
                           tiene formato [cx, cy, des_x, des_y, angle].

        Returns:
            list: Lista de obstáculos rectangulares como coordenadas de vértices.
        """
        obs_rectangle = []
        for obs in list_obs:
            if len(obs) == 5:
                cx, cy, des_x, des_y, angle = obs
                esquinas = Env.calculate_coords(cx, cy, des_x, des_y, angle)
                obs_rectangle.append(esquinas)
        return obs_rectangle

    @staticmethod
    def obs_circle_met(list_obs):
        """Filtra obstáculos circulares de la lista de entrada.

        Args:
            list_obs (list): Lista de obstáculos donde cada elemento circular
                           tiene formato [x, y, radio].

        Returns:
            list: Lista de obstáculos circulares.
        """
        obs_cir = []
        for obs in list_obs:
            if len(obs) == 3:
                obs_cir.append(obs)
        return obs_cir

    @staticmethod
    def calculate_coords(center_x, center_y, des_x, des_y, angle):
        """Calcula vértices de un rectángulo rotado.

        Calcula las coordenadas de los cuatro vértices de un rectángulo
        centrado en (center_x, center_y), expandido por des_x y des_y,
        y rotado por un ángulo dado.

        Args:
            center_x (float): Coordenada x del centro.
            center_y (float): Coordenada y del centro.
            des_x (float): Semi-ancho del rectángulo.
            des_y (float): Semi-alto del rectángulo.
            angle (float): Ángulo de rotación en radianes.

        Returns:
            list: Lista de tuplas con las coordenadas de los cuatro vértices.
        """
        esquinas = [
            (center_x - des_x, center_y + des_y),  # Esquina superior izquierda
            (center_x + des_x, center_y + des_y),  # Esquina superior derecha
            (center_x + des_x, center_y - des_y),  # Esquina inferior izquierda
            (center_x - des_x, center_y - des_y),  # Esquina inferior derecha
        ]

        list_puntos_rotados = []

        for punto in esquinas:
            x_desplazado, y_desplazado = punto[0] - center_x, punto[1] - center_y

            # Aplicar la matriz de rotacion
            x_rotado = x_desplazado * math.cos(angle) - y_desplazado * math.sin(angle)
            y_rotado = x_desplazado * math.sin(angle) + y_desplazado * math.cos(angle)

            list_puntos_rotados.append((int(center_x + x_rotado), int(center_y + y_rotado)))

        return list_puntos_rotados


class Utils:
    """Utilidades para verificar colisiones entre trayectorias y obstáculos.

    Proporciona métodos para detectar intersecciones entre rayos y diferentes
    tipos de obstáculos (círculos, rectángulos) en el entorno de planificación.

    Attributes:
        env (Env): Instancia del entorno con obstáculos.
        delta (int): Margen de seguridad para detección de colisiones.
        obs_circle (list): Lista de obstáculos circulares.
        obs_rectangle (list): Lista de obstáculos rectangulares.
        obs_boundary (list): Lista de obstáculos de límites.
    """

    def __init__(self, list_obs, field=None):
        """Inicializa las utilidades con una lista de obstáculos.

        Args:
            list_obs (list): Lista de obstáculos en el entorno.
            field: FieldGeometry con geometría del campo. Defaults to FIELD_SIM.
        """
        self.env = Env(list_obs, field=field)
        self.delta = 20
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

    def update_obstacle(self, n_list_obstacle):
        """Actualiza las listas de obstáculos con nueva información.

        Args:
            n_list_obstacle (list): Nueva lista de obstáculos.
        """
        self.obs_circle = Env.obs_circle_met(n_list_obstacle)
        self.obs_rectangle = Env.obs_rectangle_met(n_list_obstacle)

    def update_obs(self, obs_cir, obs_bound, obs_rec):
        """Actualiza directamente las listas de obstáculos.

        Args:
            obs_cir (list): Lista de obstáculos circulares.
            obs_bound (list): Lista de obstáculos de límites.
            obs_rec (list): Lista de obstáculos rectangulares.
        """
        self.obs_circle = obs_cir
        self.obs_boundary = obs_bound
        self.obs_rectangle = obs_rec

    def get_obs_vertex(self):
        """Genera coordenadas de vértices expandidos para obstáculos rectangulares.

        Calcula las coordenadas de los vértices de cada obstáculo rectangular
        expandidos por un margen delta para detección de colisiones.

        Returns:
            list: Lista de listas con vértices expandidos de cada obstáculo rectangular.
        """
        delta = self.delta
        obs_list = []
        for rect in self.obs_rectangle:
            vertex_list = [[rect[2][0] - delta, rect[2][1] - delta],    # Esquina inferior izqyuierda
                           [rect[3][0] + delta, rect[3][1] - delta],    # Esquina inferior derecha
                           [rect[1][0] + delta, rect[1][1] + delta],    # Esquiona superior derecha
                           [rect[0][0] - delta, rect[0][1] + delta]]    # Esquina superior izquierda
            obs_list.append(vertex_list)

        return obs_list

    def is_intersect_rec(self, start, end, o, d, a, b):
        """Verifica si un rayo intersecta con un segmento de línea.

        Args:
            start (Node): Nodo de inicio de la trayectoria.
            end (Node): Nodo final de la trayectoria.
            o (list): Origen del rayo [x, y].
            d (list): Dirección del rayo [dx, dy].
            a (list): Primer extremo del segmento [x, y].
            b (list): Segundo extremo del segmento [x, y].

        Returns:
            bool: True si el rayo intersecta con el segmento dentro de la trayectoria.
        """
        v1 = [o[0] - a[0], o[1] - a[1]]
        v2 = [b[0] - a[0], b[1] - a[1]]
        v3 = [-d[1], d[0]]

        div = np.dot(v2, v3)

        if div == 0:
            return False

        t1 = np.linalg.norm(np.cross(v2, v1)) / div
        t2 = np.dot(v1, v3) / div

        if t1 >= 0 and 0 <= t2 <= 1:
            shot = Node((o[0] + t1 * d[0], o[1] + t1 * d[1]))
            dist_obs = self.get_dist(start, shot)
            dist_seg = self.get_dist(start, end)
            if dist_obs <= dist_seg:
                return True

        return False

    def is_intersect_circle(self, o, d, a, r):
        """Verifica si un rayo intersecta con un círculo.

        Args:
            o (list): Origen del rayo [x, y].
            d (list): Dirección del rayo [dx, dy].
            a (list): Centro del círculo [x, y].
            r (float): Radio del círculo.

        Returns:
            bool: True si el rayo intersecta con el círculo.
        """
        d2 = np.dot(d, d)
        delta = self.delta

        if d2 == 0:
            return False

        t = np.dot([a[0] - o[0], a[1] - o[1]], d) / d2

        if 0 <= t <= 1:
            shot = Node((o[0] + t * d[0], o[1] + t * d[1]))
            if self.get_dist(shot, Node(a)) <= r + delta:
                return True

        return False

    def is_collision(self, start, end):
        """Determina si una trayectoria entre dos puntos colisiona con obstáculos.

        Args:
            start (Node): Nodo de inicio de la trayectoria.
            end (Node): Nodo final de la trayectoria.

        Returns:
            bool: True si la trayectoria colisiona con algún obstáculo.
        """
        if self.is_inside_obs(start) or self.is_inside_obs(end):
            return True

        o, d = self.get_ray(start, end)
        obs_vertex = self.get_obs_vertex()

        for (v1, v2, v3, v4) in obs_vertex:
            if self.is_intersect_rec(start, end, o, d, v1, v2):
                return True
            if self.is_intersect_rec(start, end, o, d, v2, v3):
                return True
            if self.is_intersect_rec(start, end, o, d, v3, v4):
                return True
            if self.is_intersect_rec(start, end, o, d, v4, v1):
                return True

        for (x, y, r) in self.obs_circle:
            if self.is_intersect_circle(o, d, [x, y], r):
                return True

        return False

    def is_inside_obs(self, node):
        """Verifica si un punto está dentro de algún obstáculo.

        Args:
            node (Node): Nodo a verificar.

        Returns:
            bool: True si el nodo está dentro de algún obstáculo.
        """
        delta = self.delta

        for (x, y, r) in self.obs_circle:
            if math.hypot(node.x - x, node.y - y) <= r + delta:
                return True

        for obs in self.obs_rectangle:
            if self.verificar_punto(obs, node.x, node.y):
                return True

        for (x, y, w, h) in self.obs_boundary:
            if 0 <= node.x - (x - delta) <= w + 2 * delta \
                    and 0 <= node.y - (y - delta) <= h + 2 * delta:
                return True

        return False

    @staticmethod
    def verificar_punto(obs, px, py):
        """Verifica si un punto está dentro de un rectángulo usando áreas.

        Utiliza el método de comparación de áreas de triángulos para determinar
        si un punto está dentro de un rectángulo definido por sus vértices.

        Args:
            obs (list): Lista de vértices del rectángulo [(x1,y1), (x2,y2), (x3,y3), (x4,y4)].
            px (float): Coordenada x del punto a verificar.
            py (float): Coordenada y del punto a verificar.

        Returns:
            bool: True si el punto está dentro del rectángulo.
        """
        def area(a, b, c):
            """Calcula el área de un triángulo definido por tres puntos.

            Args:
                a (tuple): Primer punto (x, y).
                b (tuple): Segundo punto (x, y).
                c (tuple): Tercer punto (x, y).

            Returns:
                float: Área del triángulo.
            """
            return abs((a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1])) / 2.0)

        punto = (px, py)
        # Calcular áreas
        total_area = (area(obs[0], obs[1], obs[2]) +
                      area(obs[0], obs[2], obs[3]))

        area1 = (area(punto, obs[0], obs[1]) +
                 area(punto, obs[1], obs[2]) +
                 area(punto, obs[2], obs[3]) +
                 area(punto, obs[3], obs[0]))
        return total_area == area1

    @staticmethod
    def get_ray(start, end):
        """Genera un rayo definido por origen y dirección desde dos puntos.

        Args:
            start (Node): Nodo de inicio.
            end (Node): Nodo final.

        Returns:
            tuple: Tupla con (origen, dirección) del rayo.
        """
        orig = [start.x, start.y]
        direc = [end.x - start.x, end.y - start.y]
        return orig, direc

    @staticmethod
    def get_dist(start, end):
        """Calcula la distancia euclidiana entre dos nodos.

        Args:
            start (Node): Nodo de inicio.
            end (Node): Nodo final.

        Returns:
            float: Distancia euclidiana entre los dos nodos.
        """
        return math.hypot(end.x - start.x, end.y - start.y)


# =============================================================================
# Utilidades de seguimiento de ruta y trigger de replanificacion
# =============================================================================

def path_closest_waypoint_idx(path, rx, ry):
    """Retorna el indice del waypoint mas cercano a la posicion (rx, ry).

    Util al recibir una ruta nueva: permite continuar desde el punto mas
    cercano a la posicion actual del robot en lugar de retroceder al inicio.

    Args:
        path (list): Lista de tuplas [(x, y), ...] que forman la ruta.
        rx (float): Coordenada x actual del robot.
        ry (float): Coordenada y actual del robot.

    Returns:
        int: Indice del waypoint mas cercano.
    """
    best_idx, best_d = 0, float('inf')
    for i, wp in enumerate(path):
        d = math.hypot(rx - wp[0], ry - wp[1])
        if d < best_d:
            best_d, best_idx = d, i
    return best_idx


def path_length_from(path, from_idx):
    """Calcula la longitud total del path desde from_idx hasta el final.

    Permite comparar la longitud restante de dos rutas para decidir si
    aceptar una ruta nueva o conservar la actual.

    Args:
        path (list): Lista de tuplas [(x, y), ...] que forman la ruta.
        from_idx (int): Indice desde el que calcular la longitud restante.

    Returns:
        float: Longitud acumulada en unidades del campo desde from_idx.
    """
    total = 0.0
    for i in range(from_idx, len(path) - 1):
        p1, p2 = path[i], path[i + 1]
        total += math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    return total


def obstacles_moved(last_positions, all_robots, robot_id, threshold=40):
    """Detecta si algun robot obstaculo cambio de posicion mas alla del umbral.

    Compara posiciones actuales de robots detectados por percepcion contra
    las ultimas posiciones registradas. Retorna True si algun obstaculo
    aparecio, desaparecio, o se movio mas del umbral dado.

    Args:
        last_positions (dict): Mapa {robot_id: (x, y)} de la ultima actualizacion.
        all_robots (list): Lista de dicts de percepcion:
            [{'id': int, 'x': float, 'y': float, 'angulo': float}, ...]
        robot_id (int): ID del robot controlado (excluido de la comparacion).
        threshold (float): Desplazamiento minimo en px para considerar cambio.

    Returns:
        bool: True si el conjunto de obstaculos cambio; False si todo igual.
    """
    current_ids = {r['id'] for r in all_robots if r['id'] != robot_id}
    if current_ids != set(last_positions.keys()):
        return True
    for r in all_robots:
        if r['id'] == robot_id:
            continue
        if r['id'] in last_positions:
            lx, ly = last_positions[r['id']]
            if math.hypot(r['x'] - lx, r['y'] - ly) > threshold:
                return True
    return False
