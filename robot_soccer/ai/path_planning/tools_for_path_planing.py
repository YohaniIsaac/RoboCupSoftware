"""
Environment for rrt_2D
@author: huiming zhou
"""
import math
import numpy as np
from robot_soccer.config import *


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None
        self.flag = "VALID"

    def __eq__(self, other):
        # Comprobar si 'other' es una instancia de Node
        if isinstance(other, Node):
            return self.x == other.x and self.y == other.y
        return False


class Env:
    """
    Define el entorno en el que se moverá el robot
    """
    def __init__(self, list_obs):
        self.x_range = (0, ANCHO_CAMPO)   # Límite del área
        self.y_range = (0, ALTO_CAMPO)    # Límite del área
        self.obs_boundary = self.obs_boundary_met()             # Límite externos del campo como obstáculos
        self.obs_circle = self.obs_circle_met(list_obs)         # Obstáculos circulares
        self.obs_rectangle = self.obs_rectangle_met(list_obs)   # Obstáculos rectangulares

    def update_obtacle(self, n_list_obs):
        self.obs_circle = self.obs_circle_met(n_list_obs)         # Obstáculos circulares
        self.obs_rectangle = self.obs_rectangle_met(n_list_obs)   # Obstáculos rectangulares

    @staticmethod
    def obs_boundary_met():
        """
        Genera una lista de los límites del entorno (borde del área) como obstáculos rectangulares delgados.
        """
        obs_boundary = [
            [0, 0, 1, ALTO_CAMPO],
            [0, ALTO_CAMPO, ANCHO_CAMPO, 1],
            [1, 0, ANCHO_CAMPO, 1],
            [ANCHO_CAMPO, 1, 1, ALTO_CAMPO]
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle_met(list_obs):
        """
        Convierte los obstáculos rectangulares de la lista de entrada en coordenadas de sus vértices.
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
        """
        Filtra los obstáculos circulares de la lista de entrada.
        """
        obs_cir = []
        for obs in list_obs:
            if len(obs) == 3:
                obs_cir.append(obs)
        return obs_cir

    @staticmethod
    def calculate_coords(center_x, center_y, des_x, des_y, angle):
        """
        Calcula los vértices del rectángulo centrado en (center_x, center_y), expandido por des_x y des_y, y rotado por
        un ángulo dado.
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
    """
    Utilidades para verificar colisiones entre una trayectoria y los obstáculos en un entorno.
    """
    def __init__(self, list_obs):
        self.env = Env(list_obs)

        self.delta = 20
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

    def update_obstacle(self, n_list_obstacle):
        self.obs_circle = Env.obs_circle_met(n_list_obstacle)
        self.obs_rectangle = Env.obs_rectangle_met(n_list_obstacle)

    def update_obs(self, obs_cir, obs_bound, obs_rec):
        """
        Actualiza las listas de obstáculos
        """
        self.obs_circle = obs_cir
        self.obs_boundary = obs_bound
        self.obs_rectangle = obs_rec

    def get_obs_vertex(self):
        """
        Genera las coordenadas de los vértices de cada obstáculo rectangular.
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
        """
        Verifica si un rayo (o, d) se cruza con un segmento (a, b).

        o    → origen del rayo
        d    → dirección del rayo
        a, b → extremos del segmento
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
        """
        Verifica si un rayo (o, d) se cruza con un círculo de centro a y radio r.
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
        """
        Determina si una trayectoria entre dos puntos colisiona con algún obstáculo.
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
        """
        Verifica si un punto está dentro de algún obstáculo.
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
        """
        Verifica si un punto (px, py) está dentro de un rectángulo representado por sus vértices.
        Utiliza el método de áreas de un rectángulo.
        """
        def area(a, b, c):
            """
            Calcula el área de un triángulo definido por tres puntos.
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
        """
        Genera un rayo (definido por un origen y una dirección) a partir de dos puntos.
        """
        orig = [start.x, start.y]
        direc = [end.x - start.x, end.y - start.y]
        return orig, direc

    @staticmethod
    def get_dist(start, end):
        """
        Calcula la distancia euclidiana entre dos puntos.
        """
        return math.hypot(end.x - start.x, end.y - start.y)
