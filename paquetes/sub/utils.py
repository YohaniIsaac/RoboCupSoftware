"""
utils for collision check
@author: huiming zhou
"""

import math
import numpy as np

from . import env
from paquetes.RRT_dinamic import Node


class Utils:
    """
    Utilidades para verificar colisiones entre una trayectoria y los obstáculos en un entorno.
    """
    def __init__(self, list_obs):
        self.env = env.Env(list_obs)

        self.delta = 0.5
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

    def update_obstacle(self, n_list_obstacle):
        self.obs_circle = env.Env.obs_circle_met(n_list_obstacle)
        self.obs_rectangle = env.Env.obs_rectangle_met(n_list_obstacle)

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
