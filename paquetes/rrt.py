"""
RRT_2D
@author: huiming zhou
"""
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Polygon

x_ancho = 1500
y_alto = 900


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class Rrt:
    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, iter_max, lista_obs):
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.vertex = [self.s_start]

        self.env = Env(lista_obs)
        self.utils = Utils(lista_obs)

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

    def planning(self):
        for i in range(self.iter_max):
            node_rand = self.generate_random_node(self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and not self.utils.is_collision(node_near, node_new):
                self.vertex.append(node_new)
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)

                if dist <= self.step_len and not self.utils.is_collision(node_new, self.s_goal):
                    self.new_state(node_new, self.s_goal)
                    return self.extract_path(node_new)

        return None

    def generate_random_node(self, goal_sample_rate):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return self.s_goal

    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))
        node_new.parent = node_start

        return node_new

    def extract_path(self, node_end):
        path = [(self.s_goal.x, self.s_goal.y)]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))

        return path

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)


#####
# ENV
#####


class Env:
    def __init__(self, lista_obs=None):
        self.x_range = (0, x_ancho)
        self.y_range = (0, y_alto)
        self.obs_boundary = self.obs_boundary()
        self.obs_circle = self.obs_circle(lista_obs)
        self.obs_rectangle = self.obs_rectangle(lista_obs)

    @staticmethod
    def obs_boundary():
        obs_boundary = [
            [0, 0, 1, y_alto],
            [0, y_alto, x_ancho, 1],
            [1, 0, x_ancho, 1],
            [x_ancho, 1, 1, y_alto]
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle(lista_obs):
        obs_rectangle = []
        for obs in lista_obs:
            if len(obs) == 5:
                cx, cy, des_x, des_y, angle = obs
                esquinas = calculate_coords(cx, cy, des_x, des_y, angle)
                obs_rectangle.append(esquinas)
        return obs_rectangle

    @staticmethod
    def obs_circle(lista_obs):
        obs_cir = []
        for obs in lista_obs:
            if len(obs) == 3:
                obs_cir.append(obs)
        return obs_cir


#######
# UTILS
#######


class Utils:
    def __init__(self, lista_obstaculos):
        self.env = Env(lista_obstaculos)

        self.delta = 0.5
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

    def update_obs(self, obs_cir, obs_bound, obs_rec):
        self.obs_circle = obs_cir
        self.obs_boundary = obs_bound
        self.obs_rectangle = obs_rec

    def get_obs_vertex(self):
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

        def area(a, b, c):
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
        orig = [start.x, start.y]
        direc = [end.x - start.x, end.y - start.y]
        return orig, direc

    @staticmethod
    def get_dist(start, end):
        return math.hypot(end.x - start.x, end.y - start.y)


def graficar(x_start, x_goal, lista_obstaculos, path):
    fig, ax = plt.subplots()

    # Graficar el punto de inicio
    ax.plot(x_start[0], x_start[1], 'go', label='Inicio')  # 'go' para un punto verde
    # Graficar el punto final
    ax.plot(x_goal[0], x_goal[1], 'ro', label='Meta')  # 'ro' para un punto rojo
    obs_rect = []
    # Graficar los obstáculos
    for obs in lista_obstaculos:
        if len(obs) == 5:
            cx, cy, des_x, des_y, angle = obs
            esquinas = calculate_coords(cx, cy, des_x, des_y, angle)
            obs_rect.append(esquinas)

            # Crear un polígono a partir de las esquinas calculadas
            rect = Polygon(esquinas, closed=True, edgecolor='black', facecolor='gray', alpha=0.7)

            # Agregar el polígono al gráfico
            ax.add_patch(rect)
        if len(obs) == 3:
            circle = patches.Circle((obs[0], obs[1]), obs[2], linewidth=1, edgecolor='black', facecolor='lightgray')
            ax.add_patch(circle)
    # Graficar el camino
    if path:
        path_x = [p[0] for p in path]
        path_y = [p[1] for p in path]
        ax.plot(path_x, path_y, 'b-', label='Camino')  # 'b-' para una línea azul

    # Configuración del gráfico
    ax.set_xlim(0, x_ancho)  # Ajusta el límite del eje x según sea necesario
    ax.set_ylim(0, y_alto)  # Ajusta el límite del eje y según sea necesario
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Planificación de Rutas con RRT')
    ax.legend()
    ax.grid()
    plt.show()


def calculate_coords(center_x, center_y, des_x, des_y, angle):
    # Calcular el vector del angulo

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


def main(start, end, lista_obstaculos):
    x_start = start  # Starting node
    x_goal = end  # Goal node
    # lista_obstaculos = []

    # lista_obstaculos = [
    #     # [700, 80, 750, 750],
    #     [400, 200, 52*2, 70*2, math.radians(180)],
    #     [400, 700, 52*2, 70*2, math.radians(90)],   # centro_x, centro_y, variacion en x segun el centro, variacion en y segun el centro, angulo en radianes
    #     [800, 400, 52*2, 70*2, math.radians(-90)],  # x,y ,ancho, alto -- x,y corresponden a la esquina inferior izquierda del rectangulo
    #     [200, 200, 50*2]
    # ]
    rrt = Rrt(start, end, 50, 0.000000001, 10000, lista_obstaculos)
    path = rrt.planning()

    path = [(int(x), int(y)) for x, y in path]


    # print(len(path))
    # for i in range(len(path)):
    #     print(path[i])
    # graficar(x_start, x_goal, lista_obstaculos, path)
    return path

# if __name__ == '__main__':
#     main()
