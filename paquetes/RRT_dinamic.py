"""
DYNAMIC_RRT_2D
@author: huiming zhou
"""

import math
import copy
import numpy as np
import time
import matplotlib.pyplot as plt
import os
# import matplotlib.patches as patches
from matplotlib.patches import Polygon

from sub import env, utils


# Representación de un nodo en RRT
class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None
        self.flag = "VALID"

    def __eq__(self, other):
        # Comprobar si 'other' es una instancia de Node
        if not isinstance(other, Node):
            return NotImplemented

        # Comparar los atributos relevantes
        return self.x == other.x and self.y == other.y


# Representación de una conexión entre dos nodos
class Edge:
    def __init__(self, n_p, n_c):
        self.parent = n_p
        self.child = n_c
        self.flag = "VALID"


# Implementación del algoritmo de RRT dinámico
class DynamicRrt:
    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, waypoint_sample_rate, iter_max, list_obs):
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len                        # Longitud de cada paso en la expansión del árbol
        self.goal_sample_rate = goal_sample_rate        # Prob de muestrear hacia el objetivo
        self.waypoint_sample_rate = waypoint_sample_rate  # Prob de muestrear hacia puntos de referencia intermedios
        self.iter_max = iter_max                        # Número máximo de iteraciones de la planificación.
        self.vertex = [self.s_start]                    # Lista de nodos visitados en el árbol.
        self.vertex_old = []
        self.vertex_new = []
        self.edges = []                                 # Lista de aristas creadas entre nodos.

        self.env = env.Env(list_obs)                            # Define el entorno con los obstáculos
        self.utils = utils.Utils(list_obs)

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary
        self.obs_add = [0, 0, 0]                        # Obstáculo dinámico que puede ser añadido en tiempo real.
        self.obs_cir_modify = []
        self.obs_rect_modify = []
        self.path = []                                  # Ruta final planificada.
        self.path_without_optimization = []
        self.waypoint = []                              # Lista de nodos intermedios que forman parte de la ruta.

        self.direct_cost_old = np.inf
        self.obs_vertex = self.utils.get_obs_vertex()
        self.ReformObsVertex()
        self.beacons = []

    def planning(self):
        """
        Ejecuta el ciclo principal de planificación.
        Genera nodos aleatorios y extiende el árbol hacia ellos.
        Si se encuentra una ruta válida hacia la meta, extrae y almacena la ruta y los puntos de referencia (waypoints).
        """
        # self.ReformObsVertex()
        for i in range(self.iter_max):
            # Genera un nodo aleatorio con una probabilidad de dirigirse hacia la meta basada en goal_sample_rate
            # Esto ayuda a explorar de manera más eficiente
            node_rand = self.generate_random_node(self.goal_sample_rate)
            # Encuentra el nodo del árbol más cercano al nodo aleatorio generado
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            # Calcula un nuevo nodo en la dirección del nodo aleatorio, extendiendo el árbol
            node_new = self.new_state(node_near, node_rand)

            if node_new and not self.utils.is_collision(node_near, node_new):
                # Node_new se agrega al conjunto de vértices (self.vertex)
                self.vertex.append(node_new)
                # Se registra una nueva arista en self.edges
                self.edges.append(Edge(node_near, node_new))

                # Calcula la distancia del nuevo nodo (node_new) a la meta (s_goal)
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)

                # Si la dist es menor o igual al paso (step_len)
                if dist <= self.step_len:
                    # Conecta el nodo (node_new) a la meta (s_goal)
                    self.new_state(node_new, self.s_goal)

                    # Extrae la ruta sin optimizar
                    self.path_without_optimization = self.extract_path(node_new)

                    # Optimiza la ruta
                    self.PathOptimization(node_new)
                    # Estrae la ruta optimizada
                    path = self.extract_path(self.s_goal)
                    self.path = path

                    # Extrae puntos clave (waypoints) del camino
                    self.waypoint = self.extract_waypoint(node_new)

                    return

        return None

    def update_obstacle(self, n_start, n_goal, n_list_obs):
        """
        Actualiza los puntos de inicio, meta y la lista de obstáculos.
        Verifica si estos continuan siendo los mismos, si no, replanifica la ruta
        """

        if self.s_start != Node(n_start):
            self.s_start = Node(n_start)
            self.vertex = [self.s_start]
            A = True
        else:
            A = False

        if self.s_goal != Node(n_goal):
            self.s_goal = Node(n_goal)
            B = True
        else:
            B = False

        diferencia_rect = self.comparar_listas(self.obs_rectangle, env.Env.obs_rectangle_met(n_list_obs))
        diferencia_circ = self.comparar_listas(self.obs_circle, env.Env.obs_circle_met(n_list_obs))

        if diferencia_rect or diferencia_circ:
            if diferencia_rect:
                self.obs_rect_modify = diferencia_rect
            if diferencia_circ:
                self.obs_cir_modify = diferencia_circ[0]

            self.obs_rectangle = env.Env.obs_rectangle_met(n_list_obs)
            self.obs_circle = env.Env.obs_circle_met(n_list_obs)

            self.env.update_obtacle(n_list_obs)
            self.utils.update_obstacle(n_list_obs)
            self.utils.get_obs_vertex()
            self.ReformObsVertex()

            C = True
        else:
            C = False
        if not A and not B and not C:
            # print("no hay cambios ")
            return
        elif A and not B and not C:
            self.planning()
            # print("uno")
            return
        elif not A and B and not C:
            self.planning()
            # print("dos")
            return
        elif not A and not B and C:
            self.InvalidateNodes()
            if self.is_path_invalid():
                path, waypoint = self.replanning()
                self.vertex_new = []
                self.path = path
                self.waypoint = waypoint
            else:
                self.TrimRRT()
            # print("tres")
            return
        elif A and B and not C:
            self.planning()
            # print("cuatro")
            return
        elif A and not B and C:
            self.replanning()
            # print("cinco")
            return
        elif not A and B and C:
            self.planning()
            # print("seis")
            return
        else:
            self.planning()
            # print("todos")
            return

    def InvalidateNodes(self):
        """
        Marca nodos como inválidos si la nueva arista colisiona con un obstáculo.
        """
        for edge in self.edges:
            if self.is_collision_obs_modify(edge.parent, edge.child):
                edge.child.flag = "INVALID"

    def is_path_invalid(self):
        """
        Comprueba si algún nodo en la ruta actual es inválido, lo que indicaría la necesidad de replantear la ruta.
        """
        for node in self.waypoint:
            if node.flag == "INVALID":
                return True

    def is_collision_obs_modify(self, start, end):
        """
        Verifica si una nueva arista colisiona con un obstáculo agregado dinámicamente.
        """

        delta = self.utils.delta
        if self.obs_cir_modify:
            obs_cir = self.obs_cir_modify

            if math.hypot(start.x - obs_cir[0], start.y - obs_cir[1]) <= obs_cir[2] + delta:
                return True

            if math.hypot(end.x - obs_cir[0], end.y - obs_cir[1]) <= obs_cir[2] + delta:
                return True

            o, d = self.utils.get_ray(start, end)

            if self.utils.is_intersect_circle(o, d, [obs_cir[0], obs_cir[1]], obs_cir[2]):
                return True

        if self.obs_rect_modify:
            obs_rect = self.obs_rect_modify

            for rect in obs_rect:
                # verifica si el punto está dentro del rectángulo
                if self.utils.verificar_punto(rect, start.x, start.y):
                    return True

                if self.utils.verificar_punto(rect, end.x, end.y):
                    return True

                o, d = self.utils.get_ray(start, end)

                v1, v2, v3, v4 = rect

                if self.utils.is_intersect_rec(start, end, o, d, v1, v2):
                    return True
                if self.utils.is_intersect_rec(start, end, o, d, v2, v3):
                    return True
                if self.utils.is_intersect_rec(start, end, o, d, v3, v4):
                    return True
                if self.utils.is_intersect_rec(start, end, o, d, v4, v1):
                    return True

        return False

    def replanning(self):
        """
        Ejecuta el algoritmo de replanificación cuando la ruta existente se vuelve inválida debido a nuevos obstáculos.
        """
        self.TrimRRT()

        for i in range(self.iter_max):
            node_rand = self.generate_random_node_replanning(self.goal_sample_rate, self.waypoint_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and not self.utils.is_collision(node_near, node_new):
                self.vertex.append(node_new)
                self.vertex_new.append(node_new)
                self.edges.append(Edge(node_near, node_new))
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)

                if dist <= self.step_len:
                    self.new_state(node_new, self.s_goal)
                    path = self.extract_path(node_new)
                    waypoint = self.extract_waypoint(node_new)
                    # print("path: ", len(path))
                    # print("waypoint: ", len(waypoint))

                    return path, waypoint

        return None

    def TrimRRT(self):
        """
        Recorta los nodos y aristas inválidos del árbol en caso de cambios en el entorno, lo que ayuda a optimizar la
        eficiencia en la planificación posterior.
        """
        for i in range(1, len(self.vertex)):
            node = self.vertex[i]
            node_p = node.parent
            if node_p.flag == "INVALID":
                node.flag = "INVALID"

        self.vertex = [node for node in self.vertex if node.flag == "VALID"]
        self.vertex_old = copy.deepcopy(self.vertex)
        self.edges = [Edge(node.parent, node) for node in self.vertex[1:len(self.vertex)]]

    def generate_random_node(self, goal_sample_rate):
        """
        Generan un nodo aleatorio, para la planificación inicial, con probabilidad de muestrear hacia el objetivo o
        puntos intermedios.
        """
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return self.s_goal

    def generate_random_node_replanning(self, goal_sample_rate, waypoint_sample_rate):
        """
        Generan un nodo aleatorio, para la replanificación, con probabilidad de muestrear hacia el objetivo o puntos
        intermedios.
        """
        delta = self.utils.delta
        p = np.random.random()

        if p < goal_sample_rate:
            return self.s_goal
        elif goal_sample_rate < p < goal_sample_rate + waypoint_sample_rate:
            return self.waypoint[np.random.randint(0, len(self.waypoint) - 1)]
        else:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

    @staticmethod
    def nearest_neighbor(node_list, n):
        """
        Encuentra el nodo más cercano en el árbol a un nodo aleatorio generado, para determinar desde dónde expandir
        el árbol.
        """
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    def new_state(self, node_start, node_end):
        """
        Calcula un nuevo nodo en la dirección del nodo generado aleatoriamente, limitando la distancia al paso máximo
        (step_len).
        """
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))
        node_new.parent = node_start

        return node_new

    def extract_path(self, node_end):
        """
        Extrae la ruta de la meta hacia el inicio, usando las relaciones padre-hijo de los nodos.
        """
        path = [(self.s_goal.x, self.s_goal.y)]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))

        return path

    def extract_waypoint(self, node_end):
        """
        Extrae los puntos de referencia (waypoints) de la meta hacia el inicio, usando las relaciones padre-hijo de
        los nodos.
        """
        waypoint = [self.s_goal]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            waypoint.append(node_now)

        return waypoint

    @staticmethod
    def comparar_listas(lista1, lista2):
        """
        Compara dos listas de obstáculos y devuelve los elementos diferentes.
        """
        diferencias = []

        # Verifica cada elemento de lista2 que no esté en lista1
        for elem in lista2:
            if elem not in lista1:
                diferencias.append(elem)

        return diferencias

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        """
        Calcula la distancia y el ángulo entre dos nodos.
        """
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    def plot_visited(self, animation=True):
        """
        Muestra los nodos que han sido visitados durante la planificación.
        """
        if animation:
            count = 0
            for node in self.vertex:
                count += 1
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])
                    if count % 10 == 0:
                        plt.pause(0.001)
        else:
            for node in self.vertex:
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")

    @staticmethod
    def plot_path(path, list_obs, path_without):
        """
        Dibuja la ruta final planificada y los obstáculos
        """
        # Crear la figura y los ejes
        plt.figure(figsize=(8, 6))
        # plt.axis("equal")  # Mantener las proporciones iguales

        plt.xlim(-100, 1500)  # Límite del eje x entre 0 y 10
        plt.ylim(-100, 900)   # Límite del eje y entre -1.5 y 1.5

        env_ = env.Env(list_obs)
        obs_circle = env_.obs_circle
        obs_rectangle = env_.obs_rectangle
        for cir in obs_circle:
            cx, cy, r = cir
            DynamicRrt.graficar_circulo((cx, cy), r)
        for rec in obs_rectangle:
            v1, v2, v3, v4 = rec
            DynamicRrt.graficar_rectangulo(v1, v2, v3, v4)

        # Graficar la ruta optimizada
        plt.plot([x[0] for x in path_without], [x[1] for x in path_without],
                 linewidth=1, color='g', label='Ruta optimizada')

        # Graficar la ruta original
        plt.plot([x[0] for x in path], [x[1] for x in path],
                 linewidth=2, color='b', label='Ruta original')

        x_goal = (1400, 100)
        plt.plot(x_goal[0], x_goal[1], 'ro',  markersize=2)
        # plt.pause(0.01)
        nombre_archivo = DynamicRrt.generar_nombre_archivo("grafico")
        plt.grid()
        plt.savefig(nombre_archivo, dpi=300, bbox_inches='tight')
        plt.close()

    @staticmethod
    def graficar_circulo(centro, radio, color='b', etiqueta=None):
        """
        Grafica un círculo dado su centro y radio.
        """
        circle = plt.Circle(centro, radio, color=color, fill=False, linewidth=2, label=etiqueta)
        plt.gca().add_patch(circle)  # Añadir el círculo al gráfico

    @staticmethod
    def graficar_rectangulo(vertice1, vertice2, vertice3, vertice4, color='r', etiqueta=None):
        """
        Grafica un rectángulo dado por sus cuatro vértices.
        """
        # Crear un polígono usando los 4 vértices
        rect = Polygon([vertice1, vertice2, vertice3, vertice4],
                       closed=True, edgecolor=color, fill=False, linewidth=2, label=etiqueta)
        plt.gca().add_patch(rect)  # Añadir el polígono al gráfico

    @staticmethod
    def generar_nombre_archivo(base_name, extension='png'):
        """
        Genera un nombre de archivo único si el original ya existe.
        """
        counter = 1
        nombre_final = f"./graf/{base_name}.{extension}"

        # Mientras el archivo exista, incrementar el contador y cambiar el nombre
        while os.path.exists(nombre_final):
            nombre_final = f"./graf/{base_name}_{counter}.{extension}"
            counter += 1

        return nombre_final

    @staticmethod
    def count_layers(node):
        """
        Cuenta la cantidad de nodos desde el nodo final hasta el nodo inicial.
        """
        layers = 0
        while node:
            layers += 1
            node = node.parent  # Se mueve al nodo padre
        return layers

    def PathOptimization(self, node):
        """
        Intenta encontrar una ruta más directa hacia la meta optimizando los nodos de la ruta.
        """
        direct_cost_new = 0.0
        node_end = self.s_goal

        while node.parent:
            node_parent = node.parent

            if not self.utils.is_collision(node_parent, node_end):
                node_end.parent = node_parent
            else:
                direct_cost_new += self.Line(node, node_end)
                node_end = node

            node = node_parent

    @staticmethod
    def Line(x_start, x_goal):
        """
        Calcula la distancia euclidiana entre dos nodos.
        """
        return math.hypot(x_goal.x - x_start.x, x_goal.y - x_start.y)

    def UpdateBeacons(self):
        """
        Actualiza los puntos de referencia (beacons) basados en los nodos cercanos a los obstáculos.
        """
        node = self.s_goal
        beacons = []
        while node.parent:
            near_vertex = [v for v in self.obs_vertex
                           if (node.x - v[0]) ** 2 + (node.y - v[1]) ** 2 < 9]
            if len(near_vertex) > 0:
                for v in near_vertex:
                    beacons.append(v)

            node = node.parent

        self.beacons = beacons

    def ReformObsVertex(self):
        """
        Reforma la lista de vértices de obstáculos para que sea más fácil de manejar.
        """
        obs_vertex = []
        for obs in self.obs_vertex:
            for vertex in obs:
                vertex = [int(vertex[0]), int(vertex[0])]
                obs_vertex.append(vertex)

        self.obs_vertex = obs_vertex


def main():
    x_start = (10, 400)  # Starting node
    x_goal = (1400, 100)  # Goal node
    list_obs = [
        [400, 400, 52*2, 70*2, math.radians(180)],
        [700, 300, 52, 70, math.radians(180)],
        [1000, 800, 50*2]
    ]
    drrt = DynamicRrt(x_start, x_goal, 10, 0.1, 0.6, 5000, list_obs)
    drrt.planning()

    # Mantiene las condiciones
    # for _ in range(5):
    #     start_time = time.time()
    #     drrt.update_obstacle(x_start, x_goal, list_obs)
    #     elapsed_time = time.time() - start_time  # Tiempo en segundos
    #     print(f"Tiempo de update_obstacle (manteniendo condiciones):\t\t {elapsed_time:.6f} segundos")
    #
    #     if drrt.path:
    #         drrt.plot_path(drrt.path, list_obs,   drrt.path_without_optimization)

    # Modifica los obstáculos
    for _ in range(10):
        list_obs[0][1] -= 25
        list_obs[1][0] += 30
        list_obs[1][1] += 50
        list_obs[2][0] += 5
        list_obs[2][1] -= 40

        print(list_obs)

        start_time = time.time()
        drrt.update_obstacle(x_start, x_goal, list_obs)
        elapsed_time = time.time() - start_time  # Tiempo en segundos
        print(f"Tiempo de update_obstacle (modificando los obstáculos):\t\t {elapsed_time:.6f} segundos")

        if drrt.path:
            drrt.plot_path(drrt.path, list_obs,   drrt.path_without_optimization)

    # Modificar la meta
    # for _ in range(5):
    #     x_goal = (x_goal[0] - var, x_goal[1] + var)
    #
    #     start_time = time.time()
    #     drrt.update_obstacle(x_start, x_goal, list_obs)
    #     elapsed_time = time.time() - start_time  # Tiempo en segundos
    #     print(f"Tiempo de update_obstacle (modificar la meta):\t\t\t\t {elapsed_time:.6f} segundos")

    # if drrt.path:
    #     drrt.plot_path(drrt.path, list_obs, drrt.path_without_optimization)
    # Modificar el inicio
    # for _ in range(5):
    #     x_start = (x_start[0] + var, x_start[1] - var)
    #
    #     start_time = time.time()
    #     drrt.update_obstacle(x_start, x_goal, list_obs)
    #     elapsed_time = time.time() - start_time  # Tiempo en segundos
    #     print(f"Tiempo de update_obstacle (modificar el inicio):\t\t\t {elapsed_time:.6f} segundos")
    #
    #     if drrt.path:
    #         drrt.plot_path(drrt.path, list_obs)


if __name__ == '__main__':
    main()
