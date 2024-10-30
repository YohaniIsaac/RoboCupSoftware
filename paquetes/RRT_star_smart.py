"""
RRT_STAR_SMART 2D
@author: huiming zhou
"""
#
# import os
# import sys
import math
import random
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon


from sub import env, utils


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


# Implementación del algoritmo RRt* smart
class RrtStarSmart:
    def __init__(self, x_start, x_goal, step_len,
                 goal_sample_rate, search_radius, iter_max, list_obs):
        self.x_start = Node(x_start)
        self.x_goal = Node(x_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.iter_max = iter_max

        self.env = env.Env(list_obs)
        self.utils = utils.Utils(list_obs)

        self.delta = self.utils.delta
        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

        self.V = [self.x_start]
        self.beacons = []
        self.beacons_radius = 2
        self.direct_cost_old = np.inf
        self.obs_vertex = self.utils.get_obs_vertex()
        self.path = None

    def planning(self):
        """
        Es el núcleo de la planificación de rutas. Realiza el muestreo, expansión, conexión y optimización de la ruta.
        """
        start_time = time.time()

        n = 0   # Guarda el valor del iterador k
        b = 2
        InitPathFlag = False
        self.ReformObsVertex()          # Ajuste de los vértices de los obstáculos
        # n y b: Se utilizan para controlar la frecuencia de muestreo utilizando balizas (beacons).

        for k in range(self.iter_max):
            if k % 200 == 0:
                print(k)

            if (k - n) % b == 0 and len(self.beacons) > 0:
                # Si (k - n) es múltiplo de b y hay balizas disponibles, realiza el muestreo usando las balizas.
                # Esto mejora la exploración guiada hacia el objetivo.
                x_rand = self.Sample(self.beacons)
            else:
                # Muestreo aleatorio estándar
                x_rand = self.Sample()

            # Encuentra el nodo en el árbol más cercano al punto muestreado x_rand
            x_nearest = self.Nearest(self.V, x_rand)
            # Intenta conectar el nodo cercano x_nearest con el punto muestreado x_rand, generando un nuevo nodo x_new
            x_new = self.Steer(x_nearest, x_rand)

            if x_new and not self.utils.is_collision(x_nearest, x_new):
                # Encuentra los nodos cercanos en el árbol al nuevo nodo x_new
                X_near = self.Near(self.V, x_new)
                # Agrega x_new al conjunto de nodos
                self.V.append(x_new)

                if X_near:
                    # Elegir padre,
                    # Para la optimización, al elegir al padre con el menor costo acumulado
                    # distancia desde el nodo inicial más la distancia al nuevo nodo
                    #
                    # Calcula el costo total para conectar cada nodo cercano (x_near) con x_new.
                    cost_list = [self.Cost(x_near) + self.Line(x_near, x_new) for x_near in X_near]
                    # Selecciona el nodo con menor costo acumulado como padre del nuevo nodo
                    x_new.parent = X_near[int(np.argmin(cost_list))]

                    # rewire: mejora los caminos dinámicamente, reconexión dinámica
                    # Si la nueva conexión a un nodo cercano reduce el costo total del camino hacia ese nodo
                    # entonces, se actualiza su padre
                    c_min = self.Cost(x_new)
                    # Revisa si conectar x_new a cada nodo cercano (x_near) reduce el costo total.
                    for x_near in X_near:
                        c_near = self.Cost(x_near)
                        c_new = c_min + self.Line(x_new, x_near)
                        if c_new < c_near:
                            # Si el nuevo costo es menor al anterior, actualiza el padre de x_near a x_new
                            x_near.parent = x_new

                # Sí se ha encontrado un camino inicial hacia el objetivo
                # Cambio de fase a una enfocada a optimizar
                if not InitPathFlag and self.InitialPathFound(x_new):
                    InitPathFlag = True
                    n = k
                # Optimización principal
                if InitPathFlag:
                    print("por llamar a la función")
                    self.PathOptimization(x_new)
                    self.path = self.ExtractPath()
                    elapsed_time = time.time() - start_time  # Tiempo en segundos
                    print(f"Tiempo de ejecución RRt* smart:\t {elapsed_time:.6f} segundos")
                    return

                if k % 5 == 0:
                    pass
        #
        # self.path = self.ExtractPath()
        # print(f"la kkk es de: {n}")
        # elapsed_time = time.time() - start_time  # Tiempo en segundos
        # print(f"Tiempo de ejecución RRt* smart:\t {elapsed_time:.6f} segundos")

    def PathOptimization(self, node):
        """
        Intenta encontrar una ruta más directa hacia la meta optimizando los nodos de la ruta.
        """
        # Inicializa el costo acumulado del camino
        direct_cost_new = 0.0
        # Representa la meta del camino (nodo objetivo)
        node_end = self.x_goal

        # Recorre los nodos hacia atrás
        while node.parent:
            # Empieza en el nodo actual y recorre hacia atras usando los nodos padres
            # Evaluar si es posible saltar nodos intermedios y conectar directamente
            # los nodos más lejanos sin colisiones
            node_parent = node.parent
            p_x = node.x
            p_y = node.y
            parent = node.parent

            # Verifica si Hay una Conexión Directa sin Colisión
            if not self.utils.is_collision(node_parent, node_end):
                node_end.parent = node_parent
            else:
                # acumula el costo de la conexión entre el nodo actual y el nodo final actual (node_end)
                direct_cost_new += self.Line(node, node_end)
                node_end = node

            # Actualiza el nodo actual al padre del nodo, para seguir evaluando más hacia atrás en la ruta.
            node = node_parent

        if direct_cost_new < self.direct_cost_old:
            # actualiza el costo total
            self.direct_cost_old = direct_cost_new
            # Ajusta las balizas (beacons) para enfocar la exploración hacia áreas más relevantes,
            # basándose en la nueva ruta encontrada
            self.UpdateBeacons()

    def UpdateBeacons(self):
        """
        Actualiza los puntos de referencia (beacons) basados en los nodos cercanos a los obstáculos.
        """
        node = self.x_goal
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
                obs_vertex.append(vertex)

        self.obs_vertex = obs_vertex

    def Steer(self, x_start, x_goal):
        """
        Expande el árbol hacia un punto objetivo, limitando el paso a la longitud máxima definida.
        """
        dist, theta = self.get_distance_and_angle(x_start, x_goal)
        dist = min(self.step_len, dist)
        node_new = Node((x_start.x + dist * math.cos(theta),
                         x_start.y + dist * math.sin(theta)))
        node_new.parent = x_start

        return node_new

    def Near(self, nodelist, node):
        """
        Encuentra los nodos cercanos dentro de un radio definido para posibles conexiones.
        """
        n = len(self.V) + 1
        r = 50 * math.sqrt((math.log(n) / n))

        dist_table = [(nd.x - node.x) ** 2 + (nd.y - node.y) ** 2 for nd in nodelist]
        X_near = [nodelist[ind] for ind in range(len(dist_table)) if dist_table[ind] <= r ** 2 and
                  not self.utils.is_collision(node, nodelist[ind])]

        return X_near

    def Sample(self, goal=None):
        """
        Genera un punto aleatorio en el espacio de búsqueda, con cierta probabilidad de elegir directamente la meta.
        """
        if goal is None:
            delta = self.utils.delta
            goal_sample_rate = self.goal_sample_rate

            if np.random.random() > goal_sample_rate:
                return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                             np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

            return self.x_goal
        else:
            R = self.beacons_radius
            r = random.uniform(0, R)
            theta = random.uniform(0, 2 * math.pi)
            ind = random.randint(0, len(goal) - 1)

            return Node((goal[ind][0] + r * math.cos(theta),
                         goal[ind][1] + r * math.sin(theta)))

    def SampleFreeSpace(self):
        """
        Genera una muestra aleatoria en el espacio libre.
        Si un número aleatorio es mayor que goal_sample_rate, selecciona una muestra al azar dentro del entorno.
        Si no, devuelve la posición del nodo objetivo (x_goal), lo que fomenta que el algoritmo explore la zona cerca
        de la meta con cierta probabilidad.
        """
        delta = self.delta

        if np.random.random() > self.goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return self.x_goal

    def ExtractPath(self):
        """
        Extrae la ruta desde la meta hacia el inicio utilizando los punteros a los nodos padres.
        """
        path = []
        node = self.x_goal

        while node.parent:
            path.append([node.x, node.y])
            node = node.parent

        path.append([self.x_start.x, self.x_start.y])

        return path

    def InitialPathFound(self, node):
        """
        Determina si se ha encontrado un camino inicial que conecta un nodo con la meta.
        Devuelve True si la distancia entre el nodo y el objetivo (x_goal) es menor que step_len.
        """
        if self.Line(node, self.x_goal) < self.step_len:
            return True

        return False

    @staticmethod
    def Nearest(nodelist, n):
        """
        Encuentra el nodo más cercano en la lista de nodos existentes.
        Devuelve el nodo con la menor distancia al objetivo.
        """
        return nodelist[int(np.argmin([(nd.x - n.x) ** 2 + (nd.y - n.y) ** 2
                                       for nd in nodelist]))]

    @staticmethod
    def Line(x_start, x_goal):
        """
        Calcula la distancia euclidiana entre dos nodos.
        """
        return math.hypot(x_goal.x - x_start.x, x_goal.y - x_start.y)

    @staticmethod
    def Cost(node):
        """
        Calcula el costo total del camino desde un nodo hasta la raíz del árbol.
        """
        cost = 0.0
        if node.parent is None:
            return cost

        while node.parent:
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent

        return cost

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        """
        Calcula la distancia y el ángulo entre dos nodos.
        """
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    @staticmethod
    def plot_path(path, list_obs, color='red'):
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
            RrtStarSmart.graficar_circulo((cx, cy), r)
        for rec in obs_rectangle:
            v1, v2, v3, v4 = rec
            RrtStarSmart.graficar_rectangulo(v1, v2, v3, v4)

        plt.plot([x[0] for x in path], [x[1] for x in path], linewidth=2, color=color)
        # plt.pause(0.01)
        # nombre_archivo = RrtStarSmart.generar_nombre_archivo("grafico")
        plt.grid()
        # plt.savefig(nombre_archivo, dpi=300, bbox_inches='tight')
        plt.show()
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


def main():
    x_start = (2, 2)  # Starting node
    x_goal = (1400, 600)  # Goal node
    list_obs = [
        [300, 250, 52*2, 70*2, math.radians(45)],
        [700, 500, 52*2, 70*2, math.radians(180)],
        [600, 100, 52*2, 70*2, math.radians(180)],
        [1000, 400, 50*2]
    ]
    rrt = RrtStarSmart(x_start, x_goal, 50, 0.50, 5, 2000, list_obs)
    rrt.planning()
    RrtStarSmart.plot_path(rrt.path, list_obs)


if __name__ == '__main__':
    for _ in range(10):
        main()
