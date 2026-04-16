import os
import logging
import time
import math
import copy
import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from .tools_for_path_planing import Env, Utils

log = logging.getLogger(__name__)


class Node:
    """Representa un nodo en el árbol RRT*.

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


# Representación de una conexión entre dos nodos
class Edge:
    """Representa una conexión entre dos nodos.

    Attributes:
        parent (Node): Nodo padre de la arista.
        child (Node): Nodo hijo de la arista.
        flag (str): Estado de la arista ('VALID' por defecto).
    """
    def __init__(self, n_p, n_c):
        """Inicializa una arista entre dos nodos.

        Args:
            n_p (Node): Nodo padre.
            n_c (Node): Nodo hijo.
        """
        self.parent = n_p
        self.child = n_c
        self.flag = "VALID"


# Implementación del algoritmo RRt* smart
class RrtStarSmart:
    """Implementación del algoritmo RRT* Smart para planificación de rutas.

    RRT* Smart es una versión optimizada del algoritmo RRT* que utiliza
    balizas para guiar la exploración y optimización de rutas dinámicamente.

    Attributes:
        x_start (Node): Nodo inicial.
        x_goal (Node): Nodo objetivo.
        step_len (float): Longitud del paso para expansión.
        goal_sample_rate (float): Probabilidad de muestrear hacia el objetivo.
        search_radius (float): Radio de búsqueda para nodos cercanos.
        iter_max (int): Número máximo de iteraciones.
        V (list): Lista de nodos válidos en el árbol.
        beacons (list): Lista de balizas para guiar la exploración.
        path (list): Ruta final encontrada.
    """
    def __init__(self, step_len, goal_sample_rate, search_radius,
                 iter_max, list_obs=None, x_start=None, x_goal=None, field=None,
                 clearance=None):
        """Inicializa el planificador RRT* Smart.

        Args:
            step_len (float): Longitud del paso para expansión del árbol.
            goal_sample_rate (float): Probabilidad de muestrear hacia el objetivo (0-1).
            search_radius (float): Radio de búsqueda para encontrar nodos cercanos.
            iter_max (int): Número máximo de iteraciones del algoritmo.
            list_obs (list, optional): Lista de obstáculos en el entorno.
            x_start (tuple, optional): Coordenadas del punto inicial.
            x_goal (tuple, optional): Coordenadas del punto objetivo.
            field (FieldGeometry, optional): Geometría del campo. Por defecto FIELD_SIM.
            clearance (int, optional): Margen de seguridad (px) añadido a cada obstáculo.
                Si es None usa PATH_PLANNING_OBSTACLE_CLEARANCE de config.
        """
        self.x_start = Node(x_start) if x_start else None
        self.vertex = [self.x_start]
        self.x_goal = Node(x_goal) if x_goal else None

        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.iter_max = iter_max
        self._field = field  # guardar para setup() y replanificaciones internas
        self._clearance = clearance  # guardar para setup() y replanificaciones
        if list_obs is not None:
            self.env = Env(list_obs, field=field)
            self.utils = Utils(list_obs, field=field, delta=clearance)
            self.obs_vertex = self.utils.get_obs_vertex()
            self.obs_circle = self.env.obs_circle
            self.obs_rectangle = self.env.obs_rectangle
            self.obs_boundary = self.env.obs_boundary

            self.delta = self.utils.delta

            self.x_range = self.env.x_range
            self.y_range = self.env.y_range

        self.beacons = []
        self.beacons_radius = 2
        self.direct_cost_old = np.inf
        self.path = None

        # Para la replanificación (dynamic)
        waypoint_sample_rate = 0.6
        self.waypoint_sample_rate = waypoint_sample_rate
        # self.vertex = [self.x_start]
        self.vertex_old = []
        self.vertex_new = []
        self.edges = []
        self.waypoint = []
        self.obs_rect_modify = []
        self.obs_cir_modify = []

        self.last_node = None

    def setup(self, x_start, x_goal, list_obs, field=None, clearance=None):
        """Configura el planificador con nuevos parámetros.

        Args:
            x_start (tuple): Coordenadas del punto inicial.
            x_goal (tuple): Coordenadas del punto objetivo.
            list_obs (list): Lista de obstáculos en el entorno.
            field (FieldGeometry, optional): Geometría del campo. Usa self._field si no se pasa.
            clearance (int, optional): Margen de seguridad (px) añadido a cada obstáculo.
                Si es None usa self._clearance (pasado en el constructor).
        """
        if x_start is not None:
            self.x_start = Node(x_start)
            self.vertex = [self.x_start]

        if x_goal is not None:
            self.x_goal = Node(x_goal)

        _field = field if field is not None else getattr(self, '_field', None)
        _clearance = clearance if clearance is not None else getattr(self, '_clearance', None)
        if list_obs is not None:
            self.env = Env(list_obs, field=_field)
            self.utils = Utils(list_obs, field=_field, delta=_clearance)
            self.obs_vertex = self.utils.get_obs_vertex()
            self.obs_circle = self.env.obs_circle
            self.obs_rectangle = self.env.obs_rectangle
            self.obs_boundary = self.env.obs_boundary

            self.delta = self.utils.delta

            self.x_range = self.env.x_range
            self.y_range = self.env.y_range

    def planning(self):
        """Ejecuta el algoritmo principal de planificación de rutas.

        Realiza el muestreo, expansión, conexión y optimización de la ruta
        utilizando el algoritmo RRT* Smart con balizas para guiar la exploración.
        """
        start_time = time.time()

        n = 0  # Guarda el valor del iterador k
        b = 2
        init_path_flag = False
        self.reform_obs_vertex()  # Ajuste de los vértices de los obstáculos
        # n y b: Se utilizan para controlar la frecuencia de muestreo utilizando balizas (beacons).

        for k in range(self.iter_max):
            if k % 200 == 0:
                log.debug(k)

            if (k - n) % b == 0 and len(self.beacons) > 0:
                # Si (k - n) es múltiplo de b y hay balizas disponibles, realiza el muestreo usando las balizas.
                # Esto mejora la exploración guiada hacia el objetivo.
                x_rand = self.sample(self.beacons)
            else:
                # Muestreo aleatorio estándar
                x_rand = self.sample()

            # Encuentra el nodo en el árbol más cercano al punto muestreado x_rand
            x_nearest = self.nearest(self.vertex, x_rand)
            # Intenta conectar el nodo cercano x_nearest con el punto muestreado x_rand, generando un nuevo nodo x_new
            x_new = self.steer(x_nearest, x_rand)

            if x_new and not self.utils.is_collision(x_nearest, x_new):
                # Encuentra los nodos cercanos en el árbol al nuevo nodo x_new
                x_near = self.near(self.vertex, x_new)
                # Agrega x_new al conjunto de nodos
                self.vertex.append(x_new)

                # self.edges.append(Edge(x_nearest, x_new))

                if x_near:
                    # Elegir padre,
                    # Para la optimización, al elegir al padre con el menor costo acumulado
                    # distancia desde el nodo inicial más la distancia al nuevo nodo
                    #
                    # Calcula el costo total para conectar cada nodo cercano (_x_near) con x_new.
                    cost_list = [self.cost(_x_near) + self.line(_x_near, x_new) for _x_near in x_near]
                    # Selecciona el nodo con menor costo acumulado como padre del nuevo nodo
                    x_new.parent = x_near[int(np.argmin(cost_list))]

                    # rewire: mejora los caminos dinámicamente, reconexión dinámica
                    # Si la nueva conexión a un nodo cercano reduce el costo total del camino hacia ese nodo
                    # entonces, se actualiza su padre
                    c_min = self.cost(x_new)
                    # Revisa si conectar x_new a cada nodo cercano (_x_near) reduce el costo total.
                    for _x_near in x_near:
                        c_near = self.cost(_x_near)
                        c_new = c_min + self.line(x_new, _x_near)
                        if c_new < c_near:
                            # Si el nuevo costo es menor al anterior, actualiza el padre de _x_near a x_new
                            _x_near.parent = x_new

                # Sí se ha encontrado un camino inicial hacia el objetivo
                # Cambio de fase a una enfocada a optimizar
                if not init_path_flag and self.initial_path_found(x_new):
                    init_path_flag = True
                    n = k
                # Optimización principal
                if init_path_flag:
                    self.waypoint = self.extract_waypoint(x_new)
                    self.path_optimization(x_new)
                    path = self.extract_path()
                    # Convertir todos los números a enteros
                    path = [[int(num) for num in sublist] for sublist in path]
                    self.path = path
                    elapsed_time = time.time() - start_time  # Tiempo en segundos
                    log.debug(
                        "Tiempo de ejecución RRt* smart:\t %.6f segundos", elapsed_time
                    )
                    self.conect_edges()

                    # layers = self.count_layers(x_new) borrar si no se usa

                    return

                if k % 5 == 0:
                    pass
        #
        # self.path = self.extract_path()
        # print(f"la kkk es de: {n}")
        # elapsed_time = time.time() - start_time  # Tiempo en segundos
        # print(f"Tiempo de ejecución RRt* smart:\t {elapsed_time:.6f} segundos")

    def path_optimization(self, node):
        """Optimiza la ruta encontrada mediante salto de nodos intermedios.

        Intenta encontrar una ruta más directa hacia la meta eliminando
        nodos intermedios innecesarios cuando existe línea de vista directa.

        Args:
            node (Node): Nodo desde el cual iniciar la optimización.
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

            # Verifica si Hay una Conexión Directa sin Colisión
            if not self.utils.is_collision(node_parent, node_end):
                node_end.parent = node_parent
            else:
                # acumula el costo de la conexión entre el nodo actual y el nodo final actual (node_end)
                direct_cost_new += self.line(node, node_end)
                node_end = node

            # Actualiza el nodo actual al padre del nodo, para seguir evaluando más hacia atrás en la ruta.
            node = node_parent

        if direct_cost_new < self.direct_cost_old:
            # actualiza el costo total
            self.direct_cost_old = direct_cost_new
            # Ajusta las balizas (beacons) para enfocar la exploración hacia áreas más relevantes,
            # basándose en la nueva ruta encontrada
            self.update_beacons()

    def update_beacons(self):
        """Actualiza los puntos de referencia (beacons) basados en los nodos cercanos a obstáculos.

        Las balizas ayudan a guiar la exploración del algoritmo hacia regiones
        que han demostrado ser útiles para encontrar rutas óptimas.
        """
        node = self.x_goal
        beacons = []
        while node.parent:
            near_vertex = [
                v
                for v in self.obs_vertex
                if (node.x - v[0]) ** 2 + (node.y - v[1]) ** 2 < 9
            ]
            if len(near_vertex) > 0:
                for v in near_vertex:
                    beacons.append(v)

            node = node.parent

        self.beacons = beacons

    def reform_obs_vertex(self):
        """Reforma la lista de vértices de obstáculos para facilitar su manejo.

        Convierte la estructura anidada de vértices de obstáculos en una
        lista plana para optimizar las operaciones de búsqueda.
        """
        obs_vertex = []
        for obs in self.obs_vertex:
            for vertex in obs:
                obs_vertex.append(vertex)

        self.obs_vertex = obs_vertex

    def steer(self, x_start, x_goal):
        """Expande el árbol hacia un punto objetivo con longitud de paso limitada.

        Args:
            x_start (Node): Nodo de inicio para la expansión.
            x_goal (Node): Nodo objetivo hacia el que expandir.

        Returns:
            Node: Nuevo nodo generado en la dirección del objetivo.
        """
        dist, theta = self.get_distance_and_angle(x_start, x_goal)

        dist = min(self.step_len, dist)
        node_new = Node(
            (x_start.x + dist * math.cos(theta), x_start.y + dist * math.sin(theta))
        )
        node_new.parent = x_start

        return node_new

    def near(self, nodelist, node):
        """Encuentra nodos cercanos dentro de un radio dinámico para conexiones RRT*.

        Args:
            nodelist (list): Lista de nodos existentes en el árbol.
            node (Node): Nodo para el cual buscar vecinos cercanos.

        Returns:
            list: Lista de nodos cercanos sin colisiones.
        """
        n = len(self.vertex) + 1
        r = 50 * math.sqrt((math.log(n) / n))

        dist_table = [(nd.x - node.x) ** 2 + (nd.y - node.y) ** 2 for nd in nodelist]
        x_near = [
            nodelist[ind]
            for ind in range(len(dist_table))
            if dist_table[ind] <= r**2
            and not self.utils.is_collision(node, nodelist[ind])
        ]

        return x_near

    def sample(self, goal=None):
        """Genera un punto aleatorio en el espacio de búsqueda.

        Con cierta probabilidad elige directamente la meta o usa balizas
        para guiar la exploración hacia regiones prometedoras.

        Args:
            goal (list, optional): Lista de balizas para muestreo guiado.

        Returns:
            Node: Nodo muestreado aleatoriamente o cerca de una baliza.
        """
        if goal is None:
            delta = self.utils.delta
            goal_sample_rate = self.goal_sample_rate

            if np.random.random() > goal_sample_rate:
                return Node(
                    (
                        np.random.uniform(
                            self.x_range[0] + delta, self.x_range[1] - delta
                        ),
                        np.random.uniform(
                            self.y_range[0] + delta, self.y_range[1] - delta
                        ),
                    )
                )

            return self.x_goal
        radius = self.beacons_radius
        r = random.uniform(0, radius)
        theta = random.uniform(0, 2 * math.pi)
        ind = random.randint(0, len(goal) - 1)

        return Node(
            (goal[ind][0] + r * math.cos(theta), goal[ind][1] + r * math.sin(theta))
        )

    def sample_free_space(self):
        """Genera una muestra aleatoria en el espacio libre.

        Si un número aleatorio es mayor que goal_sample_rate, selecciona una
        muestra al azar dentro del entorno. Si no, devuelve la posición del
        nodo objetivo, fomentando la exploración cerca de la meta.

        Returns:
            Node: Nodo muestreado en el espacio libre o el nodo objetivo.
        """
        delta = self.delta

        if np.random.random() > self.goal_sample_rate:
            return Node(
                (
                    np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                    np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta),
                )
            )

        return self.x_goal

    def extract_path(self):
        """Extrae la ruta desde la meta hacia el inicio.

        Utiliza los punteros a los nodos padres para reconstruir
        la ruta completa desde el objetivo hasta el punto inicial.

        Returns:
            list: Lista de coordenadas [x, y] que forman la ruta.
        """
        path = []
        node = self.x_goal

        while node.parent:
            path.append([node.x, node.y])
            node = node.parent

        path.append([self.x_start.x, self.x_start.y])

        return path

    def initial_path_found(self, node):
        """Determina si se ha encontrado un camino inicial al objetivo.

        Args:
            node (Node): Nodo a verificar si está cerca del objetivo.

        Returns:
            bool: True si la distancia al objetivo es menor que step_len.
        """
        if self.line(node, self.x_goal) < self.step_len:
            return True

        return False

    # ******************************************************************************************
    # ------------------------------------------ For dynamic -----------------------------------
    # ******************************************************************************************

    def update_obstacle(self, n_list_obs, start_new):
        """Actualiza obstáculos y replanifico si es necesario.

        Verifica si los obstáculos han cambiado y si la ruta actual
        sigue siendo válida. Si no, ejecuta replanificación dinámica.

        Args:
            n_list_obs (list): Nueva lista de obstáculos.
            start_new (tuple): Nueva posición de inicio.
        """
        # Encuentra deiferencias entre los obstáculos anteriores y los nuevos
        diferencia_rect = self.comparar_listas(
            self.obs_rectangle, Env.obs_rectangle_met(n_list_obs)
        )
        diferencia_circ = self.comparar_listas(
            self.obs_circle, Env.obs_circle_met(n_list_obs)
        )

        if diferencia_rect or diferencia_circ:
            if diferencia_rect:
                self.obs_rect_modify = diferencia_rect
            if diferencia_circ:
                self.obs_cir_modify = diferencia_circ[0]

            # Actualiza dentro de env los obtáculos
            self.env.update_obtacle(n_list_obs)

            # Actualiza dentro de utils los obstáculos
            self.utils.update_obstacle(n_list_obs)

            # Reasigna los valores de los obstáculos dentro de la clase
            self.obs_rectangle = self.env.obs_rectangle
            self.obs_circle = self.env.obs_circle

            # Obtiene los vertices de cada obtáculos rectángular
            # Vetices_de_los_obstaculos = self.utils.get_obs_vertex()

            # Reforma la lista de vértices de los obstáculos

            # Marca las aristas como inválidas
            self.invalidate_nodes()

            # La ruta actual es válida?
            if self.is_path_invalid():
                # self.reform_obs_vertex()
                path, waypoint = self.replanning()
                # Convertir todos los valores en enteros
                path = [[int(value) for value in sublist] for sublist in path]
                self.vertex_new = []
                self.path = path
                self.waypoint = waypoint

            else:
                self.trim_rrt()

        elif math.hypot(self.x_start.x - start_new[0],
                        self.x_start.y - start_new[1]) > 30:
            self.setup(start_new, (self.x_goal.x, self.x_goal.y), n_list_obs)
            self.planning()

        else:
            log.debug("no se encontraron diferencias, se consevó la misma ruta")

    def extract_waypoint(self, node_end):
        """Extrae los puntos de referencia (waypoints) desde la meta al inicio.

        Args:
            node_end (Node): Nodo final desde donde extraer waypoints.

        Returns:
            list: Lista de nodos que forman los waypoints de la ruta.
        """
        waypoint = [self.x_goal]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            waypoint.append(node_now)

        return waypoint

    def invalidate_nodes(self):
        """Marca nodos como inválidos si colisionan con nuevos obstáculos.

        Recorre todas las aristas del árbol y marca como inválidos
        aquellos nodos cuyas conexiones colisionan con obstáculos modificados.
        """
        for edge in self.edges:
            if self.is_collision_obs_modify(edge.parent, edge.child):
                edge.child.flag = "INVALID"
        # -------------------------------------------------------
        self.last_node = None
        nodo_now = self.x_goal
        while nodo_now.parent:
            if nodo_now.flag == "INVALID":
                self.last_node = nodo_now.parent
            nodo_now = nodo_now.parent

    def is_collision_obs_modify(self, start, end):
        """Verifica si una arista colisiona con obstáculos modificados dinámicamente.

        Args:
            start (Node): Nodo de inicio de la arista.
            end (Node): Nodo final de la arista.

        Returns:
            bool: True si la arista colisiona con algún obstáculo modificado.
        """
        delta = self.utils.delta
        if self.obs_cir_modify:
            obs_cir = self.obs_cir_modify

            if (
                math.hypot(start.x - obs_cir[0], start.y - obs_cir[1])
                <= obs_cir[2] + delta
            ):
                return True

            if math.hypot(end.x - obs_cir[0], end.y - obs_cir[1]) <= obs_cir[2] + delta:
                return True

            o, d = self.utils.get_ray(start, end)

            if self.utils.is_intersect_circle(
                o, d, [obs_cir[0], obs_cir[1]], obs_cir[2]
            ):
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

    def is_path_invalid(self):
        """Comprueba si la ruta actual contiene nodos inválidos.

        Returns:
            bool: True si algún nodo en la ruta es inválido, indicando
                  necesidad de replanificar.
        """
        for node in self.waypoint:
            if node.flag == "INVALID":
                return True
        return False

    def replanning(self):
        """Ejecuta replanificación cuando la ruta actual se vuelve inválida.

        Elimina nodos inválidos y busca una nueva ruta desde los nodos
        válidos restantes hasta el objetivo.

        Returns:
            tuple: Nueva ruta y waypoints (path, waypoint).
        """
        # Quita nodos y aristas inválidos
        self.trim_rrt()
        self.x_goal.flag = "VALID"
        # Si self.x_goal está en self.vertex, elimínalo
        if self.x_goal in self.vertex:
            self.vertex.remove(self.x_goal)

        for _ in range(self.iter_max):
            # Genera nodo aleatorio
            node_rand = self.generate_random_node_replanning(self.goal_sample_rate)
            # Encuentra el nodo más cercano en el árbol
            node_near = self.nearest(self.vertex, node_rand)
            # Calcula un nuevo nodo en la dirección del nuevo nodo generado
            node_new = self.steer(node_near, node_rand)

            if node_new and not self.utils.is_collision(node_near, node_new):
                self.vertex.append(node_new)
                self.vertex_new.append(node_new)
                self.edges.append(Edge(node_near, node_new))
                dist, _ = self.get_distance_and_angle(node_new, self.x_goal)

                if dist <= self.step_len:
                    self.x_goal.parent = None  # Eliminar la ruta anterior
                    # self.steer(node_new, self.x_goal)
                    self.path_optimization(node_new)
                    path = self.extract_path()
                    # Convertir todos los números a enteros
                    path = [[int(num) for num in sublist] for sublist in path]

                    waypoint = self.extract_waypoint(node_new)
                    self.conect_edges()
                    # print("path: ", len(path))
                    # print("waypoint: ", len(waypoint))

                    return path, waypoint
        log.debug("return none")
        return self.planning()

    def trim_rrt(self):
        """Elimina nodos inválidos del árbol manteniendo conectividad.

        Recorre la lista de nodos y elimina aquellos marcados como inválidos
        junto con sus descendientes, manteniendo solo nodos válidos y sus
        padres válidos.
        """
        for i in range(len(self.vertex) - 1):
            node = self.vertex[i]
            if node.parent:
                node_p = node.parent
                if node_p.flag == "INVALID":
                    node.flag = "INVALID"

        cleaned_nodes = []

        for node in self.vertex:
            current = node
            node_invalid = False
            node_valid = True
            aux_node = None
            # Verificar la cadena de padres
            while current:
                if current.flag == "INVALID":
                    node_invalid = True
                    node_valid = False

                if node_invalid and current.flag == "VALID":
                    aux_node = current
                    node_invalid = False

                current = current.parent
            if node_valid:
                cleaned_nodes.append(node)
            if aux_node:
                cleaned_nodes.append(aux_node)

        # Actualizar la lista de nodos con solo los válidos
        self.vertex = cleaned_nodes

    def trim_rrt2(self):
        """Versión alternativa de recorte que elimina nodos inválidos.

        Recorta los nodos y aristas inválidos del árbol en caso de cambios
        en el entorno, optimizando la eficiencia en planificación posterior.
        """
        for i in range(1, len(self.vertex) - 1):
            node = self.vertex[i]
            node_p = node.parent
            if node_p.flag == "INVALID":
                node.flag = "INVALID"

        new_v = []
        for node in reversed(self.vertex):
            if node.flag == "INVALID":
                break
            new_v.append(node)
        self.vertex = new_v

        new_v = []
        for node in reversed(self.waypoint):
            if node.flag == "INVALID":
                break
            new_v.append(node)
        self.waypoint = new_v
        self.vertex_old = copy.deepcopy(self.vertex)
        self.edges = [Edge(node.parent, node) for node in self.vertex[1 : len(self.vertex)]]
        self.waypoint = [node for node in self.waypoint if node.flag == "VALID"]
        # a = 0

    def generate_random_node_replanning(self, goal_sample_rate):
        """Genera un nodo aleatorio para replanificación.

        Con probabilidad de muestrear hacia el objetivo o puntos intermedios
        durante el proceso de replanificación dinámica.

        Args:
            goal_sample_rate (float): Probabilidad de muestrear hacia el objetivo.

        Returns:
            Node: Nodo generado aleatoriamente o hacia el objetivo.
        """
        delta = self.utils.delta
        p = np.random.random()

        if p < goal_sample_rate:
            return self.x_goal
        # elif goal_sample_rate < p < goal_sample_rate + waypoint_sample_rate:
        #     return self.waypoint[np.random.randint(0, len(self.waypoint) - 1)]
        return Node(
            (
                np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta),
            )
        )

    def conect_edges(self):
        """Genera aristas desde la meta hacia el inicio usando punteros padre.

        Crea una lista de aristas que representa la ruta final optimizada
        desde el objetivo hasta el punto de inicio.
        """
        # Crear una lista temporal para almacenar los nodos de la ruta final
        ruta_final = []

        node = self.x_goal  # Nodo objetivo desde donde empieza el recorrido

        while node.parent:
            # Nodo actual (x_new) y su padre (x_nearest)
            x_new = node
            x_nearest = node.parent

            # Añadir la arista entre el nodo actual y su padre
            self.edges.append(Edge(x_nearest, x_new))

            # Añadir el nodo actual a la ruta final
            ruta_final.append(x_new)

            # Avanzar al siguiente nodo en el camino hacia atrás
            node = node.parent

        # Añadir el nodo de inicio a la ruta final
        ruta_final.append(self.x_start)

        # Actualizar self.vertex con solo los nodos de la ruta optimizada
        self.vertex = ruta_final

    @staticmethod
    def count_layers(node):
        """Cuenta la cantidad de nodos desde el nodo final hasta el inicial.

        Args:
            node (Node): Nodo desde el cual contar hacia atrás.

        Returns:
            int: Número de capas desde el nodo dado hasta la raíz.
        """
        layers = 0
        while node:
            layers += 1
            node = node.parent  # Se mueve al nodo padre
        return layers

    @staticmethod
    def nearest(nodelist, n):
        """Encuentra el nodo más cercano en la lista de nodos existentes.

        Args:
            nodelist (list): Lista de nodos existentes.
            n (Node): Nodo objetivo para encontrar el más cercano.

        Returns:
            Node: Nodo con menor distancia euclidiana al objetivo.
        """
        return nodelist[
            int(np.argmin([(nd.x - n.x) ** 2 + (nd.y - n.y) ** 2 for nd in nodelist]))
        ]

    @staticmethod
    def line(x_start, x_goal):
        """Calcula la distancia euclidiana entre dos nodos.

        Args:
            x_start (Node): Nodo de inicio.
            x_goal (Node): Nodo de destino.

        Returns:
            float: Distancia euclidiana entre los dos nodos.
        """
        return math.hypot(x_goal.x - x_start.x, x_goal.y - x_start.y)

    @staticmethod
    def cost(node):
        """Calcula el costo total del camino desde un nodo hasta la raíz.

        Args:
            node (Node): Nodo para el cual calcular el costo acumulado.

        Returns:
            float: Costo total del camino desde el nodo hasta la raíz del árbol.
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
        """Calcula la distancia y el ángulo entre dos nodos.

        Args:
            node_start (Node): Nodo de inicio.
            node_end (Node): Nodo final.

        Returns:
            tuple: Tupla con (distancia, ángulo) entre los nodos.
        """
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    @staticmethod
    def comparar_listas(lista1, lista2):
        """Compara dos listas de obstáculos y devuelve los elementos diferentes.

        Args:
            lista1 (list): Lista de obstáculos original.
            lista2 (list): Lista de obstáculos nueva.

        Returns:
            list: Elementos que están en lista2 pero no en lista1.
        """
        diferencias = []

        # Verifica cada elemento de lista2 que no esté en lista1
        for elem in lista2:
            if elem not in lista1:
                diferencias.append(elem)

        return diferencias

    @staticmethod
    def plot_path(path, list_obs, show, tiempo=None):
        """Dibuja la ruta final planificada y los obstáculos.

        Args:
            path (list): Lista de coordenadas que forman la ruta.
            list_obs (list): Lista de obstáculos a dibujar.
            show (bool): Si True, muestra el gráfico; si False, lo guarda.
            tiempo (float, optional): Tiempo de ejecución para mostrar en título.
        """
        # Crear la figura y los ejes
        plt.figure(figsize=(8, 6))
        # plt.axis("equal")  # Mantener las proporciones iguales

        plt.xlim(-100, 1500)  # Límite del eje x entre 0 y 10
        plt.ylim(-100, 900)  # Límite del eje y entre -1.5 y 1.5

        env_ = Env(list_obs)
        obs_circle = env_.obs_circle
        obs_rectangle = env_.obs_rectangle
        for cir in obs_circle:
            cx, cy, r = cir
            RrtStarSmart.graficar_circulo((cx, cy), r)
        for rec in obs_rectangle:
            v1, v2, v3, v4 = rec
            RrtStarSmart.graficar_rectangulo(v1, v2, v3, v4)

        plt.plot([x[0] for x in path], [x[1] for x in path], linewidth=2, color="red")
        if tiempo is not None:
            plt.title(str(tiempo))
        # plt.pause(0.01)
        nombre_archivo = RrtStarSmart.generar_nombre_archivo("grafico")
        plt.grid()
        if show:
            plt.show()
        else:
            plt.savefig(nombre_archivo, dpi=300, bbox_inches="tight")
        plt.close()

    @staticmethod
    def graficar_circulo(centro, radio, color="b", etiqueta=None):
        """Grafica un círculo dado su centro y radio.

        Args:
            centro (tuple): Coordenadas (x, y) del centro del círculo.
            radio (float): Radio del círculo.
            color (str, optional): Color del círculo. Defaults to 'b'.
            etiqueta (str, optional): Etiqueta para la leyenda.
        """
        circle = plt.Circle(
            centro, radio, color=color, fill=False, linewidth=2, label=etiqueta
        )
        plt.gca().add_patch(circle)  # Añadir el círculo al gráfico

    @staticmethod
    def graficar_rectangulo(vertice1, vertice2, vertice3, vertice4, color="r", etiqueta=None):
        """Grafica un rectángulo dado por sus cuatro vértices.

        Args:
            vertice1 (tuple): Primer vértice del rectángulo.
            vertice2 (tuple): Segundo vértice del rectángulo.
            vertice3 (tuple): Tercer vértice del rectángulo.
            vertice4 (tuple): Cuarto vértice del rectángulo.
            color (str, optional): Color del rectángulo. Defaults to 'r'.
            etiqueta (str, optional): Etiqueta para la leyenda.
        """
        # Crear un polígono usando los 4 vértices
        rect = Polygon(
            [vertice1, vertice2, vertice3, vertice4],
            closed=True,
            edgecolor=color,
            fill=False,
            linewidth=2,
            label=etiqueta,
        )
        plt.gca().add_patch(rect)  # Añadir el polígono al gráfico

    @staticmethod
    def generar_nombre_archivo(base_name, extension="png"):
        """Genera un nombre de archivo único si el original ya existe.

        Args:
            base_name (str): Nombre base para el archivo.
            extension (str, optional): Extensión del archivo. Defaults to 'png'.

        Returns:
            str: Nombre de archivo único con ruta completa.
        """
        counter = 1
        nombre_final = f"./graf/{base_name}.{extension}"

        # Mientras el archivo exista, incrementar el contador y cambiar el nombre
        while os.path.exists(nombre_final):
            nombre_final = f"./graf/{base_name}_{counter}.{extension}"
            counter += 1

        return nombre_final
