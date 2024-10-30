"""
D_star_Lite 2D
@author: huiming zhou
"""

# import os
# import sys
import math
import matplotlib.pyplot as plt
import time

import env


class DStar:
    def __init__(self, s_start, s_goal, heuristic_type):

        self.s_start, self.s_goal = s_start, s_goal  # Puntos de incio y punto objetivo
        self.heuristic_type = heuristic_type  # Tipo de búsqueda

        self.Env = env.Env()  # Clase que contiene los obstáculos y bordes

        self.u_set = self.Env.motions  # conjunto de entrada factible
        self.obs = self.Env.obs  # position of obstacles
        self.x = self.Env.x_range
        self.y = self.Env.y_range

        self.g, self.rhs, self.U = {}, {}, {}
        self.km = 0

        for i in range(1, self.Env.x_range - 1):
            for j in range(1, self.Env.y_range - 1):
                self.rhs[(i, j)] = float("inf")
                self.g[(i, j)] = float("inf")

        self.rhs[self.s_goal] = 0.0
        self.U[self.s_goal] = self.CalculateKey(self.s_goal)
        self.visited = set()
        self.count = 0
        self.fig = plt.figure()

    def run(self):
        self.ComputePath()  # calcular la ruta
        # self.plot_path(self.extract_path())
        return self.extract_path()

    def monitor_obstacles(self, new_obstacle_list, interval=0.1):
        old_obstacle_set = set(self.obs)
        new_obstacle_set = set(new_obstacle_list)

        while True:
            if old_obstacle_set != new_obstacle_set:
                print("Cambios en obstáculos detectados, recalculando la ruta...")
                self.update_obstacles(list(old_obstacle_set), list(new_obstacle_set))
                self.recalculate_path()
                old_obstacle_set = new_obstacle_set

            time.sleep(interval)

    def monitor_start_goal(self, new_start, new_goal):
        """
        Verifica solo los cambios en el punto de inicio y meta, y replantea la ruta si hay cambios.
        """
        old_start = self.s_start
        old_goal = self.s_goal

        if old_start != new_start or old_goal != new_goal:
            print("Cambios en inicio o meta detectados, recalculando la ruta...")
            if old_start != new_start:
                self.s_start = new_start
            if old_goal != new_goal:
                self.s_goal = new_goal

            self.recalculate_path()

    def recalculate_path(self):
        """
        Recalcula la ruta usando D* Lite de manera eficiente, solo si ha habido cambios en los obstáculos.
        """
        s_curr = self.s_start
        s_last = self.s_start
        i = 0
        path = [self.s_start]

        while s_curr != self.s_goal:
            s_list = {}

            for s in self.get_neighbor(s_curr):
                s_list[s] = self.g[s] + self.cost(s_curr, s)

            # s_curr = min(s_list, key=s_list.get)
            s_next = min(s_list, key=s_list.get)
            path.append(s_next)

            # Verifica si el siguiente paso ya no es válido (obstáculo) o si requiere actualización
            if s_next in self.obs or self.g[s_next] != self.rhs[s_next]:
                self.UpdateVertex(s_next)
                self.ComputePath()

            s_curr = s_next
        return path

    def update_obstacles(self, old_obstacle_list, new_obstacle_list):
        """
        Actualiza los obstáculos comparando la lista antigua con la nueva.
        Solo elimina y agrega coordenadas si ha habido cambios.
        """
        # Primero eliminar los obstáculos antiguos si han cambiado
        for old_obstacle, new_obstacle in zip(old_obstacle_list, new_obstacle_list):
            if old_obstacle != new_obstacle:
                self.remove_obstacle(old_obstacle)  # Eliminar la posición anterior del obstáculo
                self.add_obstacles(new_obstacle)  # Agregar la nueva posición del obstáculo

    def remove_obstacle(self, obstacle):
        """
        Elimina las coordenadas del obstáculo en la lista de obstáculos.
        """
        if len(obstacle) == 4:
            # Rectángulo: x, y, dx, dy
            x, y, dx, dy = obstacle
            for i in range(int(x - dx), int(x + dx) + 1):
                for j in range(int(y - dy), int(y + dy) + 1):
                    if (i, j) in self.obs:
                        self.obs.remove((i, j))  # Elimina las coordenadas del rectángulo
                        plt.plot(i, j, marker='s', color='white')  # Borra el rectángulo visualmente
        elif len(obstacle) == 3:
            # Círculo: x, y, r
            x, y, r = obstacle
            for i in range(int(x - r), int(x + r) + 1):
                for j in range(int(y - r), int(y + r) + 1):
                    if (i - x) ** 2 + (j - y) ** 2 <= r ** 2:
                        if (i, j) in self.obs:
                            self.obs.remove((i, j))  # Elimina las coordenadas del círculo
                            plt.plot(i, j, marker='s', color='white')  # Borra el círculo visualmente

    def add_obstacles(self, obstacle):
        """
        Agrega obstáculos a partir de una lista de vectores. Diferencia entre rectángulos y círculos.
        """
        if len(obstacle) == 4:
            # Rectángulo: x, y, dx, dy
            x, y, dx, dy = obstacle
            for i in range(int(x - dx), int(x + dx) + 1):
                for j in range(int(y - dy), int(y + dy) + 1):
                    self.obs.add((i, j))  # Agrega las coordenadas del rectángulo
                    self.g[(i, j)] = float("inf")
                    self.rhs[(i, j)] = float("inf")
                    plt.plot(i, j, 'sk')
        elif len(obstacle) == 3:
            # Círculo: x, y, r
            x, y, r = obstacle
            for i in range(int(x - r), int(x + r) + 1):
                for j in range(int(y - r), int(y + r) + 1):
                    if (i - x) ** 2 + (j - y) ** 2 <= r ** 2:
                        self.obs.add((i, j))  # Agrega las coordenadas del círculo
                        self.g[(i, j)] = float("inf")
                        self.rhs[(i, j)] = float("inf")
                        plt.plot(i, j, 'sk')

    def ComputePath(self):
        """
        Recalcula el camino si es necesario, usando la cola de prioridad.
        Solo modifica los nodos afectados por los cambios.
        """
        while True:
            # Obtiene el nodo con la menor clave (menor prioridad)
            s, v = self.TopKey()

            # Si el nodo inicial ya está optimizado o si no hay cambios en los vecinos
            if v >= self.CalculateKey(self.s_start) and self.rhs[self.s_start] == self.g[self.s_start]:
                break

            # Remueve el nodo actual de la cola de prioridad
            k_old = v
            self.U.pop(s)
            self.visited.add(s)

            # Caso 1: Si el valor de la clave cambió
            if k_old < self.CalculateKey(s):
                self.U[s] = self.CalculateKey(s)

            # Caso 2: Si el valor g del nodo es mayor que rhs, es necesario reducir g
            elif self.g[s] > self.rhs[s]:
                self.g[s] = self.rhs[s]
                # Actualiza los vecinos para reflejar el cambio
                for s_next in self.get_neighbor(s):
                    self.UpdateVertex(s_next)

            # Caso 3: Si el valor g es mayor o igual a rhs, aumentar el costo
            else:
                self.g[s] = float("inf")
                self.UpdateVertex(s)
                for s_next in self.get_neighbor(s):
                    self.UpdateVertex(s_next)

    def UpdateVertex(self, s):
        if s != self.s_goal:
            # Recalcula el valor de rhs sólo si hay un cambio en los obstáculos o costos
            self.rhs[s] = min([self.g[s_next] + self.cost(s, s_next) for s_next in self.get_neighbor(s)],
                              default=float("inf"))

        # Si el nodo ya está en la lista de prioridad, lo eliminas
        if s in self.U:
            self.U.pop(s)

        # Solo vuelves a insertar si hay un cambio entre g y rhs
        if self.g[s] != self.rhs[s]:
            self.U[s] = self.CalculateKey(s)

    def CalculateKey(self, s):
        return [min(self.g[s], self.rhs[s]) + self.h(self.s_start, s) + self.km,
                min(self.g[s], self.rhs[s])]

    def TopKey(self):
        """
        :return: return the min key and its value.
        """

        s = min(self.U, key=self.U.get)
        return s, self.U[s]

    def h(self, s_start, s_goal):
        heuristic_type = self.heuristic_type  # heuristic type

        if heuristic_type == "manhattan":
            return abs(s_goal[0] - s_start[0]) + abs(s_goal[1] - s_start[1])
        else:
            return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):
            return float("inf")

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
        if s_start in self.obs or s_end in self.obs:
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            if s1 in self.obs or s2 in self.obs:
                return True

        return False

    def get_neighbor(self, s):
        nei_list = set()
        for u in self.u_set:
            s_next = tuple([s[i] + u[i] for i in range(2)])
            if s_next not in self.obs:
                nei_list.add(s_next)

        return nei_list

    def extract_path(self):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_start]
        s = self.s_start

        for k in range(100):
            g_list = {}
            for x in self.get_neighbor(s):
                if not self.is_collision(s, x):
                    g_list[x] = self.g[x]
            s = min(g_list, key=g_list.get)
            path.append(s)
            if s == self.s_goal:
                break

        return list(path)

    def plot_path(self, path):
        px = [x[0] for x in path]
        py = [x[1] for x in path]
        plt.plot(px, py, linewidth=2)
        plt.plot(self.s_start[0], self.s_start[1], "bs")
        plt.plot(self.s_goal[0], self.s_goal[1], "gs")

    def plot_visited(self, visited):
        color = ['gainsboro', 'lightgray', 'silver', 'darkgray',
                 'bisque', 'navajowhite', 'moccasin', 'wheat',
                 'powderblue', 'skyblue', 'lightskyblue', 'cornflowerblue']

        if self.count >= len(color) - 1:
            self.count = 0

        for x in visited:
            plt.plot(x[0], x[1], marker='s', color=color[self.count])


def main():
    s_start = (5, 5)
    s_goal = (1300, 700)

    dstar = DStar(s_start, s_goal, "euclidean")
    path = dstar.run()

    print(path)


if __name__ == '__main__':
    main()
