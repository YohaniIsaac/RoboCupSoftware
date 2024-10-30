"""
Environment for rrt_2D
@author: huiming zhou
"""
import math

ancho = 1500
alto = 900


class Env:
    """
    Define el entorno en el que se moverá el robot
    """
    def __init__(self, list_obs):
        self.x_range = (0, ancho)   # Límite del área
        self.y_range = (0, alto)    # Límite del área
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
            [0, 0, 1, alto],
            [0, alto, ancho, 1],
            [1, 0, ancho, 1],
            [ancho, 1, 1, alto]
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
