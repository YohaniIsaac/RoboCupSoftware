"""
Environment for rrt_2D
@author: huiming zhou
"""
import math 

x_ancho = 1500
y_alto = 900


class Env:
    def __init__(self, lista_obs):
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
