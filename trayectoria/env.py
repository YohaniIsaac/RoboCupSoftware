"""
Environment for rrt_2D
@author: huiming zhou
"""
x_ancho = 1080
y_alto = 720


class Env:
    def __init__(self):
        self.x_range = (0, x_ancho)
        self.y_range = (0, y_alto)
        self.obs_boundary = self.obs_boundary()
        self.obs_circle = self.obs_circle()
        self.obs_rectangle = self.obs_rectangle()

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
    def obs_rectangle():
        obs_rectangle = [
            # [14, 12, 8, 2],
            # [18, 22, 8, 3],
            # [26, 7, 2, 12],
            # [32, 14, 10, 2]
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            [500, 500, 50],
            [400, 200, 50],
            [600, 400, 50],
            [200, 100, 50],
        ]

        return obs_cir
