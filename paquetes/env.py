"""
Env 2D
@author: huiming zhou
"""
x_ancho = 1500
y_alto = 900


class Env:
    def __init__(self):
        self.x_range = x_ancho  # size of background
        self.y_range = y_alto
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),   # Define los movimientos posibles que el robot puede hacer
                        (1, 0), (1, -1), (0, -1), (-1, -1)] # Son 8 movimientos
        self.obs = self.obs_map()                           # arriba, abajo, izquierda, derecha, movimientos diagonales

    def update_obs(self, obs):
        self.obs = obs

    def obs_map(self):
        """
        Initialize obstacles' positions
        :return: map of obstacles
        """

        x = self.x_range
        y = self.y_range
        obs = set()

        for i in range(x):
            obs.add((i, 0))         # Borde inferior
        for i in range(x):
            obs.add((i, y - 1))     # Borde superior

        for i in range(y):
            obs.add((0, i))         # Borde izquierdo
        for i in range(y):
            obs.add((x - 1, i))     # Borde derecho

        # Agrega otros obtáculos en forma de líneas
        # for i in range(10, 21):
        #     obs.add((i, 15))
        # for i in range(15):
        #     obs.add((20, i))
        #
        # for i in range(15, 30):
        #     obs.add((30, i))
        # for i in range(16):
        #     obs.add((40, i))

        return obs
