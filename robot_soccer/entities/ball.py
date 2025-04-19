import numpy as np


class Ball:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def get_position(self):
        return np.array([self.x, self.y])

    def set_position(self, x, y):
        self.x = x
        self.y = y
