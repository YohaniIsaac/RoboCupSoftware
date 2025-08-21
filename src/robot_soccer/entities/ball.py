"""Módulo de la entidad pelota para el sistema de fútbol de robots.

Este módulo contiene la clase Ball que representa la pelota en el juego,
incluyendo su posición y métodos básicos de manipulación.
"""

import numpy as np


class Ball:
    """Representa la pelota en el sistema de fútbol de robots.

    Esta clase maneja la posición y el estado básico de la pelota durante el juego.
    Proporciona métodos para obtener y establecer la posición de la pelota, permitiendo
    que otros componentes del sistema (como jugadores, IA, física) puedan interactuar
    con ella.

    Attributes:
        x (int): Coordenada X de la pelota en el campo.
        y (int): Coordenada Y de la pelota en el campo.
    """

    def __init__(self, x, y):
        """Inicializa una nueva instancia de la pelota.

        Args:
            x (int): Coordenada X inicial de la pelota en el campo.
            y (int): Coordenada Y inicial de la pelota en el campo.

        Note:
            Las coordenadas deben estar dentro de los límites del campo de juego
            definidos en la configuración del sistema.
        """
        self.x = x
        self.y = y

    def get_position(self):
        """Obtiene la posición actual de la pelota.

        Returns:
            numpy.ndarray: Array de numpy conteniendo las coordenadas [x, y] de la pelota.
                          El array es de tipo float64 y shape (2,).
        """
        return np.array([self.x, self.y])

    def set_position(self, x, y):
        """Establece una nueva posición para la pelota.

        Este método actualiza directamente las coordenadas de la pelota.
        Es utilizado por el sistema de física, controles de jugadores y otros
        componentes que necesitan mover la pelota durante el juego.

        Args:
            x (int): Nueva coordenada X de la pelota.
            y (int): Nueva coordenada Y de la pelota.

        Note:
            Este método no valida si las nuevas coordenadas están dentro
            de los límites del campo. La validación debe realizarse externamente
            si es necesaria.
        """
        self.x = x
        self.y = y
