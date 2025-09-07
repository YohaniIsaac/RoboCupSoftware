"""Módulo de la entidad pelota para el sistema de fútbol de robots.

Este módulo contiene la clase Ball que representa la pelota en el juego,
incluyendo su posición y métodos básicos de manipulación.
"""

import time
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
        position_history (list): Historial de posiciones de la pelota.
        velocity_x (float): Velocidad de la pelota en el eje x.
        velocity_y (float): Velocidad de la pelota en el eje y.
        speed (float): Rapidez de la pelota (magnitud).
        last_update (float): Última actualización.
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
        self.position_history = []
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.speed = 0.0
        self.last_update = 0.0

    def get_position(self):
        """Obtiene la posición actual de la pelota.

        Returns:
            numpy.ndarray: Array de numpy conteniendo las coordenadas [x, y] de la pelota.
                          El array es de tipo float64 y shape (2,).
        """
        return np.array([self.x, self.y])

    def get_velocity(self):
        """Obtiene la posición actual de la pelota.

        Returns:
            numpy.ndarray: Array de numpy conteniendo las velocidades y rapidez [v_x, v_y, spped]
            de la pelota. El array es de tipo float64 y shape (3,).
        """
        return np.array([self.velocity_x, self.velocity_y, self.speed])

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
        self.update_velocity()


    def update_velocity(self):
        """Calcular velocidad basada en historial."""
        current_time = time.time()


        if self.position_history and current_time - self.last_update > 0.05:  # Cada 50ms
            prev_x, prev_y, prev_time = self.position_history[-1]
            dt = current_time - prev_time

            if dt > 0:
                self.velocity_x = (self.x - prev_x) / dt
                self.velocity_y = (self.y - prev_y) / dt
                self.speed = (self.velocity_x**2 + self.velocity_y**2)**0.5

        # Mantener historial (máximo 10 posiciones)
        self.position_history.append((self.x, self.y, current_time))
        if len(self.position_history) > 5:
            self.position_history.pop(0)

        self.last_update = current_time
