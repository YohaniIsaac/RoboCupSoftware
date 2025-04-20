import pygame
from robot_soccer.config import *


# ==========================================
# LOG
# ==========================================
from robot_soccer.utils.logger import get_logger
from robot_soccer.utils.logger import set_level, disable_module, enable_module
module_name = "entities.simulation"

logger = get_logger(module_name)

# Activar depuración detallada para un módulo
set_level(module_name, "WARNING")  # DEBUG, INFO, WARNING, ERROR, CRITICAL, DISABLED
# # Desactivar registro para un módulo que está generando demasiados mensajes
# disable_module("core.physics")
# # Reactivar registro para un módulo previamente desactivado
# enable_module("core.physics", "INFO")
# ==========================================


class Ball4Simulation:
    def __init__(self, masa, x, y, dx, dy, radio):
        """
        Valores iniciales para construir una pelota en la simulación

        Args:
            masa (float): Masa de la pelota
            x (int): Posición en x
            y (int): Posición en y
            dx (float): Velocidad en x
            dy (float): Velocidad en y
            radio (int): Radio de la pelota
        """
        # Posicion y velocidad del objeto
        self.x = x
        self.y = y
        self.dx = dx
        self.dy = dy
        self.identificador = 0  # Para poder determianr colisiones!!

        self.radio = radio

        self.d_fr = 0.5  # Desaceleracion por roce
        self.masa = masa  # no se está utilizando la funcion actualmente

        self.max_vel = 5  # Velocidad maxima a la que puede llegar el objeto #4
        self.f_aceleracion = .2  # Factor de aceleracion

        self.v = 0

        # Para realizar la toma de pelota y posterior disparo de la misma
        self.ball_hold = False

        diameter = 38
        self.surface = pygame.Surface((diameter, diameter), pygame.SRCALPHA)
        # Dibujar un círculo en la superficie
        pygame.draw.circle(self.surface, (244, 98, 0, 255), (diameter // 2, diameter // 2), diameter // 2)

        self.img_robot_rotated = pygame.transform.rotate(self.surface, 0)
        self.rotated_rect = self.img_robot_rotated.get_rect(center=(self.x, self.y))

        self.cooldown = 0  # Tiempo de cooldown inicial
        self.distanciaConPelota = None
        self.coordsHoldBall_x = None
        self.coordsHoldBall_y = None

    def motion_ball(self):
        """
        Genera el movimiento de la pelota, actualizando su posición
        """
        self.x = int(self.x + self.dx)
        self.y = int(self.y + self.dy)

        # Actualizar el rectángulo
        self.rotated_rect = self.surface.get_rect(center=(self.x, self.y))

        self.desaceleracion()
        self._collision_edge()

    def generationBallV2(self, fondo):
        """
        Dibuja la pelota en la superficie de fondo

        Args:
            fondo: Superficie de pygame donde dibujar
        """
        # Obtiene el rectángulo de la imagen
        self.rotated_rect = self.surface.get_rect(center=(self.x, self.y))

        # Dibujar la imagen
        fondo.blit(self.surface, self.rotated_rect.topleft)

    def _collision_edge(self):
        # Posición del robot rotado (top-left)
        robot_rect = self.rotated_rect

        # Bordes del campo
        left_edge = 0
        right_edge = ANCHO_CAMPO
        top_edge = 0
        bottom_edge = ALTO_CAMPO

        # Revisar colisiones con los bordes del campo
        colision_borde_izq = robot_rect.left < left_edge
        colision_borde_der = robot_rect.right > right_edge
        colision_borde_sup = robot_rect.top < top_edge
        colision_borde_inf = robot_rect.bottom > bottom_edge

        # Sí hay colisión, ajustar la posición y cambiar la dirección
        if colision_borde_izq:
            self.x = robot_rect.width // 2  # Ajustar posición para no salir
            self.dx = -self.dx  # Revertir la dirección horizontal

        elif colision_borde_der:
            self.x = right_edge - robot_rect.width // 2
            self.dx = -self.dx

        if colision_borde_sup:
            self.y = robot_rect.height // 2
            self.dy = -self.dy

        elif colision_borde_inf:
            self.y = (bottom_edge - robot_rect.height // 2)
            self.dy = -self.dy

    def desaceleracion(self):
        """
        Reduce la velocida del objeto debido al roce que posee
        """

        def frenado(velocidad, f_desaceleracion):
            """
            Incrementa o decrementa la velocidad hasta llegar a cero
            """
            if velocidad > 0:
                velocidad -= f_desaceleracion
                return max(velocidad, 0)

            elif velocidad < 0:
                velocidad += f_desaceleracion
                return min(velocidad, 0)

            return velocidad

        self.dx = frenado(self.dx, self.d_fr)
        self.dy = frenado(self.dy, self.d_fr)
