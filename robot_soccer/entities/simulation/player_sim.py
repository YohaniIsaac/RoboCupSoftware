import pygame
import math
import numpy as np
from robot_soccer.config import *


class Player4Simulation:
    def __init__(self, masa, x, y, angulo, dx, dy, dw, radio, identificador):
        """
        Valores iniciales para construir un jugador en la simulación

        Args:
            masa (float): Masa del jugador
            x (int): Posición en x
            y (int): Posición en y
            angulo (float): Ángulo inicial en grados
            dx (float): Velocidad en x
            dy (float): Velocidad en y
            dw (float): Velocidad angular
            radio (int): Radio del jugador
            identificador (int): ID del jugador
        """
        # Posicion y velocidad del objeto
        self.x = x
        self.y = y
        self.angulo = angulo
        self.dx = dx
        self.dy = dy
        self.dw = dw  # velocidad angular

        self.radio = radio

        self.d_fr = 0.5  # Desaceleracion por roce
        self.masa = masa  # no se está utilizando la funcion actualmente

        self.max_vel = 5  # Velocidad maxima a la que puede llegar el objeto #4
        self.f_aceleracion = .2  # Factor de aceleracion

        self.identificador = identificador  # Valor de identificacion del objeto (0 para la pelota)

        self.v = 0

        # Para realizar la toma de pelota y posterior disparo de la misma
        self.ball_hold = False

        # Cargar la imagen
        path = f"robot{self.identificador}.png"
        img_robot = pygame.image.load(str(ARUCO_MARKERS_DIR / path))
        self.img_robot = pygame.transform.scale(img_robot, (102, 135))
        self.img_robot_rotated = pygame.transform.rotate(self.img_robot, -self.angulo)
        self.rotated_rect = self.img_robot_rotated.get_rect(center=(self.x, self.y))

        # Para control de la pelota
        self.cooldown = 0  # Tiempo de cooldown inicial
        self.distanciaConPelota = None
        self.coordsHoldBall_x = None
        self.coordsHoldBall_y = None

    def intrucciones(self, punto):
        """
        Mueve el jugador hacia un punto específico

        Args:
            punto (tuple): Coordenadas (x, y) del punto destino

        Returns:
            bool: False si llegó al destino, True si sigue en movimiento
        """
        # Obtención de los parámetros de la instrucción
        x_destino, y_destino = punto

        en_curso = True
        # Evita movimientos cuando el punto se encuentra muy cercano al actual
        if abs(x_destino - self.x) <= 2:
            self.x = x_destino
            self.dx = 0
        elif x_destino > self.x:
            # Movimiento rápido si está lejos el punto
            if abs(x_destino - self.x) > 10:
                self.dx += self.f_aceleracion
                self.dx = min(self.dx, self.max_vel)
            # movimiento lento si esta cerca el punto
            else:
                self.dx += self.f_aceleracion
                self.dx = min(self.dx, 0.5)
        elif x_destino < self.x:
            # Movimiento rápido si esta lejos el punto
            if abs(x_destino - self.x) > 10:
                self.dx -= self.f_aceleracion
                self.dx = max(self.dx, -self.max_vel)
            # Movimiento más lento si está cerca el punto
            else:
                self.dx -= self.f_aceleracion
                self.dx = max(self.dx, -0.5)
        else:
            self.dx = 0

        # Igual a lo anterior, pero ahora para el eje y
        if abs(y_destino - self.y) <= 2:
            self.y = y_destino
            self.dy = 0
        elif y_destino > self.y:
            if abs(y_destino - self.y) > 10:
                self.dy += self.f_aceleracion
                self.dy = min(self.dy, self.max_vel)
            else:
                self.dy += self.f_aceleracion
                self.dy = min(self.dy, 0.5)
        elif y_destino < self.y:
            if abs(y_destino - self.y) > 10:
                self.dy -= self.f_aceleracion
                self.dy = max(self.dy, -self.max_vel)
            else:
                self.dy -= self.f_aceleracion
                self.dy = max(self.dy, -0.5)
        else:
            self.dy = 0

        # Para poder pasar a la siguiente instrucción
        if x_destino == self.x and y_destino == self.y:
            en_curso = False

        return en_curso

    def motion_player(self, pelota, jugador_1=None, jugador_2=None, jugador_3=None):
        """
        Genera el movimiento del jugador, actualizando su posición y orientación

        Args:
            pelota: Objeto pelota para interacción
            jugador_1, jugador_2, jugador_3: Otros jugadores (opcional)
        """
        # Actualiza la posición de los jugadores
        self.x = int(self.x + self.dx)
        self.y = int(self.y + self.dy)
        self.angulo = int(self.angulo - self.dw)
        self.angulo = self.angulo % 360

        # Actualizar el rectángulo rotado según la nueva posición y ángulo
        self.img_robot_rotated = pygame.transform.rotate(self.img_robot, -self.angulo)
        self.rotated_rect = self.img_robot_rotated.get_rect(center=(self.x, self.y))

        # Ajustar la posición del robot dentro de los bordes del campo de forma eficiente
        self.x = min(max(self.x, self.rotated_rect.width // 2), ANCHO_CAMPO - self.rotated_rect.width // 2)
        self.y = min(max(self.y, self.rotated_rect.height // 2), ALTO_CAMPO - self.rotated_rect.height // 2)

        # Actualizar de nuevo el rectángulo después de ajustar la posición
        self.rotated_rect = self.img_robot_rotated.get_rect(center=(self.x, self.y))

        self.desaceleracion()
        if self.identificador != 0:
            self.disparo(pelota)

    def collision_edge(self):
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

        # # Rotación del objeto
        self.angulo = int(self.angulo - self.dw) % 360

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
        self.dw = frenado(self.dw, self.d_fr)

    def teclas(self):
        """
        Permite controlar el jugador con las teclas
        """
        keys = pygame.key.get_pressed()
        if self.cooldown != 0:
            return
        # Movimiento recto
        if keys[pygame.K_w]:
            self.v += self.f_aceleracion
            self.v = min(self.v, self.max_vel)
        elif keys[pygame.K_s]:
            self.v -= self.f_aceleracion
            self.v = max(self.v, -self.max_vel)
        else:
            self.v = 0

        # Movimiento de rotación
        if keys[pygame.K_a]:
            self.dw -= 0.1
            self.dw = max(self.dw, 2)
        elif keys[pygame.K_d]:
            self.dw += 0.1
            self.dw = min(self.dw, -2)
        else:
            self.dw = 0

        # Calcula el ángulo en radianes
        angle_radians = math.radians(self.angulo + 270)

        # Obtener las componentes de velocidad
        self.dx = self.v * math.cos(angle_radians)
        self.dy = self.v * math.sin(angle_radians)

        # Actualiza la posición basándonos en las componentes de velocidad
        self.x += self.dx
        self.y += self.dy

    def disparo(self, obj):
        """
        Permite la sujeción de la pelota y su disparo

        Args:
            obj: Objeto pelota
        """
        self.distanciaConPelota = math.dist((obj.x, obj.y), (self.coordsHoldBall_x, self.coordsHoldBall_y))

        keys = pygame.key.get_pressed()

        # Tomar posesión de la pelota con la letra k
        if keys[pygame.K_k]:
            if self.distanciaConPelota < 10:
                obj.x = self.coordsHoldBall_x
                obj.y = self.coordsHoldBall_y
                obj.dx = self.dx
                obj.dy = self.dy
                self.ball_hold = True  # para el disparo
            else:
                pass
        # Mantener posesión de la pelota
        if self.ball_hold:
            obj.x = self.coordsHoldBall_x
            obj.y = self.coordsHoldBall_y
            obj.dx = self.dx
            obj.dy = self.dy
        # Disparar la pelota si se presiona la letra L
        if self.ball_hold and keys[pygame.K_l]:
            vel_disparo = 10
            angle = math.radians(self.angulo) - math.pi / 2
            obj.dx = vel_disparo * math.cos(angle)
            obj.dy = vel_disparo * math.sin(angle)
            self.ball_hold = False

    def generationRobotV2(self, fondo):
        # Rotar la imagen
        self.img_robot_rotated = pygame.transform.rotate(self.img_robot, -self.angulo)

        # Obtiene el rectangul de la iamgen
        self.rotated_rect = self.img_robot_rotated.get_rect(center=(self.x, self.y))

        # Dibujar la imagen  rotada
        fondo.blit(self.img_robot_rotated, self.rotated_rect.topleft)

        front_angle = math.radians(self.angulo) - math.pi / 2  # Convierte el ángulo a radianes

        self.coordsHoldBall_x = int(self.x + 70 * math.cos(front_angle))  # 30 es el radio del círculo
        self.coordsHoldBall_y = int(self.y + 70 * math.sin(front_angle))

        # Dibuja el punto indicando el frente
        # pygame.draw.circle(fondo, (255,255,255), (self.coordsHoldBall_x, self.coordsHoldBall_y), 5)

    def generationBallV2(self, fondo):

        # Rotar la imagen
        self.img_robot_rotated = pygame.transform.rotate(self.img_robot, -self.angulo)

        # Obtiene el rectangul de la iamgen
        self.rotated_rect = self.img_robot_rotated.get_rect(center=(self.x, self.y))

        # Dibujar la imagen  rotada
        fondo.blit(self.img_robot_rotated, self.rotated_rect.topleft)