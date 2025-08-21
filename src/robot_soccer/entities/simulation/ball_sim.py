import pygame
from robot_soccer.config import ANCHO_CAMPO, ALTO_CAMPO


class Ball4Simulation:
    """Inicializa una pelota para la simulación de fútbol robótico.

    Args:
        masa (float): Masa de la pelota en unidades de simulación.
        x (int): Posición inicial en el eje x en píxeles.
        y (int): Posición inicial en el eje y en píxeles.
        dx (float): Velocidad inicial en el eje x.
        dy (float): Velocidad inicial en el eje y.
        radio (int): Radio de la pelota en píxeles.

    Attributes:
        x (int): Posición actual en x.
        y (int): Posición actual en y.
        dx (float): Velocidad actual en x.
        dy (float): Velocidad actual en y.
        identificador (int): Identificador único (0 para pelota).
        radio (int): Radio de la pelota.
        d_fr (float): Coeficiente de desaceleración por roce.
        masa (float): Masa de la pelota.
        max_vel (int): Velocidad máxima permitida.
        f_aceleracion (float): Factor de aceleración.
        v (int): Velocidad escalar (no utilizado actualmente).
        ball_hold (bool): Estado de posesión de pelota.
        surface (pygame.Surface): Superficie de renderizado de la pelota.
        img_robot_rotated (pygame.Surface): Imagen rotada para renderizado.
        rotated_rect (pygame.Rect): Rectángulo de colisión actualizado.
        cooldown (int): Tiempo de enfriamiento para operaciones.
        distanciaConPelota (float): Distancia calculada con jugador.
        coordsHoldBall_x (int): Coordenada x para posesión de pelota.
        coordsHoldBall_y (int): Coordenada y para posesión de pelota.
    """

    def __init__(self, masa, x, y, dx, dy, radio):
        """Valores iniciales para construir una pelota en la simulación.

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
        self.f_aceleracion = 0.2  # Factor de aceleracion

        self.v = 0

        # Para realizar la toma de pelota y posterior disparo de la misma
        self.ball_hold = False

        diameter = 38
        self.surface = pygame.Surface((diameter, diameter), pygame.SRCALPHA)
        # Dibujar un círculo en la superficie
        pygame.draw.circle(
            self.surface,
            (244, 98, 0, 255),
            (diameter // 2, diameter // 2),
            diameter // 2,
        )

        self.img_robot_rotated = pygame.transform.rotate(self.surface, 0)
        self.rotated_rect = self.img_robot_rotated.get_rect(center=(self.x, self.y))

        self.cooldown = 0  # Tiempo de cooldown inicial
        self.distancia_con_pelota = None
        self.coords_hold_ball_x = None
        self.coords_hold_ball_y = None

    def motion_ball(self):
        """Actualiza la posición y estado físico de la pelota.

        Actualiza las coordenadas x, y de la pelota basándose en su velocidad actual,
        aplica desaceleración por fricción y verifica colisiones con los bordes del campo.

        Note:
            Este método debe ser llamado en cada frame de la simulación para
            mantener el movimiento físico realista de la pelota.
        """
        self.x = int(self.x + self.dx)
        self.y = int(self.y + self.dy)

        # Actualizar el rectángulo
        self.rotated_rect = self.surface.get_rect(center=(self.x, self.y))

        self.desaceleracion()
        self._collision_edge()

    def generation_ball_v2(self, fondo):
        """Renderiza la pelota en la superficie de juego.

        Args:
            fondo (pygame.Surface): Superficie de pygame donde se dibujará la pelota.

        Note:
            Actualiza automáticamente el rectángulo de colisión de la pelota
            basándose en su posición actual antes del renderizado.
        """
        # Obtiene el rectángulo de la imagen
        self.rotated_rect = self.surface.get_rect(center=(self.x, self.y))

        # Dibujar la imagen
        fondo.blit(self.surface, self.rotated_rect.topleft)

    def _collision_edge(self):
        """Detecta y maneja colisiones de la pelota con los bordes del campo.

        Verifica si la pelota ha colisionado con cualquiera de los cuatro bordes
        del campo de juego y ajusta su posición y velocidad en consecuencia,
        implementando un rebote realista.

        Note:
            Este es un método privado que se llama automáticamente desde motion_ball().
            Utiliza las constantes ANCHO_CAMPO y ALTO_CAMPO para determinar los límites.
        """
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
            self.y = bottom_edge - robot_rect.height // 2
            self.dy = -self.dy

    def desaceleracion(self):
        """Aplica desaceleración por fricción a la pelota.

        Reduce gradualmente la velocidad de la pelota en ambos ejes (x, y)
        utilizando el coeficiente de fricción hasta que se detenga completamente.

        Note:
            Utiliza la función interna frenado() para calcular la nueva velocidad
            en cada eje de manera independiente.
        """

        def frenado(velocidad, f_desaceleracion):
            """Calcula la nueva velocidad aplicando desaceleración.

            Args:
                velocidad (float): Velocidad actual del objeto.
                f_desaceleracion (float): Factor de desaceleración a aplicar.

            Returns:
                float: Nueva velocidad después de aplicar la desaceleración.
                       Se asegura de que la velocidad no cambie de signo.

            Note:
                Si la velocidad es positiva, la reduce hasta llegar a cero.
                Si es negativa, la aumenta hasta llegar a cero.
            """
            if velocidad > 0:
                velocidad -= f_desaceleracion
                return max(velocidad, 0)

            velocidad += f_desaceleracion
            return min(velocidad, 0)


        self.dx = frenado(self.dx, self.d_fr)
        self.dy = frenado(self.dy, self.d_fr)
