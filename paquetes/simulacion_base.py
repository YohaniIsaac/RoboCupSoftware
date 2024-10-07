import pygame
import math

######################
# CREACION DEL JUEGO #
######################
class Objeto:
    def __init__(self, masa, x, y, angulo, dx, dy, dw, radio, identificador):
        """
        Valores inciales para construir el objeto

        Args:
        x -- (int) Posicion en x del objeto.
        y -- (int) Posicion en y del objeto.
        equipo -- (array) Color identificador del jugador, equipo.
        angulo -- (int) Posición del angulo incial del objeto.
        dx -- (int) Variación del movimiento en x del objeto.
        dy -- (int) Variación del movimiento en y del objeto.
        dw -- (int) Variación del ángulo del objeto.
        radio -- (int) Radio del objeto.
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
        self.masa = masa  # no se esta utilizando la funcion actualmente

        self.max_vel = 5  # Velocidad maxima a la que puede llegar el objeto #4
        self.f_aceleracion = .2  # Factor de aceleracion

        self.identificador = identificador  # Valor de identificacion del objeto (0 para la pelota)

        self.v = 0

        # Para realizar la toma de pelota y posterior disparo de la misma
        self.ball_hold = False

        # Dimensiones del campo de futbol
        self.ancho_campo = 1600
        self.alto_campo = 1000

        # Imagen del robot
        if self.identificador != 0:
            # Cargar la imagen
            path = "robot" + str(self.identificador) + ".png"
            img_robot = pygame.image.load("arucoMarkers/" + path)
            self.img_robot = pygame.transform.scale(img_robot, (102, 135))
            self.img_robot_rotated = pygame.transform.rotate(self.img_robot, -self.angulo)
            self.rotated_rect = self.img_robot_rotated.get_rect(center=(self.x, self.y))
        # Imagen de la pelota
        if self.identificador == 0:
            # Crear una superficie con fondo transparente
            diameter = 38
            surface = pygame.Surface((diameter, diameter), pygame.SRCALPHA)
            # Dibujar un círculo en la superficie
            pygame.draw.circle(surface, (244, 98, 0, 255), (diameter // 2, diameter // 2), diameter // 2)

            self.img_robot = surface
            self.img_robot_rotated = pygame.transform.rotate(self.img_robot, -self.angulo)
            self.rotated_rect = self.img_robot_rotated.get_rect(center=(self.x, self.y))

        self.cooldown = 0  # Tiempo de cooldown inicial

    def intrucciones(self, punto):
        # Obtención de los parámetros de la instruccion
        x_destino, y_destino = punto

        en_curso = True
        # Evita movimientos cuando el punto se encuentra muy cercano al actual
        if abs(x_destino - self.x) <= 2:
            self.x = x_destino
            self.dx = 0

        elif x_destino > self.x:
            # Movimiento rápido si esta lejos el punto
            if abs(x_destino - self.x) > 10:
                self.dx += self.f_aceleracion
                self.dx = min(self.dx, self.max_vel)

            # movmiento lento si esta cerca el punto
            else:
                self.dx += self.f_aceleracion
                self.dx = min(self.dx, 0.5)

        elif x_destino < self.x:
            # Movimiento rápido si esta lejso el punto
            if abs(x_destino - self.x) > 10:
                self.dx -= self.f_aceleracion
                self.dx = max(self.dx, -self.max_vel)

            # Moivimiento mas lento si esta cerca el punto
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

        # Para poder pasar a la siguiente instruccion
        if x_destino == self.x and y_destino == self.y:
            en_curso = False
            # print(self.x, self.y)

        return en_curso

    def motion_player(self, pelota, jugador_1=None, jugador_2=None, jugador_3=None):
        """
        Genera el movimiento del objeto, con los valores del objeto.
        """
        # Actuliza la posicion de los jugadores
        self.x = int(self.x + self.dx)
        self.y = int(self.y + self.dy)
        self.angulo = int(self.angulo - self.dw)
        self.angulo = self.angulo % 360

        # Actualizar el rectángulo rotado según la nueva posición y ángulo
        self.img_robot_rotated = pygame.transform.rotate(self.img_robot, -self.angulo)
        self.rotated_rect = self.img_robot_rotated.get_rect(center=(self.x, self.y))

        # Ajustar la posición del robot dentro de los bordes del campo de forma eficiente
        self.x = min(max(self.x, self.rotated_rect.width // 2), self.ancho_campo - self.rotated_rect.width // 2)
        self.y = min(max(self.y, self.rotated_rect.height // 2), self.alto_campo - self.rotated_rect.height // 2)

        # Actualizar de nuevo el rectángulo después de ajustar la posición
        self.rotated_rect = self.img_robot_rotated.get_rect(center=(self.x, self.y))

        self.desaceleracion()
        if self.identificador != 0:
            self.disparo(pelota)

    def motion_ball(self):
        """
        Genera el movimiento de la pelota, con los valores del objeto.
        """
        self.x = int(self.x + self.dx)
        self.y = int(self.y + self.dy)

        self.desaceleracion()
        self.collision_edge()

    def collision_edge(self):
        # Posición del robot rotado (top-left)
        robot_rect = self.rotated_rect

        # Bordes del campo
        left_edge = 0
        right_edge = self.ancho_campo
        top_edge = 0
        bottom_edge = self.alto_campo

        # Revisar colisiones con los bordes del campo
        colision_borde_izq = robot_rect.left < left_edge
        colision_borde_der = robot_rect.right > right_edge
        colision_borde_sup = robot_rect.top < top_edge
        colision_borde_inf = robot_rect.bottom > bottom_edge

        # Si hay colisión, ajustar la posición y cambiar la dirección
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
        Ayuda a poder controlar el objeto con las teclas.
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
        #     self.v *= 0.95  # Reduce la velocidad gradualmente si no se presiona ninguna tecla

        # Movimiento de rotación
        if keys[pygame.K_a]:
            self.dw -= 0.1
            self.dw = max(self.dw, 2)
        elif keys[pygame.K_d]:
            self.dw += 0.1
            self.dw = min(self.dw, -2)
        else:
            # self.dw *= 0.9  # Reduce la velocidad de rotación gradualmente
            self.dw = 0

        # Calcula el angulo en radianes
        angle_radians = math.radians(self.angulo + 270)

        # Obtener las componentes de veloidad
        self.dx = self.v * math.cos(angle_radians)
        self.dy = self.v * math.sin(angle_radians)

        # Actualiza la posición en base a las componenetes de velocidad
        self.x += self.dx
        self.y += self.dy

    def disparo(self, obj):
        """
        Permite la sujeción de la pelota por medio de una tecla.
        Ademas al soltar la tecla hace que el robot dispare la pelota con cierta velocidad

        Args:
        obj     -- (objeto) Objeto
        """
        self.distanciaConPelota = math.dist((obj.x, obj.y), (self.coordsHoldBall_x, self.coordsHoldBall_y))

        keys = pygame.key.get_pressed()

        # Tomar posesion de la pelota con la letra k
        if keys[pygame.K_k]:
            if self.distanciaConPelota < 10:
                obj.x = self.coordsHoldBall_x
                obj.y = self.coordsHoldBall_y
                obj.dx = self.dx
                obj.dy = self.dy
                self.ball_hold = True  # para el disparo
            else:
                pass
        # Mantener posesion de la pelota
        if self.ball_hold:
            obj.x = self.coordsHoldBall_x
            obj.y = self.coordsHoldBall_y
            obj.dx = self.dx
            obj.dy = self.dy
        # Disparar la pelota si se preisona la leta L
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


## COLISIONES

def detectar_colisiones(objetos):
    colisiones_procesadas = set()  # Conjunto para evitar la doble verificación de colisiones

    # Iterar sobre cada par de objetos
    for i in range(len(objetos)):
        for j in range(i + 1, len(objetos)):  # Empezamos desde i+1 para evitar dobles verificaciones
            obj1 = objetos[i]
            obj2 = objetos[j]

            # Si cualquiera de los objetos es un robot y está sosteniendo la pelota, omitir la colisión
            if (obj1.identificador != 0 and obj1.ball_hold) or (obj2.identificador != 0 and obj2.ball_hold):
                continue

            # Crear un identificador único para este par de objetos
            par_objetos = tuple(sorted([id(obj1), id(obj2)]))

            # comprobar cooldown
            if obj1.cooldown > 0 or obj2.cooldown > 0:
                continue # Si cualñquiera de los objetos esta en cooldown, salta la verificacion

            if par_objetos not in colisiones_procesadas:
                # Verificar colisión entre obj1 y obj2
                check_collision(obj1, obj2)
                # Marcar la colisión como procesada
                colisiones_procesadas.add(par_objetos)
    for obj in objetos:
        if obj.cooldown > 0:
            obj.cooldown -= 1 # Disminuir el cooldown en cada frame

def check_collision(obj1, obj2):
    mask1 = pygame.mask.from_surface(obj1.img_robot_rotated)
    mask2 = pygame.mask.from_surface(obj2.img_robot_rotated)

    # Obtener los offsets entre las dos imagenes
    offset = (obj2.rotated_rect.left - obj1.rotated_rect.left, obj2.rotated_rect.top - obj1.rotated_rect.top)

    # Comprobar la colision usando la mascara
    collision = mask1.overlap(mask2, offset)
    if collision:
        # Calcular la dirección del golpe
        direccion = (obj2.x - obj1.x, obj2.y - obj1.y)

        # Calcular la distancia
        distancia = math.sqrt(direccion[0]**2 + direccion[1]**2)

        # Normalizar la dirección
        direccion_normal = (direccion[0] / distancia, direccion[1] / distancia)
        ####
        # Velocidad relatica en la direccion de la colision
        velocidad_relativa_x = obj2.dx - obj1.dx
        velocidad_relativa_y = obj2.dy - obj1.dy
        velocidad_normal = velocidad_relativa_x * direccion_normal[0] + velocidad_relativa_y * direccion_normal[1]

        # Si la velocidad normal es mayor a 0, los objetos ya se estan separando, no se aplica impulso
        if velocidad_normal > 0:
            return

        # Coef. de restitución (elasticidad de la colision)
        e = 1

        # Calculo de impulso escalar
        j = -(1+e) * velocidad_normal
        j /= (1 / obj1.masa + 1 / obj2.masa)

        impulso_x = j * direccion_normal[0]
        impulso_y = j * direccion_normal[1]

        # Actualizar las velocidades de los obejtos en funcion de sus masas
        obj1.dx -= impulso_x / obj1.masa
        obj1.dy -= impulso_y / obj1.masa
        obj2.dx += impulso_x / obj2.masa
        obj2.dy += impulso_y / obj2.masa

        ## Ajustar posiciones para evitar superposición
        ## Calcular la distancia de superposición (superposition_distance)7 días anteriores
        superposition_distance = (obj1.rotated_rect.width / 2 + obj2.rotated_rect.width / 2) - distancia
        factor_suavizado = 0.7
        if superposition_distance > 0:
            # Mover los objetos en direcciones opuestas para separarlos
            correction_x = direccion_normal[0] * superposition_distance * factor_suavizado
            correction_y = direccion_normal[1] * superposition_distance * factor_suavizado

            # Congelar el objeto en el límite de colisión para evitar superposición
            obj1.x -= correction_x / 2
            obj1.y -= correction_y / 2
            obj2.x += correction_x / 2
            obj2.y += correction_y / 2

        # Actualiza cooldown
        obj1.cooldown = 8
        obj2.cooldown = 8

        # Actualizar los rectángulos rotados después de mover los objetos
        obj1.rotated_rect = obj1.img_robot_rotated.get_rect(center=(obj1.x, obj1.y))
        obj2.rotated_rect = obj2.img_robot_rotated.get_rect(center=(obj2.x, obj2.y))