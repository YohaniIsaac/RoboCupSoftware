import pygame
import numpy as np
import cv2 as cv
import math

desaceleracion_roce = 0.05

class Jugador:
    def __init__(self, x, y, equipo, etiqueta, angulo, dx, dy, theta, radio):
        """
        __init__    Valores inciales para el objeto.

        Args:
        x           -- (int)    Posición en x del objeto.
        y           -- (int)    Posición en y del objeto.
        equipo      -- (array)  Color identificador del jugador, equipo.
        etiqueta    -- (array)  Color de identificación del jugador, tag.
        angulo      -- (int)    Posición del angulo incial del objeto.
        dx          -- (int)    Variación del movimiento en x del objeto.
        dy          -- (int)    Variación del movimiento en y del objeto.
        theta       -- (int)    Variación del ángulo del objeto.
        radio       -- (int)    Radio del objeto.
        """
        self.x = x
        self.y = y
        self.equipo = equipo
        self.etiqueta = etiqueta
        self.angulo = angulo
        self.dx = dx
        self.dy = dy
        self.theta = theta
        self.radio = radio

    def mover(self):
        """
        mover   Genera el movimiento del objeto, con los valores del objeto.
        """
        self.x += self.dx
        self.y += self.dy
        self.angulo += self.theta

    def colision(self, obj):
        """
        colision    Cambia la dirección de la pelota (obj), cuando ésta choca
                    con un juagor.

        Args:
        obj     -- (objeto)     Objeto que impacta al otro.
        """
        pos_x = int(self.x - obj.x)
        pos_y = int(self.y - obj.y)
        rad = int(self.radio + obj.radio)
        if abs(pos_x) < rad and abs(pos_y) < rad:
            if pos_x < 0:
                obj.dx = obj.dx * -1
            elif pos_y < 0:
                obj.dy = obj.dy * -1
            elif pos_x > 0:
                obj.dx = obj.dx * -1
            elif pos_y > 0:
                obj.dy = obj.dy * -1

    def colision_borde(self):
        """
        colision_borde  Cambia la dirección si el objeto 
                        choca con el borde del juego.
        """
        if (self.x - self.radio) < 0:
            self.dx = self.dx * -1
        elif (self.x + self.radio) > ancho:
            self.dx = self.dx * -1
        if (self.y - self.radio) < 0:
            self.dy = self.dy * -1
        elif (self.y + self.radio) > alto:
            self.dy = self.dy * -1

    def desaceleracion(self):
        """
        desaceleracion  Frena el objeto por el roce que tiene con el campo.
        """
        if abs(self.dx) > 0 or abs(self.dy) > 0:
            if self.dx > 0:
                self.dx -= desaceleracion_roce
                self.dx = max(self.dx, 0)
            elif self.dx < 0:
                self.dx += desaceleracion_roce
                self.dx = min(self.dx, 0)

            if self.dy > 0:
                self.dy -= desaceleracion_roce
                self.dy = max(self.dy, 0)
            elif self.dy < 0:
                self.dy += desaceleracion_roce
                self.dy = min(self.dy, 0)
        if self.theta > 0:
            self.theta -= desaceleracion_roce
            self.theta = min(self.theta, 0)
        if self.theta < 0: 
            self.theta += desaceleracion_roce
            self.theta = max(self.theta, 0)

    def teclas(self):
        """
        teclas  Ayuda a poder controlar el objeto a 
                por medio de diferentes teclas.
        """
        keys = pygame.key.get_pressed()
        if keys[pygame.K_w]:
            self.dy -= 0.1
            self.dy = min(self.dy, -1)
        if keys[pygame.K_s]:
            self.dy += 0.1
            self.dy = max(self.dy, 1)
        if keys[pygame.K_a]:
            self.dx -= 0.1
            self.dx = min(self.dx, -1)
        if keys[pygame.K_d]:
            self.dx += 0.1
            self.dx = max(self.dx, 1)
        if keys[pygame.K_p]:
            self.dx = 0
            self.dy = 0
            self.theta = 0
        if keys[pygame.K_q]:
            self.theta += 0.05
            self.theta = max(self.theta, 1)
        if keys[pygame.K_e]:
            self.theta -= 0.05
            self.theta = min(self.theta, -1)


    def choque(self, obj):
        """
        choque  Entrega energía a la pelota (obj)
                cuando un jugador choca con ésta.

        Args:
        obj     -- (objeto) Objeto que es impactado (pelota).
        """
        keys = pygame.key.get_pressed()
        distancia = math.sqrt((self.x - obj.x) ** 2 + (self.y - obj.y) ** 2)
        rad = self.radio + obj.radio
        if distancia < rad:

            if keys[pygame.K_k]:
                print("dentro")
                obj.dx = self.dx 
                obj.dy = self.dy 

            elif not (keys[pygame.K_k]):
                # Calcular la dirección del golpe
                direccion = (obj.x - self.x, obj.y - self.y)
                
                # Normalizar la dirección
                direccion = (direccion[0] / distancia, direccion[1] / distancia)

                # Actualizar la velocidad de la pelota según la dirección del golpe
                obj.dx, obj.dy = [direccion[0] * 3, direccion[1] * 3]
            



def circulo(img, x, y, equipo, tag, angle):
    """
    circulo     Genera los circulos.
                Círculo negro rque representa el robot.
                Círculo de color que representa el equipo.
                2 círculos de color que son los indentificadores del jugador.
                Además simula el giro de los jugadores.

    Args:
    img     -- (matriz) Imagen sobre la cuál se dibujan los jugadores.
    x       -- (int) Valor en x para cada jugador.
    y       -- (int) Valor en y para cada jugador.
    equipo  -- (array) Vector con los valores del color del equipo correspondiente.
    tag     -- (array) Vector con los valores del color de identificación del jugador.
    angle   -- (int) Valor de ángulo para modificar la dirección del jugador.
    """
    pygame.draw.circle(img, negro, (x,y), 30)
    M = cv.getRotationMatrix2D((x, y), angle, 1)
    tf1 = np.dot(M, np.array([[x], [y+15], [1]]))
    tf2 = np.dot(M, np.array([[x+15], [y-10], [1]]))
    tf3 = np.dot(M, np.array([[x-15], [y-10], [1]]))
    pygame.draw.circle(img,equipo,  (int(tf1[0]), int(tf1[1])), 10)
    pygame.draw.circle(img, tag, (int(tf2[0]), int(tf2[1])), 10)
    pygame.draw.circle(img, tag, (int(tf3[0]), int(tf3[1])), 10)

# Configuración del video
ancho = 1280
alto = 650
fps = 60
duracion = 10  # Duración en segundos

# Colores
rojo    = (0,0,255)
azul    = (255,0,0)
cian    = (0,255,255)
magenta = (255,0,255)
blanco = (255,255,255)
cesped = (40, 128, 40)
negro = (0,0,0)
naranjo = (244,98,0)

# Crear la ventana de pygame
pygame.init()
ventana = pygame.display.set_mode((ancho, alto))
pygame.display.set_caption("Video de fútbol")
reloj = pygame.time.Clock()

# Fondo incial
fondo_inicial = pygame.Surface(ventana.get_size())
fondo_inicial.fill(cesped)

# Dibujar las líneas de la cancha
pygame.draw.rect(fondo_inicial, blanco, (20, 20, ancho-40, alto-40), 2)
pygame.draw.circle(fondo_inicial, blanco, (int(ancho/2), int(alto/2)), int(146), 2)
pygame.draw.line(fondo_inicial, blanco, (ancho/2, 20), (ancho/2, alto-21), 2)
pygame.draw.rect(fondo_inicial, blanco, (0, (alto/2)-100, 22,200), 2)
pygame.draw.rect(fondo_inicial, blanco, (ancho-22, (alto/2)-100, 22,200), 2)

# Crear instancias de la clase Jugador y pelota
player_1 = Jugador(200,             int(alto/2),    rojo, cian,     0,      0,  0,  0, 30)
player_2 = Jugador(int(ancho-200),  int(alto/2),    rojo, magenta,  45,     -1,  -1,  -1.1, 30)
player_3 = Jugador(int(ancho/2),    250,            azul, cian,     180,    -1,  1,  1.26, 30)
player_4 = Jugador(int(ancho/2),    int(alto-250),  azul, magenta,  270,    1,  -1,  -1.29, 30)

pelota = Jugador(int(ancho/2), int(alto/2), (0, 0, 255), None, 270, -2, -2, -1.29, 10)

# Bucle principal para generar el video
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()
    fondo = fondo_inicial.copy()



    # Dibujar los jugadores en la ventana
    circulo(fondo, int(player_1.x), int(player_1.y) , player_1.equipo , player_1.etiqueta, player_1.angulo)
    # circulo(ventana, int(player_2.x), int(player_2.y) , player_2.equipo , player_2.etiqueta, player_2.angulo)
    # circulo(ventana, int(player_3.x), int(player_3.y) , player_3.equipo , player_3.etiqueta, player_3.angulo)
    # circulo(ventana, int(player_4.x), int(player_4.y) , player_4.equipo , player_4.etiqueta, player_4.angulo)
 
    pygame.draw.circle(fondo, naranjo, (int(pelota.x), int(pelota.y)), pelota.radio)

    # Actualizar la posición de los jugadores
    player_1.teclas()
    player_1.mover()
    player_1.desaceleracion()
    player_1.choque(pelota)

    pelota.mover()
    pelota.desaceleracion()

    # player_2.mover()
    # player_3.mover()
    # player_4.mover()

    # Cambiar el sentido en caso de colisión con el borde del campo
    pelota.colision_borde()
    player_1.colision_borde()
    player_2.colision_borde()
    player_3.colision_borde()
    player_4.colision_borde()

    # Cambiar dirección en caso de colisión con algún otro objeto
    player_1.colision(pelota)
    player_2.colision(pelota)
    player_3.colision(pelota)
    player_4.colision(pelota)

    # Actualizar la pantalla con la copia del fondo y los elementos dibujados
    ventana.blit(fondo, (0, 0))
    pygame.display.update()
    reloj.tick(fps)

    # Obtener el frame actual de la ventana de Pygame
    frame = pygame.surfarray.array3d(ventana)
    frame = cv.transpose(frame)
    frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)
    cv.imshow("asd", frame)