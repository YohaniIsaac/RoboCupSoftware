import pygame
import numpy as np
import cv2 as cv
import math


######################
# CREACION DEL JUEGO #
######################
class Objeto:
    def __init__(self, masa,x, y, equipo, etiqueta, angulo, dx, dy, theta, radio, identificador):
        """
        Valores inciales para la clase.

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
        # Posicion y velocidad del objeto
        self.x = x
        self.y = y
        self.dx = dx
        self.dy = dy
        self.angulo = angulo
        self.theta = theta              # velocidad angular

        self.equipo = equipo
        self.etiqueta = etiqueta
        self.radio = radio
        
        self.desaceleracion_roce = 0.5
        self.masa = masa                # no se esta utilizando la funcion actualmente

        self.max_vel = 10                # Velocidad maxima a la que puede llegar el objeto #4
        self.f_aceleracion = 1        # Factor de aceleracion


        self.identificador = identificador    # Valor de identificacion del objeto (0 para la pelota)
        
        if self.identificador != 0:
            path = "Tag_" + str(self.identificador) + ".png"
            arucoTag_img = pygame.image.load("arucoMarkers/" + path)
            self.arucoTag_img = pygame.transform.scale(arucoTag_img, (35,35))


        # Para realizar la toma de pelota y posterior disparo de la misma
        self.tomando_pelota = False

        # Dimensiones del campo de futbol
        self.ancho_campo = 1280
        self.alto_campo = 750



    def intrucciones(self, punto):
        # Obtención de los parámetros de la instruccion
        x_destino , y_destino = punto

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
 

    def motion_player(self, pelota, jugador_1 = None, jugador_2 = None, jugador_3 = None):
        """
        Genera el movimiento del objeto, con los valores del objeto.
        """
        self.x = int(self.x + self.dx)
        self.y = int(self.y + self.dy)

        self.x = max(self.radio, min(self.x, self.ancho_campo - self.radio))
        self.y = max(self.radio, min(self.y, self.alto_campo - self.radio))

        self.angulo = int(self.angulo - self.theta)
        self.angulo = self.angulo % 360

        # Interacción del jugador con la pelota
        self.choque(pelota)
        if jugador_1: self.choque(jugador_1)
        if jugador_2: self.choque(jugador_2)
        if jugador_3: self.choque(jugador_3)

        self.disparo(pelota)
        self.desaceleracion()
        # self.colision(pelota)

    def motion_ball(self):
        """
        Genera el movimiento de la pelota, con los valores del objeto.
        """
        self.x = int(self.x + self.dx)
        self.y = int(self.y + self.dy)

        # self.x = max(self.radio, min(self.x, self.ancho_campo - self.radio))
        # self.y = max(self.radio, min(self.y, self.alto_campo - self.radio))

        self.angulo = int(self.angulo - self.theta)
        self.angulo = self.angulo % 360

        self.desaceleracion()
        self.colision_borde()


    def colision(self, obj):
        """
        Cambia la dirección y velocidad de la pelota (obj), cuando ésta choca
        con un jugador.

        Args:
        obj     -- (objeto)     Objeto que impacta a la pelota.
        """
        pos_x = int(self.x - obj.x)
        pos_y = int(self.y - obj.y)
        rad = int(self.radio + obj.radio)

        if abs(pos_x) < rad and abs(pos_y) < rad:
            # Calcular la diferencia de velocidades entre el robot y la pelota
            diff_vel_x = obj.dx - self.dx
            diff_vel_y = obj.dy - self.dy

            # Calcular la magnitud de la diferencia de velocidades
            diff_vel_mag = math.sqrt(diff_vel_x**2 + diff_vel_y**2)

            # Si la magnitud de la diferencia de velocidades es suficientemente grande, transmitir energía
            if diff_vel_mag > 1.0:
                # Calcular el factor de rebote según la masa del robot y la pelota
                masa_total = self.masa + obj.masa
                factor_rebote = (2 * obj.masa) / masa_total


                # Ajustar la velocidad de la pelota después de la colisión
                obj.dx = self.dx + diff_vel_x * factor_rebote
                obj.dy = self.dy + diff_vel_y * factor_rebote


    def colision_borde(self):
        """
        Cambia la dirección si el objeto 
        choca con el borde del juego.
        """
        if (self.x - self.radio) < 0:
            self.dx = -self.dx
        elif (self.x + self.radio) > self.ancho_campo:
            self.dx = -self.dx 
        if (self.y - self.radio) < 0:
            self.dy = -self.dy 
        elif (self.y + self.radio) > self.alto_campo:
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

        self.dx = frenado(self.dx, self.desaceleracion_roce)
        self.dy = frenado(self.dy, self.desaceleracion_roce)
        self.theta = frenado(self.theta, self.desaceleracion_roce)

    def teclas(self):
        """
        Ayuda a poder controlar el objeto con las teclas.
        """
        keys = pygame.key.get_pressed()

        # Movimiento vertical
        if keys[pygame.K_w]:
            self.dy -= self.f_aceleracion
            self.dy = max(self.dy, -self.max_vel)
        elif keys[pygame.K_s]:
            self.dy += self.f_aceleracion
            self.dy = min(self.dy, self.max_vel)
        else:
            self.dy = 0
        if keys[pygame.K_s] and keys[pygame.K_w]:
            self.dy = 0

        # Movimiento horizontal
        if keys[pygame.K_a]:
            self.dx -= self.f_aceleracion
            self.dx = max(self.dx, -self.max_vel)
        elif keys[pygame.K_d]:
            self.dx += self.f_aceleracion
            self.dx = min(self.dx, self.max_vel)
        else:
            self.dx = 0
        if keys[pygame.K_a] and keys[pygame.K_d]:
            self.dx = 0


        # Movimiento radial
        if keys[pygame.K_q]:
            self.theta += 0.08
            self.theta = max(self.theta, 1)
        elif keys[pygame.K_e]:
            self.theta -= 0.08
            self.theta = min(self.theta, -1)
        else: self.theta = 0

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
            if self.distanciaConPelota < 15:
                obj.x = self.coordsHoldBall_x
                obj.y = self.coordsHoldBall_y

                self.tomando_pelota = True # para el disparo
            else:
                pass
        else:
            if self.tomando_pelota:
                vel_disparo = 10
                angle = math.radians(self.angulo)
                obj.dx = vel_disparo * math.cos(angle)
                obj.dy = vel_disparo * math.sin(angle)
                self.tomando_pelota = False


    def choque(self, obj):
        """
        Entrega energía a la pelota (obj)cuando un jugador choca con ésta.

        Args:
        obj     -- (objeto) Objeto que es impactado (pelota).
        """
        keys = pygame.key.get_pressed()


        self.distancia = math.dist((self.x, self.y), (obj.x, obj.y))
        self.distanciaColision = self.radio + obj.radio

        self.distanciaConPelota = math.dist((obj.x, obj.y), (self.coordsHoldBall_x, self.coordsHoldBall_y))

        # Evitar posibles divisiones por cero
        if self.distancia == 0:
            self.distancia = 1

        if self.distancia < self.distanciaColision:

            if (self.distanciaConPelota > 15 or not keys[pygame.K_k]):
                # Calcular la dirección del golpe 
                direccion = (obj.x - self.x, obj.y - self.y)
                    
                # Normalizar la dirección
                direccion = (direccion[0] / self.distancia, direccion[1] / self.distancia)

                # Calcular el factor de rebote según la masa de los objetos (jugador y pelota)
                masa_total = self.masa + obj.masa
                factor_rebote = (2 * obj.masa) / masa_total

                # Actualizar la velocidad de la pelota según la dirección del golpe y el factor de rebote
                obj.dx = direccion[0] * factor_rebote
                obj.dy = direccion[1] * factor_rebote

                # Evitar superposición de objetos después de la colisión
                overlap = self.distanciaColision - self.distancia
                obj.x += direccion[0] * overlap
                obj.y += direccion[1] * overlap



            
    def circulo(self, img, pelota_x , pelota_y, pelota_radio, naranjo):
        """
        Genera los circulos.
        Círculo negro que representa el robot.
        Círculo de color que representa el equipo.
        2 círculos de color que son los indentificadores del jugador.
        Además simula el giro de los jugadores.

        Args:
        img     -- (matriz) Imagen sobre la cuál se dibujan los jugadores.
        """
        negro = (0,0,0)
        pygame.draw.circle(img, negro, (self.x, self.y), 30)
        M = cv.getRotationMatrix2D((self.x, self.y), self.angulo, 1)
        tf1 = np.dot(M, np.array([[self.x], [self.y+15], [1]]))
        tf2 = np.dot(M, np.array([[self.x+15], [self.y-10], [1]]))
        tf3 = np.dot(M, np.array([[self.x-15], [self.y-10], [1]]))
        pygame.draw.circle(img, self.equipo,  (int(tf1[0]), int(tf1[1])), 10)
        pygame.draw.circle(img, self.etiqueta, (int(tf2[0]), int(tf2[1])), 10)
        pygame.draw.circle(img, self.etiqueta, (int(tf3[0]), int(tf3[1])), 10)
        pygame.draw.circle(img, naranjo, (pelota_x , pelota_y), pelota_radio)



        self.centro_team = int(tf1[0]), int(tf1[1])
        self.centro_tag1 = int(tf2[0]), int(tf2[1])
        self.centro_tag2 = int(tf3[0]), int(tf3[1])

        tf4 = np.dot(M, np.array([[self.x], [self.y-40], [1]]))
        self.coordsHoldBall_x = int(tf4[0]) 
        self.coordsHoldBall_x = int(tf4[1])


        pygame.draw.circle(img, (255,255,255),  (self.coordsHoldBall_x, self.coordsHoldBall_x), 1)

    def generationRobot(self, fondo):
        """
        Genera un robot ciruclar con un arugotag en la parte superior de éste
        
        Args:
        fondo   -- (matriz) Fonde del campo de futbol

        """ 
        negro = (0,0,0)
        blanco = (255,255,255)

        # Dibuja el círculo
        pygame.draw.circle(fondo, negro, (self.x, self.y), 30)

        # Rota la imagen
        rotated_image = pygame.transform.rotate(self.arucoTag_img, -self.angulo)
        rotated_rect = rotated_image.get_rect(center=(self.x, self.y))
        fondo.blit(rotated_image, rotated_rect)

        # Calcula la posición del punto que indica el frente
        front_angle = math.radians(self.angulo)  # Convierte el ángulo a radianes
        self.coordsHoldBall_x = int(self.x + 40 * math.cos(front_angle))  # 30 es el radio del círculo
        self.coordsHoldBall_y = int(self.y + 40 * math.sin(front_angle))

        
        # Dibuja el punto indicando el frente
        pygame.draw.circle(fondo, blanco, (self.coordsHoldBall_x, self.coordsHoldBall_y), 5)

    def generationBall(self, fondo):
        """
        Genera una pelota ciruclar 
        
        Args:
        fondo   -- (matriz) Fonde del campo de futbol
        
        """ 

        naranjo = (244,98,0)
        pygame.draw.circle(fondo, naranjo, (self.x , self.y), self.radio)       







#########################
# BUSQUEDA DE LA PELOTA #
#########################
class Ball:

    def __init__(self, color, centro):
        """
        Valores inciales para  la clase.

        Args:
        color      -- (array) Rangos de colores.
        centro      -- (array) Coordenadas del centro del jugador.
        """
        self.color = color
        self.x, self.y = centro
        self.vecindad = 20

        self.goles_rojo = 0
        self.goles_azul = 0
        self.pelota_fuera = True

    def seguimiento(self, hsv, img, frame):
        """
        Recorta la imagen original y hsv, para poder tener sólo la vecindad donde es posible que se mueva la pelota
        """
        # Recorta la imagen HSV y RGB
        self.roi_hsv = hsv[ self.y - self.vecindad:self.y + self.vecindad ,
                            self.x - self.vecindad:self.x + self.vecindad]
                            
        self.roi_img = img[ self.y - self.vecindad:self.y + self.vecindad , 
                            self.x - self.vecindad:self.x + self.vecindad]

        if len(self.roi_hsv) > 0: 
            # Detecta los circulos dentro del recorte y su centro 
            self.x_nuevo, self.y_nuevo, self.r_nuevo = self.detectar_circulos_color(self.roi_hsv, self.color, self.roi_img)
            cv.circle(self.roi_hsv, (self.x_nuevo, self.y_nuevo), self.r_nuevo, (0, 0, 0), 1)
            # Reescribe el centro y actualiza este en el objeto
            self.x, self.y = self.x + self.x_nuevo - self.vecindad , self.y + self.y_nuevo - self.vecindad
            # Dibuja un circulo en el centro de la pelota
            cv.circle(frame, (self.x, self.y), 1, (0, 0, 0), -1)
            cv.circle(self.roi_hsv, (self.x, self.y), self.r_nuevo, (0, 0, 0), 1)
            self.goles(frame)
            
        return self.x, self.y


    def goles(self, frame):
        """
        Detecta los goles de cada uno de los equipos ****
        """
        r = 10
        # distancia_derecha = ((self.x - x__der_arco)**2 + (self.y - y_der_arco)**2)**0.5
        # Contador de goles para el equipo azul
        if self.x + 5 >= 1260 and self.y > 225 and self.y < 325 and self.pelota_fuera:
            self.goles_azul += 1
            self.pelota_fuera = False
        
        # Contador de goles para el equipo rojo
        elif self.x - 5 <= 22  and self.y > 225 and self.y < 325 and self.pelota_fuera:
            self.goles_rojo += 1
            self.pelota_fuera = False


        font = cv.FONT_HERSHEY_SIMPLEX
        posicion_texto_rojo = (50, 50)
        posicion_texto_azul = (frame.shape[1] - 200, 50)
        color_rojo = (0, 0, 255)  # Rojo en formato BGR
        color_azul = (255, 0, 0)  # Azul en formato BGR
        goles_texto_rojo = f"Goles Rojos: {self.goles_rojo}"
        goles_texto_azul = f"Goles Azules: {self.goles_azul}"

        cv.putText(frame, goles_texto_rojo, posicion_texto_rojo, font, 0.7, color_rojo, 2, cv.LINE_AA)
        cv.putText(frame, goles_texto_azul, posicion_texto_azul, font, 0.7, color_azul, 2, cv.LINE_AA)



    @classmethod 
    def detectar_circulos_color(cls, imagen_hsv, colores, imagen_original):
        """
        Detecta los circulos de colores

        Args:
        imagen_hsv  -- (matriz) Matriz de la imagen en HSV 
        colores     -- (array) Rangos de los colores.
        imagen      -- (matriz) Matriz de la imagen en RGB

        Return:
        circulos_detetados  -- (array)  Contiene un vector por cada color detectado
                                        este contiene: el rago de colores, centro, radio
        """
        circulos_detectados = []

        color_bajo, color_alto = colores

        # Crear una máscara utilizando los rangos de color especificados
        mascara = cv.inRange(imagen_hsv, color_bajo, color_alto)

        # Aplicar la máscara a la imagen original
        imagen_filtrada = cv.bitwise_and(imagen_original, imagen_original, mask=mascara)

        # Convertir la imagen filtrada a escala de grises
        imagen_gris = cv.cvtColor(imagen_filtrada, cv.COLOR_BGR2GRAY)

        # Aplicar un filtro de suavizado para reducir el ruido
        imagen_suavizada = cv.GaussianBlur(imagen_gris, (5,5),0)

        # Aplicar la transformada de Hough para detectar círculos
        circulos = cv.HoughCircles(imagen_suavizada, cv.HOUGH_GRADIENT, 1, minDist=20,
                                    param1=15, param2=15,
                                    minRadius= 5, maxRadius= 50)

        # Si se detectaron círculos, agregarlos a la lista de circulos_detectados
        if circulos is not None:
            x , y , r = circulos[0][0][0], circulos[0][0][1], circulos[0][0][2]
        return int(x), int(y), int(r)


###########################
#  BUSQUEEDA DE JUGADORES #
###########################
# class Jugador:

#     def __init__(self, equipo, colorID, centro):
#         """
#         Valores inciales para  la clase.

#         Args:
#         equipo      -- (array) Rangos de colores del equipo.
#         colorId     -- (array) Rangos de colores del tag (un color por jugador).
#         centro      -- (array) Coordenadas del centro del jugador.
#         """
#         self.equipo = equipo
#         self.x, self.y = centro
#         self.colorID = colorID
#         self.vecindad = 70
        
#     def dibujar(self, frame):   
#         """
#         Dibuja una línea entre los tag del jugador, con variables dentro de la misma clase.
#         """

#         # Reajusta los centros de los para poder dibujarlos en el 
#         # frame principal
#         self.x_tag1, self.y_tag1 = self.tags[0][1]
#         self.x_tag1, self.y_tag1 = self.x + self.x_tag1 - self.vecindad , self.y + self.y_tag1 - self.vecindad
#         self.x_tag2, self.y_tag2 = self.tags[1][1]
#         self.x_tag2, self.y_tag2 = self.x + self.x_tag2 - self.vecindad , self.y + self.y_tag2 - self.vecindad
#         cv.line(frame, (self.x_tag1, self.y_tag1), (self.x_tag2, self.y_tag2), (255,255,255),2)

#     def seguimiento_players(self, hsv, img, frame):
#         """
#         Recorta la imagen original y hsv, para poder tener sólo la vecindad donde es posible que se mueva el jugador
#         """

#         # Recorta la imagen HSV y RGB segun la vecindad donde e sposible que se mueva el jugador
#         self.roi_hsv = hsv[ self.y - self.vecindad : self.y + self.vecindad,
#                             self.x - self.vecindad : self.x + self.vecindad]

#         self.roi_img = img[ self.y - self.vecindad : self.y + self.vecindad,
#                             self.x - self.vecindad : self.x + self.vecindad]

#         if len(self.roi_img) > 0:

#             # Detecta los circulos dentro del recorte y su centro 
#             self.team = self.detectar_circulos_color(self.roi_hsv, self.equipo, self.roi_img)
#             self.tags = self.detectar_circulos_color(self.roi_hsv, self.colorID, self.roi_img)
#             self.centros = self.detectar_centro(self.team, self.tags)

#             if len(self.centros) > 0:
#                 self.dibujar(frame)
#                 # Reescribe el centro y actualiza este en el objeto
#                 self.x_nuevo, self.y_nuevo = self.centros[0][5]
#                 self.x = self.x + self.x_nuevo - self.vecindad
#                 self.y = self.y + self.y_nuevo - self.vecindad

#                 # Dibuja un circulo en el centro del jugador
#                 cv.circle(frame, (self.x, self.y), 4, (255, 0, 255), -1)
#         return self.x, self.y

#     @classmethod
#     def detectar_circulos_color(cls, imagen_hsv, colores, imagen_original):
#         """
#         Detecta los circulos de colores

#         Args:
#         imagen_hsv  -- (matriz) Matriz de la imagen en HSV 
#         colores     -- (array) Rangos de los colores.
#         imagen      -- (matriz) Matriz de la imagen en RGB

#         Return:
#         circulos_detetados  -- (array)  Contiene un vector por cada color detectado
#                                         este contiene: el rago de colores, centro, radio
#         """
#         circulos_detectados = []

#         color_bajo, color_alto, color_bajo2, color_alto2 = colores

#         # Crear una máscara utilizando los rangos de color especificados
#         mascara = cv.inRange(imagen_hsv, color_bajo, color_alto)
#         if color_alto2 and color_bajo2 is not None:
#             mascara1 = mascara
#             mascara2 = cv.inRange(imagen_hsv, color_bajo2, color_alto2)
#             mascara = cv.add(mascara1, mascara2)

#         # Aplicar la máscara a la imagen original
#         imagen_filtrada = cv.bitwise_and(imagen_original, imagen_original, mask=mascara)

#         # Convertir la imagen filtrada a escala de grises
#         imagen_gris = cv.cvtColor(imagen_filtrada, cv.COLOR_BGR2GRAY)

#         # Aplicar un filtro de suavizado para reducir el ruido
#         imagen_suavizada = cv.GaussianBlur(imagen_gris, (5,5),0)

#         # Aplicar la transformada de Hough para detectar círculos
#         circulos = cv.HoughCircles(imagen_suavizada, cv.HOUGH_GRADIENT, 1, minDist=20,
#                                     param1=15, param2=15,
#                                     minRadius= 5, maxRadius= 50)

#         # Si se detectaron círculos, agregarlos a la lista de circulos_detectados
#         if circulos is not None:
#             circulos = np.round(circulos[0, :]).astype(int)
#             for (x, y, r) in circulos:
#                 #           (rango de colores , centro, radio)
#                 circulos_detectados.append([colores, (x,y), r])
#         return circulos_detectados

#     @classmethod
#     def detectar_centro(cls, all_equipos, all_identificadores):
#         """
#         Detecta el centro del jugador en base al circulo de color del equipo, busca los 
#         identificadores cercanos e identifica de que color son, creando un vector por cada
#         jugador encontrado, con su

#         Args:
#         all_equipos         -- (array)  Un array de vectores, donde hay un vector 
#                                         por cada circulo detectado de los colores de su
#                                         equipo, este contiene:
#                                         rango de colores, centro, radio
#         all_identificadores -- (array)  Array de vectores, por cada identificador detectado
#                                         por cada circulo del identificador contiene:
#                                         rando de colores, centro, radio 

#         Return:
#         Jugadores   -- (array)  Contiene un vector por cada jugador encontrado
#                                 Cada vector contiene:
#                                 rango de color del equipo, rango de color del identificador,
#                                 centro del circulo por quipo, centro del ciruclo del primer 
#                                 identificador, centro del circulo del segundo identificador,
#                                 centro del jugador.
#         """
#         Jugadores = []

#         for team in all_equipos:
#             x_aux, y_aux = None,None
#             for tag in all_identificadores:
#                 x_val , y_val = tag[1]
#                 x_cen , y_cen =team[1]

#                 d = math.sqrt((x_val - x_cen)**2 + (y_val - y_cen)**2) 
#                 if d <= 40:
#                     if x_aux is None and y_aux is None:
#                         x_aux, y_aux = x_val , y_val
#                     else:
#                         x_centro = (x_cen + x_aux + x_val) / 3
#                         y_centro = (y_cen + y_aux + y_val) / 3
#                         centro = (int(x_centro), int(y_centro))
#                         # color equipo, color identificador, centro circulo team, centro ID1, centro ID2, centro del jugador 
#                         Jugadores.append([team[0], tag[0], (x_cen , y_cen),(x_aux, y_aux), (x_val , y_val), centro])

#         return Jugadores


ARUCO_DICT = {
    "DICT_6X6_50": cv.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv.aruco.DICT_7X7_1000,
}


aruco_type = "DICT_7X7_1000"

arucoDict = cv.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])


def deteccionJugadoresArucoTag(frame):

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    cv.aruco_dict = cv.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])

    parameters = cv.aruco.DetectorParameters()
    detector = cv.aruco.ArucoDetector(cv.aruco_dict, parameters)


    corners, ids, rejected_img_points = detector.detectMarkers(gray)
    

    datos = []
    if ids is not None:
        for corner, aruco_id in zip(corners, ids):
            # corners[i] tiene la forma [4, 1, 2], con 4 esquinas, 1 array por esquina y 2 coordenadas (x, y)
            corner_points = corner.reshape(4, 2)  # Aplanar la matriz para obtener las esquinas
            
            # Calcular el centro (promedio de las coordenadas de las esquinas)
            center_x = np.mean(corner_points[:, 0])
            center_y = np.mean(corner_points[:, 1])
                    
            # Calcular el ángulo de rotación
            # Usaremos las primeras dos esquinas para calcular el ángulo
            # Se asume que las esquinas están ordenadas de manera consistente
            vector_1 = corner_points[1] - corner_points[0]
            angle = np.arctan2(vector_1[1], vector_1[0])
            angle_deg = np.degrees(angle)

            identificador = aruco_id[0]

            # print(f"Marcador {ids[i]} - Centro: ({center_x}, {center_y}), Ángulo: {angle_deg} grados")
            datos.append({"id": identificador, "x": center_x, "y": center_y, "angulo": angle_deg})

            # Dibujar el centro y la orientación en la imagen
            cv.circle(frame, (int(center_x), int(center_y)), 5, (0, 255, 0), -1)
            end_point = (int(center_x + 50 * np.cos(angle)), int(center_y + 50 * np.sin(angle)))
            cv.line(frame, (int(center_x), int(center_y)), end_point, (0, 255, 0), 2)
        


    return frame, datos






# def DetectarJugadoresCirculosDeColores(frame):

#     # Copia el frame para manejarlo y lo transoforma a escala HSV
#     img = np.copy(frame)
#     hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

#     if first_frame:
#         circulos_rojos      = FyC.Jugador.detectar_circulos_color(hsv, rojo, img) 
#         circulos_azul       = FyC.Jugador.detectar_circulos_color(hsv, azul, img) 
#         circulos_cian       = FyC.Jugador.detectar_circulos_color(hsv, cian, img) 
#         circulos_magenta    = FyC.Jugador.detectar_circulos_color(hsv, magenta, img) 

#         all_equipos = circulos_rojos + circulos_azul
#         all_identificadores = circulos_magenta + circulos_cian

#         Jugadores = FyC.Jugador.detectar_centro(all_equipos,all_identificadores)
        

#         equipo = Jugadores[0][0]
#         colorID = Jugadores[0][1]
#         centro = Jugadores[0][5]
        
#         players = []
#         for jugador in Jugadores:
#             equipo = jugador[0]
#             colorID = jugador[1]
#             centro2 = jugador[3]
#             centro3 = jugador[4]
#             centro = jugador[5]

#             player = FyC.Jugador(equipo, colorID, centro)
#             players.append(player)

#         first_frame = False
        
#     else:
#         for player in players:
#             x, y = player.seguimiento_players(hsv,img,frame)
#             # Enviar el centro del jugador

#             cv.imshow("jugador 1", player.roi_hsv)
#             enviar = (x,y)
#         queue.put(("juagador", enviar))
    # cv.imshow("frame jugador ", frame)
