import pygame
import numpy as np
import cv2 as cv
import math


######################
# CREACION DEL JUEGO #
######################
class Objeto:
    def __init__(self, masa,x, y, equipo, etiqueta, angulo, dx, dy, theta, radio):
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
        self.x = x
        self.y = y
        self.equipo = equipo
        self.etiqueta = etiqueta
        self.angulo = angulo
        self.dx = dx
        self.dy = dy
        self.theta = theta
        self.radio = radio
        self.desaceleracion_roce = 0.05
        self.masa = masa
        self.tomar_pelota = False
        self.ball_sujetada = 0
        self.velocidad = 2

    def mover(self):
        """
        Genera el movimiento del objeto, con los valores del objeto.
        """
        self.x += self.dx
        self.y += self.dy
        self.angulo += self.theta

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


    def colision_borde(self, ancho, alto):
        """
        Cambia la dirección si el objeto 
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
        Frena el objeto por el roce que tiene con el campo.
        """
        if abs(self.dx) > 0 or abs(self.dy) > 0:
            if self.dx > 0:
                self.dx -= self.desaceleracion_roce
                self.dx = max(self.dx, 0)
            elif self.dx < 0:
                self.dx += self.desaceleracion_roce
                self.dx = min(self.dx, 0)

            if self.dy > 0:
                self.dy -= self.desaceleracion_roce
                self.dy = max(self.dy, 0)
            elif self.dy < 0:
                self.dy += self.desaceleracion_roce
                self.dy = min(self.dy, 0)
        if self.theta > 0:
            self.theta -= self.desaceleracion_roce
            self.theta = min(self.theta, 0)
        if self.theta < 0: 
            self.theta += self.desaceleracion_roce
            self.theta = max(self.theta, 0)

    def teclas(self):
        """
        Ayuda a poder controlar el objeto a 
        por medio de diferentes teclas.
        """
        keys = pygame.key.get_pressed()

        # Movimiento vertical
        if keys[pygame.K_w]:
            self.dy -= 0.1
            self.dy = min(self.dy, -self.velocidad)
        elif keys[pygame.K_s]:
            self.dy += 0.1
            self.dy = max(self.dy, self.velocidad)
        else:
            self.dy = 0
        if keys[pygame.K_s] and keys[pygame.K_w]:
            self.dy = 0

        # Movimiento horizontal
        if keys[pygame.K_a]:
            self.dx -= 0.1
            self.dx = min(self.dx, -self.velocidad)
        elif keys[pygame.K_d]:
            self.dx += 0.1
            self.dx = max(self.dx, self.velocidad)
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
        self.distancia_ball = math.sqrt((obj.x - self.centro_tomar[0]) ** 2 + (obj.y - self.centro_tomar[1]) ** 2)

        keys = pygame.key.get_pressed()
        if keys[pygame.K_k]:
            if self.distancia_ball < 15:
                obj.x = self.posicion_pelota_agarre[0]
                obj.y = self.posicion_pelota_agarre[1]

                self.tomando_pelota = True # para el disparo
                self.ball_sujetada = 1
                print("si") 

        if not keys[pygame.K_k]:
            self.tomando_pelota = False
            if self.ball_sujetada == 1:
                factor = 5
                direccion = (obj.x - self.x, obj.y - self.y)
                direccion = (direccion[0] / self.rad, direccion[1] / self.rad)
                # Actualizar la velocidad de la pelota según la dirección del golpe y el factor de rebote
                obj.dx = direccion[0] * factor
                obj.dy = direccion[1] * factor

                self.ball_sujetada = 0


        if self.tomando_pelota:
            obj.x = self.posicion_pelota_agarre[0]
            obj.y = self.posicion_pelota_agarre[1]
            obj.dx = self.dx
            obj.dy = self.dy



    def choque(self, obj):
        """
        Entrega energía a la pelota (obj)
        cuando un jugador choca con ésta.

        Args:
        obj     -- (objeto) Objeto que es impactado (pelota).
        """
        keys = pygame.key.get_pressed()
        self.distancia = math.sqrt((self.x - obj.x) ** 2 + (self.y - obj.y) ** 2)
        self.rad = self.radio + obj.radio

        self.distancia_ball = math.sqrt((obj.x - self.centro_tomar[0]) ** 2 + (obj.y - self.centro_tomar[1]) ** 2)

        # Evitar posibles divisiones por cero
        if self.distancia == 0:
            self.distancia = 1

        if self.distancia < self.rad:

            if (self.distancia_ball > 15 or not keys[pygame.K_k]) and self.ball_sujetada == 0:
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
                overlap = self.rad - self.distancia
                obj.x += direccion[0] * overlap
                obj.y += direccion[1] * overlap

                self.ball_sujetada = 0



            
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
        self.posicion_pelota_agarre = (int(tf4[0]) , int(tf4[1]))


        tf5 = np.dot(M, np.array([[self.x], [self.y-30], [1]]))
        self.centro_tomar = (int(tf5[0]) , int(tf5[1]))


        pygame.draw.circle(img, (255,255,255),  (self.posicion_pelota_agarre[0], self.posicion_pelota_agarre[1]), 1)


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

    def seguimiento(self, hsv, img, frame):
        """
        Recorta la imagen original y hsv, para poder tener sólo la vecindad
        donde es posible que se mueva la pelota
        """
        # Recorta la imagen HSV y RGB
        self.roi_hsv = hsv[ self.y - self.vecindad:self.y + self.vecindad ,
                            self.x - self.vecindad:self.x + self.vecindad]
                            
        self.roi_img = img[ self.y - self.vecindad:self.y + self.vecindad , 
                            self.x - self.vecindad:self.x + self.vecindad]

        if len(self.roi_hsv) > 0: 
            # Detecta los circulos dentro del recorte y su centro 
            self.x_nuevo, self.y_nuevo, self.r_nuevo = self.detectar_circulos_color(self.roi_hsv, self.color, self.roi_img)
            # Reescribe el centro y actualiza este en el objeto
            self.x, self.y = self.x + self.x_nuevo - self.vecindad , self.y + self.y_nuevo - self.vecindad
            # Dibuja un circulo en el centro de la pelota
            cv.circle(frame, (self.x, self.y), 1, (255, 255, 255), -1)
        return


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
class Jugador:

    def __init__(self, equipo, colorID, centro):
        """
        Valores inciales para  la clase.

        Args:
        equipo      -- (array) Rangos de colores del equipo.
        colorId     -- (array) Rangos de colores del tag (un color por jugador).
        centro      -- (array) Coordenadas del centro del jugador.
        """
        self.equipo = equipo
        self.x, self.y = centro
        self.colorID = colorID
        self.vecindad = 70
        
    def dibujar(self, frame):   
        """
        Dibuja una línea entre los tag del jugador, con variables
        dentro de la misma clase.
        """
        # Reajusta los centros de los para poder dibujarlos en el 
        # frame principal
        self.x_tag1, self.y_tag1 = self.tags[0][1]
        self.x_tag1, self.y_tag1 = self.x + self.x_tag1 - self.vecindad , self.y + self.y_tag1 - self.vecindad
        self.x_tag2, self.y_tag2 = self.tags[1][1]
        self.x_tag2, self.y_tag2 = self.x + self.x_tag2 - self.vecindad , self.y + self.y_tag2 - self.vecindad
        cv.line(frame, (self.x_tag1, self.y_tag1), (self.x_tag2, self.y_tag2), (255,255,255),2)

    def seguimiento_players(self, hsv, img, frame):
        """
        Recorta la imagen original y hsv, para poder tener sólo la self.vecindad
        donde es posible que se mueva el jugador
        """
        # Recorta la imagen HSV y RGB
        self.roi_hsv = hsv[ self.y - self.vecindad : self.y + self.vecindad,
                            self.x - self.vecindad : self.x + self.vecindad]

        self.roi_img = img[ self.y - self.vecindad : self.y + self.vecindad,
                            self.x - self.vecindad : self.x + self.vecindad]
        if len(self.roi_img) > 0:
            # Detecta los circulos dentro del recorte y su centro 
            self.team = self.detectar_circulos_color(self.roi_hsv, self.equipo, self.roi_img)
            self.tags = self.detectar_circulos_color(self.roi_hsv, self.colorID, self.roi_img)
            self.centros = self.detectar_centro(self.team, self.tags)
            if len(self.centros) > 0:
                self.dibujar(frame)
                # Reescribe el centro y actualiza este en el objeto
                self.x_nuevo, self.y_nuevo = self.centros[0][5]
                self.x = self.x + self.x_nuevo - self.vecindad
                self.y = self.y + self.y_nuevo - self.vecindad

                # Dibuja un circulo en el centro del jugador
                cv.circle(frame, (self.x, self.y), 4, (255, 0, 255), -1)
        return

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

        color_bajo, color_alto, color_bajo2, color_alto2 = colores

        # Crear una máscara utilizando los rangos de color especificados
        mascara = cv.inRange(imagen_hsv, color_bajo, color_alto)
        if color_alto2 and color_bajo2 is not None:
            mascara1 = mascara
            mascara2 = cv.inRange(imagen_hsv, color_bajo2, color_alto2)
            mascara = cv.add(mascara1, mascara2)

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
            circulos = np.round(circulos[0, :]).astype(int)
            for (x, y, r) in circulos:
                #           (rango de colores , centro, radio)
                circulos_detectados.append([colores, (x,y), r])
        return circulos_detectados

    @classmethod
    def detectar_centro(cls, all_equipos, all_identificadores):
        """
        Detecta el centro del jugador en base al circulo de color del equipo, busca los 
        identificadores cercanos e identifica de que color son, creando un vector por cada
        jugador encontrado, con su

        Args:
        all_equipos         -- (array)  Un array de vectores, donde cada hay un vector 
                                        por cada circulo detectado de los colores de su
                                        equipo, este contiene:
                                        rango de colores, centro, radio
        all_identificadores -- (array)  Array de vectores, por cada identificador detectado
                                        por cada circulo del identificador contiene:
                                        rando de colores, centro, radio 

        Return:
        Jugadores   -- (array)  Contiene un vector por cada jugador encontrado
                                Cada vector contiene:
                                rango de color del equipo, rango de color del identificador,
                                centro del circulo por quipo, centro del ciruclo del primer 
                                identificador, centro del circulo del segundo identificador,
                                centro del jugador.
        """
        Jugadores = []

        for team in all_equipos:
            x_aux, y_aux = None,None
            for tag in all_identificadores:
                x_val , y_val = tag[1]
                x_cen , y_cen =team[1]

                d = math.sqrt((x_val - x_cen)**2 + (y_val - y_cen)**2) 
                if d <= 40:
                    if x_aux is None and y_aux is None:
                        x_aux, y_aux = x_val , y_val
                    else:
                        x_centro = (x_cen + x_aux + x_val) / 3
                        y_centro = (y_cen + y_aux + y_val) / 3
                        centro = (int(x_centro), int(y_centro))
                        # color equipo, color identificador, centro circulo team, centro ID1, centro ID2, centro del jugador 
                        Jugadores.append([team[0], tag[0], (x_cen , y_cen),(x_aux, y_aux), (x_val , y_val), centro])

        return Jugadores

