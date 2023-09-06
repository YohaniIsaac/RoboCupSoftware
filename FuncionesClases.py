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
        self.max_vel = 4
        self.f_aceleracion = 0.3

    def intrucciones(self, lista):
        # Obtención de los parámetros de la instruccion
        x_destino , y_destino = lista

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
            print(self.x, self.y)

        return en_curso 
 

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
        self.distancia_ball = math.sqrt((obj.x - self.centro_tomar[0]) ** 2 + (obj.y - self.centro_tomar[1]) ** 2)

        keys = pygame.key.get_pressed()
        if keys[pygame.K_k]:
            if self.distancia_ball < 15:
                obj.x = self.posicion_pelota_agarre[0]
                obj.y = self.posicion_pelota_agarre[1]

                self.tomando_pelota = True # para el disparo
                self.ball_sujetada = 1


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

        self.goles_rojo = 0
        self.goles_azul = 0
        self.pelota_fuera = True

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
            self.goles(frame)
            
        return self.x, self.y


    def goles(self, frame):
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
        return self.x, self.y

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


######################
#  CONTROLADORES PI  #
######################
class Control:
    def __init__(self, kp, ki):
        self.kp = kp
        self.ki = ki
        self.error_sum = [0, 0]
        self.cerca_ball = False
        self.radio_pelota = 20
        self.radio_jugador = 40


    def control_jugador_pelota(self, posicion_jugador, posicion_pelota):

        # Calcula el error en las coordenadas x e y
        error = [posicion_pelota[0] - posicion_jugador[0], posicion_pelota[1] - posicion_jugador[1]]
        print("error", error)

        # Actualiza la acumulación de errores para la componente integral
        self.error_sum[0] += error[0]
        self.error_sum[1] += error[1]
        
        # Calcula la acción de control proporcional e integral
        control = [
            self.kp * error[0] + self.ki * self.error_sum[0],
            self.kp * error[1] + self.ki * self.error_sum[1]
        ]
        
        # Calcula la siguiente posición del jugador teniendo en cuenta el control
        siguiente_posicion_jugador = [
            posicion_jugador[0] + control[0],
            posicion_jugador[1] + control[1]
        ]

        if -50 < error[0] and error[0] < 50 and -18 < error[1] and error[1] < 18:
            self.cerca_ball = True
        else:
            self.cerca_ball = False
        
        return siguiente_posicion_jugador

    def control_pelota(self, posicion_jugador, posicion_pelota, punto_objetivo):

        # Calcula el error en las coordenadas x e y
        error = [punto_objetivo[0] - posicion_pelota[0], punto_objetivo[1] - posicion_pelota[1]]
        print("error", error)

        # Actualiza la acumulación de errores para la componente integral
        self.error_sum[0] += error[0]
        self.error_sum[1] += error[1]
        
        # Calcula la acción de control proporcional e integral
        control = [
            self.kp * error[0] + self.ki * self.error_sum[0],
            self.kp * error[1] + self.ki * self.error_sum[1]
        ]
        
        # Calcula la siguiente posición del jugador teniendo en cuenta el control
        siguiente_posicion_jugador = [
            posicion_jugador[0] + control[0],
            posicion_jugador[1] + control[1]
        ]

        if error == 0:
            print("pelota en punto objetivo")

        
        return siguiente_posicion_jugador


    def calculate_next_player_position(self, current_player_pos, current_ball_pos, target_pos):
        # Calcula el error en las coordenadas x e y
        error = [target_pos[0] - current_ball_pos[0], target_pos[1] - current_ball_pos[1]]

        # Calcula la distancia entre el jugador y la pelota
        distance_to_ball = ((current_player_pos[0] - current_ball_pos[0])**2 + (current_player_pos[1] - current_ball_pos[1])**2)**0.5

        # Calcula la acción de control proporcional
        control_action = [
            self.kp * error[0],
            self.kp * error[1]
        ]

        # Si el jugador está lo suficientemente cerca de la pelota, retroalimenta con el error de la pelota
        if distance_to_ball < 40:  # Umbral de proximidad
            control_action[0] += current_ball_pos[0] - current_player_pos[0]
            control_action[1] += current_ball_pos[1] - current_player_pos[1]

        # Calcula la siguiente posición del jugador teniendo en cuenta el control_action
        next_player_pos = [
            current_player_pos[0] + control_action[0],
            current_player_pos[1] + control_action[1]
        ]

        return next_player_pos

    def calculo_puntos(ball_pos, num_points, radius, clockwise=True):
        neighboring_points = []
        angle_increment = 2 * math.pi / num_points
        
        if not clockwise:
            angle_increment *= -1

        for i in range(num_points):
            angle = i * angle_increment
            x = ball_pos[0] + radius * math.cos(angle)
            y = ball_pos[1] + radius * math.sin(angle)
            neighboring_points.append((x, y))

        return neighboring_points

    def arco(self, ball_pos, start_point, end_point, num_points, clockwise=True):
        neighboring_points = []
        
        # Calcula el radio desde la pelota a los puntos inicial y final
        start_radius = math.sqrt((start_point[0] - ball_pos[0])**2 + (start_point[1] - ball_pos[1])**2)
        end_radius = math.sqrt((end_point[0] - ball_pos[0])**2 + (end_point[1] - ball_pos[1])**2)
        
        # Calcula el ángulo inicial y final
        start_angle = math.atan2(start_point[1] - ball_pos[1], start_point[0] - ball_pos[0])
        end_angle = math.atan2(end_point[1] - ball_pos[1], end_point[0] - ball_pos[0])

        angle_increment = (end_angle - start_angle) / (num_points - 1)
        
        if not clockwise:
            angle_increment *= -1

        for i in range(num_points):
            angle = start_angle + i * angle_increment
            x = ball_pos[0] + start_radius * math.cos(angle)
            y = ball_pos[1] + start_radius * math.sin(angle)
            neighboring_points.append((int(x), int(y)))

        return neighboring_points

    def normalizar(self, vector):
        longitud = math.sqrt(vector[0] ** 2 + vector[1] ** 2)
        if longitud != 0:
            vector_normalizado = [vector[0] / longitud, vector[1] / longitud]
            return vector_normalizado
        else:
            return [0, 0]

    def controlcito(self, posicion_jugador, posicion_pelota, posicion_objetivo):

        velocidad_jugador = 10

        # Calcula la dirección desde la pelota hacia la posición objetivo
        direccion_pelota_a_objetivo = self.normalizar([posicion_objetivo[0] - posicion_pelota[0], posicion_objetivo[1] - posicion_pelota[1]])
        print("direccion_pelota_a_objetivo",direccion_pelota_a_objetivo)

        # Calcula la dirección en la que el jugador debe moverse para chocar con la pelota
        direccion_jugador_a_pelota = self.normalizar([posicion_pelota[0] - posicion_jugador[0], posicion_pelota[1] - posicion_jugador[1]])
        print("direccion_jugador a pelotaaaa", direccion_jugador_a_pelota)

        # Calcula la direccion en la que el jugador debe moverse hacia la posicion objetivo
        direccion_jugador_a_objetivo = self.normalizar([posicion_jugador[0] - posicion_pelota[0], posicion_jugador[1] - posicion_pelota[1]])
        print("direccion jugador a objetivo", direccion_jugador_a_objetivo)

        direccion_pelota_a_objetivo
        direccion_jugador_a_objetivo
        if direccion_pelota_a_objetivo != direccion_jugador_a_objetivo:

            error_direcciones = [direccion_pelota_a_objetivo[0] - direccion_jugador_a_objetivo[0], direccion_pelota_a_objetivo[1] - direccion_jugador_a_objetivo[1]]
            control_action = [self.kp * error_direcciones[0], self.kp * error_direcciones[1]]
            nueva_direccion_jugador = [direccion_jugador_a_pelota[0] + control_action[0], direccion_jugador_a_pelota[1] + control_action[1]]
            nueva_posicion_jugador = [posicion_jugador[0] + velocidad_jugador * nueva_direccion_jugador[0], posicion_jugador[1] + velocidad_jugador * nueva_direccion_jugador[1]]

            # direccion_deseada = direccion_pelota_a_objetivo
            # # Rodear la pelota en sentido horario o antihorario
            # direccion_rotacion = [direccion_deseada[0], direccion_deseada[1]]

            # nueva_posicion_jugador = [
            #     posicion_pelota[0] + (self.radio_pelota + self.radio_jugador) * direccion_rotacion[0],
            #     posicion_pelota[1] + (self.radio_pelota + self.radio_jugador) * direccion_rotacion[1]
            # ]
            return nueva_posicion_jugador

        # # Calcula el error entre las direcciones de pelota y jugador
        # error_direcciones = [direccion_pelota_a_objetivo[0] - direccion_jugador_a_pelota[0], direccion_pelota_a_objetivo[1] - direccion_jugador_a_pelota[1]]
        # print("error obtenido de las direcciones", error_direcciones)
        # # Calcula la acción de control proporcional
        # control_action = [self.kp * error_direcciones[0], self.kp * error_direcciones[1]]
        # print("control action", control_action)
        # # Calcula la nueva dirección del jugador
        # nueva_direccion_jugador = [direccion_jugador_a_pelota[0] + control_action[0], direccion_jugador_a_pelota[1] + control_action[1]]

        # nueva_posicion_jugador = [posicion_jugador[0] + velocidad_jugador * nueva_direccion_jugador[0], posicion_jugador[1] + velocidad_jugador * nueva_direccion_jugador[1]]

        return nueva_posicion_jugador


############################
#  ALGORITMO DE 90 GRADOS  #
############################
class RutaGrados:
    def __init__(self, inicio, final, ob1, ob2, ob3, ball):
        (x,y) = inicio
        self.x , self.y = incio
        self.x_final, self.y_final = final
        self.completado = False

    def 
        
