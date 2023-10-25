############################
#  ALGORITMO DE 90 GRADOS  #
############################
class RutaGrados:
    def __init__(self, inicio, final, ob1, ob2, obs_comp, ball):

        self.ancho = 1250
        self.alto = 850 

        self.incio = inicio
        self.final = final
        self.obs1 = obs1
        self.obs2 = obs2
        self.obs_comp = obs_comp
        self.obs_ball = ball

        self.radio_jugador = 30
        self.radio_pelota = 30
        self.obstaculos = [[obs1, self.radio_jugador] , [obs2, self.radio_jugador], [obs_comp, self.radio_jugador] , [ball , self.radio_pelota]]


        self.completado = False

        self.trayectorias = []

    def d_eucli(self, punto1, punto2):
        """
        Obtiene la distancia euclidiana entre dos puntos
        """
        return np.hypot(punto1[0] - punto2[0], punto1[1] - punto2[1])

    def punto_perpendicular(self, punto_inicio, punto_final, punto_obs):
        """
        Obtiene un punto dentro de la trayectoria definida por punto_inicio y punto_final, tal que la trayectoria generada entre el 
        punto_obs y la trayectoria, sea perpendicular.
        """

        # Pendiendte de la recta principal
        m = ( punto_final[1] - punto_inicio[1] ) / ( punto_final[0] - punto_inicio[0] )

        # Pendiente perpendicular a la trayectoria anterior
        m_perpendicular = -1/m

        # Ecuaciones de las dos rectas
        A = np.array([[-m , 1],[-m_perpendicular , 1]])
        B = np.array([-m*punto_final[0] + punto_final[1], -m_perpendicular*punto_obs[0] + punto_obs[1]])

        # punto de intersección entre las rectas
        punto_en_trayectoria = np.linalg.solve(A,B)

        return punto_en_trayectoria


    def distancia_segura(self, obs, trayectoria):
        """
        Determina si la distancia entre la trayectoria y el obstáculo es seguro
        """

        nodo_trayectoria = punto_perpendicular(trayectoria[0], trayectororia[1], obs[0])

        distancia = d_eucli(nodo_trayectoria, obs[0])

        if distancia <= (obs[1] + self.radio_jugador):
            return [False, nodo_trayectoria]
        else:
            return [True, nodo_trayectoria]

    def nuevos_puntos(self, obs, nodo_inter):
        """
        Genera dos nuevos puntos en base a al punto del obstáculo.
        punto_incio y punto_final son los puntos para generar la trayectoria
        """

        # Determina el sentido del vector
        vector_director = nodo_inter - obs[0]

        # Normaliza el vector director en escala unitaria
        vector_normal = vector_director / np.linalg.norm(vector_director)

        # Calcula el nuevo punto
        nuevo_punto_1 = obs[0] + ((self.radio_jugador + obs[1]) * nomal_vector)
        nuevo_punto_1 = self.verificar_punto(self.obstaculos, nuevo_punto_1)

        # Calcula el nuevo punto
        nuevo_punto_2 = obs[0] + ((self.radio_jugador + obs[1]) * (-nomal_vector))
        nuevo_punto_2 = self.verificar_punto(self.obstaculos, nuevo_punto_2)

        return nuevo_punto_1, nuevo_punto_2


    def trayecto_inicial(self):
        """ 
        Detemrina si la tratyacetoria unica desde el punto de incio hasta el final es viable
        """




    def distancia_a_trayectoria(self, trayectoria):
        """ 
        Calcular la distancia que existe entre cada uno de los obstáculos y la trayectoria
        """


        for obs in self.obstaculos:
            dist_safe, nodo_inter = distancia_segura(obs, trayectoria)
            if not dist_safe:


    def generador_trayectorias(self):
        """
        Genera cada una de las trayectorias y verifica la viabilidad de cada una de ellas
        """


