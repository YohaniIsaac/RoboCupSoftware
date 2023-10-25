import numpy as np
import matplotlib.pyplot as plt

############################
#  ALGORITMO DE 90 GRADOS  #
############################
class RutaGrados:
    def __init__(self, radio_jugador, radio_pelota, d_seguridad):

        self.ancho = 1250
        self.alto = 850 

        self.radio_jugador = radio_jugador
        self.radio_pelota = radio_pelota

        self.d_of_security = d_seguridad


    def inicial(self, inicio, final, obs1, obs2, obs3, ball):
        """
        Inicia las variables para encontrar la ruta

        Return
        Retorna una ruta
        """

        self.inicio = inicio
        self.final = final
        self.obs1 = obs1
        self.obs2 = obs2
        self.obs_comp = obs3
        self.obs_ball = ball
        self.obstaculos = [[obs1, self.radio_jugador] , [obs2, self.radio_jugador], [self.obs_comp, self.radio_jugador] , [ball , self.radio_pelota]]

        return self.generator()

    def generator(self):
        """
        en base a las otras funciones de la clase, encuentra las posibles trayectorias y selecciona la mas corta de éstas.

        Return:
        ruta    -- (lista) contiene los vectores para generar la ruta desde el punto de inicio hasta el final
        """
        puntos_inicio = [self.inicio]
        final = self.final
        vectores = []
        g = 0
        for inicio in puntos_inicio:
            g += 1
            print("------------------------- %i -------------------------------------------" %g)
            print("puntos incio:   ", puntos_inicio)
            print("incio en evaluacion", inicio)
            for obs, radio in self.obstaculos:
                if self.evaluacion_de_obstaculo(obs, inicio,final):
                    # Busca punto en pre-trayectoria generada por incio-final
                    # Determina si la pre-trayectroria pasa por el obstaculos
                    d_safe , punto_en_trayectoria = self.distancia_segura(inicio, final, obs, radio)
                    # si la distancia no es segura (puede chocar)
                    if not d_safe:
                        print("no safe")
                        punto_1, punto_2 = self.nuevos_puntos(obs, radio, punto_en_trayectoria)
                        aux = False
                        if self.punto_en_mapa(punto_1):
                            vectores.append([inicio, punto_1])
                            puntos_inicio.append(punto_1)
                            print("agregó punto 1 : ", punto_1)
                            aux = True

                        if self.punto_en_mapa(punto_2):
                            vectores.append([inicio, punto_2])
                            puntos_inicio.append(punto_2)
                            print("agregó punto 2 : ", punto_2)
                            aux = True
                        if aux:
                            break
                    else:
                        vectores.append([inicio, final])
        print("vectores", vectores)

        if len(vectores) == 0:
            ruta = [inicio, final]
            return ruta
        else:
            trayectorias = self.generar_rutas(vectores, self.inicio, self.final)
            print("trayectorias completitas:   ", trayectorias)
            ruta = self.mejor_ruta(trayectorias)
            return ruta

    def evaluacion_de_obstaculo(self, obs, inicio, fin):
        """
        determina si el obstáculo esta dentro de la trayectoria y por ende si es necesario evaluarlo

        Args: 
        obs         -- (vector) coordenadas x,y del obstaculos
        incio       -- (vector) coordenadas x,y del punto de inicio de la pre-trayectoria
        fin         -- (vector) coordenadas x,y del punto final de la pre-trayectoria

        Return
        bool        -- Determina si es necesario evaluar el obstáculo True si HAY que evaluarlo 
                        False si NO HAY  que evaluarlo
        """
        if obs[0] > inicio[0] and obs[0] < fin[0]:
            return True
        else: 
            return False


    def punto_perpendicular(self, punto_inicio, punto_final, punto_obs):
        """
        Obtiene un punto dentro de la pre-trayectoria definida por punto_inicio y punto_final, tal que la trayectoria generada entre el 
        punto_obs y la pre-trayectoria, sean perpendiculares.

        Args:
        punto_inicio    -- (vector) coordenadas x,y del punto de incio
        punto_final     -- (vector) coordenadas x,y del punto de final
        punto_obs       -- (vector) coordenadas x,y del obstaculo

        Return:
        punto_en_trayectoria    -- (vector) coordenadas x,y del nuevo punto en la pre-trayectoria
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
                
    def distancia_segura(self, punto_inicio, punto_final, punto_obs, radio, plus=2):
        """
        Determina si la distancia entre la pre-trayectoria y el obstáculo es seguro, y calcula el punto en la trayectoria

        Args:
        punto_inicio    -- (vector) coordenadas x,y del punto de incio
        punto_final     -- (vector) coordenadas x,y del punto de final
        punto_obs       -- (vector) coordenadas x,y del obstaculo
        radio           -- (int) radio del obstaculo que se está analizando
        plus            -- (int) numero de seguridad para que no se rocen los objetos
        """
        nodo_trayectoria = self.punto_perpendicular(punto_inicio, punto_final, punto_obs)
        distancia = self.d_eucli(nodo_trayectoria, punto_obs)
        print("distancia : ", distancia)
        print("nodo en trayectoria: ", nodo_trayectoria)

        if distancia <= (radio + self.radio_jugador + plus):
            return [False, nodo_trayectoria]
        else:
            return [True, nodo_trayectoria]

    def d_eucli(self, punto1, punto2):
        """
        Obtiene la distancia euclidiana entre dos puntos
        """
        return np.hypot(punto1[0] - punto2[0], punto1[1] - punto2[1])

    def nuevos_puntos(self, obs, radio, punto_en_trayectoria):
        """
        Genera dos nuevos puntos en base al obstaculo y al punto en el trayectoria.

        Args:
        obs                     -- (vector) coordenadas x,y del obstaculo
        radio                   -- (int) radio del obstaculo
        punto_en_trayectoria    -- (vector) coordendas x,y del punto en la trayectoria (generado por: punto_perpendicular)

        Return:
        nuevo_punto_1, nuevo_punto_2 -- (vector) coordenas x,y de los nuevos puntos encontrados
        """

        # Determina el sentido del vector
        vector_director = punto_en_trayectoria - obs

        # Normaliza el vector director en escala unitaria
        vector_normal = vector_director / np.linalg.norm(vector_director)

        distancia_segura = self.radio_jugador + radio + self.d_of_security 

        # Calcula el nuevo punto
        nuevo_punto_1 = obs + distancia_segura * vector_normal

        # Calcula el nuevo punto
        nuevo_punto_2 = obs + distancia_segura * (-vector_normal)


        return np.round(nuevo_punto_1).astype(int), np.round(nuevo_punto_2).astype(int)

    def punto_en_mapa(self, punto):
        """
        verifica que el punto dado se encuentra dentro del campo de juego

        Args: 
        punto       -- (vector) coordenadas x,y del punto 

        Return:
        True        -- esta dentro del campo 
        False       -- Esta fuera del campo
        """
        if 0 <= punto[0] < self.ancho and 0 <= punto[1] < self.alto:
            return True
        else:
            return False

    def encontrar_rutas(self, vectores, actual, destino, ruta_actual, rutas_encontradas):
        """
        funcion recursiva que busca rutas desde un punto de inicio hasta un punto finals sobre una
        una lista de vectores

        Args:
        vectores            -- (list) lista de vectores que define las conexiones entre los nodos
        ruta_actual         -- (vector) coordenadas x,y del punto actual
        destino             -- (vector) coordenadas del nodo destino al que queremos llegar
        ruta_actual         -- (list) lista que almacena la ruta actual que estamos explorando
        rutas_encontradas   -- (list) listya que almacena todas las rutas válidas encontradas
        """

        # Verifica que se ha encontrado una ruta que llega desde el punto de incio hasta el final
        if np.array_equal(actual, destino):
            # Si se encontró una ruta, entonces se copia en rutas_encontradas
            rutas_encontradas.append(ruta_actual.copy())
            return
        # itera sobre la misma función tal que busca la conexion que existe entre cada uno de los vectores 
        for vector in vectores:
            origen, siguiente = vector
            # if origen == actual and siguiente not in ruta_actual:
            if np.array_equal(origen,actual) and not any(np.all(siguiente == nodo) for nodo in ruta_actual):
                ruta_actual.append(siguiente)
                self.encontrar_rutas(vectores, siguiente, destino, ruta_actual, rutas_encontradas)
                ruta_actual.pop()

    def generar_rutas(self, vectores, inicio, final):
        """
        inicializa las listas de ruta_actual y rutas_encontradas

        Args:
        vectores          -- (list) lista de vectores que define las conexiones entre los nodos
        inicio            -- (vector) coordenadas x,y del punto de inicio
        final             -- (vector) coordenadas x,y del punto de final
        """
        rutas_encontradas = []
        comienzo = [inicio] 
        self.encontrar_rutas(vectores, inicio, final, comienzo, rutas_encontradas)
        return rutas_encontradas

    def mejor_ruta(self, trayectorias):
        """
        itera por sobre todas las trayectorias que se encontraron, verifica que no haya ningún obstaculos 
        y retorna la ruta con la distancia más corta desde el incio al final.

        Args:
        trayectoria     -- (matriz) en cada fila tiene una posible trayectoria

        Return:
        ruta_menor_distancia -- (list) lista que contiene la trayectoria con la menor distancia hasta el punto final
        """
        nuevas_rutas = [ruta for ruta in trayectorias if all(self.distancia_segura(ruta[i], ruta[i+1], obs, radio)[0] for i in range(len(ruta)-1) for obs,radio in self.obstaculos)]
        print("nuevas :", nuevas_rutas)
        ruta_menor_distancia = min( nuevas_rutas, key=lambda ruta: sum(self.d_eucli(ruta[i], ruta[i+1]) for i in range(len(ruta) - 1)) )

        return ruta_menor_distancia

def main():
    inicio = np.array([10,10]) 
    final = np.array([1000, 700])

    obs1 = np.array([200,100])
    obs2 = np.array([400,300])
    obs3 = np.array([600, 350])
    ball = np.array([800, 650])
    radio_jugador = 30
    radio_pelota = 10
    d_seguridad = 10

    # plt.scatter(inicio[0], inicio[1], color = "blue")
    # plt.scatter(final[0], final[1], color = "green")

    # plt.grid(True)
    # plt.show()
    ruta = RutaGrados(radio_jugador, radio_pelota, d_seguridad)
    trayectoria = ruta.inicial(inicio, final, obs1, obs2, obs3, ball)
    print(trayectoria)

    graf_obs(obs1, obs2, obs3,ball, radio_jugador, radio_pelota)
    x = []
    y = []
    for xi,yi in trayectoria:
        x.append(xi)
        y.append(yi)
    plt.plot(x,y, marker='o', linestyle='-')
    plt.grid()
    plt.show()

def graf_obs(obs1, obs2, obs3, ball, radio_jugador, radio_pelota):
    angulo = np.linspace(0, 2 * np.pi, 100)

    plt.scatter(obs1[0], obs1[1], color = "black")
    x_cir = obs1[0] + radio_jugador * np.cos(angulo)
    y_cir = obs1[1] + radio_jugador * np.sin(angulo)
    plt.plot(x_cir, y_cir)
    plt.scatter(obs2[0], obs2[1], color = "black")
    x_cir = obs2[0] + radio_jugador * np.cos(angulo)
    y_cir = obs2[1] + radio_jugador * np.sin(angulo)
    plt.plot(x_cir, y_cir)
    plt.scatter(obs3[0], obs3[1], color = "black")
    x_cir = obs3[0] + radio_jugador * np.cos(angulo)
    y_cir = obs3[1] + radio_jugador * np.sin(angulo)
    plt.plot(x_cir, y_cir)
    plt.scatter(ball[0], ball[1], color = "orange")
    x_cir = ball[0] + radio_pelota * np.cos(angulo)
    y_cir = ball[1] + radio_pelota * np.sin(angulo)
    plt.plot(x_cir, y_cir)


if __name__ == '__main__':
    main()
