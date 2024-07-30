from multiprocessing import Process, Queue, Event
import time
import keyboard
import random
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation 
import numpy as np


############################
#  ALGORITMO DE 90 GRADOS  #
############################
class Ruta:
    def __init__(self):
        # Valores de incio
        self.x = 1
        self.y = 2

        # Valores del punto de meta
        self.x_goal = 17
        self.y_goal = 11

        # Valores del obstáculo
        self.x_obs = 5
        self.y_obs = 4

        self.x_obs2 = 14
        self.y_obs2 = 8

        self.aumentar_obs1 = True
        self.aumentar_obs2 = True
        

        self.fig, self.ax = plt.subplots()
        self.ani = FuncAnimation(self.fig, self.update,    frames=range(100), interval=100)  # 200 frames, 100 ms entre cada frame    

    def trayectoria_final(self):
        '''
        En base a los puntos obtenidos, calcula las diferentes trayectorias posibles.

        Return:
        Todas las trayectorias, con los puntos ordenados

        '''
        trayectoria_0 = [np.array([self.x, self.y])]
        trayectoria_1 = [np.array([self.x, self.y])]
        trayectoria_2 = [np.array([self.x, self.y])]
        trayectoria_3 = [np.array([self.x, self.y])]
        trayectoria_4 = [np.array([self.x, self.y])]
        trayectoria_5 = [np.array([self.x, self.y])]
        trayectoria_6 = [np.array([self.x, self.y])]

        sub_goal_1, sub_goal_2 = self.trayectoria((self.x,self.y),(self.x_goal,self.y_goal),self.obstaculos[0])

        if sub_goal_1 is not None:
            # Grafica los puntos obtenidos
            plt.scatter(sub_goal_1[0], sub_goal_1[1], c='m')
            trayectoria_1.append(sub_goal_1)
            trayectoria_2.append(sub_goal_1)

            # Encuentra otros nuevos puntos en base al siguiente obstaculo
            sub_goal_3, sub_goal_4 = self.trayectoria(sub_goal_1,(self.x_goal,self.y_goal),self.obstaculos[1])

            # Encuentra otros nuevos puntos en base al siguiente obstaculo
            if sub_goal_3 is not None:
                plt.scatter(sub_goal_3[0], sub_goal_3[1], c='m')
                trayectoria_1.append(sub_goal_3)
                trayectoria_1.append(np.array([self.x_goal,self.y_goal]))
            else:
                trayectoria_1.append(np.array([self.x_goal,self.y_goal]))

            if sub_goal_4 is not None:
                plt.scatter(sub_goal_4[0], sub_goal_4[1], c='m')
                trayectoria_2.append(sub_goal_4)
                trayectoria_2.append(np.array([self.x_goal,self.y_goal]))
            else:
                trayectoria_2.append(np.array([self.x_goal,self.y_goal]))


        if sub_goal_2 is not None:
            plt.scatter(sub_goal_2[0], sub_goal_2[1], c='m')
            trayectoria_3.append(sub_goal_2)
            trayectoria_4.append(sub_goal_2)

            sub_goal_5, sub_goal_6 = self.trayectoria(sub_goal_2,(self.x_goal,self.y_goal),self.obstaculos[1])

            if sub_goal_5 is not None:
                plt.scatter(sub_goal_5[0], sub_goal_5[1], c='m')
                trayectoria_3.append(sub_goal_5)
                trayectoria_3.append(np.array([self.x_goal,self.y_goal]))
            else:
                trayectoria_3.append(np.array([self.x_goal,self.y_goal]))

            if sub_goal_6 is not None:
                plt.scatter(sub_goal_6[0], sub_goal_6[1], c='m')
                trayectoria_4.append(sub_goal_6)
                trayectoria_4.append(np.array([self.x_goal,self.y_goal]))
            else:
                trayectoria_4.append(np.array([self.x_goal,self.y_goal]))

        if sub_goal_1 is None and sub_goal_2 is None:
            sub_goal_1, sub_goal_2 = self.trayectoria((self.x,self.y),(self.x_goal,self.y_goal),self.obstaculos[1])
            
            if sub_goal_1 is not None:
                # Grafica los puntos obtenidos
                plt.scatter(sub_goal_1[0], sub_goal_1[1], c='m')
                trayectoria_5.append(sub_goal_1)
                trayectoria_5.append(np.array([self.x_goal,self.y_goal]))
            else: 
                trayectoria_5.append(np.array([self.x_goal,self.y_goal]))

            if sub_goal_2 is not None:
                plt.scatter(sub_goal_2[0], sub_goal_2[1], c='m')
                trayectoria_6.append(sub_goal_2)
                trayectoria_6.append(np.array([self.x_goal,self.y_goal]))
            else: 
                trayectoria_6.append(np.array([self.x_goal,self.y_goal]))
        # else:
        #     trayectoria_0.append(np.array([self.x_goal,self.y_goal]))
        if len(trayectoria_0) < 2:
            trayectoria_0 = None
        if len(trayectoria_1) < 2:
            trayectoria_1 = None
        if len(trayectoria_2) < 2:
            trayectoria_2 = None
        if len(trayectoria_3) < 2:
            trayectoria_3 = None
        if len(trayectoria_4) < 2:
            trayectoria_4 = None
        if len(trayectoria_5) < 2:
            trayectoria_5 = None
        if len(trayectoria_6) < 2:
            trayectoria_6 = None


        return trayectoria_0, trayectoria_1, trayectoria_2, trayectoria_3, trayectoria_4, trayectoria_5, trayectoria_6

    def distancia_puntos(self, puntos):
        '''
        Calcula la distancia que existe entre todos los puntos de la lista de puntos

        Args:
        puntos  -- (list) Lista de puntos ordenados (x,y)
        '''
        distancia_total = 0
        for i in range(len(puntos) - 1):
            a = puntos[i]
            b = puntos[i + 1]

            aux = np.hypot(a[0] - b[0] , a[1] - b[1])
            distancia_total = distancia_total + aux
        print(distancia_total)
        return distancia_total

    def update(self, frame):
        '''
        Modifca los valores de los obtáculos, permitiendo que éstos
        se muevan en el gráfco para entregar un dinamismo.
        Además recalcula las trayectorias y deja en negro la trayectoria mas corta.
        '''
        # Valores mínimo y máximo
        min_1 = 3
        max_1 = 8
        min_2 = 6
        max_2 = 10
        # Borra el gráfico anterior
        self.ax.clear()

        # Nuevos puntos
        if self.aumentar_obs1:
            self.y_obs += 0.1
        else:
            self.y_obs -= 0.1

        # Cambia la dirección cuando se alcanza el valor máximo o mínimo
        if self.y_obs >= max_1:
            self.aumentar_obs1 = False
        elif self.y_obs <= min_1:
            self.aumentar_obs1 = True

        # Nuevos puntos
        if self.aumentar_obs2:
            self.y_obs2 += 0.25
        else:
            self.y_obs2 -= 0.25

        # Cambia la dirección cuando se alcanza el valor máximo o mínimo
        if self.y_obs2 >= max_1:
            self.aumentar_obs2 = False
        elif self.y_obs2 <= min_1:
            self.aumentar_obs2 = True

        distancia_trayectorias = []

        #obstaculos2 = ordenar(x,y,obstaculos)
        self.obstaculos = np.array([ (self.x_obs, self.y_obs), (self.x_obs2, self.y_obs2) ])

        t_0, t_1, t_2, t_3, t_4, t_5, t_6 = self.trayectoria_final()

        # if t_0 is not None:
        #     x_0 , y_0 = zip(*t_0)
        #     d_0 = self.distancia_puntos(t_0)
        #     distancia_trayectorias.append((d_0, x_0, y_0))
        #     # plt.plot(x_0, y_0)
        if t_1 is not None:
            x_1 , y_1 = zip(*t_1)
            d_1 = self.distancia_puntos(t_1)
            distancia_trayectorias.append(np.array( [d_1, 1] ))
            plt.plot(x_1, y_1, )
        if t_2 is not None:
            x_2 , y_2 = zip(*t_2)
            d_2 = self.distancia_puntos(t_2)
            distancia_trayectorias.append((d_2, 2))
            plt.plot(x_2, y_2)
        if t_3 is not None:
            x_3 , y_3 = zip(*t_3)
            d_3 = self.distancia_puntos(t_3)
            distancia_trayectorias.append((d_3, 3))
            plt.plot(x_3, y_3)
        if t_4 is not None:
            x_4 , y_4 = zip(*t_4)
            d_4 = self.distancia_puntos(t_4)
            distancia_trayectorias.append((d_4, 4))
            plt.plot(x_4, y_4)
        if t_5 is not None:
            x_5 , y_5 = zip(*t_5)
            d_5 = self.distancia_puntos(t_5)
            distancia_trayectorias.append((d_5, 5))
            plt.plot(x_5, y_5)
        if t_6 is not None:
            x_6 , y_6 = zip(*t_6)
            d_6 = self.distancia_puntos(t_6)
            distancia_trayectorias.append((d_6, 6))
            plt.plot(x_6, y_6)

        valor_minimo = min(distancia_trayectorias, key=lambda x: x[0])
        if valor_minimo[1] == 1:
            plt.plot(x_1, y_1, c='k')
        elif valor_minimo[1] == 2:
            plt.plot(x_2, y_2, c= 'k')
        elif valor_minimo[1] == 3:
            plt.plot(x_3, y_3, c= 'k')
        elif valor_minimo[1] == 4:
            plt.plot(x_4, y_4, c= 'k')
        elif valor_minimo[1] == 5:
            plt.plot(x_5, y_5, c= 'k')
        elif valor_minimo[1] == 6:
            plt.plot(x_6, y_6, c= 'k')

       # Dibuja el grafico
        plt.scatter(self.x, self.y, c='g')
        plt.scatter(self.x_goal, self.y_goal,c='b')
        plt.scatter(self.x_obs, self.y_obs,c='k')
        plt.scatter(self.x_obs2, self.y_obs2,c='k')

        plt.grid(True)


        plt.title(f'Frame {frame}')

    def ordenar(self, x,y, lista):
        '''
        ordena los valores de la lista desde el que se encuentra mas cercano a x,y 
        hasta el mas lejano

        Args: 
        x       -- (int) coordenada x del objeto
        y       -- (int) coordenada y del objeto

        Return:
        new_list -- (lista) nueva lista ordenada y con un valor extra de la distancia al objeto
        '''

        new_list = []

        for i in lista:
            distancia = np.hypot(x - i[0], y - i[1])

            new_list.append((i[0],i[1], distancia))

        return sorted(new_list, key=lambda x: x[2])

    def trayectoria(self, inicio, goal, obstaculos, dist_safe= 3):
        '''
        Busca dos puntos nuevos tal que estos dos se encuentren a uns distancia segura
        del obstáculo, moviendo 90 grados hacia ambos sentidos.

        Args:
        inicio      -- (tuple) Coordenadas x,y del incio de la trayectoria
        goal        -- (tuple) Coordeandas x,y del final de la trayectoria
        obstaculos  -- (tuple) Coordenadas x,y del obstáculo 
        dist_safe   -- (int) El radio de seguridad para evitar el obstáculo

        Return:
        nuevo_punto_1   -- (tuple) Coordendas del nuevo punto para evitar el obstaculo 
        nuevo_punto_2   -- (tuple) Coordendas del nuevo punto para evitar el obstaculo
        '''

        x = inicio[0]
        y = inicio[1]

        x_goal = goal[0]
        y_goal = goal[1]

        x_obs = obstaculos[0]
        y_obs = obstaculos[1]

        m = (y_goal - y) / (x_goal - x)

        m_obs = -1/m

        A = np.array([[-m , 1],[-m_obs , 1]])
        B = np.array([-m*x_goal + y_goal, -m_obs*x_obs + y_obs])

        # punto de intersección entre las rectas
        inter = np.linalg.solve(A,B)
        #print(inter[0], inter[1])
        # plt.scatter(inter[0],inter[1], c='c')

        dist_obs_trayect = np.hypot(x_obs - inter[0] , y_obs - inter[1])

        if dist_obs_trayect <= dist_safe:
            punto_0 = np.array([x_obs, y_obs])
            punto_1 = np.array([inter[0], inter[1]])

            # Determina el sentido del vector
            vector_director = punto_1 - punto_0

            # Normaliza el vector director en unuitario
            nomal_vector = vector_director / np.linalg.norm(vector_director)

            # Calcula el nuevo punto
            nuevo_punto_1 = punto_0 + dist_safe * nomal_vector
            nuevo_punto_1 = self.verificar_punto(self.obstaculos, nuevo_punto_1)

            # Calcula el nuevo punto
            nuevo_punto_2 = punto_0 + dist_safe * (-nomal_vector)
            nuevo_punto_2 = self.verificar_punto(self.obstaculos, nuevo_punto_2)

            return nuevo_punto_1, nuevo_punto_2
        else:
            return None, None

    def verificar(self, obstaculos, puntos, dist_safe = 1 , mapa = (30,30)):
        '''
        Verifica que los puntos obtenidos se encuentran lejos del radio seguro, es decir, 
        que no hay obstáculos, cercanos ni las paredes para que el robot no choque,
        haciendo viable los puntos.


        '''

        puntos_validos = []

        for punto in puntos:
            if punto[0] < mapa[0] and punto[0] > 0 and punto[1] < mapa[1] and punto[1] > 0:
                dentro_mapa = True

            else:
                dentro_mapa = False

            for obs in obstaculos:
                distancia = np.hypot(punto[0] - obs[0], punto[1] - obs[1])
                if distancia <= dist_safe:
                    safe_for_obstaculos = False
                    break
                else:
                    safe_for_obstaculos = True

            if dentro_mapa and safe_for_obstaculos:
                puntos_validos.append(punto)
        return puntos_validos

    def verificar_punto(self, obstaculos, punto, dist_safe = 2, mapa = (20,20)):
        '''
        Verifica que el punto obtenido se encuentran lejos del radio seguro, es decir, 
        que no hay obstáculos cercanos, ni paredes para que el robot no choque,
        haciendo viable el punto.


        '''
        if punto[0] < mapa[0] and punto[0] > 0 and punto[1] < mapa[1] and punto[1] > 0:
            dentro_mapa = True

        else:
            dentro_mapa = False

        for obs in obstaculos:
            distancia = np.hypot(punto[0] - obs[0], punto[1] - obs[1])
            if distancia <= dist_safe:
                safe_for_obstaculos = False
                break
            else:
                safe_for_obstaculos = True

        if dentro_mapa and safe_for_obstaculos:
            return punto
        else: 
            return None

def main():
    # Instanciar una clase
    animacion = Ruta()

    # Muestra el gráfico
    plt.show()

if __name__ == "__main__":
    main()