from multiprocessing import Process, Queue, Event
import time
import keyboard
import random
import math
import matplotlib.pyplot as plt
import numpy as np


############################
#  ALGORITMO DE 90 GRADOS  #
############################
# class RutaGrados:
#     def __init__(self, inicio, final, obs, ball):
#         (x,y) = inicio
#         self.x , self.y = incio
#         self.x_final, self.y_final = final
#         self.obstaculos = obs
#         self.completado = False

#     def trayectoria(self):
        

#     def distancia_obtaculos(self):
#         for i in self.obstaculos:
#             distancia = sqrt((x2 - x1)^2 + (y2 - y1)^2)

def trayectoria(x,y, x_goal, y_goal, obstaculos):

    sup = []
    inf = []

    for i in obstaculos:
        x_obs = i[0]
        y_obs = i[1]
        m = (y_goal - y) / (x_goal - x)

        m_obs = -1/m

        A = np.array([[-m , 1],[-m_obs , 1]])
        B = np.array([-m*x_goal + y_goal, -m_obs*x_obs + y_obs])

        # punto de intersección entre las rectas
        solucion = np.linalg.solve(A,B)
        print(solucion[0], solucion[1])
        # plt.scatter(solucion[0],solucion[1], c='c')


        dist_obs_trayect = np.hypot(x_obs- solucion[0] , y_obs-solucion[1])
        print("distancia trayectoria obstaculo", dist_obs_trayect)
        if dist_obs_trayect < 3:
            d = 5
            x_sub1 = x_obs + np.sqrt(d**2/(1+m_obs**2))
            x_sub2 = x_obs - np.sqrt(d**2/(1+m_obs**2))
            
            y_sub1 = m_obs*(x_sub1 - x_obs) + y_obs
            y_sub2 = m_obs*(x_sub2 - x_obs) + y_obs

            if y_sub1 > y_sub2:
                sup.apped([x_sub1,y_sub1])
                inf.apped([x_sub2,y_sub2])
            else:
                inf.append([x_sub1, y_sub1])
                sup.append([x_sub2, y_sub2])

            print("primero", x_sub1, y_sub1, "segundo", x_sub2, y_sub2)
            plt.scatter(x_sub1, y_sub1, c='m')
            plt.scatter(x_sub2, y_sub2, c='m')

            aux1 = np.hypot(x_sub1 - x_obs, y_sub1 - y_obs)
            aux2 = np.hypot(x_sub2 - x_obs, y_sub2 - y_obs)
            print("aux1", aux1, "   ", "aux2", aux2)
    print(sup[0])
    plt.plot([x,sup[0][0],sup[1][0],x_goal], [y,sup[0][1],sup[1][1], y_goal])
    plt.plot([x,inf[0][0],inf[1][0],x_goal], [y,inf[0][1],inf[1][1], y_goal])

def subgoal():
    

def main():


    # Define los colores de los puntos
    # r = rojo, b = azul, k = negro

    # Valores de incio
    x = 1
    y = 2
    plt.scatter(x,y, c='g')

    # Valores del punto de meta
    x_goal = 17
    y_goal = 11
    plt.scatter(x_goal,y_goal,c='b')

    # plt.plot([x,x_goal], [y,y_goal])

    # Valores del obstáculo
    x_obs = 5
    y_obs = 4
    plt.scatter(x_obs,y_obs,c='k')

    x_obs2 = 10
    y_obs2 = 8
    plt.scatter(x_obs2,y_obs2,c='k')

    x_obs3 = 14
    y_obs3 = 4
    plt.scatter(x_obs3,y_obs3,c='k')

    obstaculos = np.array([[x_obs,y_obs],[x_obs2,y_obs2], [x_obs3,y_obs3]])
    trayectoria(x,y,x_goal,y_goal,obstaculos)
    # trayectoria(1,2, 7,5, 6,3)

    # Configura los límites de los ejes
    # plt.xlim(-5, 10)
    # plt.ylim(-5, 10)
    plt.grid(True)

    # Muestra el gráfico
    plt.show()






if __name__ == "__main__":
    main()