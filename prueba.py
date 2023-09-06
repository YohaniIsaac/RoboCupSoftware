from multiprocessing import Process, Queue, Event
import time
import keyboard
import random
import math
import matplotlib.pyplot as plt
import numpy as np



############################
#  CAMPOS DE POTENCIALES   #
############################

################################################################
# Función para calcular el campo de atracción hacia el objetivo
def campo_atraccion(inicio, meta, scale=1.0):
    return scale * (meta - inicio)

# Función para calcular el campo de repulsión del obstáculo
def campo_repulsion(inicio, obstaculos, radio_repulsion, scale=1.0):
    distancia = np.linalg.norm(inicio - obstaculos)
    if distancia < radio_repulsion:
        print("si")
        return scale * (inicio - obstaculos) / distancia

    else:
        return np.zeros_like(inicio)

def main():
    # Parámetros
    start_point = np.array([0, 25])  # Punto de inicio
    goal_point = np.array([500, 500])   # Punto objetivo
    obstacle_point = np.array([100, 100])  # Punto del obstáculo
    obstacle_point2 = np.array([200, 450])  # Punto del obstáculo
    radio_repulsion = 200         # Radio de repulsión
    plt.scatter(start_point[0],start_point[1], c='g')

    # Parámetros de escala para los campos de atracción y repulsión
    attraction_scale = 1.0
    repulsion_scale = 500

    # Número de iteraciones
    iteraciones = 100

    # Tasa de aprendizaje para actualizar la posición del punto de inicio
    learning_rate = 0.07

    # Bucle de optimización
    for _ in range(iteraciones):
        # Calcular los campos de atracción y repulsión
        atraccion = campo_atraccion(start_point, goal_point, attraction_scale)
        repulsion = campo_repulsion(start_point, obstacle_point, radio_repulsion, repulsion_scale)
        repulsion2 = campo_repulsion(start_point, obstacle_point2, radio_repulsion, repulsion_scale)
        #print("repulsionnnn" , repulsion)
        # Calcular la dirección resultante
        direction = atraccion + repulsion + repulsion2
        #print("resultante", direction)
        # Actualizar la posición del punto de inicio
        start_point = start_point + learning_rate * direction

        #print(start_point)
        plt.scatter(start_point[0], start_point[1], c='r')


    # Imprimir la posición final del punto de inicio
    print("Posición final del punto de inicio:", start_point)
    plt.scatter(goal_point[0],goal_point[1], c='b')
    plt.scatter(obstacle_point[0],obstacle_point[1], c='k')
    plt.scatter(obstacle_point2[0],obstacle_point2[1], c='k')
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()