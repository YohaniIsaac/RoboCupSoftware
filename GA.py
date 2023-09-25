import random
import matplotlib.pyplot as plt
import numpy as np
from deap import base, creator, tools, algorithms

x_ancho = 1250
y_alto = 850
punto_inicial = [10.0 , 10.0]
punto_final = [600.0 , 400.0]

# distancia mínima entre puntos: 707.2481884034769


# Definir la función de aptitud (menor distancia)
def calcular_distancia(a, b):
    # Calcular la distancia Euclidiana entre dos puntos a y b
    return np.hypot(a[0] - b[0] , a[1] - b[1])


def funcion_aptitud(individuo):
    distancia_total = 0
    # Calcular la distancia total desde el punto inicial hasta el final pasando por los puntos intermedios
    distancia_total += calcular_distancia(punto_inicial , individuo[0][0])

    for i in range(len(individuo) - 1):
        distancia_total += calcular_distancia(individuo[i][0], individuo[i+1][0])
    distancia_total += calcular_distancia(individuo[-1][0], punto_final)
    return distancia_total,

# Definir la función para generar un valor de coordenada (x o y)
def generar_valor():
    return [random.uniform(0, 1250), random.uniform(0, 850)]  # Generar un vector de 2 dimensiones


# Definir una función de mutación personalizada para una lista anidada
def mutar_lista_anidada(individuo, mu, sigma, indpb):
    for nivel1 in individuo:
        for vector in nivel1:
            for i in range(len(vector)):
                if random.random() < indpb:
                    vector[i] += random.gauss(mu, sigma)
                    # Asegurarse de que los valores estén dentro de tus rangos requeridos si es necesario


# Define el tipo de problema (maximización o minimización)
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))

# Crear una clase para el tipo de individuo que es una lista de vectores de dimensión 2
creator.create("Individual", list, fitness=creator.FitnessMin, vector_size=2)

# Configura la toolbox DEAP
toolbox = base.Toolbox()
# Attribute generator
# toolbox.register("attr_x", random.uniform, 0, x_ancho)  # Rango de la primera coordenada (x)
# toolbox.register("attr_y", random.uniform, 0, y_alto)   # Rango de la segunda coordenada (y)

toolbox.register("vector", tools.initRepeat, list, generar_valor, n=1)  # Generar vectores de 2 dimensiones

# Structure initializers
toolbox.register("individuo", tools.initRepeat, creator.Individual, toolbox.vector, n=10)  # 10 vectores
toolbox.register("poblacion", tools.initRepeat, list, toolbox.individuo)


toolbox.register("evaluate", funcion_aptitud)
toolbox.register("mate", tools.cxTwoPoint)
toolbox.register("mutate", mutar_lista_anidada, mu=0, sigma=10, indpb=0.2)
toolbox.register("select", tools.selTournament, tournsize=3)


if __name__ == "__main__":
    # Configura la población inicial
    poblacion = toolbox.poblacion(n=100)  # Población inicial de 50 individuos

    #Evaluar a toda la poblacion
    fitnesses = list(map(toolbox.evaluate, poblacion))

    for ind, fit in zip(poblacion, fitnesses):
        ind.fitness.values = fit

    # CXPB es la probabilidad de cruzar a dos individuos
    # MUTPB es la probabilidad de mutar a un individuo
    CXPB, MUTPB = 0.5, 0.2

    # Extraer el fitness de nuestros individuos
    fits = [ind.fitness.values[0] for ind in poblacion]

    #Variable para guardar el numero de la geneeracion 
    g = 0

    # Comenzar evlocion 
    while max(fits) > 100 and g < 400:
        # Crea un nueva generacion
        g = g + 1
        print("----- generation %i  ---------" % g)

        # Seleccionar individuos para la nueva generacion
        offspring = toolbox.select(poblacion, len(poblacion))

        # Clonar individuos para la nueva generacion
        offspring = list(map(toolbox.clone , offspring))

        # Aplicar crossover y mutar a la nueva generacion
        for child1, child2 in zip(offspring[::2], offspring[1::2]):
            if random.random() < CXPB:
                toolbox.mate(child1, child2)
                del child1.fitness.values
                del child2.fitness.values

        for mutant in offspring:
            if random.random() < MUTPB:
                toolbox.mutate(mutant)
                del mutant.fitness.values

        # Evaluar alos individuos con un fitness invalido
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit
        poblacion[:] = offspring

        # Crear lista para imprimir
        fits = [ind.fitness.values[0] for ind in poblacion]

        length = len(poblacion)
        mean = sum(fits) / length
        sum2 = sum(x*x for x in fits)
        std = abs(sum2 / length - mean**2)**0.5

        print("  Min  %s" % min(fits))
        print("  Max  %s" % max(fits))
        print("  Avg  %s" % mean)
        print("  Std  %s" % std)
        print("  Indv  %s" % poblacion[:1])
        puntos = poblacion[:1]

    # Extraer las coordenadas x e y de la estructura anidada
    coordenadas_x = []
    coordenadas_y = []
    coordenadas_x.append(punto_inicial[0])
    coordenadas_y.append(punto_inicial[1])

    for nivel1 in puntos:
        for nivel2 in nivel1:
            for punto in nivel2:
                x, y = punto
                coordenadas_x.append(x)
                coordenadas_y.append(y)

    coordenadas_x.append(punto_final[0])
    coordenadas_y.append(punto_final[1])

    # Crear el gráfico de dispersión (scatter plot)
    plt.plot(coordenadas_x, coordenadas_y, marker='^', linestyle='-', markersize=5, label='Línea')

    plt.xlabel('Coordenada X')
    plt.ylabel('Coordenada Y')
    plt.title('Gráfico de Puntos')
    plt.legend()
    plt.scatter(punto_inicial[0], punto_inicial[1], marker = 's', color='r')
    plt.scatter(punto_final[0], punto_final[1], marker = 'x', color='k')
    plt.grid(True)

    # Mostrar el gráfico
    plt.show()








