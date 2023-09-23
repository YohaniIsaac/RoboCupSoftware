import random
from deap import base, creator, tools, algorithms

# Define el problema de planificación de rutas
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMin)

# Función para generar una ruta aleatoria
def generate_individual():
    return [random.randint(0, 100) for _ in range(10)]  # 10 puntos en la ruta

# Función para evaluar la aptitud de una ruta (aquí, simplemente la suma de distancias)
def evaluate(individual):
    return sum(individual),

# Configura la toolbox DEAP
toolbox = base.Toolbox()
toolbox.register("individual", tools.initIterate, creator.Individual, generate_individual)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)
toolbox.register("evaluate", evaluate)
toolbox.register("mate", tools.cxTwoPoint)
toolbox.register("mutate", tools.mutShuffleIndexes, indpb=0.1)
toolbox.register("select", tools.selTournament, tournsize=3)

if __name__ == "__main__":
    # Configura la población inicial
    population = toolbox.population(n=50)

    # Define estadísticas para realizar un seguimiento del progreso
    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register("min", min)
    stats.register("avg", lambda x: sum(x) / len(x))

    # Define un objeto de registro para guardar los resultados
    logbook = tools.Logbook()
    logbook.header = ["gen", "evals"] + stats.fields

    # Configura los parámetros del algoritmo genético
    cxpb, mutpb, ngen = 0.7, 0.2, 100

    # Ejecuta el algoritmo genético
    algorithms.eaSimple(population, toolbox, cxpb, mutpb, ngen, stats=stats, halloffame=None)

    # Imprime las estadísticas finales
    print("Mejor ruta encontrada:", population[0])
    print("Distancia mínima:", population[0].fitness.values[0])
