import random
from deap import base, creator, tools, algorithms

# Define el tipo de problema (maximización o minimización)
creator.create("FitnessMax", base.Fitness, weights=(1.0,))

# Define la estructura del individuo (en este caso, una cadena de bits)
creator.create("Individual", list, fitness=creator.FitnessMax)

# Configura la toolbox DEAP
toolbox = base.Toolbox()
# Attribute generator 
toolbox.register("attr_bool", random.randint, 0, 1)
# Structure initializers
toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_bool, 100)

toolbox.register("population", tools.initRepeat, list, toolbox.individual)


def evalOneMax(individual):
    return sum(individual),


toolbox.register("evaluate", evalOneMax)
toolbox.register("mate", tools.cxTwoPoint)
toolbox.register("mutate", tools.mutFlipBit, indpb=0.05)  # Probabilidad de mutación de un bit
toolbox.register("select", tools.selTournament, tournsize=3)


if __name__ == "__main__":
    # Configura la población inicial
    population = toolbox.population(n=300)  # Población inicial de 50 individuos

    #Evaluar a toda la poblacion
    fitnesses = list(map(toolbox.evaluate, population))

    for ind, fit in zip(population, fitnesses):
        ind.fitness.values = fit

    # CXPB es la probabilidad de cruzar a dos individuos
    # MUTPB es la probabilidad de mutar a un individuo
    CXPB, MUTPB = 0.5, 0.2

    # Extraer el fitness de nuestros individuos
    fits = [ind.fitness.values[0] for ind in population]

    #Variable para guardar la geneeracion 

    g = 0

    # Comenzar evlocion 
    while max(fits) < 100 and g < 1000:
        # Crea un nueva generacion
        g = g + 1
        print("----- generation %i  ---------" % g)

        # Seleccionar individuos para la nueva generacion
        offspring = toolbox.select(population, len(population))

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
        population[:] = offspring

        # Crear lista para imprimir
        fits = [ind.fitness.values[0] for ind in population]

        length = len(population)
        mean = sum(fits) / length
        sum2 = sum(x*x for x in fits)
        std = abs(sum2 / length - mean**2)**0.5

        print("  Min  %s" % min(fits))
        print("  Max  %s" % max(fits))
        print("  Avg  %s" % mean)
        print("  Std  %s" % std)
        print("  Indv  %s" % population[:1])
