import numpy as np
import matplotlib.pyplot as plt

# Implementación de Ramer-Douglas-Peucker


def graficar_rutas(ruta_original, ruta_suavizada):
    """
    Grafica las rutas original y suavizada.

    :param ruta_original: Lista de tuplas que representan la ruta original.
    :param ruta_suavizada: Lista de tuplas que representan la ruta suavizada.
    """
    # Extraer coordenadas de las rutas
    x_original, y_original = zip(*ruta_original)
    x_suavizada, y_suavizada = zip(*ruta_suavizada)

    # Crear la figura y los ejes
    plt.figure(figsize=(10, 6))

    # Graficar la ruta original
    plt.plot(x_original, y_original, marker='o', label='Ruta Original', color='blue', markersize=5)

    # Graficar la ruta suavizada
    plt.plot(x_suavizada, y_suavizada, marker='o', label='Ruta Suavizada', color='red', markersize=5)

    # Configuraciones de la gráfica
    plt.title('Comparación entre Ruta Original y Ruta Suavizada')
    plt.xlabel('Coordenada X')
    plt.ylabel('Coordenada Y')
    plt.legend()
    plt.grid()
    plt.axis('equal')  # Para mantener la relación de aspecto
    plt.show()

def perpendicular_distance(point, start, end):
    """Calcula la distancia perpendicular desde un punto a una línea."""
    if np.array_equal(start, end):
        return np.linalg.norm(point - start)

    # La línea se define por dos puntos
    line_vec = end - start
    line_mag = np.linalg.norm(line_vec)
    line_unit_vec = line_vec / line_mag
    point_vec = point - start
    t = np.dot(point_vec, line_unit_vec)

    # Proyección del punto sobre la línea
    if t < 0:
        nearest = start
    elif t > line_mag:
        nearest = end
    else:
        nearest = start + t * line_unit_vec

    return np.linalg.norm(point - nearest)


def douglas_peucker(points, epsilon):
    """Reduce los puntos en la curva utilizando el algoritmo de Ramer-Douglas-Peucker."""
    # Distancia máxima permitida
    dmax = 0.0
    index = 0
    end = len(points) - 1

    # Recurre a los puntos más distantes de la línea
    for i in range(1, end):
        d = perpendicular_distance(points[i], points[0], points[end])
        if d > dmax:
            index = i
            dmax = d

    # Si la distancia máxima es mayor que epsilon, se guarda el punto
    if dmax >= epsilon:
        # Recurre en dos segmentos
        rec_results1 = douglas_peucker(points[:index + 1], epsilon)
        rec_results2 = douglas_peucker(points[index:], epsilon)

        # Combina los resultados
        return rec_results1[:-1] + rec_results2
    else:
        return [points[0], points[end]]


# Ejemplo de uso
ruta = [
    (800, 600),
    (755.8724596406473, 562.1269677217381),
    (707.1818232220695, 573.4945499252614),
    (667.8741075331378, 604.3960626280096),
    (619.2515447974017, 592.7407465443761),
    (589.2886846834278, 579.2072476416264),
    (590.6884689224602, 529.2268454415224),
    (625.0993909365933, 492.9516687205573),
    (590.9844027917369, 456.3980425820152),
    (576.6236800203586, 408.5047273753643),
    (566.7716847834673, 359.4849539175823),
    (585.7735561256296, 313.23639842126533),
    (547.5891897480151, 280.95708361533895),
    (497.845360869571, 275.9022355946686),
    (460.6985955338555, 242.43407423057087),
    (413.3552905316128, 226.35267281574946),
    (387.84452896550636, 183.35033515467262),
    (346.5968454015942, 155.09031745100924),
    (299.6275666469492, 137.947188994731),
    (253.63094364181697, 157.55103229454693),
    (214.7670572090177, 126.09315043103803),
    (166.69798274899813, 112.33218823513421),
    (146.7884972439311, 66.46703384835027),
    (108.04796456927772, 34.85736557922545),
    (64.47581340553906, 59.38220284605164),
    (51.147949541699774, 11.191248873060328),
    (2, 2)
]

# Suavizar la ruta
epsilon = 50  # Define un umbral para la suavización
ruta_suavizada = douglas_peucker(np.array(ruta), epsilon)

print("Ruta suavizada:")
for punto in ruta_suavizada:
    print(punto)
graficar_rutas(ruta, ruta_suavizada)

