import math
import paquetes.rrt_star_smart as rrt


def calcular_posicion_objetivo(posicion_rival, radio=50):
    """
    Calcula una posición objetivo alrededor del robot rival para orbitarlo.

    Args:
        posicion_rival (list): Coordenadas [x, y] del robot rival.
        radio (float): Radio de la órbita alrededor del rival.

    Returns:
        list: Coordenadas [x, y] de la posición objetivo.
    """
    # Ángulo inicial (puedes variarlo para cambiar la posición de la órbita)
    angulo = math.radians(45)  # 45 grados, por ejemplo

    # Calcular la posición objetivo
    x_objetivo = posicion_rival[0] + radio * math.cos(angulo)
    y_objetivo = posicion_rival[1] + radio * math.sin(angulo)

    return [x_objetivo, y_objetivo]


def generar_trayectoria(self, accion):
    # Aquí llamas a tu función para generar trayectorias
    print(f"Generando trayectoria para la acción: {accion}")
    # Lógica para generar la trayectoria
    pass