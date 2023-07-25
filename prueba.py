def linear_trajectory(start, end, num_points):
    """
    Genera una trayectoria lineal entre dos puntos.

    Args:
        start (tuple): Coordenadas (x, y) del punto inicial.
        end (tuple): Coordenadas (x, y) del punto final.
        num_points (int): Número de puntos intermedios para la interpolación.

    Returns:
        list: Lista de códigos G que representan la trayectoria lineal.
    """
    trajectory = []
    x0, y0 = start
    x1, y1 = end

    for i in range(num_points + 1):
        x = x0 + (x1 - x0) * i / num_points
        y = y0 + (y1 - y0) * i / num_points
        g_code = f"G01 X{x:.2f} Y{y:.2f}"  # G01 es el código G para movimiento lineal
        trajectory.append(g_code)

    return trajectory


def quadratic_trajectory(start, control, end, num_points):
    """
    Genera una trayectoria cuadrática entre tres puntos.

    Args:
        start (tuple): Coordenadas (x, y) del punto inicial.
        control (tuple): Coordenadas (x, y) del punto de control.
        end (tuple): Coordenadas (x, y) del punto final.
        num_points (int): Número de puntos intermedios para la interpolación.

    Returns:
        list: Lista de códigos G que representan la trayectoria cuadrática.
    """
    trajectory = []
    x0, y0 = start
    cx, cy = control
    x1, y1 = end

    for i in range(num_points + 1):
        t = i / num_points
        x = (1 - t) ** 2 * x0 + 2 * (1 - t) * t * cx + t ** 2 * x1
        y = (1 - t) ** 2 * y0 + 2 * (1 - t) * t * cy + t ** 2 * y1
        g_code = f"G01 X{x:.2f} Y{y:.2f}"  # G01 es el código G para movimiento lineal
        trajectory.append(g_code)

    return trajectory


def generate_g_code(traj_list):
    """
    Genera el código G combinando varias trayectorias.

    Args:
        traj_list (list): Lista de trayectorias, cada una representada como una lista de códigos G.

    Returns:
        list: Lista con el código G resultante, que representa la unión de todas las trayectorias.
    """
    g_code = []
    for traj in traj_list:
        g_code.extend(traj)
    return g_code
