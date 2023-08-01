import math


def generador_gcode(x_final, y_final, tipo):

    gcode = []

    if tipo == "lineal":
        # G01
        gocde.append(("G01", x_final, y_final))


    elif tipo == "horario":
        # G02


    elif tipo == "antihorario":
        # G03

def interprete_gcode(gcode):
    for i in gcode:
        tipo = i[0]
        x_destino = i[1]
        y_destino = i[2]

        if tipo == "G01":
            if x_destino > self.x: 
                self.dx += 0.1
                self.dx = min(self.dx, 2) 
            elif x_destino < self.x: 
                self.dx -= 0.1
                self.dx = max(self.dx, -2)
            else:
                self.dx = 0


            if y_destino > self.y: 
                self.dy += 0.1
                self.dy = min(self.dy, 2) 
            elif y_destino < self.y: 
                self.dy -= 0.1
                self.dy = max(self.dy, -2)
            else:
                self.dy = 0







##########################################################################

def interpretar_codigo_g(codigo_g):
    # Definir posición inicial
    x_actual, y_actual = 0, 0

    # Lista para almacenar las trayectorias
    trayectorias = []

    for comando in codigo_g:
        tipo = comando[0]
        valores = comando[1:]

        if tipo == "G00":  # Movimiento rápido (lineal)
            x_destino, y_destino = valores
            trayectorias.append((x_actual, y_actual, x_destino, y_destino))
            x_actual, y_actual = x_destino, y_destino

        elif tipo == "G01":  # Movimiento lineal
            x_destino, y_destino = valores
            distancia = math.sqrt((x_destino - x_actual)**2 + (y_destino - y_actual)**2)
            pasos = int(distancia / 0.1)  # Intervalo de 0.1 para ejemplo
            for i in range(pasos):
                t = (i + 1) / pasos
                x_interpolado = x_actual + t * (x_destino - x_actual)
                y_interpolado = y_actual + t * (y_destino - y_actual)
                trayectorias.append((x_interpolado, y_interpolado))
            x_actual, y_actual = x_destino, y_destino

        elif tipo == "G02":  # Movimiento circular en sentido horario (arco)
            x_centro, y_centro, x_final, y_final = valores
            # Cálculo del arco y generación de trayectorias (implementación opcional)
            # ...

        elif tipo == "G03":  # Movimiento circular en sentido antihorario (arco)
            x_centro, y_centro, x_final, y_final = valores
            # Cálculo del arco y generación de trayectorias (implementación opcional)
            # ...

    return trayectorias

# Ejemplo de uso
codigo_g_ejemplo = [("G00", 10, 10), ("G01", 20, 30), ("G00", 0, 0)]
print(codigo_g_ejemplo)
trayectorias_generadas = interpretar_codigo_g(codigo_g_ejemplo)
print(trayectorias_generadas)




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


# a = linear_trajectory((100,100),(350,10),5)

# print(a)


#################################################################################

def generar_codigo_g(x_inicial, y_inicial, x_final, y_final, tipo_movimiento="lineal", x_centro=None, y_centro=None):
    codigo_g = []

    if tipo_movimiento == "lineal":
        # Movimiento lineal (G01)
        codigo_g.append(("G01", x_final, y_final))

    elif tipo_movimiento == "horario":
        # Movimiento circular en sentido horario (G02)
        if x_centro is None or y_centro is None:
            raise ValueError("Faltan coordenadas del centro para el movimiento circular horario.")
        
        radio = math.sqrt((x_centro - x_inicial) ** 2 + (y_centro - y_inicial) ** 2)
        codigo_g.append(("G02", x_centro, y_centro, x_final, y_final, radio))

    elif tipo_movimiento == "antihorario":
        # Movimiento circular en sentido antihorario (G03)
        if x_centro is None or y_centro is None:
            raise ValueError("Faltan coordenadas del centro para el movimiento circular antihorario.")
        
        radio = math.sqrt((x_centro - x_inicial) ** 2 + (y_centro - y_inicial) ** 2)
        codigo_g.append(("G03", x_centro, y_centro, x_final, y_final, radio))

    return codigo_g

# Ejemplo de uso:
codigo_g_lineal = generar_codigo_g(0, 0, 20, 30, tipo_movimiento="lineal")
codigo_g_horario = generar_codigo_g(0, 0, 20, 30, tipo_movimiento="horario", x_centro=10, y_centro=10)
codigo_g_antihorario = generar_codigo_g(0, 0, 20, 30, tipo_movimiento="antihorario", x_centro=10, y_centro=10)

print(codigo_g_lineal)
print(codigo_g_horario)
print(codigo_g_antihorario)
