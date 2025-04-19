import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox
import numpy as np
import paquetes.tools as tools
from paquetes.MS_logicDifusse import FuzzyRobotTeamManager
# from MS import RobotStateMachine
# team_red = FuzzyRobotTeamManager(1, 2, 'red')
# team_blue = FuzzyRobotTeamManager(3, 4, 'blue')
from config import *


# Posiciones iniciales de jugadores y pelota
posiciones_jugadores = {
    1: [200, 200, 0],   # [x, y, ángulo]
    2: [200, 700, 0],  # [x, y, ángulo]
    3: [1200, 200, 0],   # [x, y, ángulo]
    4: [1200, 700, 0],   # [x, y, ángulo]
}
posicion_pelota = [750, 450]  # Coordenadas de la pelota [x, y]
# players = {
#     1: tools.Player(1, 200, 200, 90),
#     2: tools.Player(2, 200, 700, 180),
#     3: tools.Player(3, 1200, 200, 270),
#     4: tools.Player(4, 1200, 700, 0),
# }
player_1 = tools.Player(1, 200, 200, 90, 'red')
player_2 = tools.Player(2, 200, 700, 180, 'red')
player_3 = tools.Player(3, 1200, 200, 270, 'blue')
player_4 = tools.Player(4, 1200, 700, 0, 'blue')

ball = tools.Ball(750, 450)

team_red = FuzzyRobotTeamManager(player_1, player_2, player_3, player_4, ball)
team_blue = FuzzyRobotTeamManager(player_1, player_2, player_3, player_4, ball, team='blue')


# Figuras y ejes
fig, ax = plt.subplots(figsize=(10, 6))
plt.subplots_adjust(bottom=0.2)  # Espacio para los TextBox
ax.set_xlim(0, ANCHO_CAMPO)
ax.set_ylim(0, ALTO_CAMPO)
ax.set_aspect('equal')
ax.set_title("Tablero interactivo - Mueve los jugadores y la pelota")
ax.axvline(ANCHO_CAMPO * 0.3, color='gray', linestyle='--', label='Línea neutral izquierda')
ax.axvline(ANCHO_CAMPO * 0.7, color='gray', linestyle='--', label='Línea neutral derecha')
ax.grid(True)
# Dibujar el campo
ax.plot([0, 0, ANCHO_CAMPO, ANCHO_CAMPO, 0], [0, ALTO_CAMPO, ALTO_CAMPO, 0, 0], 'k-', lw=2)

# Elementos gráficos
circulos_jugadores = {}
texto_jugadores = {}
pelota, = ax.plot(posicion_pelota[0], posicion_pelota[1], 'go', markersize=10, label='Pelota')
for id, (x, y, _) in posiciones_jugadores.items():
    if id <= 2:
        circulos_jugadores[id], = ax.plot(x, y, 'ro', markersize=12, label=f'Jugador {id}')
        texto_jugadores[id] = ax.text(x + 20, y + 20, f'{id}', color='red', fontsize=10)
    else:
        circulos_jugadores[id], = ax.plot(x, y, 'bo', markersize=12, label=f'Jugador {id}')
        texto_jugadores[id] = ax.text(x + 20, y + 20, f'{id}', color='blue', fontsize=10)

# Widgets
selected_id = None
textboxes = {}

def actualizar_estado():
    """
    Actualiza las posiciones gráficas de los jugadores y la pelota.
    """
    # Actualiza jugadores
    for id, (x, y, _) in posiciones_jugadores.items():
        circulos_jugadores[id].set_data([x], [y])  # Corregido: pasar listas o tuplas
        texto_jugadores[id].set_position((x + 20, y + 20))

    # Actualiza pelota
    pelota.set_data([posicion_pelota[0]], [posicion_pelota[1]])  # Corregido: pasar listas o tuplas
    ax.axvline(ANCHO_CAMPO * 0.3, color='gray', linestyle='--', label='Línea neutral izquierda')
    ax.axvline(ANCHO_CAMPO * 0.7, color='gray', linestyle='--', label='Línea neutral derecha')
    plt.draw()

def mover_elemento(event):
    """
    Al hacer clic, selecciona un jugador o la pelota para moverla.
    """
    global selected_id
    if event.xdata is None or event.ydata is None:
        return

    # Encuentra el elemento más cercano
    min_dist = float('inf')
    seleccionado = None
    for id, (x, y, _) in posiciones_jugadores.items():
        dist = np.sqrt((event.xdata - x) ** 2 + (event.ydata - y) ** 2)
        if dist < min_dist:
            min_dist = dist
            seleccionado = id

    # Calcula distancia a la pelota
    dist_pelota = np.sqrt((event.xdata - posicion_pelota[0]) ** 2 + (event.ydata - posicion_pelota[1]) ** 2)
    if dist_pelota < min_dist:
        min_dist = dist_pelota
        seleccionado = 'Pelota'

    # Si está cerca de un jugador o la pelota, abre un cuadro de diálogo
    if min_dist < 50:
        selected_id = seleccionado
        abrir_cuadro_dialogo()

def abrir_cuadro_dialogo():
    """
    Abre cuadros de texto para modificar coordenadas y ángulo.
    """
    global textboxes
    for box in textboxes.values():
        box.ax.remove()
    textboxes.clear()

    # Coordenadas actuales
    if selected_id == 'Pelota':
        x, y = posicion_pelota
        inputs = {'X': x, 'Y': y}
    else:
        x, y, angulo = posiciones_jugadores[selected_id]
        inputs = {'X': x, 'Y': y, 'Ángulo': angulo}

    # Crear cuadros de texto
    for i, (label, value) in enumerate(inputs.items()):
        axbox = plt.axes([0.2, 0.05 + i * 0.05, 0.1, 0.04])
        textbox = TextBox(axbox, f'{label}:', initial=f'{value:.2f}')
        textboxes[label] = textbox

    # Botón para guardar
    axboton = plt.axes([0.5, 0.05, 0.1, 0.04])
    boton_guardar = TextBox(axboton, "Guardar")
    boton_guardar.on_submit(guardar_cambios)

    plt.draw()

def guardar_cambios(_=None):
    """
    Guarda las nuevas coordenadas o ángulo introducidos por el usuario.
    """
    global selected_id

    try:
        x = float(textboxes['X'].text)
        y = float(textboxes['Y'].text)
        if selected_id == 'Pelota':
            posicion_pelota[0] = max(0, min(ANCHO_CAMPO, x))
            posicion_pelota[1] = max(0, min(ALTO_CAMPO, y))
        else:
            angulo = float(textboxes['Ángulo'].text)
            posiciones_jugadores[selected_id] = [
                max(0, min(ANCHO_CAMPO, x)),
                max(0, min(ALTO_CAMPO, y)),
                angulo
            ]
    except ValueError:
        print("Entrada inválida. Por favor, introduce números válidos.")

    for player_id, (x, y, angle) in posiciones_jugadores.items():
        if player_id == player_1.id:
            player_1.set_position(x, y)
            player_1.set_angle(angle)
        elif player_id == player_2.id:
            player_2.set_position(x, y)
            player_2.set_angle(angle)
        if player_id == player_3.id:
            player_3.set_position(x, y)
            player_3.set_angle(angle)
        if player_id == player_4.id:
            player_4.set_position(x, y)
            player_4.set_angle(angle)

    ball.set_position(posicion_pelota[0], posicion_pelota[1])

    # Control del sitema difuso se envia el objeto instanciado
    # _, infor_robots = tools.position_ball(players, ball)

    team_red_info = team_red.evaluar_msLogicDifusse()
    team_blue_info = team_blue.evaluar_msLogicDifusse()
    # print("----------- TEAM RED -----------")
    # for clave, valor in team_red_info.items():
    #     print(f"{clave}: {valor}")
    # print("----------- TEAM BLUE -----------")
    # for clave, valor in team_blue_info.items():
    #     print(f"{clave}: {valor}")

    actualizar_estado()




def manejar_tecla(event):
    """
    Maneja la pulsación de teclas. Guarda los cambios si se presiona Enter.
    """
    if event.key == 'enter':  # Verifica si se presionó Enter
        guardar_cambios()

# Conecta el evento de teclado al canvas de Matplotlib
fig.canvas.mpl_connect('key_press_event', manejar_tecla)


# Conectar eventos
fig.canvas.mpl_connect('button_press_event', mover_elemento)

plt.show()
