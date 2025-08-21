"""Test para arbol de decisiones."""

import logging
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox
import numpy as np

from robot_soccer.entities.player import Player
from robot_soccer.entities.ball import Ball
from robot_soccer.ai.fuzzy_logic.game_context import FuzzyRobotTeamManager
from robot_soccer.ai.behavior_tree.manager import BehaviorManager
from robot_soccer.config import ANCHO_CAMPO, ALTO_CAMPO

# Configuración global del logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(levelname)-8s - %(filename)-15s - %(message)s'
)

# Control de niveles por módulo
logging.getLogger('robot_soccer.core').setLevel(logging.INFO)
logging.getLogger('robot_soccer.core.process').setLevel(logging.DEBUG)
logging.getLogger('robot_soccer.core.physics').setLevel(logging.WARNING)
logging.getLogger('robot_soccer.perception').setLevel(logging.INFO)
logging.getLogger('robot_soccer.entities').setLevel(logging.ERROR)
logging.getLogger('robot_soccer.ai').setLevel(logging.DEBUG)
logging.getLogger('robot_soccer.ai.path_planning').setLevel(logging.WARNING)
logging.getLogger('robot_soccer.utils').setLevel(logging.INFO)

# Librerías externas
logging.getLogger('pygame').setLevel(logging.WARNING)
logging.getLogger('opencv').setLevel(logging.ERROR)
logging.getLogger('numpy').setLevel(logging.ERROR)

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
ball = Ball(750, 450)
player_1 = Player(1, 200, 200, 90, 'red')
player_2 = Player(2, 200, 700, 180, 'red')
player_3 = Player(3, 1200, 200, 270,  'blue')
player_4 = Player(4, 1200, 700, 0, 'blue')


game_context_team_red = FuzzyRobotTeamManager(
    [player_1, player_2, player_3, player_4], ball, team='red'
)
game_context_team_blue = FuzzyRobotTeamManager(
    [player_1, player_2, player_3, player_4], ball, team='blue'
)

# Inicializar los gestores de comportamiento
behavior_manager_red = BehaviorManager(
    [player_1, player_2, player_3, player_4], ball, team='red'
)
behavior_manager_blue = BehaviorManager(
    [player_1, player_2, player_3, player_4], ball, team='blue'
)

# Figuras y ejes
fig, ax = plt.subplots(figsize=(10, 6))
plt.subplots_adjust(bottom=0.2)  # Espacio para los TextBox
ax.set_xlim(0, ANCHO_CAMPO)
ax.set_ylim(0, ALTO_CAMPO)
ax.set_aspect('equal')
ax.set_title("Tablero interactivo - Mueve los jugadores y la pelota")
ax.axvline(
    ANCHO_CAMPO * 0.3, color='gray', linestyle='--', label='Línea neutral izquierda'
)
ax.axvline(
    ANCHO_CAMPO * 0.7, color='gray', linestyle='--', label='Línea neutral derecha'
)
ax.grid(True)

# Dibujar el campo
ax.plot(
    [0, 0, ANCHO_CAMPO, ANCHO_CAMPO, 0], [0, ALTO_CAMPO, ALTO_CAMPO, 0, 0], 'k-', lw=2
)

# Elementos gráficos
circulos_jugadores = {}
texto_jugadores = {}
pelota, = ax.plot(
    posicion_pelota[0], posicion_pelota[1], 'go', markersize=10, label='Pelota'
)
for ide, (x, y, _) in posiciones_jugadores.items():
    if ide <= 2:
        circulos_jugadores[ide], = ax.plot(
            x, y, 'ro', markersize=12, label=f'Jugador {ide}'
        )
        texto_jugadores[ide] = ax.text(
            x + 20, y + 20, f'{ide}', color='red', fontsize=10
        )
    else:
        circulos_jugadores[ide], = ax.plot(
            x, y, 'bo', markersize=12, label=f'Jugador {ide}'
        )
        texto_jugadores[ide] = ax.text(
            x + 20, y + 20, f'{ide}', color='blue', fontsize=10
        )

# Widgets
SELECTED_ID = None
textboxes = {}

def actualizar_estado():
    """
    Actualiza las posiciones gráficas de los jugadores y la pelota.
    """
    # Actualiza jugadores
    for player_id, (x, y, _) in posiciones_jugadores.items():
        circulos_jugadores[player_id].set_data([x], [y])  # Pasar listas o tuplas
        texto_jugadores[player_id].set_position((x + 20, y + 20))

    # Actualiza pelota
    pelota.set_data([posicion_pelota[0]], [posicion_pelota[1]])  # Pasar listas o tuplas
    ax.axvline(
        ANCHO_CAMPO * 0.3, color='gray', linestyle='--', label='Línea neutral izquierda'
    )
    ax.axvline(
        ANCHO_CAMPO * 0.7, color='gray', linestyle='--', label='Línea neutral derecha'
    )
    plt.draw()

def mover_elemento(event):
    """
    Al hacer clic, selecciona un jugador o la pelota para moverla.
    """
    global SELECTED_ID
    if event.xdata is None or event.ydata is None:
        return

    # Encuentra el elemento más cercano
    min_dist = float('inf')
    seleccionado = None
    for player_id, (x, y, _) in posiciones_jugadores.items():
        dist = np.sqrt((event.xdata - x) ** 2 + (event.ydata - y) ** 2)
        if dist < min_dist:
            min_dist = dist
            seleccionado = player_id

    # Calcula distancia a la pelota
    dist_pelota = np.sqrt(
        (event.xdata - posicion_pelota[0]) ** 2 +
        (event.ydata - posicion_pelota[1]) ** 2
    )
    if dist_pelota < min_dist:
        min_dist = dist_pelota
        seleccionado = 'Pelota'

    # Si está cerca de un jugador o la pelota, abre un cuadro de diálogo
    if min_dist < 50:
        SELECTED_ID = seleccionado
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
    if SELECTED_ID == 'Pelota':
        xp, yp = posicion_pelota
        inputs = {'X': xp, 'Y': yp}
    else:
        xp, yp, angulo = posiciones_jugadores[SELECTED_ID]
        inputs = {'X': xp, 'Y': yp, 'Ángulo': angulo}

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
    global SELECTED_ID

    logging.debug(" ================== INTENTO NUEVO ================== \n")


    try:
        xi = float(textboxes['X'].text)
        yi = float(textboxes['Y'].text)
        if SELECTED_ID == 'Pelota':
            posicion_pelota[0] = max(0, min(ANCHO_CAMPO, xi))
            posicion_pelota[1] = max(0, min(ALTO_CAMPO, yi))
        else:
            angulo = float(textboxes['Ángulo'].text)
            posiciones_jugadores[SELECTED_ID] = [
                max(0, min(ANCHO_CAMPO, xi)),
                max(0, min(ALTO_CAMPO, yi)),
                angulo
            ]
    except ValueError:
        logging.error("Entrada inválida. Por favor, introduce números válidos.")

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
    logging.info("\n pos_player_1: (%d, %d) \t pos_player_2: (%d, %d) \t "
                "pos_player_3: (%d, %d) \t pos_player_4: (%d, %d) \n"
                " angulo_plasyer_1: %d \t\t angulo_plasyer_2: %d \t\t"
                "angulo_plasyer_3: %d \t\t angulo_plasyer_4: %d \n"
                " posicion de la pelota: (%d, %d)",
                player_1.x, player_1.y,
                player_2.x, player_2.y,
                player_3.x, player_3.y,
                player_4.x, player_4.y,
                player_1.angle, player_2.angle, player_3.angle, player_4.angle,
                ball.x, ball.y)
    # Control del sitema difuso se envia el objeto instanciado
    # _, infor_robots = tools.position_ball(players, ball)
    red_context = game_context_team_red.evaluar_ms_logic_difusse()
    blue_context = game_context_team_blue.evaluar_ms_logic_difusse()

    # Determina rol de jugadores
    r_posesion, r_proximidad, r_zona = game_context_team_red.evaluar_ms_logic_difusse()
    b_posesion, b_proximidad, b_zona = game_context_team_blue.evaluar_ms_logic_difusse()

    # 2. Actualizar contexto en los gestores de comportamiento
    behavior_manager_red.update_game_context(red_context)
    behavior_manager_blue.update_game_context(blue_context)
    # 3. Ejecutar árboles de comportamiento
    behavior_manager_red.update()
    behavior_manager_blue.update()

    # state_manager_team_red.evaluar_admEstados(r_possesion, r_proximidad, r_zona)
    # state_manager_team_blue.evaluar_admEstados(b_possesion, b_proximidad, b_zona)

    # robot_controller.execute_team_strategy(estado_robot_ataque, estado_robot_defensa)
    # robot_controller.execute_team_strategy(estado_robot_ataque, estado_robot_defensa)

    logging.debug("\t\t\t\t %s TEAM RED %s \n" +
                 "\t\t\t\t\t\t posesion: %.2f  prxomidad: %.2f" +
                 "zona: %.2f  rol_player1: %d  rol_player2: %d",
                 "-" * 33, "-" * 33,
                 r_posesion, r_proximidad, r_zona, player_1.rol, player_2.rol,)
    logging.debug("\t\t\t\t %s TEAM BLUE %s \n" +
                 "\t\t\t\t\t\t posesion: %.2f  prxomidad: %.2f" +
                 "zona: %.2f  rol_player3: %d  rol_player4: %d",
                 "-" * 33, "-" * 33,
                 b_posesion, b_proximidad, b_zona, player_3.rol, player_4.rol)

    logging.debug(behavior_manager_red.get_current_state(1))
    logging.debug(behavior_manager_red.get_current_state(2))

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
