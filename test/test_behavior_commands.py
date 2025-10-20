import time
import logging
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, FancyArrowPatch
from matplotlib.widgets import TextBox, Button, RadioButtons
from robot_soccer.entities.player import Player
from robot_soccer.entities.ball import Ball
from robot_soccer.ai.fuzzy_logic.game_context import FuzzyRobotTeamManager
from robot_soccer.ai.behavior_tree.manager import BehaviorManager
from robot_soccer.ai.behavior_tree.base import NodeStatus, get_global_tracer
from robot_soccer.config import (ROL_ATACANTE, ROL_DEFENSIVO, ANCHO_CAMPO, ALTO_CAMPO, LARGO_ARCO,
                                 ZONA_IZQUIERDA, ZONA_DERECHA)
from config_test import *
from tools_fuzzy import get_fuzzy_variables


# Clase para el formatter con colores
class ColoredFormatter(logging.Formatter):
    """Formatter que agrega colores a los logs"""

    # Códigos ANSI para colores
    RESET = '\033[0m'
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    MAGENTA = '\033[35m'
    CYAN = '\033[36m'

    def format(self, record):
        # Aplicar colores según el nivel
        if record.levelname == 'ERROR':
            color = self.RED
        elif record.levelname == 'WARNING':
            color = self.YELLOW
        elif record.levelname == 'INFO':
            color = self.BLUE
        elif record.levelname == 'DEBUG':
            color = self.GREEN
        else:
            color = self.RESET  # Sin color para otros niveles

        # Colorear toda la línea
        formatted_message = super().format(record)
        record.msg = f"{color}{formatted_message}{self.RESET}"

        # Sobrescribir el mensaje para que se aplique el color a toda la línea
        return record.msg

# Configuración global del logging (tu configuración original)
logging.basicConfig(
    level=logging.INFO,
    format="%(levelname)-6s - %(filename)-25s - %(message)s"
)
# Aplicar el formatter con colores a todos los handlers existentes
colored_formatter = ColoredFormatter("%(levelname)-6s - %(filename)-25s - %(message)s")
for handler in logging.root.handlers:
    handler.setFormatter(colored_formatter)

# Control de niveles por módulo
logging.getLogger("robot_soccer.ai.behavior_tree").setLevel(logging.WARNING)
logging.getLogger("robot_soccer.ai.fuzzy_logic").setLevel(logging.DEBUG)
logging.getLogger("robot_soccer.ai.path_planning").setLevel(logging.WARNING)
logging.getLogger("robot_soccer.ai.role_assignment").setLevel(logging.WARNING)

logging.getLogger("robot_soccer.core").setLevel(logging.WARNING)
logging.getLogger("robot_soccer.core.process").setLevel(logging.WARNING)
logging.getLogger("robot_soccer.core.physics").setLevel(logging.WARNING)

logging.getLogger("robot_soccer.perception").setLevel(logging.WARNING)

logging.getLogger("robot_soccer.entities").setLevel(logging.WARNING)


logging.getLogger("robot_soccer.controllers.robot_command_manager").setLevel(
    logging.DEBUG
)

logging.getLogger("robot_soccer.utils").setLevel(logging.INFO)

# Librerías externas
logging.getLogger("pygame").setLevel(logging.WARNING)
logging.getLogger("opencv").setLevel(logging.ERROR)
logging.getLogger("numpy").setLevel(logging.ERROR)

log = logging.getLogger(__name__)
log.setLevel(logging.DEBUG)
# Extender el tracer global existente con funcionalidades adicionales
class TracerExtension:
    """
    Extensión para el tracer global que añade funcionalidad para coordenadas
    """

    def __init__(self):
        self.planned_movements = {}  # Diccionario para almacenar movimientos planificados por robot
        self.action_details = {}  # Detalles adicionales de acciones por robot
        self.robot_actions = {}  # Acciones por robot

    def clear(self):
        self.planned_movements = {}
        self.action_details = {}
        self.robot_actions = {}

    def set_planned_movement(
        self, robot_id, target_pos, action_type="move", additional_info=None
    ):
        """Registra un movimiento planificado para un robot"""
        self.planned_movements[robot_id] = {
            "target_pos": target_pos,
            "action_type": action_type,
            "additional_info": additional_info or {},
        }

    def set_action_details(self, robot_id, action_name, details):
        """Registra detalles adicionales de una acción"""
        self.action_details[robot_id] = {"action": action_name, "details": details}

    def set_robot_action(self, robot_id, action_name):
        """Registra la acción actual de un robot específico"""
        self.robot_actions[robot_id] = action_name


# Crear extensión global
tracer_extension = TracerExtension()


class ImprovedBehaviorDebugger:
    """
    Herramienta mejorada para depurar árboles de comportamiento de robots de fútbol.
    Permite edición manual completa de posiciones y visualización clara de comportamientos.
    """

    def __init__(self):
        # Variables de inicialización
        self.positions = None
        self.ball = None
        self.player_1 = None
        self.player_2 = None
        self.player_3 = None
        self.player_4 = None
        self.players = None
        self.context_red = None
        self.context_blue = None
        self.behavior_red = None
        self.behavior_blue = None
        self.team_managers = None
        self.context_managers = None
        self.current_states = None
        self.fig = None
        self.gs = None
        self.field_ax = None
        self.actions_ax = None
        self.behavior_ax = None
        self.edit_ax = None
        self.details_ax = None
        self.params_ax = None
        self.ball_circle = None
        self.ball_label = None
        self.player_circles = None
        self.player_directions = None
        self.player_labels = None
        self.player_info_texts = None
        self.highlight_circle = None
        self.robot_selector = None
        self.analyze_button = None
        self.roles_button = None
        self.reset_button = None
        self.debug_level = None

        # Elementos para mostrar coordenadas de destino
        self.target_markers = {}  # Marcadores de destino en el campo
        self.movement_arrows = {}  # Flechas de movimiento planificado

        # Obtener el tracer global original y la extensión
        self.tracer = get_global_tracer()
        self.tracer_ext = tracer_extension

        # Posiciones iniciales
        self.setup_entities()

        # Estado de simulación
        self.selected_id = None
        self.focused_robot_id = 1
        self.execution_depth = 1
        self.last_behavior_update = time.time()

        # Estado para arrastrar objetos
        self.dragging = False
        self.drag_object = None

        # Widgets de edición
        self.position_widgets = {}  # Para X, Y de cada robot
        self.angle_widgets = {}  # Para ángulos
        self.ball_widgets = {}  # Para la pelota

        # Historial de acciones
        self.action_history = {i: [] for i in range(1, 5)}
        self.max_history_length = 5


        self.simulation_active = False
        self.physics_timer = None
        self.ball_physics = {
            'velocity_x': 0.0,
            'velocity_y': 0.0,
            'friction': 0.95,  # Factor de fricción (0.95 = pierde 5% velocidad por frame)
            'min_speed': 2.0   # Velocidad mínima antes de detenerse
        }

        # Estado de simulación
        self.simulation_active = False
        self.physics_timer = None
        self.ball_velocity_arrow = None  # Para mostrar vector de velocidad
        # Crear gestores de comportamiento
        self.setup_managers()

        # Configurar interfaz gráfica
        self.setup_ui()

    def setup_entities(self):
        """Inicializa jugadores y pelota"""
        self.positions = {
            "ball": [750, 450],
            1: [200, 200, 0],
            2: [200, 700, 0],
            3: [1200, 200, 0],
            4: [1200, 700, 0],
        }

        # Crear objetos
        self.ball = Ball(self.positions["ball"][0], self.positions["ball"][1])
        self.player_1 = Player(
            1, self.positions[1][0], self.positions[1][1], self.positions[1][2], "red"
        )
        self.player_2 = Player(
            2, self.positions[2][0], self.positions[2][1], self.positions[2][2], "red"
        )
        self.player_3 = Player(
            3, self.positions[3][0], self.positions[3][1], self.positions[3][2], "blue"
        )
        self.player_4 = Player(
            4, self.positions[4][0], self.positions[4][1], self.positions[4][2], "blue"
        )

        self.players = {
            1: self.player_1,
            2: self.player_2,
            3: self.player_3,
            4: self.player_4,
        }

        # Asignar roles iniciales
        self.player_1.set_rol(ROL_ATACANTE)
        self.player_2.set_rol(ROL_DEFENSIVO)
        self.player_3.set_rol(ROL_ATACANTE)
        self.player_4.set_rol(ROL_DEFENSIVO)

    def setup_managers(self):
        """Inicializa gestores de comportamiento"""
        all_players = [self.player_1, self.player_2, self.player_3, self.player_4]

        # Crear gestores de contexto
        self.context_red = FuzzyRobotTeamManager(all_players, self.ball, team="red")
        self.context_blue = FuzzyRobotTeamManager(all_players, self.ball, team="blue")

        # Crear gestores de comportamiento
        self.behavior_red = BehaviorManager(
            all_players, self.ball, team="red", use_real_robots=False
        )
        self.behavior_blue = BehaviorManager(
            all_players, self.ball, team="blue", use_real_robots=False
        )

        self.team_managers = {"red": self.behavior_red, "blue": self.behavior_blue}

        self.context_managers = {"red": self.context_red, "blue": self.context_blue}

        # Obtener estados iniciales
        self.current_states = {}
        for player_id in range(1, 5):
            team = "red" if player_id <= 2 else "blue"
            manager = self.team_managers[team]
            self.current_states[player_id] = manager.get_current_state(player_id)

    def setup_ui(self):
        """Configura la interfaz gráfica mejorada"""
        # Crear figura principal con mejor distribución
        self.fig = plt.figure(figsize=(16, 8))
        self.gs = self.fig.add_gridspec(
            4,
            3,
            width_ratios=[2, 1, 1],
            height_ratios=[2, 1, 1, 0.8],
            hspace=0.3,
            wspace=0.3,
        )

        # Panel del campo (más grande)
        self.field_ax = self.fig.add_subplot(self.gs[0:3, 0:2])
        self.setup_field_view()

        # Panel de acciones planificadas (nuevo)
        self.actions_ax = self.fig.add_subplot(self.gs[0, 2])
        self.actions_ax.set_title("Secuencia de Acciones Planificadas")
        self.actions_ax.axis("off")

        # Panel de comportamiento actual
        self.behavior_ax = self.fig.add_subplot(self.gs[3, 0:2])
        self.behavior_ax.set_title("Estado Actual")
        self.behavior_ax.axis("off")

        # Panel de edición de posiciones
        self.edit_ax = self.fig.add_subplot(self.gs[1, 2])
        self.edit_ax.set_title("Edición Manual")
        self.edit_ax.axis("off")

        # Panel de detalles del árbol
        self.details_ax = self.fig.add_subplot(self.gs[2, 2])
        self.details_ax.set_title("Detalles del Árbol de Comportamiento")
        self.details_ax.axis("off")

        # Panel de parámetros afinables
        self.params_ax = self.fig.add_subplot(self.gs[3, 2])
        self.params_ax.set_title("Parámetros Afinables")
        self.params_ax.axis("off")

        # Añadir controles
        self.add_controls()

        # Añadir widgets de edición
        self.add_position_editors()

        # Conectar eventos
        self.fig.canvas.mpl_connect(
            "button_press_event", lambda event: self.on_mouse_press(event)
        )
        self.fig.canvas.mpl_connect(
            "button_release_event", lambda event: self.on_mouse_release(event)
        )
        self.fig.canvas.mpl_connect(
            "motion_notify_event", lambda event: self.on_mouse_motion(event)
        )

        # Actualizar visualizaciones
        self.update_all_views()

    def setup_field_view(self):
        """Configura la vista del campo de juego"""
        self.field_ax.set_xlim(-50, ANCHO_CAMPO + 50)
        self.field_ax.set_ylim(-50, ALTO_CAMPO + 50)
        self.field_ax.set_title(
            "Campo de Juego - Click para seleccionar, arrastra para mover"
        )
        self.field_ax.set_aspect("equal")
        self.field_ax.grid(True, alpha=0.3)

        # Dibujar campo
        field_rect = Rectangle(
            (0, 0), ANCHO_CAMPO, ALTO_CAMPO, fill=False, edgecolor="black", linewidth=2
        )
        self.field_ax.add_patch(field_rect)

        # Líneas de zona
        self.field_ax.axvline(
            ANCHO_CAMPO * ZONA_IZQUIERDA, color="gray", linestyle="--", alpha=0.5
        )
        self.field_ax.axvline(
            ANCHO_CAMPO * ZONA_DERECHA, color="gray", linestyle="--", alpha=0.5
        )

        # Etiquetas de zona
        self.field_ax.text(
            ANCHO_CAMPO * ZONA_IZQUIERDA / 2,
            ALTO_CAMPO + 20,
            "Zona Defensiva",
            ha="center",
            fontsize=10,
            color="red",
            alpha=0.7,
        )
        self.field_ax.text(
            ANCHO_CAMPO * (ZONA_IZQUIERDA + ZONA_DERECHA) / 2,
            ALTO_CAMPO + 20,
            "Zona Neutral",
            ha="center",
            fontsize=10,
            color="gray",
            alpha=0.7,
        )
        self.field_ax.text(
            ANCHO_CAMPO * (ZONA_DERECHA + 1.0) / 2,
            ALTO_CAMPO + 20,
            "Zona Ofensiva",
            ha="center",
            fontsize=10,
            color="blue",
            alpha=0.7,
        )

        # Porterías
        self.field_ax.add_patch(
            Rectangle(
                (0, (ALTO_CAMPO / 2) - (LARGO_ARCO / 2)),
                10,
                LARGO_ARCO,
                color="red",
                alpha=0.3,
            )
        )
        self.field_ax.add_patch(
            Rectangle(
                (ANCHO_CAMPO - 10, (ALTO_CAMPO / 2) - (LARGO_ARCO / 2)),
                10,
                LARGO_ARCO,
                color="blue",
                alpha=0.3,
            )
        )

        # Crear elementos visuales
        self.create_field_elements()

    def create_field_elements(self):
        """Crea los elementos visuales del campo"""
        # Pelota
        self.ball_circle = Circle(
            (self.ball.x, self.ball.y), 30, color="orange", alpha=0.8, picker=True
        )
        self.field_ax.add_patch(self.ball_circle)
        self.ball_label = self.field_ax.text(
            self.ball.x,
            self.ball.y - 45,
            "Pelota",
            ha="center",
            fontsize=9,
            color="darkorange",
        )

        # Jugadores
        self.player_circles = {}
        self.player_directions = {}
        self.player_labels = {}
        self.player_info_texts = {}

        for player_id, player in self.players.items():
            color = "red" if player.team == "red" else "blue"

            # Círculo del jugador
            circle = Circle(
                (player.x, player.y), 40, color=color, alpha=0.7, picker=True
            )
            self.field_ax.add_patch(circle)
            self.player_circles[player_id] = circle

            # Línea de dirección
            angle_rad = np.radians(player.angle)
            dx = 60 * np.cos(angle_rad)
            dy = 60 * np.sin(angle_rad)
            arrow = FancyArrowPatch(
                (player.x, player.y),
                (player.x + dx, player.y + dy),
                arrowstyle="->",
                color="black",
                linewidth=2,
                mutation_scale=20,
            )
            self.field_ax.add_patch(arrow)
            self.player_directions[player_id] = arrow

            # Etiqueta
            text = self.field_ax.text(
                player.x,
                player.y,
                f"{player_id}",
                ha="center",
                va="center",
                color="white",
                fontsize=14,
                fontweight="bold",
            )
            self.player_labels[player_id] = text

            # Información adicional
            rol_text = "ATK" if player.rol == ROL_ATACANTE else "DEF"
            info = self.field_ax.text(
                player.x,
                player.y - 55,
                rol_text,
                ha="center",
                fontsize=9,
                color=color,
                fontweight="bold",
            )
            self.player_info_texts[player_id] = info

            # Inicializar marcadores de destino (ocultos inicialmente)
            target_marker = Circle(
                (0, 0),
                25,
                color=color,
                alpha=0.3,
                linestyle="--",
                fill=False,
                linewidth=2,
                visible=False,
            )
            self.field_ax.add_patch(target_marker)
            self.target_markers[player_id] = target_marker

            # Inicializar flechas de movimiento (ocultas inicialmente)
            movement_arrow = FancyArrowPatch(
                (0, 0),
                (0, 0),
                arrowstyle="->",
                color=color,
                alpha=0.6,
                linewidth=2,
                linestyle=":",
                visible=False,
            )
            self.field_ax.add_patch(movement_arrow)
            self.movement_arrows[player_id] = movement_arrow

        # Destacar robot enfocado
        self.create_highlight()

    def create_highlight(self):
        """Crea el resaltado para el robot enfocado"""
        player = self.players[self.focused_robot_id]
        self.highlight_circle = Circle(
            (player.x, player.y),
            55,
            color="yellow",
            alpha=0.3,
            fill=False,
            linewidth=3,
            linestyle="--",
        )
        self.field_ax.add_patch(self.highlight_circle)

    def add_controls(self):
        """Añade controles interactivos mejorados"""
        # Selector de robot
        robot_selector_ax = plt.axes((0.02, 0.8, 0.08, 0.15))
        self.robot_selector = RadioButtons(
            robot_selector_ax,
            ["Robot 1", "Robot 2", "Robot 3", "Robot 4"],
            active=self.focused_robot_id - 1,
        )
        self.robot_selector.on_clicked(self.on_robot_selected)

        # Botones de acción
        reset_button_ax = plt.axes((0.02, 0.75, 0.08, 0.04))
        self.reset_button = Button(reset_button_ax, "Reset Posiciones")
        self.reset_button.on_clicked(self.reset_positions)

        roles_button_ax = plt.axes((0.02, 0.7, 0.08, 0.04))
        self.roles_button = Button(roles_button_ax, "Actualizar Roles")
        self.roles_button.on_clicked(self.update_roles)

        analyze_button_ax = plt.axes((0.02, 0.65, 0.08, 0.04))
        self.analyze_button = Button(analyze_button_ax, "Analizar")
        self.analyze_button.on_clicked(self.analyze_behaviors)

        # Niveles de depuración
        # debug_level_ax = plt.axes((0.02, 0.6, 0.08, 0.14))
        # self.debug_level = RadioButtons(
        #     debug_level_ax, ["Básico", "Detallado", "Completo"], active=0
        # )
        # self.debug_level.on_clicked(self.on_debug_level_changed)

        sim_button_ax = plt.axes((0.02, 0.6, 0.08, 0.04))
        self.sim_button = Button(sim_button_ax, "▶ Simular")
        self.sim_button.on_clicked(self.toggle_simulation)

        export_button_ax = plt.axes((0.02, 0.55, 0.08, 0.04))
        self.export_button = Button(export_button_ax, "Exportar")
        self.export_button.on_clicked(self.export_game_context)

    def add_position_editors(self):
        """Añade widgets para editar posiciones manualmente"""
        # Limpiar panel de edición
        self.edit_ax.clear()
        self.edit_ax.set_title(
            "Edición Manual de Posiciones",
            y=0.55,
            x=0.4,
            fontsize=12,
            fontweight="bold",
        )
        self.edit_ax.axis("off")

        # ============= VARIABLES DE LAYOUT =============
        # Posiciones de texto (coordenadas del subplot)
        text_x_start = -0.35
        text_x_spacing = 0.235  # Espaciado uniforme entre columnas de texto
        text_y_start = 0.22
        text_y_spacing = 0.25  # Espaciado entre filas

        # Posiciones de widgets (coordenadas de figura)
        widget_base_x = 0.638  # Posición inicial de widgets
        widget_base_y = 0.44  # Posición Y base de widgets
        widget_x_spacing = 0.04  # Espaciado horizontal entre widgets
        widget_y_spacing = 0.0365  # Espaciado vertical entre filas
        widget_width = 0.035  # Ancho de cada TextBox
        widget_height = 0.025  # Alto de cada TextBox

        x_offset = 0.12  # Offset general (mantener por compatibilidad)

        # ============= ENCABEZADOS =============
        headers = ["Objeto", "X", "Y", "VX", "VY", "Ángulo"]
        header_x_positions = [
            text_x_start,                           # "Objeto"
            text_x_start + text_x_spacing,         # "X"
            text_x_start + text_x_spacing * 2,     # "Y"
            text_x_start + text_x_spacing * 3,     # "VX"
            text_x_start + text_x_spacing * 4,     # "VY"
            text_x_start + text_x_spacing * 5      # "Ángulo"
        ]

        for i, (header, x_pos) in enumerate(zip(headers, header_x_positions)):
            color = "red" if header in ["VX", "VY"] else "black"
            self.edit_ax.text(
                x_pos,
                text_y_start + 0.05,
                header,
                fontsize=10,
                fontweight="bold",
                color=color
            )

        # ============= EDITOR PARA LA PELOTA =============
        pelota_y_pos = text_y_start - text_y_spacing

        # Texto "Pelota"
        self.edit_ax.text(
            text_x_start,
            pelota_y_pos + 0.1,
            "Pelota",
            fontsize=11
        )

        # Posiciones de widgets para la pelota
        pelota_widget_positions = [
            widget_base_x + x_offset,                      # X
            widget_base_x + x_offset + widget_x_spacing,   # Y
            widget_base_x + x_offset + widget_x_spacing * 2, # VX
            widget_base_x + x_offset + widget_x_spacing * 3, # VY
        ]

        # Crear TextBoxes para la pelota
        ball_x_ax = self.fig.add_axes([pelota_widget_positions[0], widget_base_y, widget_width, widget_height])
        ball_y_ax = self.fig.add_axes([pelota_widget_positions[1], widget_base_y, widget_width, widget_height])
        ball_vx_ax = self.fig.add_axes([pelota_widget_positions[2], widget_base_y, widget_width, widget_height])
        ball_vy_ax = self.fig.add_axes([pelota_widget_positions[3], widget_base_y, widget_width, widget_height])

        # Inicializar TextBoxes
        self.ball_widgets["x"] = TextBox(ball_x_ax, "", initial=f"{self.ball.x:.0f}")
        self.ball_widgets["y"] = TextBox(ball_y_ax, "", initial=f"{self.ball.y:.0f}")
        self.ball_widgets["vx"] = TextBox(ball_vx_ax, "", initial=f"{self.ball_physics['velocity_x']:.1f}")
        self.ball_widgets["vy"] = TextBox(ball_vy_ax, "", initial=f"{self.ball_physics['velocity_y']:.1f}")

        # Conectar eventos
        self.ball_widgets["x"].on_submit(lambda text: self.update_ball_position())
        self.ball_widgets["y"].on_submit(lambda text: self.update_ball_position())
        self.ball_widgets["vx"].on_submit(lambda text: self.update_ball_velocity())
        self.ball_widgets["vy"].on_submit(lambda text: self.update_ball_velocity())

        # ============= EDITORES PARA ROBOTS =============
        for i, player_id in enumerate([1, 2, 3, 4]):
            # Posición Y para esta fila
            robot_y_pos = text_y_start - (i + 2) * text_y_spacing
            widget_y_pos = widget_base_y - (i + 1) * widget_y_spacing

            player = self.players[player_id]
            color = "red" if player.team == "red" else "blue"

            # Texto del robot
            self.edit_ax.text(
                text_x_start,
                robot_y_pos + 0.1,
                f"Robot {player_id}",
                fontsize=11,
                color=color
            )

            # Posiciones de widgets para este robot
            robot_widget_positions = [
                widget_base_x + x_offset,                        # X
                widget_base_x + x_offset + widget_x_spacing,     # Y
                widget_base_x + x_offset + widget_x_spacing * 4, # Ángulo (saltamos VX, VY)
            ]

            # Crear TextBoxes para este robot
            x_ax = self.fig.add_axes([robot_widget_positions[0], widget_y_pos, widget_width, widget_height])
            y_ax = self.fig.add_axes([robot_widget_positions[1], widget_y_pos, widget_width, widget_height])
            angle_ax = self.fig.add_axes([robot_widget_positions[2], widget_y_pos, widget_width, widget_height])

            x_box = TextBox(x_ax, "", initial=f"{player.x:.0f}")
            y_box = TextBox(y_ax, "", initial=f"{player.y:.0f}")
            angle_box = TextBox(angle_ax, "", initial=f"{player.angle:.0f}")

            # Conectar eventos
            x_box.on_submit(lambda text, pid=player_id: self.update_robot_position(pid))
            y_box.on_submit(lambda text, pid=player_id: self.update_robot_position(pid))
            angle_box.on_submit(lambda text, pid=player_id: self.update_robot_position(pid))

            # Guardar referencias
            self.position_widgets[player_id] = {
                "x": x_box,
                "y": y_box,
                "angle": angle_box,
                "axes": [x_ax, y_ax, angle_ax],
            }

    def update_ball_position(self):
        """Actualiza la posición de la pelota desde los TextBoxes"""
        try:
            x = float(self.ball_widgets["x"].text)
            y = float(self.ball_widgets["y"].text)

            # Limitar dentro del campo
            x = max(0, min(ANCHO_CAMPO, x))
            y = max(0, min(ALTO_CAMPO, y))

            # Actualizar
            self.ball.set_position(x, y)
            self.positions["ball"] = [x, y]

            # Actualizar visuales
            self.ball_circle.center = (x, y)
            self.ball_label.set_position((x, y - 45))

            # Analizar
            self.analyze_behaviors()
            self.fig.canvas.draw_idle()

        except ValueError:
            pass

    def update_ball_velocity(self):
        """Actualiza la velocidad de la pelota desde los TextBoxes."""
        try:
            vx = float(self.ball_widgets["vx"].text)
            vy = float(self.ball_widgets["vy"].text)

            # Limitar valores razonables
            vx = max(-50, min(50, vx))
            vy = max(-50, min(50, vy))

            # Actualizar parámetros de física
            self.ball_physics['velocity_x'] = vx
            self.ball_physics['velocity_y'] = vy

            log.info(f" -- Velocidad actualizada: vx={vx:.1f}, vy={vy:.1f}")

            # Actualizar TextBoxes si los valores fueron limitados
            self.ball_widgets["vx"].set_val(f"{vx:.1f}")
            self.ball_widgets["vy"].set_val(f"{vy:.1f}")

        except ValueError:
            # Restaurar valores anteriores en caso de error
            self.ball_widgets["vx"].set_val(f"{self.ball_physics['velocity_x']:.1f}")
            self.ball_widgets["vy"].set_val(f"{self.ball_physics['velocity_y']:.1f}")

    def update_robot_position(self, player_id):
        """Actualiza la posición de un robot desde los TextBoxes"""
        try:
            widgets = self.position_widgets[player_id]
            x = float(widgets["x"].text)
            y = float(widgets["y"].text)
            angle = float(widgets["angle"].text) % 360

            # Limitar dentro del campo
            x = max(0, min(ANCHO_CAMPO, x))
            y = max(0, min(ALTO_CAMPO, y))

            # Actualizar
            player = self.players[player_id]
            player.set_position(x, y)
            player.set_angle(angle)
            self.positions[player_id] = [x, y, angle]

            # Actualizar visuales
            self.update_player_visual(player_id)

            # Analizar
            self.analyze_behaviors()
            self.fig.canvas.draw_idle()

        except ValueError:
            pass

    def update_player_visual(self, player_id):
        """Actualiza los elementos visuales de un jugador"""
        player = self.players[player_id]

        # Círculo
        self.player_circles[player_id].center = (player.x, player.y)

        # Dirección
        angle_rad = np.radians(player.angle)
        dx = 60 * np.cos(angle_rad)
        dy = 60 * np.sin(angle_rad)
        self.player_directions[player_id].set_positions(
            (player.x, player.y), (player.x + dx, player.y + dy)
        )

        # Etiquetas
        self.player_labels[player_id].set_position((player.x, player.y))
        self.player_info_texts[player_id].set_position((player.x, player.y - 55))

        # Destacado si es necesario
        if player_id == self.focused_robot_id:
            self.highlight_circle.center = (player.x, player.y)

    def update_movement_visualization(self):
        """Actualiza la visualización de movimientos planificados en el campo"""
        # Limpiar visualizaciones anteriores
        for player_id in range(1, 5):
            self.target_markers[player_id].set_visible(False)
            self.movement_arrows[player_id].set_visible(False)

        # Mostrar movimientos planificados desde la extensión del tracer
        for player_id, movement in self.tracer_ext.planned_movements.items():
            if "target_pos" in movement:
                target_x, target_y = movement["target_pos"]
                player = self.players[player_id]

                # Mostrar marcador de destino
                self.target_markers[player_id].center = (target_x, target_y)
                self.target_markers[player_id].set_visible(True)

                # Mostrar flecha de movimiento
                self.movement_arrows[player_id].set_positions(
                    (player.x, player.y), (target_x, target_y)
                )
                self.movement_arrows[player_id].set_visible(True)

    def update_actions_view(self):
        """Actualiza la visualización de acciones planificadas con coordenadas"""
        self.actions_ax.clear()
        self.actions_ax.set_title(
            "Secuencia de Acciones Planificadas", y=1.05, fontsize=12, fontweight="bold"
        )
        self.actions_ax.axis("off")

        # ============= VARIABLES DE LAYOUT =============
        # Posiciones de texto base
        text_x_start = -0.35        # Alineación izquierda del texto
        text_y_start = 1.0         # Posición Y inicial
        text_y_spacing = 0.34      # Espaciado entre robots

        # Offsets internos para diferentes tipos de texto
        action_y_offset = -0.05     # Offset para línea de acción
        coord_y_offset = -0.1     # Offset para coordenadas
        desc_y_offset = -0.15      # Offset para descripción
        history_y_offset = -0.2   # Offset para historial

        # Configuración de fuentes y colores
        robot_title_fontsize = 10
        action_fontsize = 8
        desc_fontsize = 9
        coord_fontsize = 8
        history_fontsize = 8

        # Mostrar acciones para cada robot
        for i, player_id in enumerate([1, 2, 3, 4]):
            player = self.players[player_id]
            team_color = "red" if player.team == "red" else "blue"

            # Calcular posición Y base para este robot
            robot_y_base = text_y_start - i * text_y_spacing

            # Título del robot
            rol_text = "ATACANTE" if player.rol == ROL_ATACANTE else "DEFENSOR"
            self.actions_ax.text(
                text_x_start,
                robot_y_base,
                f"Robot {player_id} ({rol_text}):",
                fontsize=robot_title_fontsize,
                fontweight="bold",
                color=team_color,
            )

            # Buscar si hay una acción registrada para este robot específico
            action_found = False
            action_name = None

            # Primero verificar si hay una acción específica para este robot en la extensión
            if player_id in self.tracer_ext.robot_actions:
                action_name = self.tracer_ext.robot_actions[player_id]
                action_found = True

            # Si no hay acción específica, usar la acción general del tracer si es el robot enfocado
            elif self.tracer.next_action and self.focused_robot_id == player_id:
                action_name = self.tracer.next_action["name"]
                action_found = True

            # Si encontramos una acción, mostrarla
            if action_found and action_name:
                # Formatear nombre de acción
                formatted_action = " ".join(
                    word.capitalize() for word in action_name.split("_")
                )

                # Crear descripción detallada según la acción
                action_desc = self.get_action_description_with_coords(
                    player_id, action_name
                )

                # Mostrar secuencia
                self.actions_ax.text(
                    text_x_start,
                    robot_y_base + action_y_offset,
                    f"➜ {formatted_action}",
                    fontsize=action_fontsize,
                    color="darkgreen",
                    fontweight="bold",
                )
                self.actions_ax.text(
                    text_x_start,
                    robot_y_base + desc_y_offset,
                    f"   {action_desc}",
                    fontsize=desc_fontsize,
                    color="gray"
                )

                # Añadir coordenadas específicas si están disponibles
                coord_info = self.get_coordinate_info(player_id)
                if coord_info:
                    self.actions_ax.text(
                        text_x_start,
                        robot_y_base + coord_y_offset,
                        f"{coord_info}",
                        fontsize=coord_fontsize,
                        color="blue"
                    )

                # Añadir al historial
                if (
                    len(self.action_history[player_id]) == 0
                    or self.action_history[player_id][-1] != formatted_action
                ):
                    self.action_history[player_id].append(formatted_action)
                    if len(self.action_history[player_id]) > self.max_history_length:
                        self.action_history[player_id].pop(0)
            else:
                # Si no hay acción, mostrar estado de espera
                self.actions_ax.text(
                    text_x_start,
                    robot_y_base + action_y_offset,
                    "➜ En espera...",
                    fontsize=action_fontsize,
                    color="gray",
                    style="italic",
                )

            # Mostrar historial reciente
            if self.action_history[player_id]:
                history_text = " → ".join(self.action_history[player_id][-2:])
                self.actions_ax.text(
                    text_x_start,
                    robot_y_base + history_y_offset,
                    f"Historial: {history_text}",
                    fontsize=history_fontsize,
                    color="lightgray",
                    style="italic",
                )

    def get_action_description_with_coords(self, player_id, action):
        """Genera una descripción detallada de la acción incluyendo coordenadas"""
        descriptions = {
            "move_to_ball": f"Robot {player_id} Moverse a pelota en ({self.ball.x:.0f}, {self.ball.y:.0f})",
            "capture_ball": "Capturar la pelota con el dribbler",
            "dribble_forward": "Avanzar con pelota hacia portería rival",
            "shoot_to_goal": "Disparar a portería con potencia máxima",
            "pass_to_teammate": "Pasar la pelota a compañero mejor posicionado",
            "intercept_ball": "Interceptar trayectoria de la pelota",
            "block_opponent": "Bloquear al rival más peligroso",
            "move_to_defensive_position": "Posicionarse defensivamente",
            "move_to_support_position": "Buscar posición de apoyo ofensivo",
            "position_to_defend_goal": "Defender la portería propia",
        }

        return descriptions.get(action, "Ejecutar acción")

    def get_coordinate_info(self, player_id):
        """Obtiene información de coordenadas para un robot específico"""
        if player_id in self.tracer_ext.planned_movements:
            movement = self.tracer_ext.planned_movements[player_id]
            target_pos = movement["target_pos"]
            # action_type = movement.get('action_type', 'move')

            # Calcular distancia
            player = self.players[player_id]
            distance = np.sqrt(
                (target_pos[0] - player.x) ** 2 + (target_pos[1] - player.y) ** 2
            )

            coord_text = f"Destino: ({target_pos[0]:.0f}, {target_pos[1]:.0f})"
            coord_text += f" | Distancia: {distance:.0f} px"

            # Añadir información adicional si está disponible
            if "additional_info" in movement and movement["additional_info"]:
                for key, value in movement["additional_info"].items():
                    coord_text += f" | {key}: {value}"

            return coord_text

        if player_id in self.tracer_ext.action_details:
            details = self.tracer_ext.action_details[player_id]["details"]
            if "target_pos" in details:
                target_pos = details["target_pos"]
                return f"Objetivo: ({target_pos[0]:.0f}, {target_pos[1]:.0f})"

        return None

    def on_mouse_press(self, event):
        """Maneja los eventos de click"""
        if event.inaxes != self.field_ax:
            return

        if event.button != 1:
            return

        # Buscar objeto bajo el cursor
        min_dist = float("inf")
        selected_obj = None

        # Comprobar pelota
        dist_to_ball = np.sqrt(
            (event.xdata - self.ball.x) ** 2 + (event.ydata - self.ball.y) ** 2
        )
        if dist_to_ball < 30:
            min_dist = dist_to_ball
            selected_obj = "ball"

        # Comprobar robots
        for player_id, player in self.players.items():
            dist = np.sqrt(
                (event.xdata - player.x) ** 2 + (event.ydata - player.y) ** 2
            )
            if dist < 40 and dist < min_dist:
                min_dist = dist
                selected_obj = player_id

        # Iniciar arrastre si se seleccionó algo
        if selected_obj is not None:
            self.dragging = True
            self.drag_object = selected_obj

            # Si es un robot, actualizarlo como enfocado
            if selected_obj != "ball":
                self.focused_robot_id = selected_obj
                self.robot_selector.set_active(int(selected_obj) - 1)
                self.update_focus()

    def on_mouse_release(self, event):
        """Maneja la liberación del mouse"""
        if self.dragging:
            self.dragging = False
            self.drag_object = None

            # Actualizar TextBoxes con las nuevas posiciones
            self.update_textboxes()

            # Analizar comportamientos
            self.analyze_behaviors()

    def on_mouse_motion(self, event):
        """Maneja el movimiento del mouse durante el arrastre"""
        if not self.dragging or self.drag_object is None:
            return

        if event.inaxes != self.field_ax or event.xdata is None or event.ydata is None:
            return

        # Calcular nueva posición directamente desde el mouse
        new_x = event.xdata
        new_y = event.ydata

        # Limitar dentro del campo
        new_x = max(0, min(ANCHO_CAMPO, new_x))
        new_y = max(0, min(ALTO_CAMPO, new_y))

        # Actualizar posición
        if self.drag_object == "ball":
            self.ball.set_position(new_x, new_y)
            self.ball_circle.center = (new_x, new_y)
            self.ball_label.set_position((new_x, new_y - 45))
            self.positions["ball"] = [new_x, new_y]
        else:
            player_id = self.drag_object
            player = self.players[player_id]
            player.set_position(new_x, new_y)
            self.positions[player_id][0] = new_x
            self.positions[player_id][1] = new_y
            self.update_player_visual(player_id)

        self.fig.canvas.draw_idle()

    def update_textboxes(self):
        """Actualiza los valores en los TextBoxes después de arrastrar"""
        # Actualizar pelota
        self.ball_widgets["x"].set_val(f"{self.ball.x:.0f}")
        self.ball_widgets["y"].set_val(f"{self.ball.y:.0f}")

        # Actualizar robots
        for player_id, widgets in self.position_widgets.items():
            player = self.players[player_id]
            widgets["x"].set_val(f"{player.x:.0f}")
            widgets["y"].set_val(f"{player.y:.0f}")
            widgets["angle"].set_val(f"{player.angle:.0f}")

    def on_robot_selected(self, label):
        """Maneja la selección de robot"""
        robot_id = int(label.split()[-1])
        self.focused_robot_id = robot_id
        self.update_focus()
        self.analyze_behaviors()

    def update_focus(self):
        """Actualiza el destacado del robot enfocado"""
        player = self.players[self.focused_robot_id]
        self.highlight_circle.center = (player.x, player.y)
        self.fig.canvas.draw_idle()

    def on_debug_level_changed(self, label):
        """Maneja el cambio de nivel de depuración"""
        if label == "Básico":
            self.execution_depth = 1
        elif label == "Detallado":
            self.execution_depth = 3
        else:
            self.execution_depth = 10

        self.update_all_views()

    def reset_positions(self, event=None):
        """Resetea las posiciones a los valores iniciales"""
        # Resetear posiciones
        self.positions = {
            "ball": [750, 450],
            1: [200, 200, 0],
            2: [200, 700, 0],
            3: [1200, 200, 0],
            4: [1200, 700, 0],
        }

        # Actualizar objetos
        self.ball.set_position(self.positions["ball"][0], self.positions["ball"][1])

        for player_id, (x, y, angle) in list(self.positions.items())[1:]:
            self.players[player_id].set_position(x, y)
            self.players[player_id].set_angle(angle)

        # Actualizar visuales
        self.update_field_view()
        self.update_textboxes()

        # Limpiar historial
        self.action_history = {i: [] for i in range(1, 5)}

        # Analizar
        self.analyze_behaviors()

    def analyze_behaviors(self, event=None):
        """Analiza los comportamientos con el contexto actual"""
        # Limpiar tracer original y extensión
        self.tracer.clear()
        self.tracer_ext.clear()

        # Debug: Verificar que los tracers están limpios
        log.debug("Tracer limpiado. Trace items: %i", len(self.tracer.trace))
        log.debug("Extension limpiada. Robot actions: %i", len(self.tracer_ext.robot_actions))

        # Actualizar lógica difusa
        red_context = self.context_red.evaluar_ms_logic_difusse()
        blue_context = self.context_blue.evaluar_ms_logic_difusse()

        # Actualizar contexto
        self.behavior_red.update_game_context(red_context)
        self.behavior_blue.update_game_context(blue_context)

        # Ejecutar árboles sin movimiento real
        self._tick_behaviors_without_movement()

        # Debug: Verificar que se generaron acciones
        log.debug("Después de tick. Trace items: %i", len(self.tracer.trace))
        log.debug("Robot actions generadas: %s", self.tracer_ext.robot_actions)
        log.debug("Planned movements: %s", list(self.tracer_ext.planned_movements.keys()))

        # Obtener estados
        for player_id in range(1, 5):
            team = "red" if player_id <= 2 else "blue"
            manager = self.team_managers[team]
            self.current_states[player_id] = manager.get_current_state(player_id)

        # Actualizar vistas
        self.update_all_views()

    def update_roles(self, event=None):
        """Actualiza los roles de los jugadores"""
        # Los roles se actualizan automáticamente en evaluar_ms_logic_difusse
        self.analyze_behaviors()

        # Actualizar etiquetas de rol
        for player_id, player in self.players.items():
            rol_text = "ATK" if player.rol == ROL_ATACANTE else "DEF"
            self.player_info_texts[player_id].set_text(rol_text)

    def _tick_behaviors_without_movement(self):
        """Ejecuta los árboles sin permitir movimiento real pero capturando las coordenadas planificadas"""

        # Clase mock mejorada para el gestor de comandos
        class EnhancedMockCommandManager:
            def __init__(self, tracer_ext, players, ball, team_players):
                self.last_command = None
                self.tracer_ext = tracer_ext
                self.players = players
                self.ball = ball
                self.team_players = team_players

            def move_robot_to(
                self, player_id, target_pos, target_angle=None, speed_factor=1.0
            ):
                log.debug("Mock move_robot_to called for robot %s to %s", player_id, target_pos)
                self.last_command = {
                    "type": "move",
                    "player_id": player_id,
                    "target_pos": target_pos,
                }
                # Registrar movimiento planificado
                additional_info = {}
                if target_angle is not None:
                    additional_info["target_angle"] = target_angle
                if speed_factor != 1.0:
                    additional_info["speed_factor"] = speed_factor

                self.tracer_ext.set_planned_movement(
                    player_id, target_pos, "move_to_position", additional_info
                )

                # Registrar la acción específica para este robot
                self.tracer_ext.set_robot_action(player_id, "move_to_ball")
                log.debug("Registered action 'move_to_ball' for robot %i", player_id)
                return False

            def rotate_robot_to(self, player_id, target_angle):
                log.debug(
                    "Mock rotate_robot_to called for robot %i to angle %s", player_id, target_angle
                )
                self.last_command = {
                    "type": "rotate",
                    "player_id": player_id,
                    "target_angle": target_angle,
                }
                player = self.players[player_id]
                self.tracer_ext.set_action_details(
                    player_id,
                    "rotate_to_angle",
                    {"target_angle": target_angle, "current_angle": player.angle},
                )
                self.tracer_ext.set_robot_action(player_id, "rotate_to_angle")
                return False

            def capture_ball(self, player_id):
                log.debug("Mock capture_ball called for robot %i", player_id)
                self.last_command = {"type": "capture", "player_id": player_id}
                # Para capturar, el destino es la posición de la pelota
                ball_pos = (self.ball.x, self.ball.y)
                self.tracer_ext.set_planned_movement(
                    player_id, ball_pos, "capture_ball", {"action": "capture_ball"}
                )
                self.tracer_ext.set_robot_action(player_id, "capture_ball")
                return False

            def kick_ball(self, player_id, target_pos, ball, power=1.0):
                self.last_command = {
                    "type": "kick",
                    "player_id": player_id,
                    "target_pos": target_pos,
                }
                self.tracer_ext.set_action_details(
                    player_id,
                    "kick_ball",
                    {
                        "target_pos": target_pos,
                        "power": power,
                        "ball_pos": (ball.x, ball.y),
                    },
                )
                self.tracer_ext.set_robot_action(player_id, "kick_ball")
                return True

            def move_with_ball(self, player_id, target_pos, ball, speed_factor=0.7):
                self.last_command = {
                    "type": "move_with_ball",
                    "player_id": player_id,
                    "target_pos": target_pos,
                }
                self.tracer_ext.set_planned_movement(
                    player_id,
                    target_pos,
                    "move_with_ball",
                    {"speed_factor": speed_factor, "has_ball": True},
                )
                self.tracer_ext.set_robot_action(player_id, "move_with_ball")
                return False

        # Guardar gestores originales
        original_red = self.behavior_red.command_manager
        original_blue = self.behavior_blue.command_manager

        try:
            # Reemplazar con mocks mejorados
            mock_red = EnhancedMockCommandManager(
                self.tracer_ext, self.players, self.ball, [self.player_1, self.player_2]
            )
            mock_blue = EnhancedMockCommandManager(
                self.tracer_ext, self.players, self.ball, [self.player_3, self.player_4]
            )

            self.behavior_red.command_manager = mock_red
            self.behavior_blue.command_manager = mock_blue

            # Propagar a blackboards
            for blackboard in self.behavior_red.blackboards.values():
                blackboard.command_manager = mock_red
            for blackboard in self.behavior_blue.blackboards.values():
                blackboard.command_manager = mock_blue

            # Ejecutar comportamientos por robot individual para capturar acciones específicas
            for player_id in [1, 2]:  # Equipo rojo
                player = self.players[player_id]
                blackboard = self.behavior_red.blackboards[player_id]

                # Ejecutar el árbol correspondiente según el rol del jugador
                if player.rol == ROL_ATACANTE:
                    status = self.behavior_red.attacker_tree.tick(blackboard)
                else:  # ROL_DEFENSIVO
                    status = self.behavior_red.defender_tree.tick(blackboard)

                # Registrar que se ejecutó una acción para este robot
                if self.tracer.next_action:
                    self.tracer_ext.set_robot_action(
                        player_id, self.tracer.next_action["name"]
                    )

            for player_id in [3, 4]:  # Equipo azul
                player = self.players[player_id]
                blackboard = self.behavior_blue.blackboards[player_id]

                # Ejecutar el árbol correspondiente según el rol del jugador
                if player.rol == ROL_ATACANTE:
                    status = self.behavior_blue.attacker_tree.tick(blackboard)
                else:  # ROL_DEFENSIVO
                    status = self.behavior_blue.defender_tree.tick(blackboard)

                # Registrar que se ejecutó una acción para este robot
                if self.tracer.next_action:
                    self.tracer_ext.set_robot_action(
                        player_id, self.tracer.next_action["name"]
                    )

        finally:
            # Restaurar
            self.behavior_red.command_manager = original_red
            self.behavior_blue.command_manager = original_blue

            for blackboard in self.behavior_red.blackboards.values():
                blackboard.command_manager = original_red
            for blackboard in self.behavior_blue.blackboards.values():
                blackboard.command_manager = original_blue

    def update_all_views(self):
        """Actualiza todas las visualizaciones"""
        self.update_field_view()
        self.update_behavior_view()
        self.update_actions_view()
        self.update_details_view()
        self.update_params_view()
        self.update_movement_visualization()
        self.fig.canvas.draw_idle()

    def update_field_view(self):
        """Actualiza la vista del campo"""
        # Actualizar pelota
        self.ball_circle.center = (self.ball.x, self.ball.y)
        self.ball_label.set_position((self.ball.x, self.ball.y - 45))

        # Actualizar jugadores
        for player_id in range(1, 5):
            self.update_player_visual(player_id)

    def update_behavior_view(self):
        """Actualiza la vista de comportamiento actual"""
        self.behavior_ax.clear()
        self.behavior_ax.set_title(
            f"Estado Robot {self.focused_robot_id}",
            y=TITLE_DIFUSE_LOGIC_Y,
            x=TITLE_DIFUSE_LOGIC_X,
            fontweight="bold",
            fontsize=TITLE_HEADER_FONTSIZE,
        )
        self.behavior_ax.axis("off")

        if self.focused_robot_id not in self.current_states:
            return

        state = self.current_states[self.focused_robot_id]
        player = self.players[self.focused_robot_id]

        # Determinar el equipo y obtener el contexto de lógica difusa correspondiente
        team = "red" if self.focused_robot_id <= 2 else "blue"
        fuzzy_manager = self.context_managers[team]

        # Obtener las variables de entrada y salida del sistema difuso
        fuzzy_vars = get_fuzzy_variables(fuzzy_manager, self.focused_robot_id, self.players, self.ball)

        # Veolocida de la pelota
        speed = getattr(self.ball, 'speed', 0)
        velocity_info = "VELOCIDAD PELOTA\n"
        velocity_info += f"Velocidad: {speed:.2f} px/s\n"

        # COLUMNA 1: SISTEMA POSESIÓN (entradas y salida)
        posesion_info = "$\\bf{SISTEMA \\ POSESIÓN}$\n"
        posesion_info += "Entradas:\n"
        posesion_info += f"• Dist A1: {fuzzy_vars['inputs']['distancia_aliado1']:.0f}\n"
        posesion_info += f"• Dist A2: {fuzzy_vars['inputs']['distancia_aliado2']:.0f}\n"
        posesion_info += f"• Dist R1: {fuzzy_vars['inputs']['distancia_rival1']:.0f}\n"
        posesion_info += f"• Dist R2: {fuzzy_vars['inputs']['distancia_rival2']:.0f}\n"
        posesion_info += f"• Ori A1: {fuzzy_vars['inputs']['orientacion_aliado1']:.2f}\n"
        posesion_info += f"• Ori A2: {fuzzy_vars['inputs']['orientacion_aliado2']:.2f}\n"
        posesion_info += f"• Ori R1: {fuzzy_vars['inputs']['orientacion_rival1']:.2f}\n"
        posesion_info += f"• Ori R2: {fuzzy_vars['inputs']['orientacion_rival2']:.2f}\n"
        posesion_info += f"• Vel: {fuzzy_vars['inputs']['velocidad_pelota']:.1f}\n"
        posesion_info += f"• Dir: {fuzzy_vars['inputs']['direccion_movimiento']:.1f}\n\n"
        posesion_info += f"Salida:\n"
        posesion_info += f"• Posesión: {fuzzy_vars['outputs']['posesion_pelota']:.2f}"


        # COLUMNA 2: SISTEMA PROXIMIDAD (entradas y salida)
        proximidad_info = "$\\bf{SISTEMA \\ PROXIMIDAD}$\n"
        proximidad_info += "Entradas:\n"
        proximidad_info += f"• Posesión R: {fuzzy_vars['inputs']['posesion_pelota_result']:.2f}\n"
        proximidad_info += f"• Ventaja P: {fuzzy_vars['inputs']['ventaja_proximidad']:.0f}\n"
        proximidad_info += f"• Vel Pelota: {fuzzy_vars['inputs']['velocidad_pelota']:.1f}\n\n"
        proximidad_info += f"Salida:\n"
        proximidad_info += f"• Proximidad: {fuzzy_vars['outputs']['proximidad_equipo']:.2f}"

        # COLUMNA 3: SISTEMA ZONA (entradas y salida)
        zona_info = "$\\bf{SISTEMA \\ ZONA}$\n"
        zona_info += "Entradas:\n"
        zona_info += f"• Pos X: {fuzzy_vars['inputs']['posicion_x']:.0f}\n"
        zona_info += f"• Dir Mov: {fuzzy_vars['inputs']['direccion_movimiento']:.1f}\n\n"
        zona_info += f"Salida:\n"
        zona_info += f"• Zona: {fuzzy_vars['outputs']['zona_pelota']:.2f}"


        # Información del robot (parte superior)
        info_basic = f"Equipo: {'Rojo' if self.focused_robot_id <= 2 else 'Azul'}\n"
        info_basic += f"Rol: {state.get('rol', 'Desconocido')}\n"

        self.behavior_ax.text(
            TEXT_X_POSITION_GENERAL,
            TEXT_Y_POSITION_GENERAL,
            info_basic,
            ha="left",
            va="top",
            transform=self.behavior_ax.transAxes,
            fontsize=FONT_SIZE_GENERAL,
            linespacing=TEXT_LINESPACING_GENERAL,
        )
        self.behavior_ax.text(
            TEXT_X_POSITION_VELOCIDAD,
            TEXT_Y_POSITION_VELOCIDAD,
            velocity_info,
            ha="left",
            va="top",
            transform=self.behavior_ax.transAxes,
            fontsize=FONT_SIZE_VELOCIDAD,
            linespacing=TEXT_LINESPACING_VELOCIDAD,
        )
        self.behavior_ax.text(
            TEXT_X_POSITION_POSESION,
            TEXT_Y_POSITION_POSESION,
            posesion_info,
            ha="left",
            va="top",
            transform=self.behavior_ax.transAxes,
            fontsize=FONT_SIZE_POSESION,
            linespacing=TEXT_LINESPACING_POSESION,
        )
        self.behavior_ax.text(
            TEXT_X_POSITION_PROXIMIDAD,
            TEXT_Y_POSITION_PROXIMIDAD,
            proximidad_info,
            ha="left",
            va="top",
            transform=self.behavior_ax.transAxes,
            fontsize=FONT_SIZE_PROXIMIDAD,
            linespacing=TEXT_LINESPACING_PROXIMIDAD,
        )
        self.behavior_ax.text(
            TEXT_X_POSITION_ZONA,
            TEXT_Y_POSITION_ZONA,
            zona_info,
            ha="left",
            va="top",
            transform=self.behavior_ax.transAxes,
            fontsize=FONT_SIZE_ZONA,
            linespacing=TEXT_LINESPACING_ZONA,
        )

    def update_details_view(self):
        """Actualiza la vista de detalles del árbol con jerarquía y condiciones"""
        self.details_ax.clear()
        self.details_ax.set_title(
            "Árbol de Decisiones y Parámetros",
            y=-0.3,
            x=-1.5,
            fontweight="bold",
            fontsize=12,
        )
        self.details_ax.axis("off")

        # ============= VARIABLES DE LAYOUT =============
        text_x_col1 = -2.5   # Columna 1: Condiciones Evaluadas
        text_x_col2 = -1.9   # Columna 2: Parámetros Clave
        text_x_col3 = -1.3    # Columna 3: Flujo de Decisiones
        text_y_position = -0.2
        text_fontsize = 8
        text_linespacing = 1.3
        text_family = "monospace"

        # ============= GENERAR CONTENIDO =============
        player = self.players[self.focused_robot_id]
        blackboard = None
        team = "red" if self.focused_robot_id <= 2 else "blue"
        manager = self.team_managers[team]

        if self.focused_robot_id in manager.blackboards:
            blackboard = manager.blackboards[self.focused_robot_id]

        # ============= COLUMNA 1: CONDICIONES EVALUADAS =============
        col1_text = f"Robot {self.focused_robot_id} ({'ATK' if player.rol == ROL_ATACANTE else 'DEF'})\n\n"
        col1_text += "CONDICIONES EVALUADAS:\n"

        if self.tracer.conditions_met:
            for cond in self.tracer.conditions_met[-8:]:  # Últimas 8 condiciones
                result_symbol = "✓" if cond['result'] else "✗"
                cond_name = cond['name'].replace("_", " ").title()
                col1_text += f"  {result_symbol} {cond_name}\n"
        else:
            col1_text += "  (Sin condiciones)\n"

        # ============= COLUMNA 2: PARÁMETROS CLAVE =============
        col2_text = "\n\n"  # Espaciado para alinear
        col2_text += "PARÁMETROS CLAVE:\n"

        if blackboard:
            # Distancias
            dist_ball = player.distance_to_ball(self.ball)
            col2_text += f"• Dist pelota: {dist_ball:.0f} px\n"

            # Umbrales de lógica difusa
            col2_text += f"• Posesión: {blackboard.posesion_pelota:.2f}"
            if blackboard.posesion_pelota < 0.3:
                col2_text += " [ALIADA]\n"
            elif blackboard.posesion_pelota > 0.7:
                col2_text += " [RIVAL]\n"
            else:
                col2_text += " [LIBRE]\n"

            col2_text += f"• Proximidad: {blackboard.proximidad_equipo:.2f}"
            if blackboard.proximidad_equipo < 0.8:
                col2_text += " [CERCA]\n"
            elif blackboard.proximidad_equipo > 1.2:
                col2_text += " [LEJOS]\n"
            else:
                col2_text += " [NEUTRAL]\n"

            col2_text += f"• Zona: {blackboard.zona_pelota:.2f}"
            if blackboard.zona_pelota < 0.4:
                col2_text += " [DEFENSIVA]\n"
            elif blackboard.zona_pelota > 1.6:
                col2_text += " [OFENSIVA]\n"
            else:
                col2_text += " [NEUTRAL]\n"

            # Ángulos
            angle_diff = player.angle_difference_ball(self.ball)
            angle_diff_degrees = np.degrees(angle_diff)  # Convertir radianes a grados
            col2_text += f"• Ángulo pelota: {angle_diff_degrees:.0f}°\n"

            # Estado
            col2_text += f"• Tiene pelota: {'SÍ' if player.has_ball() else 'NO'}\n"
        else:
            col2_text += "  (Sin parámetros)\n"

        # ============= COLUMNA 3: FLUJO DE DECISIONES =============
        col3_text = "\n\n"  # Espaciado para alinear
        col3_text += "FLUJO DE DECISIONES:\n"

        if self.tracer.trace:
            # Procesar traza con indentación según tipo de nodo
            indent_level = 0
            last_node_type = None

            for node in self.tracer.trace[-15:]:  # Últimas 15 entradas
                node_type = node['type']
                node_name = node['name']
                status = node['status']

                # Ajustar indentación según tipo de nodo
                if 'Selector' in node_type or 'Sequence' in node_type:
                    if last_node_type in ['ActionNode', 'ConditionNode']:
                        indent_level = max(0, indent_level - 1)
                    indent = "  " * indent_level
                    indent_level += 1
                elif 'Condition' in node_type or 'Action' in node_type:
                    indent = "  " * indent_level
                else:
                    indent = "  " * indent_level

                # Símbolos de estado
                status_symbols = {
                    NodeStatus.SUCCESS: "✓",
                    NodeStatus.FAILURE: "✗",
                    NodeStatus.RUNNING: "▶",
                    NodeStatus.INVALID: "?",
                }
                symbol = status_symbols.get(status, "?")

                # Formatear nombre del nodo
                formatted_name = node_name.replace("_", " ").title()

                # Tipo de nodo (simplificado)
                if 'Selector' in node_type:
                    type_tag = "[SEL]"
                elif 'Sequence' in node_type:
                    type_tag = "[SEQ]"
                elif 'Condition' in node_type:
                    type_tag = "[IF?]"
                elif 'Action' in node_type:
                    type_tag = "[DO!]"
                elif 'Inverter' in node_type:
                    type_tag = "[NOT]"
                else:
                    type_tag = "[---]"

                col3_text += f"{indent}{symbol} {type_tag} {formatted_name}\n"
                last_node_type = node_type
        else:
            col3_text += "No hay traza disponible.\n"
            col3_text += "Presiona 'Analizar' para ejecutar.\n"

        # ============= RENDERIZAR COLUMNAS =============
        # Columna 1: Condiciones Evaluadas
        self.details_ax.text(
            text_x_col1,
            text_y_position,
            col1_text,
            ha="left",
            va="top",
            transform=self.details_ax.transAxes,
            fontsize=text_fontsize,
            linespacing=text_linespacing,
            family=text_family,
        )

        # Columna 2: Parámetros Clave
        self.details_ax.text(
            text_x_col2,
            text_y_position,
            col2_text,
            ha="left",
            va="top",
            transform=self.details_ax.transAxes,
            fontsize=text_fontsize,
            linespacing=text_linespacing,
            family=text_family,
        )

        # Columna 3: Flujo de Decisiones
        self.details_ax.text(
            text_x_col3,
            text_y_position,
            col3_text,
            ha="left",
            va="top",
            transform=self.details_ax.transAxes,
            fontsize=text_fontsize,
            linespacing=text_linespacing,
            family=text_family,
        )

    def update_params_view(self):
        """Actualiza la vista de parámetros afinables del sistema"""
        self.params_ax.clear()
        self.params_ax.set_title(
            "Árbol de comportamiento",
            y=1.55,
            x=0.42,
            fontweight="bold",
            fontsize=10,
        )
        self.params_ax.axis("off")

        # Configuración de layout
        text_x_col1 = -0.15  # Columna izquierda
        text_x_col2 = 0.55  # Columna derecha
        text_y = 1.45
        fontsize = 9
        linespacing = 1.4

        # ============= COLUMNA IZQUIERDA =============
        params_text_col1 = "═══ UMBRALES CLAVE ═══\n\n"

        # GRUPO 1: Umbrales de Lógica Difusa
        params_text_col1 += "▶ Lógica Difusa:\n"
        params_text_col1 += "  Posesión Aliada: < 0.3\n"
        params_text_col1 += "  Posesión Rival:  > 0.7\n"
        params_text_col1 += "  Prox. Cerca:     < 0.8\n"
        params_text_col1 += "  Prox. Lejos:     > 1.2\n"
        params_text_col1 += "  Zona Defensiva:  < 0.4\n"
        params_text_col1 += "  Zona Ofensiva:   > 1.6\n\n"

        # GRUPO 2: Distancias de Acción
        params_text_col1 += "▶ Distancias (px):\n"
        params_text_col1 += "  Captura pelota:  50-60\n"
        params_text_col1 += "  Rango tiro:      < 400\n"
        params_text_col1 += "  Rango pase:      100-600\n"
        params_text_col1 += "  Aproximación:    25-50\n"

        # ============= COLUMNA DERECHA =============
        params_text_col2 = "\n\n"  # Espaciado para alinear con col1

        # GRUPO 3: Ángulos
        params_text_col2 += "▶ Ángulos:\n"
        params_text_col2 += "  Orientación OK:  < 45°\n"
        params_text_col2 += "  Ajuste rotación: > 10°\n"
        params_text_col2 += "  Ajuste pase:     > 15°\n\n"

        # GRUPO 4: Velocidades
        params_text_col2 += "▶ Velocidades:\n"
        params_text_col2 += "  Dribbling:       0.7x\n"
        params_text_col2 += "  Disparo:         0.8x\n"
        params_text_col2 += "  Intercepción:    1.5x\n"

        # Renderizar columna izquierda
        self.params_ax.text(
            text_x_col1,
            text_y,
            params_text_col1,
            ha="left",
            va="top",
            transform=self.params_ax.transAxes,
            fontsize=fontsize,
            linespacing=linespacing,
            family="monospace",
        )

        # Renderizar columna derecha
        self.params_ax.text(
            text_x_col2,
            text_y,
            params_text_col2,
            ha="left",
            va="top",
            transform=self.params_ax.transAxes,
            fontsize=fontsize,
            linespacing=linespacing,
            family="monospace",
        )

    def export_game_context(self, event=None):
        """Exporta el contexto completo del juego a un archivo de texto"""
        import datetime

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"game_context_{timestamp}.txt"

        try:
            with open(filename, 'w', encoding='utf-8') as f:
                f.write("=" * 80 + "\n")
                f.write("CONTEXTO DEL JUEGO - DEPURACIÓN ROBOCUP\n")
                f.write(f"Fecha: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write("=" * 80 + "\n\n")

                # ===== POSICIONES ACTUALES =====
                f.write("█ POSICIONES ACTUALES\n")
                f.write("-" * 80 + "\n")
                f.write(f"Pelota: X={self.ball.x:.1f}, Y={self.ball.y:.1f}\n")
                f.write(f"Velocidad Pelota: VX={self.ball_physics['velocity_x']:.2f}, VY={self.ball_physics['velocity_y']:.2f}\n\n")

                for player_id in [1, 2, 3, 4]:
                    player = self.players[player_id]
                    rol = "ATACANTE" if player.rol == ROL_ATACANTE else "DEFENSOR"
                    team = "ROJO" if player.team == "red" else "AZUL"
                    f.write(f"Robot {player_id} ({team} - {rol}):\n")
                    f.write(f"  Posición: X={player.x:.1f}, Y={player.y:.1f}, Ángulo={player.angle:.1f}°\n")
                    f.write(f"  Distancia a pelota: {player.distance_to_ball(self.ball):.1f} px\n")
                    angle_diff = player.angle_difference_ball(self.ball)
                    f.write(f"  Ángulo a pelota: {np.degrees(angle_diff):.1f}°\n")
                    f.write(f"  Tiene pelota: {'SÍ' if player.has_ball() else 'NO'}\n\n")

                # ===== LÓGICA DIFUSA =====
                f.write("\n" + "█" * 80 + "\n")
                f.write("█ LÓGICA DIFUSA - PARÁMETROS ACTUALES\n")
                f.write("█" * 80 + "\n\n")

                for player_id in [1, 2, 3, 4]:
                    player = self.players[player_id]
                    team = "red" if player_id <= 2 else "blue"
                    manager = self.team_managers[team]

                    if player_id in manager.blackboards:
                        blackboard = manager.blackboards[player_id]
                        f.write(f"Robot {player_id}:\n")
                        f.write(f"  • Posesión pelota: {blackboard.posesion_pelota:.3f}")
                        if blackboard.posesion_pelota < 0.3:
                            f.write(" [ALIADA]\n")
                        elif blackboard.posesion_pelota > 0.7:
                            f.write(" [RIVAL]\n")
                        else:
                            f.write(" [LIBRE]\n")

                        f.write(f"  • Proximidad equipo: {blackboard.proximidad_equipo:.3f}")
                        if blackboard.proximidad_equipo < 0.8:
                            f.write(" [CERCA]\n")
                        elif blackboard.proximidad_equipo > 1.2:
                            f.write(" [LEJOS]\n")
                        else:
                            f.write(" [NEUTRAL]\n")

                        f.write(f"  • Zona pelota: {blackboard.zona_pelota:.3f}")
                        if blackboard.zona_pelota < 0.4:
                            f.write(" [DEFENSIVA]\n")
                        elif blackboard.zona_pelota > 1.6:
                            f.write(" [OFENSIVA]\n")
                        else:
                            f.write(" [NEUTRAL]\n")
                        f.write("\n")

                # ===== UMBRALES Y PARÁMETROS =====
                f.write("\n" + "█" * 80 + "\n")
                f.write("█ UMBRALES Y PARÁMETROS DEL SISTEMA\n")
                f.write("█" * 80 + "\n\n")

                f.write("▶ LÓGICA DIFUSA:\n")
                f.write("  • Posesión Aliada:  < 0.3\n")
                f.write("  • Posesión Rival:   > 0.7\n")
                f.write("  • Proximidad Cerca: < 0.8\n")
                f.write("  • Proximidad Lejos: > 1.2\n")
                f.write("  • Zona Defensiva:   < 0.4\n")
                f.write("  • Zona Ofensiva:    > 1.6\n\n")

                f.write("▶ DISTANCIAS (píxeles):\n")
                f.write("  • Captura pelota:   50-60 px\n")
                f.write("  • Rango de tiro:    < 400 px\n")
                f.write("  • Rango de pase:    100-600 px\n")
                f.write("  • Aproximación:     25-50 px\n\n")

                f.write("▶ ÁNGULOS:\n")
                f.write("  • Orientación OK:   < 45°\n")
                f.write("  • Ajuste rotación:  > 10°\n")
                f.write("  • Ajuste pase:      > 15°\n\n")

                f.write("▶ VELOCIDADES:\n")
                f.write("  • Dribbling:        0.7x\n")
                f.write("  • Disparo:          0.8x\n")
                f.write("  • Intercepción:     1.5x\n\n")

                # ===== ÁRBOL DE DECISIONES =====
                f.write("\n" + "█" * 80 + "\n")
                f.write("█ ÁRBOL DE DECISIONES - TRAZA DE EJECUCIÓN\n")
                f.write("█" * 80 + "\n\n")

                if self.tracer.trace:
                    f.write("Últimas 20 decisiones del árbol:\n\n")
                    for idx, node in enumerate(self.tracer.trace[-20:], 1):
                        status_symbols = {
                            NodeStatus.SUCCESS: "✓",
                            NodeStatus.FAILURE: "✗",
                            NodeStatus.RUNNING: "▶",
                            NodeStatus.INVALID: "?",
                        }
                        symbol = status_symbols.get(node['status'], "?")
                        node_type = node['type']
                        node_name = node['name'].replace("_", " ").title()

                        f.write(f"{idx:2d}. [{symbol}] {node_type:20s} - {node_name}\n")
                else:
                    f.write("No hay traza disponible. Ejecuta 'Analizar' primero.\n")

                # ===== CONDICIONES EVALUADAS =====
                if self.tracer.conditions_met:
                    f.write("\n\nCondiciones evaluadas recientemente:\n\n")
                    for cond in self.tracer.conditions_met[-10:]:
                        result = "✓ TRUE " if cond['result'] else "✗ FALSE"
                        cond_name = cond['name'].replace("_", " ").title()
                        f.write(f"  {result} - {cond_name}\n")

                # ===== ACCIONES PLANIFICADAS =====
                f.write("\n\n" + "█" * 80 + "\n")
                f.write("█ ACCIONES PLANIFICADAS\n")
                f.write("█" * 80 + "\n\n")

                for player_id in [1, 2, 3, 4]:
                    if player_id in self.tracer_ext.robot_actions:
                        action = self.tracer_ext.robot_actions[player_id]
                        f.write(f"Robot {player_id}: {action.replace('_', ' ').title()}\n")

                        if player_id in self.tracer_ext.planned_movements:
                            movement = self.tracer_ext.planned_movements[player_id]
                            target = movement['target_pos']
                            f.write(f"  → Destino: ({target[0]:.1f}, {target[1]:.1f})\n")
                    else:
                        f.write(f"Robot {player_id}: Sin acción asignada\n")

                f.write("\n" + "=" * 80 + "\n")
                f.write("FIN DEL REPORTE\n")
                f.write("=" * 80 + "\n")

            log.info(f"✅ Contexto exportado exitosamente: {filename}")
            print(f"\n✅ Contexto del juego exportado a: {filename}")

        except Exception as e:
            log.error(f"❌ Error al exportar contexto: {e}")
            print(f"\n❌ Error al exportar: {e}")

    @staticmethod
    def run():
        """Inicia la herramienta de depuración"""
        plt.subplots_adjust(left=0.12, right=0.95, top=0.95, bottom=0.05)
        plt.show()

    # NUEVOS métodos para agregar:
    def toggle_simulation(self, event):
        """Activa/desactiva simulación de física"""
        if self.simulation_active:
            self.stop_simulation()
        else:
            self.start_simulation()

    def start_simulation(self):
        """Inicia la simulación continua"""
        self.simulation_active = True
        self.sim_button.label.set_text("|| Parar")

        # Timer que actualiza cada 50ms (20 FPS)
        self.physics_timer = self.fig.canvas.new_timer(interval=50)
        self.physics_timer.add_callback(self.update_physics)
        self.physics_timer.start()

        log.info("Simulación iniciada")

    def stop_simulation(self):
        """Para la simulación"""
        self.simulation_active = False
        self.sim_button.label.set_text("▶ Simular")

        if self.physics_timer:
            self.physics_timer.stop()
            self.physics_timer = None

        log.info("⏹ Simulación detenida")

    def update_physics(self):
        """Actualiza solo la posición de la pelota existente"""
        if not self.simulation_active:
            return

        # Aplicar movimiento simple
        if abs(self.ball_physics['velocity_x']) > self.ball_physics['min_speed'] or \
           abs(self.ball_physics['velocity_y']) > self.ball_physics['min_speed']:

            # Calcular nueva posición
            new_x = self.ball.x + self.ball_physics['velocity_x']
            new_y = self.ball.y + self.ball_physics['velocity_y']

            # Mantener dentro del campo
            new_x = max(30, min(ANCHO_CAMPO - 30, new_x))
            new_y = max(30, min(ALTO_CAMPO - 30, new_y))

            # Actualizar la pelota (esto actualiza automáticamente ball.speed por tu nueva clase)
            self.ball.set_position(new_x, new_y)

            # Actualizar SOLO los elementos visuales existentes
            self.ball_circle.center = (new_x, new_y)  # Mover círculo existente
            self.ball_label.set_position((new_x, new_y - 45))  # Mover etiqueta existente

            # Aplicar fricción simple
            self.ball_physics['velocity_x'] *= self.ball_physics['friction']
            self.ball_physics['velocity_y'] *= self.ball_physics['friction']

            # Refrescar SOLO el gráfico (sin crear nada nuevo)
            self.fig.canvas.draw_idle()

            # Analizar comportamientos (opcional, cada ciertos frames)
            if hasattr(self, '_physics_counter'):
                self._physics_counter += 1
            else:
                self._physics_counter = 0

            # Solo analizar cada 5 frames para no saturar
            if self._physics_counter % 5 == 0:
                self.analyze_behaviors()
        else:
            # Pelota se detuvo
            self.stop_simulation()
            log.info("⏹ Pelota detenida - simulación parada")

    def handle_wall_bounces(self, new_x, new_y):
        """Maneja rebotes con las paredes del campo"""
        bounce_factor = 0.8  # La pelota pierde energía al rebotar

        # Rebote en paredes verticales (izquierda/derecha)
        if new_x <= 30:  # Borde izquierdo
            new_x = 30
            self.ball_physics['velocity_x'] = -self.ball_physics['velocity_x'] * bounce_factor
        elif new_x >= ANCHO_CAMPO - 30:  # Borde derecho
            new_x = ANCHO_CAMPO - 30
            self.ball_physics['velocity_x'] = -self.ball_physics['velocity_x'] * bounce_factor

        # Rebote en paredes horizontales (arriba/abajo)
        if new_y <= 30:  # Borde superior
            new_y = 30
            self.ball_physics['velocity_y'] = -self.ball_physics['velocity_y'] * bounce_factor
        elif new_y >= ALTO_CAMPO - 30:  # Borde inferior
            new_y = ALTO_CAMPO - 30
            self.ball_physics['velocity_y'] = -self.ball_physics['velocity_y'] * bounce_factor

        return new_x, new_y

    # def check_robot_kicks(self, ball_x, ball_y):
    #     """Verifica si algún robot está cerca y puede 'patear' la pelota"""
    #     kick_distance = 50  # Distancia para que un robot pueda patear
    #     kick_strength = 40  # Fuerza de la patada
    #
    #     for player_id, player in self.players.items():
    #         distance = np.sqrt((player.x - ball_x)**2 + (player.y - ball_y)**2)
    #
    #         if distance < kick_distance:
    #             # Calcular dirección de la patada (robot hacia pelota)
    #             angle_to_ball = np.arctan2(ball_y - player.y, ball_x - player.x)
    #
    #             # Añadir algo de aleatoriedad a la dirección
    #             angle_variation = (np.random.random() - 0.5) * 0.3  # ±0.15 radianes
    #             kick_angle = angle_to_ball + angle_variation
    #
    #             # Aplicar velocidad en esa dirección
    #             self.ball_physics['velocity_x'] += kick_strength * np.cos(kick_angle)
    #             self.ball_physics['velocity_y'] += kick_strength * np.sin(kick_angle)
    #
    #             print(f"Robot {player_id} pateó la pelota!")
    #             break  # Solo un robot puede patear por frame

    # def simulate_kick(self, event):
    #     """Simula una patada específica para testing"""
    #     # Encontrar el robot más cercano a la pelota
    #     closest_distance = float('inf')
    #     closest_robot = None
    #
    #     for player_id, player in self.players.items():
    #         distance = np.sqrt((player.x - self.ball.x)**2 + (player.y - self.ball.y)**2)
    #         if distance < closest_distance:
    #             closest_distance = distance
    #             closest_robot = player
    #
    #     if closest_robot and closest_distance < 100:
    #         # Calcular dirección hacia la portería rival
    #         target_goal_x = ANCHO_CAMPO if closest_robot.team == "red" else 0
    #         angle_to_goal = np.arctan2(ALTO_CAMPO/2 - self.ball.y, target_goal_x - self.ball.x)
    #
    #         # Aplicar patada hacia la portería
    #         kick_power = 60
    #         self.ball_physics['velocity_x'] = kick_power * np.cos(angle_to_goal)
    #         self.ball_physics['velocity_y'] = kick_power * np.sin(angle_to_goal)
    #
    #         print(f"{closest_robot.team.upper()} pateó hacia portería!")
    #
    #         # Iniciar simulación si no está activa
    #         if not self.simulation_active:
    #             self.start_simulation()
    #     else:
    #         print("No hay robots cerca de la pelota")

    def update_velocity_arrow(self):
        """Actualiza la flecha que muestra la velocidad de la pelota"""
        if self.ball_velocity_arrow is None:
            # Crear flecha inicial (oculta)
            self.ball_velocity_arrow = FancyArrowPatch(
                (0, 0), (0, 0),
                arrowstyle="->",
                color="red",
                linewidth=3,
                mutation_scale=25,
                visible=False
            )
            self.field_ax.add_patch(self.ball_velocity_arrow)

        # Mostrar flecha solo si hay velocidad significativa
        speed = np.sqrt(self.ball_physics['velocity_x']**2 + self.ball_physics['velocity_y']**2)

        if speed > 5:
            # Escalar la flecha según la velocidad (máximo 100 píxeles)
            scale_factor = min(speed * 2, 100)
            arrow_end_x = self.ball.x + (self.ball_physics['velocity_x'] / speed) * scale_factor
            arrow_end_y = self.ball.y + (self.ball_physics['velocity_y'] / speed) * scale_factor

            self.ball_velocity_arrow.set_positions((self.ball.x, self.ball.y),
                                                   (arrow_end_x, arrow_end_y))
            self.ball_velocity_arrow.set_visible(True)

            # Actualizar label de la pelota con velocidad
            speed_text = f"Pelota\n{speed:.1f} px/s"
            self.ball_label.set_text(speed_text)
        else:
            self.ball_velocity_arrow.set_visible(False)
            self.ball_label.set_text("Pelota")

    def update_ball_visuals(self):
        """Actualiza solo los elementos visuales de la pelota"""
        self.ball_circle.center = (self.ball.x, self.ball.y)
        self.ball_label.set_position((self.ball.x, self.ball.y - 45))
        self.fig.canvas.draw_idle()


if __name__ == "__main__":
    try:
        debugger = ImprovedBehaviorDebugger()
        debugger.run()
    except KeyboardInterrupt:
        print("\nDepurador cerrado por el usuario")
    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()
