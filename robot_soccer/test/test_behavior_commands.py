#!/usr/bin/env python3
"""
Script de prueba para verificar el funcionamiento de los árboles de comportamiento
y los comandos generados para los robots en un entorno controlado.
"""

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import numpy as np
import time
import threading
import sys
import logging
from robot_soccer.entities.player import Player
from robot_soccer.entities.ball import Ball
from robot_soccer.ai.fuzzy_logic.game_context import FuzzyRobotTeamManager
from robot_soccer.ai.behavior_tree.manager import BehaviorManager
from robot_soccer.config import *

# Configuración de logging para ver todos los comandos generados
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        # logging.FileHandler('behavior_test.log')
    ]
)


class CommandRecorder:
    """
    Clase para registrar y visualizar los comandos enviados a los robots.
    Reemplaza la comunicación real para pruebas.
    """

    def __init__(self):
        self.commands = []
        self.logger = logging.getLogger("command_recorder")

    def record_command(self, robot_id, command_type, **params):
        """Registra un comando para un robot."""
        command = {
            'timestamp': time.time(),
            'robot_id': robot_id,
            'type': command_type,
            'params': params
        }
        self.commands.append(command)
        self.logger.info(f"Robot {robot_id}: {command_type} - {params}")

    def get_commands_for_robot(self, robot_id):
        """Devuelve todos los comandos registrados para un robot específico."""
        return [cmd for cmd in self.commands if cmd['robot_id'] == robot_id]

    def clear(self):
        """Limpia el historial de comandos."""
        self.commands = []

    def print_summary(self):
        """Imprime un resumen de los comandos registrados."""
        if not self.commands:
            print("No se han registrado comandos.")
            return

        print("\n=== RESUMEN DE COMANDOS ===")
        robots = set(cmd['robot_id'] for cmd in self.commands)

        for robot_id in sorted(robots):
            robot_commands = self.get_commands_for_robot(robot_id)
            print(f"\nRobot {robot_id}: {len(robot_commands)} comandos")

            # Contar tipos de comandos
            command_types = {}
            for cmd in robot_commands:
                cmd_type = cmd['type']
                command_types[cmd_type] = command_types.get(cmd_type, 0) + 1

            # Mostrar conteo por tipo
            for cmd_type, count in command_types.items():
                print(f"  - {cmd_type}: {count}")

            # Mostrar los últimos 3 comandos
            if robot_commands:
                print(f"  Últimos comandos:")
                for cmd in robot_commands[-3:]:
                    params_str = ', '.join(f"{k}={v}" for k, v in cmd['params'].items())
                    print(f"    - {cmd['type']}: {params_str}")


# Sobrescribir el método _send_motor_commands en DifferentialDriveController
# para que registre los comandos en lugar de enviarlos
from robot_soccer.controllers.differential_drive import DifferentialDriveController

original_send_motor_commands = DifferentialDriveController._send_motor_commands
command_recorder = CommandRecorder()


def mock_send_motor_commands(self, robot, left_speed, right_speed):
    """Versión de prueba que registra los comandos en lugar de enviarlos."""
    # Registrar el comando
    command_recorder.record_command(
        robot.id,
        'motors',
        left_speed=round(left_speed, 2),
        right_speed=round(right_speed, 2)
    )

    # Llamar al método original para actualizar la simulación
    original_send_motor_commands(self, robot, left_speed, right_speed)


# Sobrescribir el método
DifferentialDriveController._send_motor_commands = mock_send_motor_commands

# También necesitamos sobrescribir otros métodos que enviarían comandos
from robot_soccer.controllers.robot_command_manager import RobotCommandManager

original_kick_ball = RobotCommandManager._execute_kick_ball


def mock_execute_kick_ball(self, player, target_pos, ball, power):
    """Versión de prueba que registra la acción de pateo."""
    command_recorder.record_command(
        player.id,
        'kick',
        target_x=round(target_pos[0], 2),
        target_y=round(target_pos[1], 2),
        power=round(power, 2)
    )

    # Llamar al método original para la simulación
    return original_kick_ball(self, player, target_pos, ball, power)


RobotCommandManager._execute_kick_ball = mock_execute_kick_ball


# Sobrescribir el método para el dribbler
def mock_set_dribbler(self, robot_id, power):
    """Versión de prueba que registra la acción del dribbler."""
    command_recorder.record_command(
        robot_id,
        'dribbler',
        power=round(power, 2)
    )

    # No hay un método original para llamar en este caso
    return True


# Añadir este método a RobotCommandManager si no existe
if not hasattr(RobotCommandManager, 'set_dribbler'):
    RobotCommandManager.set_dribbler = mock_set_dribbler


class BehaviorTester:
    """
    Clase para probar los árboles de comportamiento en escenarios controlados.
    """

    def __init__(self):
        # Crear campo y objetos
        self.setup_field()

        # Inicializar gestores de comportamiento
        self.setup_behavior_managers()

        # Configuración para pruebas
        self.test_scenarios = [
            self.test_scenario_1,
            self.test_scenario_2,
            self.test_scenario_3,
            self.test_scenario_4,
            self.test_scenario_5
        ]
        self.current_test = 0

        # Configurar interfaz gráfica
        self.setup_ui()

    def setup_field(self):
        """Configura el campo y los objetos del juego."""
        # Posiciones iniciales
        self.ball = Ball(750, 450)
        self.player_1 = Player(1, 200, 200, 90, 'red')
        self.player_2 = Player(2, 200, 700, 180, 'red')
        self.player_3 = Player(3, 1200, 200, 270, 'blue')
        self.player_4 = Player(4, 1200, 700, 0, 'blue')

        # Lista de todos los jugadores
        self.all_players = [self.player_1, self.player_2, self.player_3, self.player_4]

        # Configurar roles iniciales
        self.player_1.set_rol(ROL_ATACANTE)
        self.player_2.set_rol(ROL_DEFENSIVO)
        self.player_3.set_rol(ROL_ATACANTE)
        self.player_4.set_rol(ROL_DEFENSIVO)

    def setup_behavior_managers(self):
        """Inicializa los gestores de comportamiento para ambos equipos."""
        # Crear gestores de contexto del juego
        self.context_red = FuzzyRobotTeamManager(
            self.all_players, self.ball, team='red'
        )
        self.context_blue = FuzzyRobotTeamManager(
            self.all_players, self.ball, team='blue'
        )

        # Crear gestores de comportamiento
        self.behavior_red = BehaviorManager(
            self.all_players, self.ball, team='red'
        )
        self.behavior_blue = BehaviorManager(
            self.all_players, self.ball, team='blue'
        )

    def setup_ui(self):
        """Configura la interfaz gráfica para visualización."""
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(18, 8))

        # Configurar gráfico del campo
        self.ax1.set_xlim(0, ANCHO_CAMPO)
        self.ax1.set_ylim(0, ALTO_CAMPO)
        self.ax1.set_title("Campo de Juego")
        self.ax1.grid(True)

        # Dibujar líneas del campo
        self.ax1.plot([0, 0, ANCHO_CAMPO, ANCHO_CAMPO, 0],
                      [0, ALTO_CAMPO, ALTO_CAMPO, 0, 0], 'k-', lw=2)
        self.ax1.axvline(ANCHO_CAMPO * 0.3, color='gray', linestyle='--')
        self.ax1.axvline(ANCHO_CAMPO * 0.7, color='gray', linestyle='--')

        # Inicializar objetos visuales
        self.ball_circle = Circle((self.ball.x, self.ball.y), 15, color='orange')
        self.ax1.add_patch(self.ball_circle)

        self.player_circles = []
        self.player_directions = []

        for player in self.all_players:
            if player.team == 'red':
                color = 'red'
            else:
                color = 'blue'

            circle = Circle((player.x, player.y), 30, color=color, alpha=0.7)
            self.ax1.add_patch(circle)
            self.player_circles.append(circle)

            # Añadir línea de dirección
            line, = self.ax1.plot([player.x, player.x + 50 * np.cos(np.radians(player.angle))],
                                  [player.y, player.y + 50 * np.sin(np.radians(player.angle))],
                                  color='black', lw=2)
            self.player_directions.append(line)

            # Añadir texto con ID
            self.ax1.text(player.x, player.y, str(player.id),
                          ha='center', va='center', color='white')

        # Configurar gráfico de registro de comandos
        self.ax2.set_title("Registro de Comandos")
        self.ax2.axis('off')
        self.command_text = self.ax2.text(0.05, 0.95, "Esperando comandos...",
                                          va='top', ha='left', fontsize=10,
                                          transform=self.ax2.transAxes)

        # Botones de control
        self.run_button_ax = plt.axes([0.7, 0.05, 0.1, 0.04])
        self.run_button = plt.Button(self.run_button_ax, 'Ejecutar prueba')
        self.run_button.on_clicked(self.run_current_test)

        self.next_button_ax = plt.axes([0.85, 0.05, 0.1, 0.04])
        self.next_button = plt.Button(self.next_button_ax, 'Siguiente')
        self.next_button.on_clicked(self.next_test)

    def update_visualizations(self):
        """Actualiza las visualizaciones con las posiciones actuales."""
        # Actualizar posición de la pelota
        self.ball_circle.center = (self.ball.x, self.ball.y)

        # Actualizar posiciones de los jugadores
        for i, player in enumerate(self.all_players):
            self.player_circles[i].center = (player.x, player.y)
            # Actualizar dirección
            angle_rad = np.radians(player.angle)
            self.player_directions[i].set_data(
                [player.x, player.x + 50 * np.cos(angle_rad)],
                [player.y, player.y + 50 * np.sin(angle_rad)]
            )

        # Actualizar registro de comandos
        commands_text = "Últimos comandos:\n\n"

        # Mostrar los últimos 10 comandos
        last_commands = command_recorder.commands[-10:] if command_recorder.commands else []

        for cmd in last_commands:
            robot_id = cmd['robot_id']
            cmd_type = cmd['type']
            params_str = ', '.join(f"{k}={v}" for k, v in cmd['params'].items())

            # Colorear según equipo
            if robot_id in [1, 2]:
                color = "red"
            else:
                color = "blue"

            commands_text += f"Robot {robot_id} ({color}): {cmd_type} - {params_str}\n"

        self.command_text.set_text(commands_text)

        # Actualizar canvas
        self.fig.canvas.draw_idle()

    def run_simulation_step(self):
        """Ejecuta un paso de la simulación."""
        # Evaluar contexto del juego
        red_context = self.context_red.evaluar_msLogicDifusse()
        blue_context = self.context_blue.evaluar_msLogicDifusse()

        # Actualizar contexto en los gestores de comportamiento
        self.behavior_red.update_game_context(red_context)
        self.behavior_blue.update_game_context(blue_context)

        # Ejecutar árboles de comportamiento
        self.behavior_red.update()
        self.behavior_blue.update()

        # Actualizar visualizaciones
        self.update_visualizations()

    def run_current_test(self, event=None):
        """Ejecuta el escenario de prueba actual."""
        if 0 <= self.current_test < len(self.test_scenarios):
            # Limpiar registro de comandos
            command_recorder.clear()

            # Ejecutar el escenario de prueba
            test_func = self.test_scenarios[self.current_test]
            test_func()

            # Imprimir resumen
            command_recorder.print_summary()
        else:
            print("No hay más escenarios de prueba.")

    def next_test(self, event=None):
        """Avanza al siguiente escenario de prueba."""
        self.current_test = (self.current_test + 1) % len(self.test_scenarios)
        print(f"Seleccionado escenario {self.current_test + 1}")

    # Escenarios de prueba
    def test_scenario_1(self):
        """Escenario 1: Pelota libre en medio campo, jugadores alejados."""
        print("\n=== ESCENARIO 1: Pelota libre en medio campo ===")
        # Posicionar objetos
        self.ball.set_position(750, 450)
        self.player_1.set_position(300, 200)
        self.player_1.set_angle(90)
        self.player_2.set_position(200, 700)
        self.player_2.set_angle(180)
        self.player_3.set_position(1200, 200)
        self.player_3.set_angle(270)
        self.player_4.set_position(1200, 700)
        self.player_4.set_angle(0)

        # Ejecutar varios pasos de simulación
        for _ in range(10):
            self.run_simulation_step()
            time.sleep(0.5)

    def test_scenario_2(self):
        """Escenario 2: Jugador rojo cercano a la pelota (captura)."""
        print("\n=== ESCENARIO 2: Jugador rojo cercano a la pelota (captura) ===")
        # Posicionar objetos
        self.ball.set_position(500, 400)
        self.player_1.set_position(560, 400)  # Cercano a la pelota
        self.player_1.set_angle(180)  # Mirando hacia la pelota
        self.player_2.set_position(200, 700)
        self.player_3.set_position(1200, 200)
        self.player_4.set_position(1200, 700)

        # Ejecutar varios pasos de simulación
        for i in range(15):
            # Acercar gradualmente para ver la captura
            if i < 5:
                self.player_1.set_position(self.player_1.x - 10, self.player_1.y)

            self.run_simulation_step()
            time.sleep(0.5)

    def test_scenario_3(self):
        """Escenario 3: Jugador rojo con la pelota (tiro a portería)."""
        print("\n=== ESCENARIO 3: Jugador rojo con la pelota (tiro a portería) ===")
        # Posicionar objetos
        self.ball.set_position(1200, 450)  # Cerca de la portería rival
        self.player_1.set_position(1180, 450)  # Junto a la pelota
        self.player_1.set_angle(0)  # Mirando hacia la portería
        self.player_1.ball_hold = True  # Tiene la pelota
        self.player_2.set_position(800, 700)
        self.player_3.set_position(1300, 400)
        self.player_4.set_position(1300, 500)

        # Ejecutar varios pasos de simulación
        for _ in range(10):
            self.run_simulation_step()
            time.sleep(0.5)

    def test_scenario_4(self):
        """Escenario 4: Defensa (jugador azul atacando)."""
        print("\n=== ESCENARIO 4: Defensa (jugador azul atacando) ===")
        # Posicionar objetos
        self.ball.set_position(300, 450)  # Pelota en zona defensiva roja
        self.player_3.set_position(320, 450)  # Rival cerca de la pelota
        self.player_3.set_angle(180)  # Mirando hacia la portería
        self.player_3.ball_hold = True  # Tiene la pelota
        self.player_1.set_position(500, 400)
        self.player_2.set_position(200, 450)  # Defensor cerca de la portería
        self.player_4.set_position(1200, 700)

        # Ejecutar varios pasos de simulación
        for _ in range(10):
            self.run_simulation_step()
            time.sleep(0.5)

    def test_scenario_5(self):
        """Escenario 5: Cambio dinámico de roles (pelota moviéndose)."""
        print("\n=== ESCENARIO 5: Cambio dinámico de roles (pelota moviéndose) ===")
        # Posicionar objetos inicialmente
        self.ball.set_position(500, 450)
        self.player_1.set_position(600, 400)
        self.player_2.set_position(300, 500)
        self.player_3.set_position(900, 400)
        self.player_4.set_position(1100, 500)

        # Mover la pelota gradualmente hacia el lado azul
        for i in range(10):
            # Mover la pelota
            new_x = 500 + i * 70
            self.ball.set_position(new_x, 450)

            # Ejecutar simulación
            self.run_simulation_step()
            time.sleep(0.5)

    def run(self):
        """Inicia la prueba interactiva."""
        plt.tight_layout()
        plt.show()


# Ejecutar pruebas
if __name__ == "__main__":
    tester = BehaviorTester()
    tester.run()