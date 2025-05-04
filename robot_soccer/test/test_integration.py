#!/usr/bin/env python3
"""
Prueba de integración completa del sistema de control de robots.
Conecta los árboles de comportamiento con la comunicación real al Arduino.
"""

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import numpy as np
import time
import threading
import sys
import logging
import argparse
from robot_soccer.entities.player import Player
from robot_soccer.entities.ball import Ball
from robot_soccer.ai.fuzzy_logic.game_context import FuzzyRobotTeamManager
from robot_soccer.ai.behavior_tree.manager import BehaviorManager
from robot_soccer.config import *
from robot_soccer.communication.serial_manager import SerialManager
from robot_soccer.communication.command_protocol import RobotCommandProtocol

# Configuración de logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        # logging.FileHandler('integration_test.log')
    ]
)

logger = logging.getLogger("integration_test")


class IntegrationTester:
    """
    Clase para probar la integración completa del sistema de control de robots.
    """

    def __init__(self, serial_port=None, use_arduino=False, show_visualization=True):
        """
        Inicializa el entorno de prueba.

        Args:
            serial_port: Puerto serial para comunicarse con Arduino (si use_arduino=True)
            use_arduino: Si True, envía comandos reales al Arduino
            show_visualization: Si True, muestra visualización gráfica
        """
        self.use_arduino = use_arduino
        self.show_visualization = show_visualization
        self.is_running = False
        self.simulation_thread = None

        # Inicializar objetos del juego
        self.setup_game_objects()

        # Inicializar comunicación con Arduino si es necesario
        self.serial_manager = None
        if use_arduino and serial_port:
            self.serial_manager = SerialManager(serial_port)
            if not self.serial_manager.connect():
                logger.error("No se pudo conectar con Arduino. Usando modo simulación.")
                self.use_arduino = False
                self.serial_manager = None

        # Configurar gestores de comportamiento con conexión a Arduino
        self.setup_behavior_managers()

        # Configurar visualización si está activada
        if show_visualization:
            self.setup_visualization()

    def setup_game_objects(self):
        """Configura los objetos del juego (jugadores y pelota)."""
        # Crear pelota
        self.ball = Ball(750, 450)

        # Crear jugadores
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
        """Configura los gestores de comportamiento con conexión a Arduino si está disponible."""
        # Crear gestores de contexto del juego
        self.context_red = FuzzyRobotTeamManager(
            self.all_players, self.ball, team='red'
        )
        self.context_blue = FuzzyRobotTeamManager(
            self.all_players, self.ball, team='blue'
        )

        # Crear gestores de comportamiento con acceso a comunicación
        self.behavior_red = BehaviorManager(
            self.all_players,
            self.ball,
            team='red',
            use_real_robots=self.use_arduino,
            serial_port=self.serial_manager.port if self.serial_manager else None
        )

        self.behavior_blue = BehaviorManager(
            self.all_players,
            self.ball,
            team='blue',
            use_real_robots=self.use_arduino,
            serial_port=self.serial_manager.port if self.serial_manager else None
        )

    def setup_visualization(self):
        """Configura la visualización gráfica."""
        self.fig, self.ax = plt.subplots(figsize=(10, 8))

        # Configurar gráfico del campo
        self.ax.set_xlim(0, ANCHO_CAMPO)
        self.ax.set_ylim(0, ALTO_CAMPO)
        self.ax.set_title("Prueba de Integración")
        self.ax.grid(True)

        # Dibujar líneas del campo
        self.ax.plot([0, 0, ANCHO_CAMPO, ANCHO_CAMPO, 0],
                     [0, ALTO_CAMPO, ALTO_CAMPO, 0, 0], 'k-', lw=2)
        self.ax.axvline(ANCHO_CAMPO * 0.3, color='gray', linestyle='--')
        self.ax.axvline(ANCHO_CAMPO * 0.7, color='gray', linestyle='--')

        # Inicializar objetos visuales
        self.ball_circle = Circle((self.ball.x, self.ball.y), 15, color='orange')
        self.ax.add_patch(self.ball_circle)

        self.player_circles = []
        self.player_directions = []

        for player in self.all_players:
            if player.team == 'red':
                color = 'red'
            else:
                color = 'blue'

            circle = Circle((player.x, player.y), 30, color=color, alpha=0.7)
            self.ax.add_patch(circle)
            self.player_circles.append(circle)

            # Añadir línea de dirección
            line, = self.ax.plot([player.x, player.x + 50 * np.cos(np.radians(player.angle))],
                                 [player.y, player.y + 50 * np.sin(np.radians(player.angle))],
                                 color='black', lw=2)
            self.player_directions.append(line)

            # Añadir texto con ID y rol
            text = self.ax.text(player.x, player.y + 40, f"{player.id} ({player.rol})",
                                ha='center', va='center', color='black')

            # Añadir al jugador para fácil seguimiento
            player._circle = circle
            player._direction = line
            player._text = text

        # Botones de control
        self.scenario_1_button_ax = plt.axes([0.15, 0.02, 0.15, 0.04])
        self.scenario_1_button = plt.Button(self.scenario_1_button_ax, 'Escenario 1')
        self.scenario_1_button.on_clicked(lambda e: self.run_scenario_1())

        self.scenario_2_button_ax = plt.axes([0.35, 0.02, 0.15, 0.04])
        self.scenario_2_button = plt.Button(self.scenario_2_button_ax, 'Escenario 2')
        self.scenario_2_button.on_clicked(lambda e: self.run_scenario_2())

        self.scenario_3_button_ax = plt.axes([0.55, 0.02, 0.15, 0.04])
        self.scenario_3_button = plt.Button(self.scenario_3_button_ax, 'Escenario 3')
        self.scenario_3_button.on_clicked(lambda e: self.run_scenario_3())

        self.stop_button_ax = plt.axes([0.75, 0.02, 0.15, 0.04])
        self.stop_button = plt.Button(self.stop_button_ax, 'DETENER')
        self.stop_button.on_clicked(lambda e: self.stop_simulation())

    def update_visualization(self):
        """Actualiza la visualización gráfica con las posiciones actuales."""
        if not self.show_visualization:
            return

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

            # Actualizar texto con información del rol
            player._text.set_position((player.x, player.y + 40))
            player._text.set_text(f"{player.id} ({player.rol})")

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
        self.update_visualization()

    def simulation_loop(self, duration=30, update_interval=0.1):
        """
        Bucle principal de simulación.

        Args:
            duration: Duración máxima en segundos (None para ejecutar indefinidamente)
            update_interval: Intervalo entre actualizaciones en segundos
        """
        start_time = time.time()

        try:
            while self.is_running:
                # Verificar si se ha alcanzado la duración máxima
                if duration is not None and time.time() - start_time > duration:
                    logger.info(f"Simulación completada (duración: {duration}s)")
                    break

                # Ejecutar un paso de simulación
                self.run_simulation_step()

                # Esperar antes de la siguiente actualización
                time.sleep(update_interval)

        except Exception as e:
            logger.error(f"Error en simulación: {e}")

        finally:
            self.is_running = False

    def start_simulation(self, duration=None, update_interval=0.1):
        """
        Inicia la simulación en un hilo separado.

        Args:
            duration: Duración máxima en segundos (None para ejecutar indefinidamente)
            update_interval: Intervalo entre actualizaciones en segundos
        """
        if self.is_running:
            logger.warning("La simulación ya está en ejecución")
            return

        # Marcar como en ejecución
        self.is_running = True

        # Iniciar hilo de simulación
        self.simulation_thread = threading.Thread(
            target=self.simulation_loop,
            args=(duration, update_interval)
        )
        self.simulation_thread.daemon = True
        self.simulation_thread.start()

        logger.info("Simulación iniciada")

    def stop_simulation(self):
        """Detiene la simulación."""
        self.is_running = False

        if self.simulation_thread:
            self.simulation_thread.join(timeout=1)
            self.simulation_thread = None

        # Detener todos los robots si se está usando Arduino
        if self.use_arduino and self.serial_manager:
            for player in self.all_players:
                command = RobotCommandProtocol.format_stop_command(player.id)
                self.serial_manager.send_command(command)

        logger.info("Simulación detenida")

    def run_scenario_1(self):
        """
        Escenario 1: Pelota libre en medio campo, jugadores alejados.
        """
        # Detener simulación anterior si existe
        self.stop_simulation()

        logger.info("=== ESCENARIO 1: Pelota libre en medio campo ===")

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

        # Actualizar visualización
        self.update_visualization()

        # Iniciar simulación
        self.start_simulation(duration=30)

    def run_scenario_2(self):
        """
        Escenario 2: Jugador rojo cercano a la pelota (captura).
        """
        # Detener simulación anterior si existe
        self.stop_simulation()

        logger.info("=== ESCENARIO 2: Jugador rojo cercano a la pelota (captura) ===")

        # Posicionar objetos
        self.ball.set_position(500, 400)
        self.player_1.set_position(560, 400)  # Cercano a la pelota
        self.player_1.set_angle(180)  # Mirando hacia la pelota
        self.player_2.set_position(200, 700)
        self.player_3.set_position(1200, 200)
        self.player_4.set_position(1200, 700)

        # Actualizar visualización
        self.update_visualization()

        # Iniciar simulación
        self.start_simulation(duration=30)

    def run_scenario_3(self):
        """
        Escenario 3: Jugador rojo con la pelota (tiro a portería).
        """
        # Detener simulación anterior si existe
        self.stop_simulation()

        logger.info("=== ESCENARIO 3: Jugador rojo con la pelota (tiro a portería) ===")

        # Posicionar objetos
        self.ball.set_position(1200, 450)  # Cerca de la portería rival
        self.player_1.set_position(1180, 450)  # Junto a la pelota
        self.player_1.set_angle(0)  # Mirando hacia la portería
        self.player_1.ball_hold = True  # Tiene la pelota
        self.player_2.set_position(800, 700)
        self.player_3.set_position(1300, 400)
        self.player_4.set_position(1300, 500)

        # Actualizar visualización
        self.update_visualization()

        # Iniciar simulación
        self.start_simulation(duration=30)

    def run(self):
        """Inicia la prueba interactiva."""
        if self.show_visualization:
            plt.tight_layout()
            plt.show()
        else:
            # Modo no interactivo, ejecutar un escenario predeterminado
            self.run_scenario_1()

            # Esperar a que termine la simulación
            while self.is_running:
                time.sleep(0.1)

    def cleanup(self):
        """Libera recursos y cierra conexiones."""
        self.stop_simulation()

        # Cerrar comunicación con Arduino
        if self.serial_manager:
            self.serial_manager.disconnect()

        # Cerrar gestores de comportamiento
        if hasattr(self, 'behavior_red'):
            self.behavior_red.shutdown()
        if hasattr(self, 'behavior_blue'):
            self.behavior_blue.shutdown()


def main():
    """Función principal."""
    parser = argparse.ArgumentParser(description='Prueba de integración del sistema de control de robots')
    parser.add_argument('--port', help='Puerto serial (ej: /dev/ttyUSB0, COM3)')
    parser.add_argument('--use-arduino', action='store_true', help='Usar comunicación real con Arduino')
    parser.add_argument('--no-viz', action='store_true', help='Deshabilitar visualización')

    args = parser.parse_args()

    # Comprobar si se requiere puerto pero no se proporcionó
    if args.use_arduino and not args.port:
        parser.error("--use-arduino requiere --port")
        return

    # Crear tester de integración
    tester = IntegrationTester(
        serial_port=args.port,
        use_arduino=args.use_arduino,
        show_visualization=not args.no_viz
    )

    try:
        tester.run()
    finally:
        tester.cleanup()


if __name__ == "__main__":
    main()
