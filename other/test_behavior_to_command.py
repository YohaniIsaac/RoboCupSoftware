#!/usr/bin/env python3
"""
Script simplificado para probar la traducción de comportamientos a comandos.
Este script muestra claramente qué comandos se generan para cada comportamiento.
"""

import logging
import time
from robot_soccer.entities.player import Player
from robot_soccer.entities.ball import Ball
from robot_soccer.ai.behavior_tree.manager import BehaviorManager
from robot_soccer.config import *

# Configurar logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(levelname)s - %(name)s - %(message)s'
)


# Crear un manejador especial para la clase CommandFilter
class CommandFilter(logging.Filter):
    """Filtro para mostrar solo los mensajes de comando."""

    def filter(self, record):
        return "motor" in record.getMessage().lower() or \
            "kick" in record.getMessage().lower() or \
            "dribbler" in record.getMessage().lower()


# Configurar el logger para capturar comandos
command_logger = logging.getLogger("controllers.differential_drive")
console_handler = logging.StreamHandler()
console_handler.addFilter(CommandFilter())
command_logger.addHandler(console_handler)


class SimpleBehaviorTest:
    """Prueba simple de comportamientos y su traducción a comandos."""

    def __init__(self):
        """Inicializa el entorno de prueba."""
        self.setup_players_and_ball()
        self.setup_behavior_manager()

    def setup_players_and_ball(self):
        """Configura jugadores y pelota."""
        self.ball = Ball(750, 450)
        self.player_1 = Player(1, 500, 300, 0, 'red')
        self.player_2 = Player(2, 200, 700, 90, 'red')

        self.players = [self.player_1, self.player_2]

        # Configurar roles
        self.player_1.set_rol(ROL_ATACANTE)
        self.player_2.set_rol(ROL_DEFENSIVO)

    def setup_behavior_manager(self):
        """Configura el gestor de comportamientos."""
        self.behavior_manager = BehaviorManager(
            self.players, self.ball, team='red'
        )

    def test_move_to_ball(self):
        """Prueba el comportamiento de moverse a la pelota."""
        print("\n=== PRUEBA: Mover hacia la pelota ===")

        # Colocar pelota alejada del jugador
        self.ball.set_position(700, 300)
        self.player_1.set_position(500, 300)

        # Ejecutar algunos pasos del árbol de comportamiento
        print("Ejecutando comportamiento (mostrando solo comandos generados):")
        for i in range(10):
            # Actualizar gestores
            self.behavior_manager.update()
            time.sleep(0.1)

    def test_capture_ball(self):
        """Prueba el comportamiento de capturar la pelota."""
        print("\n=== PRUEBA: Capturar la pelota ===")

        # Colocar pelota cerca del jugador
        self.ball.set_position(530, 300)
        self.player_1.set_position(500, 300)

        # Ejecutar algunos pasos del árbol de comportamiento
        print("Ejecutando comportamiento (mostrando solo comandos generados):")
        for i in range(10):
            # Actualizar gestores
            self.behavior_manager.update()
            time.sleep(0.1)

    def test_shoot_to_goal(self):
        """Prueba el comportamiento de tiro a portería."""
        print("\n=== PRUEBA: Tiro a portería ===")

        # Colocar jugador cerca de la portería con la pelota
        self.ball.set_position(1200, 450)
        self.player_1.set_position(1200, 450)
        self.player_1.set_angle(0)  # Apuntando a la portería
        self.player_1.ball_hold = True  # Ya tiene la pelota

        # Ejecutar algunos pasos del árbol de comportamiento
        print("Ejecutando comportamiento (mostrando solo comandos generados):")
        for i in range(10):
            # Actualizar gestores
            self.behavior_manager.update()
            time.sleep(0.1)

    def test_defensive_position(self):
        """Prueba el comportamiento de posicionamiento defensivo."""
        print("\n=== PRUEBA: Posicionamiento defensivo ===")

        # Colocar pelota en zona defensiva
        self.ball.set_position(300, 450)
        self.player_2.set_position(400, 450)

        # Ejecutar algunos pasos del árbol de comportamiento
        print("Ejecutando comportamiento (mostrando solo comandos generados):")
        for i in range(10):
            # Actualizar gestores
            self.behavior_manager.update()
            time.sleep(0.1)

    def test_intercept_ball(self):
        """Prueba el comportamiento de interceptar la pelota."""
        print("\n=== PRUEBA: Interceptar la pelota en movimiento ===")

        # Colocar pelota con velocidad
        self.ball.set_position(600, 300)

        # Simular pelota en movimiento
        if hasattr(self.ball, 'dx') and hasattr(self.ball, 'dy'):
            self.ball.dx = 5  # Velocidad en x
            self.ball.dy = 0  # Velocidad en y

        self.player_1.set_position(500, 350)

        # Ejecutar algunos pasos del árbol de comportamiento
        print("Ejecutando comportamiento (mostrando solo comandos generados):")
        for i in range(10):
            # Simular movimiento de la pelota
            if hasattr(self.ball, 'x') and hasattr(self.ball, 'dx'):
                self.ball.x += self.ball.dx
                self.ball.y += self.ball.dy

            # Actualizar gestores
            self.behavior_manager.update()
            time.sleep(0.1)

    def run_all_tests(self):
        """Ejecuta todas las pruebas en secuencia."""
        tests = [
            self.test_move_to_ball,
            self.test_capture_ball,
            self.test_shoot_to_goal,
            self.test_defensive_position,
            self.test_intercept_ball
        ]

        for test in tests:
            test()
            time.sleep(1)  # Pausa entre pruebas


if __name__ == "__main__":
    tester = SimpleBehaviorTest()
    tester.run_all_tests()
