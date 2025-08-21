#!/usr/bin/env python3
"""Robot Soccer Simulation."""

import logging
import multiprocessing
from robot_soccer.core.game_controller import execute_multiprocessing

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

def main():
    """Función principal que inicia la simulación del juego."""
    # Configurar multiprocessing si es necesario
    multiprocessing.set_start_method('spawn')

    # Crear e iniciar el controlador principal del juego
    game = execute_multiprocessing()
    game.run()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nPrograma terminado por el usuario")
    except Exception as e:
        print(f"Error inesperado: {e}")
