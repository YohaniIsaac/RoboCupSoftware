#!/usr/bin/env python3
"""
Robot Soccer Simulation - Main Entry Point
"""
import multiprocessing
from core.game_controller import execute_multiprocessing


def main():
    """
    Función principal que inicia la simulación del juego
    """
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
