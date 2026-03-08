"""Ejemplo de uso del RFTransmitter.

Este script demuestra cómo enviar comandos a los robots y al tablero.
"""

import time

from robot_soccer.controllers.rf_transmitter import RFTransmitter, RobotCommand, TableroCommand


def main():
    """Demuestra el uso del RFTransmitter con robots y tablero."""
    # Crear transmisor (ajusta el puerto según tu sistema)
    transmitter = RFTransmitter(port='/dev/ttyUSB0', baudrate=115200)

    # Conectar
    if not transmitter.connect():
        print("No se pudo conectar al transmisor")
        return

    try:
        print("\n=== Demo de control de robots ===\n")

        # Reset del tablero
        print("1. Reseteando tablero...")
        transmitter.send_tablero_command(TableroCommand.RESET_GOALS)
        transmitter.send_tablero_command(TableroCommand.RESET_TIME)
        time.sleep(1)

        # Iniciar cronómetro
        print("2. Iniciando cronómetro...")
        transmitter.send_tablero_command(TableroCommand.TOGGLE_PAUSE)
        time.sleep(1)

        # Control de Robot 1
        print("3. Robot 1 - Adelante")
        transmitter.send_robot_command(1, RobotCommand.FORWARD)
        time.sleep(0.5)

        print("4. Robot 1 - Girar izquierda")
        transmitter.send_robot_command(1, RobotCommand.LEFT)
        time.sleep(0.5)

        print("5. Robot 1 - Patear")
        transmitter.send_robot_command(1, RobotCommand.KICK)
        time.sleep(0.5)

        # Control de Robot 2
        print("6. Robot 2 - Adelante")
        transmitter.send_robot_command(2, RobotCommand.FORWARD)
        time.sleep(0.5)

        print("7. Robot 2 - Activar rodillo")
        transmitter.send_robot_command(2, RobotCommand.ROLLER_ON)
        time.sleep(1)

        print("8. Robot 2 - Desactivar rodillo")
        transmitter.send_robot_command(2, RobotCommand.ROLLER_OFF)
        time.sleep(0.5)

        # Registrar gol
        print("9. ¡GOL del equipo 1!")
        transmitter.send_tablero_command(TableroCommand.GOAL_TEAM1)
        time.sleep(1)

        # Pausar juego
        print("10. Pausando cronómetro...")
        transmitter.send_tablero_command(TableroCommand.TOGGLE_PAUSE)

        print("\n=== Demo completado ===")

    except KeyboardInterrupt:
        print("\n\nInterrumpido por el usuario")

    finally:
        # Desconectar
        transmitter.disconnect()


if __name__ == "__main__":
    main()
