#!/usr/bin/env python3
"""Script de calibración - PASO 2: Ajustar bias para movimiento recto.

Mueve el robot hacia adelante a velocidad media y permite ajustar
el bias hasta que vaya en línea recta. El bias compensa la diferencia
física entre los motores izquierdo y derecho.

Uso:
    python scripts/calibrate_robot_bias.py --robot-id 0

Controles:
    ESPACIO     : Mover/detener robot (toggle)
    x           : Detener robot
    LEFT/RIGHT  : Bias +/-0.002 (fino)
    a/d         : Bias +/-0.01 (grueso)
    UP/DOWN     : PWM de prueba +/-1
    c           : Limpiar trayectoria
    g           : Guardar bias en JSON
    ESC         : Salir

Proceso:
    1. Presiona ESPACIO para mover el robot hacia adelante
    2. Observa si el robot tuerce a la izquierda o derecha
    3. Ajusta el bias con las flechas hasta que vaya recto
    4. La trayectoria se dibuja en pantalla (rojo=inicio, verde=actual)
    5. Presiona 'g' para guardar cuando esté satisfecho
"""

import sys
import logging
import argparse
from pathlib import Path

logging.basicConfig(level=logging.INFO, format='%(levelname)s [%(name)s]: %(message)s')
logging.getLogger('robot_soccer.communication.rf_controller').setLevel(logging.INFO)
logging.getLogger('robot_soccer.communication.serial_manager').setLevel(logging.INFO)

sys.path.insert(0, str(Path(__file__).parent))

# pylint: disable=import-error,wrong-import-position
from multiprocess_calibration.calibrate_bias_mp import run_bias_calibration


def main():
    parser = argparse.ArgumentParser(
        description='Calibracion - Paso 2: Ajustar bias para movimiento recto'
    )
    parser.add_argument('--robot-id', type=int, required=True, choices=[0, 1, 2, 3],
                        help='ID del robot (0-3)')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0',
                        help='Puerto serial RF (default: /dev/ttyUSB0)')
    parser.add_argument('--camera', type=int, default=None,
                        help='ID de camara (default: auto-detectar)')
    args = parser.parse_args()

    print("\n" + "=" * 60)
    print("CALIBRACION - PASO 2: BIAS PARA MOVIMIENTO RECTO")
    print("=" * 60)
    print("\nObjetivo:")
    print("  El robot debe ir en linea recta cuando ambas ruedas")
    print("  reciben el mismo PWM. El bias compensa asimetrias.")
    print("\nControles:")
    print("  ESPACIO       Mover/detener (toggle)")
    print("  LEFT/RIGHT    Bias +/-0.002 (fino)")
    print("  a/d           Bias +/-0.01 (grueso)")
    print("  UP/DOWN       PWM de prueba +/-1")
    print("  c             Limpiar trayectoria")
    print("  g             Guardar bias")
    print("  ESC           Salir")
    print("=" * 60 + "\n")

    run_bias_calibration(
        robot_id=args.robot_id,
        serial_port=args.port,
        camera_id=args.camera
    )


if __name__ == '__main__':
    main()
