#!/usr/bin/env python3
"""Script de calibración interactiva para parámetros de control de velocidad.

Este script permite ajustar los parámetros de velocidad del robot en tiempo real
para encontrar los valores óptimos de forma empírica.

Utiliza multiprocessing para separar la percepción del control, permitiendo
detección continua sin interrupciones.

Uso:
    # Robot con ID 0 (default)
    python scripts/calibrate_velocity_control.py --robot-id 0

    # Robot con ID específico (0, 1, 2, 3)
    python scripts/calibrate_velocity_control.py --robot-id 2

    # Especificar cámara y puerto serial
    python scripts/calibrate_velocity_control.py --robot-id 1 --camera-id 2 --serial-port /dev/ttyUSB0

Controles:
    MOVIMIENTO DEL WAYPOINT:
    Flechas (↑↓←→): Mover waypoint objetivo (10px por paso)

    VELOCIDAD LINEAL (±0.01 por pulsación):
    W/S: Aumentar/Disminuir max_linear_speed
    A/D: Aumentar/Disminuir min_linear_speed

    VELOCIDAD ROTACIÓN (±0.01 por pulsación):
    Q/E: Aumentar/Disminuir max_rotation_speed
    Z/C: Aumentar/Disminuir min_rotation_speed

    CONTROL DE MOVIMIENTO:
    ESPACIO: START/STOP - Iniciar o pausar movimiento del robot
    X: Cancelar waypoint actual y detener robot

    OTROS CONTROLES:
    ENTER: Guardar parámetros a config.py
    ESC: Salir

Objetivo:
    - Observar cómo el robot alcanza el waypoint
    - Ajustar parámetros hasta lograr movimiento suave sin oscilaciones
    - MIN y MAX son límites ABSOLUTOS (si MIN=MAX, velocidad constante)

Arquitectura:
    - Proceso de percepción: Detección continua de robots con ArUco
    - Proceso de control: UI, teclado y comandos RF
    - Comunicación no bloqueante entre procesos via pipes
"""

import sys
import logging
import argparse
from pathlib import Path

# Agregar scripts al path para importar módulos de multiprocess_calibration
SCRIPTS_DIR = Path(__file__).parent
sys.path.insert(0, str(SCRIPTS_DIR))

from multiprocess_calibration.calibrate_velocity_mp import run_calibration

# Configurar logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(processName)-12s] %(levelname)-8s %(message)s',
    datefmt='%H:%M:%S'
)

log = logging.getLogger(__name__)


def main():
    """Función principal - launcher de calibración."""
    parser = argparse.ArgumentParser(
        description='Calibración de velocidad con multiprocessing',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument(
        '--robot-id',
        type=int,
        choices=[0, 1, 2, 3],
        required=True,
        help='ID del robot a controlar (0-3)'
    )
    parser.add_argument(
        '--serial-port',
        type=str,
        default='/dev/ttyUSB0',
        help='Puerto serial para comunicación RF (default: /dev/ttyUSB0)'
    )
    parser.add_argument(
        '--camera-id',
        type=int,
        default=None,
        help='ID de la cámara (si no se especifica, auto-detecta DroidCam)'
    )

    args = parser.parse_args()

    # Ejecutar calibración con multiprocessing
    run_calibration(
        robot_id=args.robot_id,
        serial_port=args.serial_port,
        camera_id=args.camera_id
    )


if __name__ == '__main__':
    main()
