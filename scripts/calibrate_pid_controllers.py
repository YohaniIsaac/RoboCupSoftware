#!/usr/bin/env python3
"""Script de calibración interactiva para parámetros PID del controlador.

Este script permite ajustar los 6 parámetros del controlador PID en tiempo real
para optimizar el seguimiento de trayectoria del robot.

Utiliza multiprocessing para separar la percepción del control, permitiendo
detección continua sin interrupciones.

Uso:
    # Robot con ID 0 (default)
    python scripts/calibrate_pid_controllers.py --robot-id 0

    # Robot con ID específico (0, 1, 2, 3)
    python scripts/calibrate_pid_controllers.py --robot-id 2

    # Especificar cámara y puerto serial
    python scripts/calibrate_pid_controllers.py --robot-id 1 --camera-id 2 --serial-port /dev/ttyUSB0

Controles:
    MOVIMIENTO DEL WAYPOINT:
    Flechas (↑↓←→): Mover waypoint objetivo (10px por paso)

    PID DE POSICIÓN (±0.001 por pulsación):
    Q/A: Aumentar/Disminuir kp_pos (Proporcional)
    W/S: Aumentar/Disminuir ki_pos (Integral)
    E/D: Aumentar/Disminuir kd_pos (Derivativo)

    PID ANGULAR (±0.001 por pulsación):
    R/F: Aumentar/Disminuir kp_angle (Proporcional)
    T/G: Aumentar/Disminuir ki_angle (Integral)
    Y/H: Aumentar/Disminuir kd_angle (Derivativo)

    AJUSTES FINOS (±0.0001 por pulsación):
    Mantener SHIFT + teclas anteriores para ajustes más finos

    CONTROL DE MOVIMIENTO:
    ESPACIO: START/STOP - Iniciar o pausar movimiento del robot
    X: Cancelar waypoint actual y detener robot
    C: Limpiar integral windup (resetear términos integrales)

    OTROS CONTROLES:
    ENTER: Guardar parámetros PID a config.py
    ESC: Salir

Objetivo:
    - Observar la respuesta del robot al alcanzar waypoints
    - Ajustar PID para eliminar:
      * Overshoot (robot se pasa del objetivo)
      * Oscilaciones (robot oscila alrededor del objetivo)
      * Error estacionario (robot no llega exactamente al objetivo)
      * Tiempo de asentamiento largo (robot tarda mucho en estabilizarse)

Metodología Ziegler-Nichols Modificada:
    1. Empezar con todos los valores en 0
    2. Incrementar Kp hasta que aparezcan oscilaciones sostenidas
    3. Reducir Kp a ~60% de ese valor crítico
    4. Agregar Kd para reducir overshoot
    5. Agregar Ki solo si hay error estacionario

Arquitectura:
    - Proceso de percepción: Detección continua de robots con ArUco
    - Proceso de control: UI, teclado y comandos RF con ajuste PID en tiempo real
    - Comunicación no bloqueante entre procesos via pipes
"""

import sys
import logging
import argparse
from pathlib import Path

# Agregar scripts al path para importar módulos de multiprocess_calibration
SCRIPTS_DIR = Path(__file__).parent
sys.path.insert(0, str(SCRIPTS_DIR))

from multiprocess_calibration.calibrate_pid_mp import run_pid_calibration

# Configurar logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(processName)-12s] %(levelname)-8s %(message)s',
    datefmt='%H:%M:%S'
)

log = logging.getLogger(__name__)


def main():
    """Función principal - launcher de calibración PID."""
    parser = argparse.ArgumentParser(
        description='Calibración de controladores PID con multiprocessing',
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

    # Ejecutar calibración PID con multiprocessing
    run_pid_calibration(
        robot_id=args.robot_id,
        serial_port=args.serial_port,
        camera_id=args.camera_id
    )


if __name__ == '__main__':
    main()
