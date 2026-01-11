#!/usr/bin/env python3
"""Script de calibración - PASO 1: Determinar rango PWM útil de cada robot.

Este es el primer paso del proceso de calibración de motores. Permite
determinar el rango PWM donde el robot se mueve adecuadamente y la cámara
puede detectarlo consistentemente.

Uso:
    python scripts/calibrate_robot_pwm_range.py --robot-id 0

Controles:
    MOVIMIENTO:
    ESPACIO     : Iniciar movimiento hacia adelante
    BACKSPACE   : Iniciar movimiento hacia atrás
    x           : Detener movimiento manualmente

    AJUSTAR PWM DE PRUEBA:
    ↑/↓         : PWM ±5
    w/s         : PWM ±1 (fino)

    AJUSTAR RANGO PWM (para guardar):
    n/m         : PWM_min ±1
    ,/.         : PWM_max ±1
    r           : Sugerencias basadas en PWM actual
    g           : Guardar rango en JSON

    OTROS:
    +/-         : Duración ±0.5s
    [/]         : Duración ±0.1s (fino)
    ESC         : Salir
"""

import sys
import logging
import argparse
from pathlib import Path

# Configurar logging
logging.basicConfig(
    level=logging.INFO,
    format='%(levelname)s [%(name)s]: %(message)s'
)

# Control de niveles por módulo
logging.getLogger('robot_soccer.communication.rf_controller').setLevel(logging.INFO)
logging.getLogger('robot_soccer.communication.serial_manager').setLevel(logging.INFO)

# Agregar scripts al path
sys.path.insert(0, str(Path(__file__).parent))

# Importar módulo multiprocessing
# pylint: disable=import-error,wrong-import-position
from multiprocess_calibration.calibrate_pwm_range_mp import run_pwm_range_finder


def main():
    """Función principal."""
    parser = argparse.ArgumentParser(
        description='Calibración - Paso 1: Determinar rango PWM útil de cada robot'
    )
    parser.add_argument(
        '--robot-id',
        type=int,
        required=True,
        choices=[0, 1, 2, 3],
        help='ID del robot a probar (0-3)'
    )
    parser.add_argument(
        '--port',
        type=str,
        default='/dev/ttyUSB0',
        help='Puerto serial RF (default: /dev/ttyUSB0)'
    )
    parser.add_argument(
        '--camera',
        type=int,
        default=None,
        help='ID de cámara (default: auto-detectar)'
    )

    args = parser.parse_args()

    print("\n" + "=" * 70)
    print("CALIBRACIÓN - PASO 1: DETERMINACIÓN DE RANGO PWM ÚTIL")
    print("=" * 70)
    print("\nObjetivo:")
    print("  Determinar el rango PWM [min, max] donde el robot se mueve")
    print("  adecuadamente y la cámara puede detectarlo consistentemente.")
    print("\nProceso:")
    print("  1. Empieza con PWM bajo (20-30)")
    print("  2. Mueve el robot con ESPACIO (adelante) o BACKSPACE (atrás)")
    print("  3. Aumenta PWM gradualmente con ↑ (+5) o w (+1)")
    print("  4. Observa el contador de 'Detecciones'")
    print("  5. Cuando la cámara deje de detectar consistentemente,")
    print("     ese es tu PWM_max útil")
    print("\nControles:")
    print("  ESPACIO     → Mover adelante")
    print("  BACKSPACE   → Mover atrás")
    print("  x           → Detener manualmente")
    print("  ↑/↓         → PWM ±5")
    print("  w/s         → PWM ±1 (fino)")
    print("  +/-         → Duración ±0.5s")
    print("  [/]         → Duración ±0.1s (fino)")
    print("  ESC         → Salir")
    print("=" * 70 + "\n")

    # Ejecutar con multiprocessing
    run_pwm_range_finder(
        robot_id=args.robot_id,
        serial_port=args.port,
        camera_id=args.camera
    )


if __name__ == '__main__':
    main()
