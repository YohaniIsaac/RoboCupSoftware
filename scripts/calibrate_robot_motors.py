#!/usr/bin/env python3
"""Script de calibración interactiva de motores por robot (Multiprocessing).

Este script permite ajustar factores de calibración individuales para
compensar diferencias físicas entre robots (motores, fricción, peso).

VERSIÓN MULTIPROCESSING:
- Proceso 1: Percepción (detección ArUco) → ~30-60 FPS
- Proceso 2: Control (UI + RF + calibración) → Envío de comandos fluido

Uso:
    python scripts/calibrate_robot_motors.py [--robot-id 0] [--camera-id 2]

Controles de Calibración:
    AJUSTE GRUESO:
    q/a: Aumentar/Disminuir max_speed_left (±0.05)
    w/s: Aumentar/Disminuir max_speed_right (±0.05)
    e/d: Aumentar/Disminuir bias_correction (±0.01)

    AJUSTE FINO:
    1/2: Aumentar/Disminuir max_speed_left (±0.01)
    3/4: Aumentar/Disminuir max_speed_right (±0.01)
    5/6: Aumentar/Disminuir bias_correction (±0.005)

    Flechas: Mover robot (duración configurable, default 0.3s)
        ↑: Adelante
        ↓: Atrás
        ←: Girar izquierda
        →: Girar derecha

    [/]: Disminuir/Aumentar duración (±0.05s, ajuste grueso)
    -/=: Disminuir/Aumentar duración (±0.01s, ajuste fino)
    ESPACIO: Detener robot inmediatamente

    Rango de duración: 0.05s - 5.0s (default: 0.3s)
    r (minúscula): Resetear calibración a valores neutros (1.0, 1.0, 0.0)
    ENTER: Guardar calibración actual
    ESC: Salir sin guardar

IMPORTANTE:
    - Al presionar flecha, el robot se mueve por X segundos y se detiene automáticamente
    - Usa [/] para ajustes gruesos (±0.05s) o -/= para ajustes finos (±0.01s)
    - Para movimientos óptimos, empieza con 0.3s y ajusta según necesites
    - Presiona ESPACIO para cancelar el movimiento antes de que termine

Objetivo:
    Ajustar los valores hasta que el robot se mueva:
    - Recto sin desviarse (ajustar bias_correction)
    - A la misma velocidad que otros robots (ajustar max_speed_*)
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
logging.getLogger('robot_soccer.core').setLevel(logging.ERROR)
logging.getLogger('robot_soccer.perception').setLevel(logging.ERROR)
logging.getLogger('robot_soccer.entities').setLevel(logging.ERROR)
logging.getLogger('robot_soccer.ai').setLevel(logging.ERROR)
logging.getLogger('robot_soccer.utils').setLevel(logging.ERROR)
logging.getLogger('robot_soccer.communication.rf_controller').setLevel(logging.INFO)
logging.getLogger('robot_soccer.communication.serial_manager').setLevel(logging.INFO)

# Agregar scripts al path
sys.path.insert(0, str(Path(__file__).parent))

# Importar módulo de calibración multiprocess
from multiprocess_calibration.calibrate_motors_mp import run_motors_calibration


def main():
    """Función principal."""
    parser = argparse.ArgumentParser(
        description='Calibración interactiva de motores (multiprocessing)'
    )
    parser.add_argument(
        '--robot-id',
        type=int,
        default=0,
        choices=[0, 1, 2, 3],
        help='ID del robot a calibrar (0-3)'
    )
    parser.add_argument(
        '--camera-id',
        type=int,
        default=None,
        help='ID de la cámara (None = auto-detectar DroidCam)'
    )
    parser.add_argument(
        '--serial-port',
        type=str,
        default='/dev/ttyUSB0',
        help='Puerto serial del transmisor RF'
    )

    args = parser.parse_args()

    print("\n" + "=" * 70)
    print("CALIBRACIÓN DE MOTORES - VERSIÓN MULTIPROCESSING")
    print("=" * 70)
    print("\nVentajas de esta versión:")
    print("  ✅ Detección ArUco en proceso separado (~30-60 FPS)")
    print("  ✅ Comandos RF enviados cada ~10ms (movimiento fluido)")
    print("  ✅ Sin lag en la UI")
    print("  ✅ Usa la misma arquitectura que src/")
    print("\n" + "=" * 70 + "\n")

    # Ejecutar calibración multiprocess
    run_motors_calibration(
        robot_id=args.robot_id,
        serial_port=args.serial_port,
        camera_id=args.camera_id
    )


if __name__ == "__main__":
    main()
