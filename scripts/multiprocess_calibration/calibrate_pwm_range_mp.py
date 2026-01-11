"""Módulo de apoyo para búsqueda de rango PWM con multiprocessing.

Este módulo ejecuta dos procesos independientes:
1. Proceso de percepción: Detección continua de robots con ArUco
2. Proceso de control: UI simple + RF para probar diferentes PWM

Este módulo NO debe ejecutarse directamente.
"""

import sys
import logging
import multiprocessing
from pathlib import Path

# Agregar src al path
ROOT_DIR = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))

from robot_soccer.utils.camera_utils import get_camera_index  # pylint: disable=wrong-import-position

# Importar desde el mismo directorio
# pylint: disable=import-error,wrong-import-position,wrong-import-order
from multiprocess_calibration.perception_process import perception_loop
from multiprocess_calibration.control_process_pwm_range import control_loop_pwm_range

log = logging.getLogger(__name__)


def run_pwm_range_finder(robot_id, serial_port='/dev/ttyUSB0', camera_id=None):
    """Ejecuta la búsqueda de rango PWM con multiprocessing.

    Args:
        robot_id: ID del robot a probar (0-3)
        serial_port: Puerto serial para comunicación RF
        camera_id: ID de la cámara (None para auto-detectar)
    """
    # Auto-detectar cámara si no se especificó
    if camera_id is None:
        camera_id = get_camera_index(prefer_droidcam=True, fallback_index=0)
        log.info("📷 Cámara auto-detectada: /dev/video%d", camera_id)

    try:
        log.info("=" * 70)
        log.info("BÚSQUEDA DE RANGO PWM ÚTIL - MULTIPROCESSING")
        log.info("=" * 70)
        log.info("Robot ID: %d", robot_id)
        log.info("Puerto serial: %s", serial_port)
        log.info("Cámara: /dev/video%d", camera_id)
        log.info("=" * 70)

        # Crear pipes de comunicación
        robot_positions_send, robot_positions_recv = multiprocessing.Pipe()
        frame_send, frame_recv = multiprocessing.Pipe()

        # Lista de procesos
        processes = []

        # PROCESO 1: Percepción
        log.info("\n🎥 Inicializando proceso de percepción...")
        p1 = multiprocessing.Process(
            target=perception_loop,
            args=(robot_positions_send, frame_send, camera_id),
            name="Perception"
        )
        processes.append(p1)

        # PROCESO 2: Control PWM Range
        log.info("🎮 Inicializando proceso de control...")
        p2 = multiprocessing.Process(
            target=control_loop_pwm_range,
            args=(robot_positions_recv, frame_recv, robot_id, serial_port),
            name="ControlPWMRange"
        )
        processes.append(p2)

        # Iniciar procesos
        log.info("\n▶️  Iniciando %d proceso(s)...", len(processes))
        for proc in processes:
            proc.start()
            log.info("   ✓ %s iniciado (PID: %d)", proc.name, proc.pid)

        log.info("\n" + "=" * 70)
        log.info("Sistema en ejecución.")
        log.info("Usa ESPACIO/BACKSPACE para mover, ↑/↓ para ajustar PWM.")
        log.info("Presiona 'ESC' en la ventana para detener.")
        log.info("=" * 70)
        log.info("")

        # Esperar a que los procesos terminen
        for proc in processes:
            proc.join()

        log.info("\n✅ Búsqueda de rango PWM finalizada")

    except KeyboardInterrupt:
        log.info("\n⏹️  Búsqueda interrumpida por usuario")
        for proc in processes:
            if proc.is_alive():
                proc.terminate()
                proc.join(timeout=2)
    except Exception as e:
        log.error("\n❌ Error en búsqueda de rango PWM: %s", e)
        for proc in processes:
            if proc.is_alive():
                proc.terminate()
                proc.join(timeout=2)
