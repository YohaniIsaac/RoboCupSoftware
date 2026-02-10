"""Módulo de apoyo para calibración de umbrales de comportamiento con multiprocessing.

Este módulo proporciona la función run_behavior_calibration() que ejecuta dos procesos independientes:
1. Proceso de percepción: Detección continua de robots con ArUco
2. Proceso de control: Comandos RF, UI y ajuste de umbrales de comportamiento

La comunicación entre procesos es no bloqueante usando pipes.

Este módulo NO debe ejecutarse directamente. Usar scripts/calibrate_behavior_thresholds.py
"""

import sys
import logging
import multiprocessing
from pathlib import Path

# Agregar src y scripts al path
ROOT_DIR = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))

from robot_soccer.utils.camera_utils import get_camera_index

# Importar desde el mismo directorio
from multiprocess_calibration.perception_process import perception_loop
from multiprocess_calibration.control_process_behavior import control_loop_behavior

log = logging.getLogger(__name__)


def run_behavior_calibration(robot_id, serial_port='/dev/ttyUSB0', camera_id=None):
    """Ejecuta la calibración de umbrales de comportamiento con multiprocessing.

    Args:
        robot_id: ID del robot a controlar (0-3)
        serial_port: Puerto serial para comunicación RF
        camera_id: ID de la cámara (None para auto-detectar DroidCam)
    """
    # Auto-detectar cámara si no se especificó
    if camera_id is None:
        camera_id = get_camera_index(prefer_droidcam=True, fallback_index=0)
        log.info(f"📷 Cámara auto-detectada: /dev/video{camera_id}")

    try:
        log.info("=" * 70)
        log.info("CALIBRACIÓN DE UMBRALES DE COMPORTAMIENTO - MULTIPROCESSING")
        log.info("=" * 70)
        log.info(f"Robot ID: {robot_id}")
        log.info(f"Puerto serial: {serial_port}")
        log.info(f"Cámara: /dev/video{camera_id}")
        log.info("=" * 70)

        # Crear pipes de comunicación entre procesos
        # Pipe para enviar posiciones de robots: percepción -> control
        robot_positions_send, robot_positions_recv = multiprocessing.Pipe()

        # Pipe para enviar frames procesados: percepción -> control (para visualización)
        frame_send, frame_recv = multiprocessing.Pipe()

        # Lista de procesos
        processes = []

        # PROCESO 1: Percepción (detección continua de robots)
        log.info("\n🎥 Inicializando proceso de percepción...")
        p1 = multiprocessing.Process(
            target=perception_loop,
            args=(robot_positions_send, frame_send, camera_id),
            name="Perception"
        )
        processes.append(p1)

        # PROCESO 2: Control de comportamiento (UI, teclado, RF, ajuste de thresholds)
        log.info("🎮 Inicializando proceso de control de comportamiento...")
        p2 = multiprocessing.Process(
            target=control_loop_behavior,
            args=(robot_positions_recv, frame_recv, robot_id, serial_port),
            name="ControlBehavior"
        )
        processes.append(p2)

        # Iniciar todos los procesos
        log.info(f"\n▶️  Iniciando {len(processes)} proceso(s)...")
        for proc in processes:
            proc.start()
            log.info(f"   ✓ {proc.name} iniciado (PID: {proc.pid})")

        log.info("\n" + "=" * 70)
        log.info("Sistema en ejecución. Presiona 'ESC' en la ventana para detener.")
        log.info("=" * 70)
        log.info("")

        # Esperar a que los procesos terminen
        for proc in processes:
            proc.join()

        log.info("\n✅ Calibración de comportamiento finalizada")

    except KeyboardInterrupt:
        log.info("\n⏹️  Calibración interrumpida por usuario")
        # Terminar procesos si están activos
        for proc in processes:
            if proc.is_alive():
                proc.terminate()
                proc.join(timeout=2)
    except Exception as e:
        log.error(f"\n❌ Error en calibración de comportamiento: {e}")
        # Terminar procesos si están activos
        for proc in processes:
            if proc.is_alive():
                proc.terminate()
                proc.join(timeout=2)
