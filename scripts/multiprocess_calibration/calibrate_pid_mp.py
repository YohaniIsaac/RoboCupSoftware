"""Módulo de apoyo para calibración PID con multiprocessing.

Este módulo proporciona la función run_pid_calibration() que ejecuta dos procesos independientes:
1. Proceso de percepción: Detección continua de robots con ArUco
2. Proceso de control: Comandos RF, UI y ajuste de parámetros PID

La comunicación entre procesos es no bloqueante usando pipes.

Este módulo NO debe ejecutarse directamente. Usar scripts/calibrate_pid_controllers.py
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
from multiprocess_calibration.perception_process_pid import perception_loop_pid
from multiprocess_calibration.control_process_pid_pure import control_loop_pid
from multiprocess_calibration.visualization_process_pid import visualization_loop_pid

log = logging.getLogger(__name__)


def run_pid_calibration(robot_id, serial_port='/dev/ttyUSB0', camera_id=None):
    """Ejecuta la calibración PID con multiprocessing.

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
        log.info("CALIBRACIÓN DE CONTROLADORES PID - 3 PROCESOS")
        log.info("=" * 70)
        log.info(f"Robot ID: {robot_id}")
        log.info(f"Puerto serial: {serial_port}")
        log.info(f"Cámara: /dev/video{camera_id}")
        log.info("\nArquitectura:")
        log.info("  PROCESO 1: Percepción Ultra-Rápida (28-40 FPS)")
        log.info("  PROCESO 2: Control PID Puro (100-200 Hz)")
        log.info("  PROCESO 3: Visualización (28-40 FPS)")
        log.info("=" * 70)

        # ===== CREAR PIPES DE COMUNICACIÓN =====
        # Pipe 1: Percepción → Control (datos posición, ~100 bytes)
        perception_to_control_send, perception_to_control_recv = multiprocessing.Pipe()

        # Pipe 2: Percepción → Visualización (frames procesados, ~921KB)
        perception_to_viz_send, perception_to_viz_recv = multiprocessing.Pipe()

        # Pipe 3: Control → Visualización (estado PID, ~200 bytes)
        control_to_viz_send, control_to_viz_recv = multiprocessing.Pipe()

        # Pipe 4: Visualización → Control (comandos teclado/mouse, ~100 bytes, bidireccional)
        viz_to_control_send, viz_to_control_recv = multiprocessing.Pipe()

        # Lista de procesos
        processes = []

        # ===== PROCESO 1: Percepción Ultra-Rápida =====
        log.info("\n🎥 Inicializando proceso de percepción ultra-rápida...")
        p1 = multiprocessing.Process(
            target=perception_loop_pid,
            args=(
                perception_to_control_send,  # Envía datos posición a Control
                perception_to_viz_send,      # Envía frames a Visualización
                robot_id,
                camera_id
            ),
            name="PerceptionFast"
        )
        processes.append(p1)

        # ===== PROCESO 2: Control PID Puro =====
        log.info("🎮 Inicializando proceso de control PID puro (sin UI)...")
        p2 = multiprocessing.Process(
            target=control_loop_pid,
            args=(
                perception_to_control_recv,  # Recibe datos posición de Percepción
                control_to_viz_send,         # Envía estado a Visualización
                viz_to_control_recv,         # Recibe comandos de Visualización
                robot_id,
                serial_port
            ),
            name="ControlPID"
        )
        processes.append(p2)

        # ===== PROCESO 3: Visualización =====
        log.info("🖥️  Inicializando proceso de visualización...")
        p3 = multiprocessing.Process(
            target=visualization_loop_pid,
            args=(
                perception_to_viz_recv,      # Recibe frames de Percepción
                control_to_viz_recv,         # Recibe estado de Control
                viz_to_control_send          # Envía comandos a Control
            ),
            name="Visualization"
        )
        processes.append(p3)

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

        log.info("\n✅ Calibración PID finalizada")

    except KeyboardInterrupt:
        log.info("\n⏹️  Calibración interrumpida por usuario")
        # Terminar procesos si están activos
        for proc in processes:
            if proc.is_alive():
                proc.terminate()
                proc.join(timeout=2)
    except Exception as e:
        log.error(f"\n❌ Error en calibración PID: {e}")
        # Terminar procesos si están activos
        for proc in processes:
            if proc.is_alive():
                proc.terminate()
                proc.join(timeout=2)
