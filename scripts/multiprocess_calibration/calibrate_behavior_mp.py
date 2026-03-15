"""Módulo de apoyo para calibración de umbrales de comportamiento con multiprocessing.

Este módulo proporciona la función run_behavior_calibration() que ejecuta tres procesos:
1. Proceso de percepción: Detección continua de robots con ArUco (usa shared memory)
2. Proceso de control: Comandos RF, lógica de movimiento
3. Proceso de visualización: UI, teclado, mouse, paneles de ajuste

Arquitectura igual a calibrate_pid_mp.py y calibrate_bias_mp.py.
La comunicación entre procesos usa pipes no bloqueantes y shared memory para frames.

Este módulo NO debe ejecutarse directamente. Usar scripts/calibrate_behavior_thresholds.py
"""

import sys
import logging
import multiprocessing
from multiprocessing import Value, shared_memory
from pathlib import Path

ROOT_DIR = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))

from robot_soccer.config import CAMERA_PERSPECTIVE_HEIGHT, CAMERA_PERSPECTIVE_WIDTH
from robot_soccer.utils.camera_utils import get_camera_index

from multiprocess_calibration.perception_process_pid import perception_loop_pid
from multiprocess_calibration.control_process_behavior import control_loop_behavior_pure
from multiprocess_calibration.visualization_process_behavior import visualization_loop_behavior

log = logging.getLogger(__name__)

SHM_NAME_BASE = "robot_calib_behavior"


def run_behavior_calibration(robot_id, serial_port='/dev/ttyUSB0', camera_id=None):
    """Ejecuta la calibración de umbrales de comportamiento con multiprocessing (3 procesos).

    Args:
        robot_id: ID del robot a controlar (0-3)
        serial_port: Puerto serial para comunicación RF
        camera_id: ID de la cámara (None para auto-detectar DroidCam)
    """
    if camera_id is None:
        camera_id = get_camera_index(prefer_droidcam=True, fallback_index=0)
        log.info(f"📷 Cámara auto-detectada: /dev/video{camera_id}")

    try:
        log.info("=" * 70)
        log.info("CALIBRACIÓN DE UMBRALES DE COMPORTAMIENTO - 3 PROCESOS")
        log.info("=" * 70)
        log.info(f"Robot ID: {robot_id}")
        log.info(f"Puerto serial: {serial_port}")
        log.info(f"Cámara: /dev/video{camera_id}")
        log.info("\nArquitectura:")
        log.info("  PROCESO 1: Percepción Ultra-Rápida (28-40 FPS)")
        log.info("  PROCESO 2: Control Behavior Puro (100-200 Hz)")
        log.info("  PROCESO 3: Visualización (UI + Teclado + Mouse)")
        log.info("=" * 70)

        # Limpiar SHM huérfana si existe (por si script anterior no terminó bien)
        shm_name = f"{SHM_NAME_BASE}_{robot_id}"
        try:
            old_shm = shared_memory.SharedMemory(name=shm_name)
            old_shm.close()
            old_shm.unlink()
            log.info(f"🧹 Limpiando SHM huérfana: {shm_name}")
        except FileNotFoundError:
            pass

        # Crear shared memory para frames
        frame_size = CAMERA_PERSPECTIVE_HEIGHT * CAMERA_PERSPECTIVE_WIDTH * 3
        shm = shared_memory.SharedMemory(create=True, name=shm_name, size=frame_size)
        frame_counter = Value('i', 0)
        log.info(f"📦 Shared memory creada: {shm.name} ({frame_size} bytes)")

        # Crear pipes de comunicación entre procesos
        # Pipe 1: Percepción → Control (datos posición, ~100 bytes)
        perception_to_control_send, perception_to_control_recv = multiprocessing.Pipe()

        # Pipe 2: Percepción → Visualización (metadata, ~100 bytes)
        perception_to_viz_send, perception_to_viz_recv = multiprocessing.Pipe()

        # Pipe 3: Control → Visualización (estado behavior, ~200 bytes)
        control_to_viz_send, control_to_viz_recv = multiprocessing.Pipe()

        # Pipe 4: Visualización → Control (comandos teclado/mouse, ~100 bytes, bidireccional)
        viz_to_control_send, viz_to_control_recv = multiprocessing.Pipe()

        # Lista de procesos
        processes = []

        # PROCESO 1: Percepción Ultra-Rápida
        log.info("\n🎥 Inicializando proceso de percepción ultra-rápida...")
        p1 = multiprocessing.Process(
            target=perception_loop_pid,
            args=(
                perception_to_control_send,
                perception_to_viz_send,
                robot_id,
                camera_id,
                shm.name,
                frame_counter
            ),
            name="PerceptionFast"
        )
        processes.append(p1)

        # PROCESO 2: Control Behavior Puro (sin UI)
        log.info("🎮 Inicializando proceso de control behavior puro (sin UI)...")
        p2 = multiprocessing.Process(
            target=control_loop_behavior_pure,
            args=(
                perception_to_control_recv,
                control_to_viz_send,
                viz_to_control_recv,
                robot_id,
                serial_port
            ),
            name="ControlBehavior"
        )
        processes.append(p2)

        # PROCESO 3: Visualización
        log.info("🖥️  Inicializando proceso de visualización...")
        p3 = multiprocessing.Process(
            target=visualization_loop_behavior,
            args=(
                perception_to_viz_recv,
                control_to_viz_recv,
                viz_to_control_send,
                shm.name,
                frame_counter
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

        log.info("\n✅ Calibración de comportamiento finalizada")

    except KeyboardInterrupt:
        log.info("\n⏹️  Calibración interrumpida por usuario")
        for proc in processes:
            if proc.is_alive():
                proc.terminate()
                proc.join(timeout=2)
    except Exception as e:
        log.error(f"\n❌ Error en calibración de comportamiento: {e}")
        for proc in processes:
            if proc.is_alive():
                proc.terminate()
                proc.join(timeout=2)
    finally:
        # Limpiar shared memory
        try:
            shm.close()
            shm.unlink()
            log.info("🧹 Shared memory liberada")
        except Exception:
            pass
