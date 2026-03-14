"""Módulo multiprocessing para calibración de bias.

Ejecuta tres procesos independientes:
1. Proceso de percepción: Detección ArUco del robot (usa shared memory)
2. Proceso de control: UI + RF para ajustar bias
3. Proceso de visualización: UI, video, teclado

Arquitectura similar a calibrate_pwm_range_mp.py y calibrate_pid_mp.py.

NO ejecutar directamente.
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
from multiprocess_calibration.control_process_bias import control_loop_bias
from multiprocess_calibration.visualization_process_bias import visualization_loop_bias

log = logging.getLogger(__name__)

SHM_NAME_BASE = "robot_calib_bias"


def run_bias_calibration(robot_id, serial_port='/dev/ttyUSB0', camera_id=None):
    """Ejecuta calibración de bias con multiprocessing (3 procesos).

    Args:
        robot_id: ID del robot a probar (0-3)
        serial_port: Puerto serial para comunicación RF
        camera_id: ID de la cámara (None para auto-detectar)
    """
    if camera_id is None:
        camera_id = get_camera_index(prefer_droidcam=True, fallback_index=0)
        log.info("📷 Cámara auto-detectada: /dev/video%d", camera_id)

    try:
        log.info("=" * 70)
        log.info("CALIBRACIÓN DE BIAS - 3 PROCESOS")
        log.info("=" * 70)
        log.info("Robot ID: %d", robot_id)
        log.info("Puerto serial: %s", serial_port)
        log.info("Cámara: /dev/video%d", camera_id)
        log.info("\nArquitectura:")
        log.info("  PROCESO 1: Percepción Ultra-Rápida (28-40 FPS)")
        log.info("  PROCESO 2: Control Bias (RF commands)")
        log.info("  PROCESO 3: Visualización (UI + Video)")
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

        # Crear pipes de comunicación
        # Pipe 1: Percepción → Control (datos posición)
        perception_to_control_send, perception_to_control_recv = multiprocessing.Pipe()

        # Pipe 2: Percepción → Visualización (metadata)
        perception_to_viz_send, perception_to_viz_recv = multiprocessing.Pipe()

        # Pipe 3: Control → Visualización (estado bias, estadísticas)
        control_to_viz_send, control_to_viz_recv = multiprocessing.Pipe()

        # Pipe 4: Visualización → Control (comandos teclado)
        viz_to_control_send, viz_to_control_recv = multiprocessing.Pipe()

        # Pipe 5: Control → Percepción (señal de reset de estadísticas)
        control_to_perception_send, control_to_perception_recv = multiprocessing.Pipe()

        # Lista de procesos
        processes = []

        # PROCESO 1: Percepción Ultra-Rápida
        log.info("\n🎥 Inicializando proceso de percepción ultra-rápida...")
        p1 = multiprocessing.Process(
            target=perception_loop_pid,
            args=(
                perception_to_control_send,  # Envía datos posición a Control
                perception_to_viz_send,       # Envía metadata a Visualización
                robot_id,
                camera_id,
                shm.name,                     # Shared memory para frames
                frame_counter,
                control_to_perception_send    # Pipe para reset de estadísticas
            ),
            name="PerceptionBias"
        )
        processes.append(p1)

        # PROCESO 2: Control Bias
        log.info("🎮 Inicializando proceso de control Bias...")
        p2 = multiprocessing.Process(
            target=control_loop_bias,
            args=(
                perception_to_control_recv,   # Recibe datos posición de Percepción
                control_to_viz_send,          # Envía estado a Visualización
                viz_to_control_recv,          # Recibe comandos de Visualización
                control_to_perception_send,   # Envía señales a Percepción (reset stats)
                robot_id,
                serial_port
            ),
            name="ControlBias"
        )
        processes.append(p2)

        # PROCESO 3: Visualización Bias
        log.info("🖥️  Inicializando proceso de visualización Bias...")
        p3 = multiprocessing.Process(
            target=visualization_loop_bias,
            args=(
                perception_to_viz_recv,        # Recibe metadata de Percepción
                control_to_viz_recv,          # Recibe estado de Control
                viz_to_control_send,          # Envía comandos a Control
                shm.name,                     # Shared memory para frames
                frame_counter
            ),
            name="VisualizationBias"
        )
        processes.append(p3)

        # Iniciar procesos
        log.info(f"\n▶️  Iniciando {len(processes)} proceso(s)...")
        for proc in processes:
            proc.start()
            log.info(f"   ✓ {proc.name} iniciado (PID: {proc.pid})")

        log.info("\n" + "=" * 70)
        log.info("Sistema en ejecución.")
        log.info("Usa ESPACIO para mover, LEFT/RIGHT para bias fino, a/d para bias grueso.")
        log.info("Presiona 'ESC' en la ventana para detener.")
        log.info("=" * 70)
        log.info("")

        # Esperar a que los procesos terminen
        for proc in processes:
            proc.join()

        log.info("\n✅ Calibración de bias finalizada")

    except KeyboardInterrupt:
        log.info("\n⏹️  Calibración interrumpida por usuario")
        for proc in processes:
            if proc.is_alive():
                proc.terminate()
                proc.join(timeout=2)
    except Exception as e:
        log.error("\n❌ Error en calibración de bias: %s", e)
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
