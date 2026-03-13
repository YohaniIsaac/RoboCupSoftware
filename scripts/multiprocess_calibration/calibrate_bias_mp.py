"""Módulo multiprocessing para calibración de bias.

Ejecuta dos procesos:
1. Percepción: Detección ArUco del robot
2. Control: UI + RF para ajustar bias

NO ejecutar directamente.
"""

import sys
import logging
import multiprocessing
from pathlib import Path

ROOT_DIR = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))

from robot_soccer.utils.camera_utils import get_camera_index  # pylint: disable=wrong-import-position

# pylint: disable=import-error,wrong-import-position,wrong-import-order
from multiprocess_calibration.perception_process_pwm_range import perception_loop_pwm_range
from multiprocess_calibration.control_process_bias import control_loop_bias

log = logging.getLogger(__name__)


def run_bias_calibration(robot_id, serial_port='/dev/ttyUSB0', camera_id=None):
    """Ejecuta calibración de bias con multiprocessing."""
    if camera_id is None:
        camera_id = get_camera_index(prefer_droidcam=True, fallback_index=0)

    processes = []
    try:
        robot_positions_send, robot_positions_recv = multiprocessing.Pipe()

        p1 = multiprocessing.Process(
            target=perception_loop_pwm_range,
            args=(robot_positions_send, robot_id, camera_id),
            name="PerceptionBias"
        )
        processes.append(p1)

        p2 = multiprocessing.Process(
            target=control_loop_bias,
            args=(robot_positions_recv, None, robot_id, serial_port),
            name="ControlBias"
        )
        processes.append(p2)

        for proc in processes:
            proc.start()

        for proc in processes:
            proc.join()

    except KeyboardInterrupt:
        log.info("Calibracion interrumpida")
        for proc in processes:
            if proc.is_alive():
                proc.terminate()
                proc.join(timeout=2)
