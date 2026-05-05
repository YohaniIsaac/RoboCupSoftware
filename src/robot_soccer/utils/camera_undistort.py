"""Utilidad de corrección de distorsión de lente para cámara overhead.

Carga los coeficientes intrínsecos K y D desde camera_intrinsics.json
(generado por scripts/calibrate_intrinsic.py) y aplica cv2.undistort()
antes del warpPerspective en el pipeline de percepción.

Si el archivo no existe, todas las funciones son no-op: el pipeline
funciona sin corrección, igual que antes de la calibración.

Uso:
    from robot_soccer.utils.camera_undistort import load_intrinsics, undistort_frame

    K, D = load_intrinsics()          # llamar UNA vez al inicio del proceso
    frame = undistort_frame(frame, K, D)  # antes de warpPerspective en cada frame
"""
import json
import logging
import numpy as np
from pathlib import Path

log = logging.getLogger(__name__)

_INTRINSICS_PATH = Path(__file__).parent.parent / "config" / "camera_intrinsics.json"


def load_intrinsics():
    """Carga K y D desde camera_intrinsics.json.

    Returns:
        tuple: (K, D) como arrays numpy, o (None, None) si el archivo no existe.
    """
    if not _INTRINSICS_PATH.exists():
        log.info("camera_intrinsics.json no encontrado — undistort desactivado")
        return None, None

    try:
        with open(_INTRINSICS_PATH, "r") as f:
            data = json.load(f)
        K = np.array(data["K"], dtype=np.float64)
        D = np.array(data["D"], dtype=np.float64)
        log.info("Intrínsecos cargados desde %s (reproyección: %.3fpx)",
                 _INTRINSICS_PATH.name, data.get("reprojection_error", float("nan")))
        return K, D
    except Exception as e:
        log.error("Error cargando camera_intrinsics.json: %s — undistort desactivado", e)
        return None, None


def undistort_frame(frame, K, D):
    """Aplica cv2.undistort al frame. No-op si K o D es None."""
    if K is None or D is None:
        return frame
    import cv2
    return cv2.undistort(frame, K, D)
