"""Proceso de percepción para detección de robots.

Este proceso se ejecuta independientemente y:
- Captura frames de la cámara continuamente
- Detecta robots usando ArUco tags
- Envía posiciones de robots via pipe sin bloquearse
"""

import sys
import time
import logging
from pathlib import Path

import cv2
import numpy as np

# Agregar src al path
ROOT_DIR = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))

from robot_soccer.perception.player_tracking import deteccion_jugadores_aruco_tag
from robot_soccer.config import (
    CAMERA_PERSPECTIVE_ENABLED,
    CAMERA_PERSPECTIVE_SRC_POINTS,
    CAMERA_PERSPECTIVE_WIDTH,
    CAMERA_PERSPECTIVE_HEIGHT,
)

log = logging.getLogger(__name__)


def perception_loop(robot_positions_pipe, frame_pipe, camera_id):
    """Bucle principal del proceso de percepción.

    Args:
        robot_positions_pipe: Pipe para enviar posiciones de robots al proceso de control
        frame_pipe: Pipe para enviar frames procesados al proceso de control (para visualización)
        camera_id: ID de la cámara a usar
    """
    log.info(f"🎥 Proceso de percepción iniciado con cámara /dev/video{camera_id}")

    # Abrir cámara
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        log.error(f"❌ No se pudo abrir la cámara {camera_id}")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Configurar transformación de perspectiva si está habilitada
    perspective_matrix = None
    if CAMERA_PERSPECTIVE_ENABLED:
        src_points = np.float32(CAMERA_PERSPECTIVE_SRC_POINTS)
        dst_points = np.float32([
            [0, 0],
            [CAMERA_PERSPECTIVE_WIDTH - 1, 0],
            [CAMERA_PERSPECTIVE_WIDTH - 1, CAMERA_PERSPECTIVE_HEIGHT - 1],
            [0, CAMERA_PERSPECTIVE_HEIGHT - 1]
        ])
        perspective_matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        log.info("✅ Transformación de perspectiva configurada")

    log.info("✅ Cámara iniciada - Comenzando detección...")

    frame_count = 0
    last_log_time = time.time()

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                log.warning("⚠️  No se pudo leer frame de la cámara")
                time.sleep(0.1)
                continue

            # Aplicar transformación de perspectiva
            if perspective_matrix is not None:
                frame = cv2.warpPerspective(
                    frame, perspective_matrix,
                    (CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT)
                )

            # Detectar robots
            frame_with_markers, robots_data = deteccion_jugadores_aruco_tag(frame, use_camera=True)

            # Enviar posiciones de robots al proceso de control (sin bloqueo)
            # Usar try/except para no bloquearse si el otro proceso no está listo
            try:
                if robot_positions_pipe.poll():  # Si hay datos viejos, leerlos primero para vaciar el pipe
                    _ = robot_positions_pipe.recv()

                robot_positions_pipe.send({
                    'robots': robots_data,
                    'timestamp': time.time(),
                    'frame_count': frame_count
                })
            except Exception as e:
                # Continuar aunque falle el envío
                pass

            # Enviar frame procesado para visualización (sin bloqueo)
            try:
                if frame_pipe.poll():  # Vaciar pipe si tiene datos viejos
                    _ = frame_pipe.recv()

                frame_pipe.send(frame_with_markers)
            except Exception as e:
                pass

            frame_count += 1

            # Log cada 2 segundos
            current_time = time.time()
            if current_time - last_log_time >= 2.0:
                fps = frame_count / (current_time - last_log_time + 0.001)
                log.debug(f"📊 Percepción: {fps:.1f} FPS - Robots detectados: {len(robots_data)}")
                frame_count = 0
                last_log_time = current_time

            # Pequeña pausa para no saturar el CPU
            time.sleep(0.001)

    except KeyboardInterrupt:
        log.info("⏹️  Proceso de percepción detenido por usuario")
    except Exception as e:
        log.error(f"❌ Error en proceso de percepción: {e}")
    finally:
        cap.release()
        log.info("🔌 Cámara cerrada")
