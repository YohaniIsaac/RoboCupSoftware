"""Proceso de percepción ULTRA-RÁPIDO para calibración PWM Range.

Este proceso está optimizado para máxima velocidad:
- Sin pre-procesamiento de imagen (sharpening, CLAHE, bilateral)
- Sin dibujos en frames
- Sin transformación de perspectiva
- Sin predicciones
- Sin envío de frames
- Solo detección ArUco del robot específico

Objetivo: 28-40 FPS (vs 10-13 FPS del proceso estándar)

Ganancia de velocidad:
- Pre-procesamiento eliminado: ~35ms
- Dibujo eliminado: ~18ms
- Transformación perspectiva eliminada: ~7ms
- Total ahorrado: ~60ms de ~75-100ms = mejora 2-3x
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

from robot_soccer.perception.player_tracking import create_aruco_detector

log = logging.getLogger(__name__)


class PerceptionStats:
    """Estadísticas de detección en tiempo real.

    Usa __slots__ para eficiencia de memoria.
    """

    __slots__ = ('frames_analyzed', 'frames_detected', 'start_time',
                 'last_fps_calc_time', 'fps')

    def __init__(self):
        self.frames_analyzed = 0
        self.frames_detected = 0
        self.start_time = time.time()
        self.last_fps_calc_time = time.time()
        self.fps = 0.0

    def update(self, detected: bool):
        """Actualiza estadísticas con el resultado de detección.

        Args:
            detected: True si el robot fue detectado en este frame
        """
        self.frames_analyzed += 1
        if detected:
            self.frames_detected += 1

        # Calcular FPS cada 10 frames para reducir overhead
        if self.frames_analyzed % 10 == 0:
            current_time = time.time()
            elapsed = current_time - self.last_fps_calc_time
            if elapsed > 0:
                self.fps = 10.0 / elapsed
                self.last_fps_calc_time = current_time

    def get_detection_rate(self) -> float:
        """Calcula tasa de detección (0.0-1.0).

        Returns:
            float: Porcentaje de frames con detección exitosa
        """
        if self.frames_analyzed == 0:
            return 0.0
        return self.frames_detected / self.frames_analyzed

    def reset(self):
        """Resetea contadores (mantiene FPS actual)."""
        self.frames_analyzed = 0
        self.frames_detected = 0
        self.start_time = time.time()

    def to_dict(self) -> dict:
        """Convierte a diccionario para envío por pipe.

        Returns:
            dict: Estadísticas en formato serializable
        """
        return {
            'frames_analyzed': self.frames_analyzed,
            'frames_detected': self.frames_detected,
            'detection_rate': self.get_detection_rate(),
            'fps': round(self.fps, 1)
        }


def detect_aruco_fast(frame, robot_id: int, detector):
    """Detección ArUco ULTRA-RÁPIDA sin procesamiento adicional.

    Usa detector pre-creado de create_aruco_detector() para evitar
    recrear el detector cada frame (~0.5ms de ahorro).

    Args:
        frame: Frame BGR de la cámara (640x480)
        robot_id: ID del robot a detectar (0-3)
        detector: cv2.aruco.ArucoDetector pre-creado

    Returns:
        tuple: (detected: bool, robot_data: dict or None)
            - detected: True si se encontró el robot
            - robot_data: {'x': int, 'y': int, 'angle': float} o None
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    # Buscar el robot específico
    if ids is not None:
        for corner, aruco_id in zip(corners, ids):
            if aruco_id[0] == robot_id:
                # Calcular centro (promedio de las 4 esquinas)
                corner_points = corner.reshape(4, 2)
                center_x = int(np.mean(corner_points[:, 0]))
                center_y = int(np.mean(corner_points[:, 1]))

                # Calcular ángulo (vector de esquina 0 a esquina 1)
                vector_1 = corner_points[1] - corner_points[0]
                angle = np.arctan2(vector_1[1], vector_1[0])
                angle_deg = np.degrees(angle)

                return True, {
                    'x': center_x,
                    'y': center_y,
                    'angle': angle_deg
                }

    # No detectado
    return False, None


def perception_loop_pwm_range(robot_positions_pipe, robot_id: int, camera_id: int):
    """Bucle principal del proceso de percepción ULTRA-RÁPIDO.

    Diseñado específicamente para calibración PWM Range donde SOLO importa:
    - ¿Se detectó el robot? (sí/no)
    - Posición básica (x, y, ángulo)
    - Estadísticas de detección en tiempo real
    - FPS máximo posible

    Args:
        robot_positions_pipe: Pipe para enviar datos al proceso de control
        robot_id: ID del robot a detectar (0-3)
        camera_id: ID de la cámara a usar

    Envía por pipe cada frame:
        {
            'robot_detected': bool,
            'robot_data': {'x': int, 'y': int, 'angle': float} or None,
            'stats': {
                'frames_analyzed': int,
                'frames_detected': int,
                'detection_rate': float (0.0-1.0),
                'fps': float
            },
            'timestamp': float
        }
    """
    log.info("🚀 Proceso de percepción ULTRA-RÁPIDO iniciado")
    log.info(f"   Robot objetivo: ID {robot_id}")
    log.info(f"   Cámara: /dev/video{camera_id}")
    log.info("   Optimizaciones:")
    log.info("     ✓ Sin pre-procesamiento de imagen")
    log.info("     ✓ Sin dibujos en frames")
    log.info("     ✓ Sin transformación de perspectiva")
    log.info("     ✓ Sin envío de frames por pipe")
    log.info("     → Ganancia esperada: 2-3x velocidad (28-40 FPS)")

    # Abrir cámara
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        log.error(f"❌ No se pudo abrir la cámara {camera_id}")
        return

    # Configuración básica de resolución
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 60)  # Intentar 60 FPS
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Buffer mínimo

    # Configuración de exposure (solo si la cámara lo soporta)
    try:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # Manual mode
        cap.set(cv2.CAP_PROP_EXPOSURE, -6)  # Exposure bajo (menos motion blur)
        cap.set(cv2.CAP_PROP_BRIGHTNESS, 150)
        cap.set(cv2.CAP_PROP_CONTRAST, 130)
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    except Exception as e:
        log.warning(f"⚠️  No se pudieron aplicar todas las configuraciones: {e}")

    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    log.info(f"📷 Cámara configurada: FPS objetivo={actual_fps:.1f}")

    # Crear detector ArUco UNA sola vez (reutilizable entre frames)
    aruco_detector = create_aruco_detector(use_camera=True)
    log.info("✅ Detector ArUco creado (reutilizable)")

    # Estadísticas
    stats = PerceptionStats()

    log.info("✅ Cámara iniciada - Comenzando detección ultra-rápida...")

    try:
        while True:
            # Capturar frame
            ret, frame = cap.read()
            if not ret:
                log.warning("⚠️  No se pudo leer frame")
                time.sleep(0.01)
                continue

            # Detección RÁPIDA (sin pre-procesamiento)
            detected, robot_data = detect_aruco_fast(frame, robot_id, aruco_detector)

            # Actualizar estadísticas
            stats.update(detected)

            # Enviar datos al proceso de control (sin bloqueo)
            try:
                # Vaciar pipe si tiene datos viejos (non-blocking)
                if robot_positions_pipe.poll():
                    _ = robot_positions_pipe.recv()

                # Enviar nuevo paquete de datos
                robot_positions_pipe.send({
                    'robot_detected': detected,
                    'robot_data': robot_data,  # None si no detectado
                    'stats': stats.to_dict(),
                    'timestamp': time.time()
                })
            except Exception:
                # Continuar aunque falle el envío
                pass

            # Log cada 100 frames (~3 segundos a 30 FPS)
            if stats.frames_analyzed % 100 == 0:
                log.debug(
                    f"📊 Percepción: {stats.fps:.1f} FPS | "
                    f"Detección: {stats.get_detection_rate()*100:.1f}% "
                    f"({stats.frames_detected}/{stats.frames_analyzed})"
                )

            # Mínima pausa para no saturar CPU (ajustable)
            time.sleep(0.0001)  # 0.1ms - casi imperceptible

    except KeyboardInterrupt:
        log.info("⏹️  Proceso de percepción detenido por usuario")
    except Exception as e:
        log.error(f"❌ Error en proceso de percepción: {e}", exc_info=True)
    finally:
        cap.release()
        log.info("🔌 Cámara cerrada")
        log.info(
            f"📊 Stats finales: {stats.fps:.1f} FPS, "
            f"{stats.get_detection_rate()*100:.1f}% detección"
        )
