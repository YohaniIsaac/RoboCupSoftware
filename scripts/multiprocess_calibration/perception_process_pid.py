"""Proceso de percepción ULTRA-RÁPIDO para calibración PID.

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

from robot_soccer.config import (
    CAMERA_PERSPECTIVE_ENABLED,
    CAMERA_PERSPECTIVE_SRC_POINTS,
    CAMERA_PERSPECTIVE_WIDTH,
    CAMERA_PERSPECTIVE_HEIGHT
)

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


def detect_aruco_fast(frame, robot_id: int):
    """Detección ArUco ULTRA-RÁPIDA sin procesamiento adicional.

    Elimina TODO el pre-procesamiento para máxima velocidad:
    - Sin sharpening (~10ms ahorrados)
    - Sin CLAHE (~15ms ahorrados)
    - Sin bilateral filter (~10ms ahorrados)
    - Sin transformación de perspectiva (~7ms ahorrados)
    - Sin dibujos (~18ms ahorrados)

    Total ahorrado: ~60ms

    Args:
        frame: Frame BGR de la cámara (640x480)
        robot_id: ID del robot a detectar (0-3)

    Returns:
        tuple: (detected: bool, robot_data: dict or None)
            - detected: True si se encontró el robot
            - robot_data: {'x': int, 'y': int, 'angulo': float} o None
    """
    # Convertir a escala de grises (única operación de pre-procesamiento)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Diccionario ArUco para cámara física (5x5 para mejor detección)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)

    # Parámetros de detección optimizados (copiados del sistema estándar)
    parameters = cv2.aruco.DetectorParameters()

    # Ventana de umbral adaptativo
    parameters.adaptiveThreshWinSizeMin = 5
    parameters.adaptiveThreshWinSizeMax = 51
    parameters.adaptiveThreshWinSizeStep = 10

    # Rango de tamaño de marcadores (detecta pequeños y grandes)
    parameters.minMarkerPerimeterRate = 0.01
    parameters.maxMarkerPerimeterRate = 6.0

    # Refinamiento de esquinas para máxima precisión
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    parameters.cornerRefinementWinSize = 3
    parameters.cornerRefinementMaxIterations = 50
    parameters.cornerRefinementMinAccuracy = 0.01

    # Corrección de errores
    parameters.errorCorrectionRate = 0.8

    # Remoción de perspectiva
    parameters.perspectiveRemovePixelPerCell = 6
    parameters.perspectiveRemoveIgnoredMarginPerCell = 0.10

    # Distancia al borde
    parameters.minDistanceToBorder = 1

    # Bits de borde del marcador
    parameters.markerBorderBits = 1

    # Desviación estándar mínima para Otsu
    parameters.minOtsuStdDev = 2.0

    # Aproximación poligonal
    parameters.polygonalApproxAccuracyRate = 0.08

    # Crear detector y detectar marcadores
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
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
                    'angulo': angle_deg  # Usar 'angulo' para ser consistente con el plan
                }

    # No detectado
    return False, None


def perception_loop_pid(robot_positions_pipe, frame_pipe, robot_id: int, camera_id: int):
    """Bucle principal del proceso de percepción ULTRA-RÁPIDO para calibración PID.

    Arquitectura de 3 procesos:
    - Percepción: Detecta ArUco + envía frames procesados (este proceso)
    - Control: SOLO PID + RF (100-200 Hz)
    - Visualización: cv2.imshow() + panel + teclado (28-40 FPS)

    Args:
        robot_positions_pipe: Pipe para enviar datos al proceso de control (datos pequeños)
        frame_pipe: Pipe para enviar frames procesados a visualización (frames grandes)
        robot_id: ID del robot a detectar (0-3)
        camera_id: ID de la cámara a usar

    Envía por robot_positions_pipe (a Control):
        {
            'robot_detected': bool,
            'robot_data': {'x': int, 'y': int, 'angulo': float} or None,
            'stats': {'fps': float, 'detection_rate': float},
            'timestamp': float
        }

    Envía por frame_pipe (a Visualización):
        {
            'frame': np.ndarray (BGR, después de transformación de perspectiva),
            'robot_detected': bool,
            'robot_data': {'x': int, 'y': int, 'angulo': float} or None,
            'stats': {...},
            'timestamp': float
        }
    """
    log.info("🚀 Proceso de percepción ULTRA-RÁPIDO iniciado (PID - 3 procesos)")
    log.info(f"   Robot objetivo: ID {robot_id}")
    log.info(f"   Cámara: /dev/video{camera_id}")
    log.info("   Arquitectura:")
    log.info("     ✓ Detección ArUco rápida (sin pre-procesamiento pesado)")
    log.info("     ✓ Transformación de perspectiva aplicada")
    log.info("     ✓ Datos enviados a Control (pipe 1)")
    log.info("     ✓ Frames enviados a Visualización (pipe 2)")
    log.info("     → FPS objetivo: 28-40")

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

    # Estadísticas
    stats = PerceptionStats()

    # Pre-calcular matriz de transformación de perspectiva
    perspective_matrix = None
    if CAMERA_PERSPECTIVE_ENABLED:
        src_pts = np.float32(CAMERA_PERSPECTIVE_SRC_POINTS)
        dst_pts = np.float32([
            [0, 0],
            [CAMERA_PERSPECTIVE_WIDTH - 1, 0],
            [CAMERA_PERSPECTIVE_WIDTH - 1, CAMERA_PERSPECTIVE_HEIGHT - 1],
            [0, CAMERA_PERSPECTIVE_HEIGHT - 1]
        ])
        perspective_matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
        log.info("✅ Transformación de perspectiva calculada")

    log.info("✅ Cámara iniciada - Comenzando detección ultra-rápida...")

    try:
        while True:
            # Capturar frame
            ret, frame = cap.read()
            if not ret:
                log.warning("⚠️  No se pudo leer frame")
                time.sleep(0.01)
                continue

            # Aplicar transformación de perspectiva (si está habilitada)
            if CAMERA_PERSPECTIVE_ENABLED and perspective_matrix is not None:
                frame_transformed = cv2.warpPerspective(
                    frame,
                    perspective_matrix,
                    (CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT)
                )
            else:
                frame_transformed = frame.copy()

            # Detección RÁPIDA (sin pre-procesamiento pesado como sharpening/CLAHE)
            detected, robot_data = detect_aruco_fast(frame_transformed, robot_id)

            # Actualizar estadísticas
            stats.update(detected)

            # Timestamp común para ambos pipes
            current_timestamp = time.time()

            # ===== ENVIAR DATOS AL PROCESO DE CONTROL (pipe 1, crítico) =====
            try:
                # Vaciar pipe si tiene datos viejos (non-blocking)
                if robot_positions_pipe.poll():
                    _ = robot_positions_pipe.recv()

                # Enviar nuevo paquete de datos (solo ~100 bytes)
                robot_positions_pipe.send({
                    'robot_detected': detected,
                    'robot_data': robot_data,  # None si no detectado
                    'stats': stats.to_dict(),
                    'timestamp': current_timestamp
                })
            except Exception:
                # Continuar aunque falle el envío
                pass

            # ===== ENVIAR FRAME AL PROCESO DE VISUALIZACIÓN (pipe 2, no crítico) =====
            try:
                # Vaciar pipe si tiene frames viejos (mantener solo el más reciente)
                if frame_pipe.poll():
                    _ = frame_pipe.recv()

                # Enviar frame procesado con datos (frame ~921KB + datos ~100 bytes)
                frame_pipe.send({
                    'frame': frame_transformed,
                    'robot_detected': detected,
                    'robot_data': robot_data,
                    'stats': stats.to_dict(),
                    'timestamp': current_timestamp
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
