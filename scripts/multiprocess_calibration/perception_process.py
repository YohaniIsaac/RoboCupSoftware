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

from robot_soccer.perception.player_tracking import (
    create_aruco_detector,
    deteccion_jugadores_aruco_tag,
)
from robot_soccer.config import (
    CAMERA_PERSPECTIVE_ENABLED,
    CAMERA_PERSPECTIVE_SRC_POINTS,
    CAMERA_PERSPECTIVE_WIDTH,
    CAMERA_PERSPECTIVE_HEIGHT,
)
from robot_soccer.utils.camera_undistort import load_intrinsics, undistort_frame

log = logging.getLogger(__name__)


def perception_loop(robot_positions_pipe, frame_pipe, camera_id):
    """Bucle principal del proceso de percepción.

    Args:
        robot_positions_pipe: Pipe para enviar posiciones de robots al proceso de control
        frame_pipe: Pipe para enviar frames procesados al proceso de control (para visualización)
        camera_id: ID de la cámara a usar
    """
    log.info(f"🎥 Proceso de percepción iniciado con cámara /dev/video{camera_id}")

    _K, _D = load_intrinsics()

    # Abrir cámara
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        log.error(f"❌ No se pudo abrir la cámara {camera_id}")
        return

    # Configuración básica de resolución
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # ===== OPTIMIZACIÓN PARA DETECCIÓN EN MOVIMIENTO =====
    # 1. FPS alto para capturar más frames
    cap.set(cv2.CAP_PROP_FPS, 60)  # Intentar 60 FPS

    # 2. Reducir buffer de frames (evitar frames viejos)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    # 3. Auto-exposure deshabilitado (si la cámara lo soporta)
    # Exposure bajo = menos motion blur, pero necesita más luz
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # 1 = manual mode
    cap.set(cv2.CAP_PROP_EXPOSURE, -6)  # Valor bajo (rango típico: -13 a -1)

    # 4. Aumentar brillo y contraste para compensar exposure bajo
    cap.set(cv2.CAP_PROP_BRIGHTNESS, 150)  # Rango 0-255
    cap.set(cv2.CAP_PROP_CONTRAST, 130)
    cap.set(cv2.CAP_PROP_SATURATION, 128)

    # 5. Autofocus deshabilitado (si la cámara lo soporta)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_FOCUS, 0)  # Infinito

    # Log de configuración aplicada
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    actual_exposure = cap.get(cv2.CAP_PROP_EXPOSURE)
    log.info(f"📷 Configuración cámara: FPS={actual_fps:.1f}, Exposure={actual_exposure}")

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

    # Crear detector ArUco UNA sola vez (reutilizable entre frames)
    aruco_detector = create_aruco_detector(use_camera=True)
    log.info("✅ Detector ArUco creado (reutilizable)")

    # ===== IDs PERMITIDOS =====
    # Solo detectar robots con IDs 0, 1, 2, 3 (rechazar falsos positivos)
    ALLOWED_ROBOT_IDS = {0, 1, 2, 3}
    log.info(f"🤖 IDs permitidos: {sorted(ALLOWED_ROBOT_IDS)} - Otros marcadores serán rechazados")

    # ===== PREDICCIÓN DE POSICIÓN =====
    # Mantener última posición conocida de cada robot para predicción
    last_known_positions = {}  # {robot_id: {'x': x, 'y': y, 'angulo': ang, 'timestamp': t, 'vx': vx, 'vy': vy}}
    MAX_PREDICTION_TIME = 0.5  # Máximo 500ms de predicción
    LOST_DETECTION_COUNT = {}  # {robot_id: frames_sin_detección}

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                log.warning("⚠️  No se pudo leer frame de la cámara")
                time.sleep(0.1)
                continue

            frame = undistort_frame(frame, _K, _D)

            # Aplicar transformación de perspectiva
            if perspective_matrix is not None:
                frame = cv2.warpPerspective(
                    frame, perspective_matrix,
                    (CAMERA_PERSPECTIVE_WIDTH, CAMERA_PERSPECTIVE_HEIGHT)
                )

            # Detectar robots (solo IDs permitidos: 0, 1, 2, 3)
            frame_with_markers, robots_data = deteccion_jugadores_aruco_tag(
                frame,
                aruco_detector,
                allowed_ids=ALLOWED_ROBOT_IDS
            )

            # ===== PREDICCIÓN Y TRACKING =====
            current_time = time.time()
            detected_ids = {robot['id'] for robot in robots_data}

            # Actualizar posiciones conocidas y calcular velocidades
            for robot in robots_data:
                robot_id = robot['id']
                if robot_id in last_known_positions:
                    # Calcular velocidad estimada
                    prev = last_known_positions[robot_id]
                    dt = current_time - prev['timestamp']
                    if dt > 0:
                        vx = (robot['x'] - prev['x']) / dt
                        vy = (robot['y'] - prev['y']) / dt
                    else:
                        vx, vy = prev.get('vx', 0), prev.get('vy', 0)
                else:
                    vx, vy = 0, 0

                last_known_positions[robot_id] = {
                    'x': robot['x'],
                    'y': robot['y'],
                    'angulo': robot['angulo'],
                    'timestamp': current_time,
                    'vx': vx,
                    'vy': vy,
                    'esquinas': robot['esquinas']
                }
                LOST_DETECTION_COUNT[robot_id] = 0  # Reset contador

            # Predecir posición de robots no detectados
            for robot_id, last_pos in list(last_known_positions.items()):
                if robot_id not in detected_ids:
                    time_since_detection = current_time - last_pos['timestamp']

                    # Si no ha pasado mucho tiempo, predecir posición
                    if time_since_detection < MAX_PREDICTION_TIME:
                        LOST_DETECTION_COUNT[robot_id] = LOST_DETECTION_COUNT.get(robot_id, 0) + 1

                        # Predicción lineal simple
                        predicted_x = int(last_pos['x'] + last_pos['vx'] * time_since_detection)
                        predicted_y = int(last_pos['y'] + last_pos['vy'] * time_since_detection)

                        # Añadir robot predicho a robots_data
                        robots_data.append({
                            'id': robot_id,
                            'x': predicted_x,
                            'y': predicted_y,
                            'angulo': last_pos['angulo'],
                            'esquinas': last_pos['esquinas'],
                            'predicted': True  # Flag para indicar que es predicción
                        })

                        # Dibujar predicción en naranja
                        cv2.circle(frame_with_markers, (predicted_x, predicted_y), 8, (0, 165, 255), 2)
                        cv2.putText(frame_with_markers, f"PRED {robot_id}",
                                   (predicted_x + 10, predicted_y - 10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)

                        log.debug(f"🔮 Robot {robot_id} predicho (perdido por {LOST_DETECTION_COUNT[robot_id]} frames)")
                    else:
                        # Demasiado tiempo sin detectar, eliminar de tracking
                        log.warning(f"⚠️  Robot {robot_id} perdido definitivamente ({time_since_detection:.2f}s)")
                        del last_known_positions[robot_id]
                        if robot_id in LOST_DETECTION_COUNT:
                            del LOST_DETECTION_COUNT[robot_id]

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
