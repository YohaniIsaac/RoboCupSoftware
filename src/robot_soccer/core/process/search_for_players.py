"""Módulo para detección y seguimiento de jugadores en el campo de fútbol robot.

Este módulo contiene las funciones necesarias para el procesamiento de video
en tiempo real para detectar jugadores utilizando ArUco tags. Funciona como
un proceso independiente que recibe frames de video y envía las coordenadas
de los jugadores detectados.
"""
import logging

import cv2 as cv
from robot_soccer.perception.player_tracking import (
    create_aruco_detector,
    deteccion_jugadores_aruco_tag,
)
from robot_soccer.core.shared_frame import SharedFrameReader

log = logging.getLogger(__name__)


def busqueda_player(frame_config, player_send, use_camera=False, enable_planning=True):
    """Ejecuta la búsqueda y detección continua de jugadores en el campo.

    Args:
        frame_config (dict): Configuración de shared memory (de SharedFrameWriter.config()).
        player_send (multiprocessing.Pipe): Tubería para enviar coordenadas de jugadores.
        use_camera (bool): Si True usa diccionario de cámara. Default: False.
        enable_planning (bool): Si True, envía coordenadas al planificador. Default: True.
    """
    logging.basicConfig(
        level=logging.INFO,
        format='%(levelname)-8s - %(filename)-15s - %(message)s'
    )

    # Crear detector UNA sola vez (reutilizado entre frames)
    detector = create_aruco_detector(use_camera=use_camera)

    log.info("Iniciando búsqueda de jugadores...")
    log.info("Esperando frames...")

    reader = SharedFrameReader(frame_config)

    try:
        frame_count = 0
        while True:
            frame = reader.read(blocking_timeout=5.0)
            if frame is None:
                log.error("Timeout: No se recibió frame en 5 segundos")
                continue

            frame_count += 1
            if frame_count == 1:
                log.info("Primer frame recibido")
            elif frame_count % 60 == 0:
                log.debug("Frames procesados: %d", frame_count)

            img = frame  # Ya es copia local del shared memory

            salida, datos = deteccion_jugadores_aruco_tag(img, detector)

            if enable_planning:
                player_send.send(datos)

            cv.imshow("deteccion", salida)

            k = cv.waitKey(1) & 0xFF
            if k == 27:
                break
        cv.destroyAllWindows()

    except Exception as e:
        log.error("Error en búsqueda de jugadores: %s", e)
    finally:
        reader.cleanup()
