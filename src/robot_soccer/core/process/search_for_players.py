"""Módulo para detección y seguimiento de jugadores en el campo de fútbol robot.

Este módulo contiene las funciones necesarias para el procesamiento de video
en tiempo real para detectar jugadores utilizando ArUco tags. Funciona como
un proceso independiente que recibe frames de video y envía las coordenadas
de los jugadores detectados.
"""
import logging

import numpy as np
import cv2 as cv
from robot_soccer.perception.player_tracking import (
    create_aruco_detector,
    deteccion_jugadores_aruco_tag,
)

log = logging.getLogger(__name__)


def busqueda_player(fr2player_recv, player_send, use_camera=False, enable_planning=True):
    """Ejecuta la búsqueda y detección continua de jugadores en el campo.

    Esta función implementa un bucle de procesamiento de video en tiempo real
    que recibe frames a través de una tubería de multiproceso, detecta jugadores
    usando ArUco tags y envía las coordenadas detectadas a través de otra tubería.

    Args:
        fr2player_recv (multiprocessing.Pipe): Tubería para recibir frames de video.
        player_send (multiprocessing.Pipe): Tubería para enviar las coordenadas
            de los jugadores detectados.
        use_camera (bool): Si True usa diccionario de cámara configurado en config.py,
            si False usa diccionario de simulación. Default: False.
        enable_planning (bool): Si True, envía coordenadas al proceso de planificación.
            Default: True.
    """
    logging.basicConfig(
        level=logging.INFO,
        format='%(levelname)-8s - %(filename)-15s - %(message)s'
    )

    # Crear detector UNA sola vez (reutilizado entre frames)
    detector = create_aruco_detector(use_camera=use_camera)

    log.info("Iniciando búsqueda de jugadores...")
    log.info("Esperando frames...")

    try:
        frame_count = 0
        while True:
            if fr2player_recv.poll(timeout=5):
                frame = fr2player_recv.recv()
                frame_count += 1

                if frame_count == 1:
                    log.info("Primer frame recibido")
                elif frame_count % 60 == 0:
                    log.debug("Frames procesados: %d", frame_count)
            else:
                log.error("Timeout: No se recibió frame en 5 segundos")
                continue

            # TODO(perf): Copia innecesaria si el frame ya viene de shared memory.
            img = np.copy(frame)

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
