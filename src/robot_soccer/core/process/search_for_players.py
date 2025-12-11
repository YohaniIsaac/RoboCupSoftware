"""Módulo para detección y seguimiento de jugadores en el campo de fútbol robot.

Este módulo contiene las funciones necesarias para el procesamiento de video
en tiempo real para detectar jugadores utilizando ArUco tags. Funciona como
un proceso independiente que recibe frames de video y envía las coordenadas
de los jugadores detectados.
"""
import logging

import numpy as np
import cv2 as cv
from robot_soccer.perception.player_tracking import deteccion_jugadores_aruco_tag

log = logging.getLogger(__name__)


def busqueda_player(fr2player_recv, player_send, use_camera=False, enable_planning=True):
    """Ejecuta la búsqueda y detección continua de jugadores en el campo.

    Esta función implementa un bucle de procesamiento de video en tiempo real
    que recibe frames a través de una tubería de multiproceso, detecta jugadores
    usando ArUco tags y envía las coordenadas detectadas a través de otra tubería.

    La función utiliza OpenCV para mostrar la imagen procesada con las detecciones
    visualizadas y permite salir del bucle presionando la tecla ESC.

    Args:
        fr2player_recv (multiprocessing.Pipe): Tubería para recibir frames de video.
            Los frames deben ser arrays de NumPy en formato BGR.
        playerSend (multiprocessing.Pipe): Tubería para enviar las coordenadas
            de los jugadores detectados. Envía una estructura de datos con
            información de posición y orientación de cada jugador.
        use_camera (bool): Si True usa diccionario 6x6 (cámara física),
            si False usa 7x7 (simulación). Default: False.
        enable_planning (bool): Si True, envía coordenadas al proceso de planificación.
            Default: True.

    Raises:
        Exception: Captura cualquier excepción durante el procesamiento y la
            registra en los logs. Continúa ejecutándose a menos que sea una
            excepción crítica del sistema.

    Note:
        - La función está diseñada para ejecutarse como un proceso independiente
        - Utiliza deteccionJugadoresArucoTag() del módulo player_tracking
        - Muestra una ventana OpenCV llamada "deteccion" con los resultados
        - Presionar ESC (código 27) termina el bucle y cierra las ventanas
        - Los frames recibidos son copiados antes del procesamiento

    Example:
        Típicamente llamada desde un proceso padre usando multiprocessing:

        >>> import multiprocessing
        >>> fr2player_env, fr2player_recv = multiprocessing.Pipe()
        >>> playerSend, playerReceived = multiprocessing.Pipe()
        >>> p3 = multiprocessing.Process(target=busqueda_player,
        ...                             args=(fr2player_recv, playerSend))
        >>> p3.start()
    """
    # Configurar logging para este proceso
    logging.basicConfig(
        level=logging.INFO,
        format='%(levelname)-8s - %(filename)-15s - %(message)s'
    )

    # first_frame = True
    # dentro = True

    # Log del diccionario ArUco utilizado
    diccionario = "DICT_6X6_1000" if use_camera else "DICT_7X7_1000"
    log.info("Iniciando búsqueda de jugadores...")
    log.info("   Diccionario ArUco: %s", diccionario)
    log.info("Esperando frames...")

    try:
        frame_count = 0
        while True:
            # Recibir datos como bytes a través de la tubería
            if fr2player_recv.poll(timeout=5):  # Esperar máximo 5 segundos
                frame = fr2player_recv.recv()
                frame_count += 1

                # Log cada 60 frames
                if frame_count == 1:
                    log.info("✅ Primer frame recibido")
                elif frame_count % 60 == 0:
                    log.debug("Frames procesados: %d", frame_count)
            else:
                log.error("❌ Timeout: No se recibió frame en 5 segundos")
                log.error("   Verifica que el proceso de cámara/simulación esté enviando frames")
                continue

            img = np.copy(frame)

            salida, datos = deteccion_jugadores_aruco_tag(img, use_camera=use_camera)

            # Enviar coordenadas solo si el módulo de planificación está activo
            if enable_planning:
                player_send.send(datos)

            # if dentro:
            #     # print(datos)
            #     dentro = False

            # track_rob.DetectarJugadoresCirculosDeColores(frame)

            cv.imshow("deteccion", salida)

            k = cv.waitKey(1) & 0xFF
            if k == 27:
                break
        cv.destroyAllWindows()

    except Exception as e:
        print(f"error en busqueda de jugadores {e}")
