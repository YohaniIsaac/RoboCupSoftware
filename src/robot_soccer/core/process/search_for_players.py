"""Módulo para detección y seguimiento de jugadores en el campo de fútbol robot.

Este módulo contiene las funciones necesarias para el procesamiento de video
en tiempo real para detectar jugadores utilizando ArUco tags. Funciona como
un proceso independiente que recibe frames de video y envía las coordenadas
de los jugadores detectados.
"""
import numpy as np
import cv2 as cv
from robot_soccer.perception.player_tracking import deteccion_jugadores_aruco_tag


def busqueda_player(fr2player_recv, player_send):
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
    # first_frame = True
    # dentro = True
    try:
        while True:
            # Recibir datos como bytes a través de la tubería
            frame = fr2player_recv.recv()
            img = np.copy(frame)

            salida, datos = deteccion_jugadores_aruco_tag(img)
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
