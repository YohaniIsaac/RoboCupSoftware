"""Módulo de planificación de trayectorias para robot soccer.

Este módulo contiene las funciones principales para la planificación de rutas
utilizando el algoritmo RRT* Smart en un entorno de fútbol de robots.
Maneja la recepción de datos de sensores y genera trayectorias libres de colisiones.
"""
import math
import time
import cv2 as cv
from robot_soccer.ai.path_planning.rrt_star_smart import RrtStarSmart
from robot_soccer.core.shared_frame import SharedFrameReader


def trayectoria(ball_received, player_received, frame_config):
    """Función principal para la planificación de trayectorias en tiempo real.

    Esta función ejecuta un bucle continuo que recibe datos de la pelota y jugadores,
    calcula una trayectoria libre de colisiones utilizando RRT* Smart y visualiza
    el resultado en una ventana de OpenCV.

    Args:
        ball_received : multiprocessing.Pipe
            Canal de comunicación para recibir las coordenadas de la pelota (x, y).
        player_received : multiprocessing.Pipe
            Canal de comunicación para recibir las coordenadas y información de los jugadores.
            Formato esperado: [{"id": int, "x": float, "y": float, "angulo": float}, ...]
        fr2traj_recv : multiprocessing.Pipe
            Canal para recibir frames de imagen para visualización de la trayectoria.

    Notes:
        - Utiliza un retraso inicial de 2 segundos antes de comenzar la planificación
        - El robot objetivo está definido por `que_robot_mover = 1`
        - El punto de destino está fijo en (100, 500)
        - Los obstáculos incluyen la pelota (radio 30) y otros jugadores (52x70)
        - La visualización se muestra en una ventana llamada "ruta"
        - Presionar ESC (código 27) termina la ejecución

    Raises:
        Exception
            Captura cualquier excepción durante la ejecución y la imprime con el prefijo
            "error en trayectoria".

    Examples:
        Esta función está diseñada para ejecutarse en un proceso separado:

        >>> import multiprocessing
        >>> p4 = multiprocessing.Process(
        ...     target=trayectoria,
        ...     args=(ballReceived, playerReceived, fr2traj_recv)
        ... )
        >>> p4.start()
    """
    reader = SharedFrameReader(frame_config)

    try:
        # Inicializar el tiempo de inicio para el retraso
        start_time = time.time()
        delay_seconds = 2  # Por ejemplo, un retraso de 5 segundos
        que_robot_mover = 1
        first = True
        path_rrt = RrtStarSmart(50, 0.50, 5, 10000,
                                None, None, None)

        while True:
            frame = reader.read(blocking_timeout=5.0)
            if frame is None:
                continue
            x_ball, y_ball = ball_received.recv()
            coords_players = player_received.recv()

            final = (100, 500)
            inicio = None
            path = []

            if time.time() - start_time >= delay_seconds and que_robot_mover is not None:

                radio_ball = 30
                lista_obstaculos = []

                ball = [x_ball, y_ball, radio_ball]
                for info in coords_players:
                    if info["id"] == que_robot_mover:
                        inicio = (info["x"], info["y"])
                    else:
                        lista_obstaculos.append([info["x"], info["y"], 52, 70, math.radians(info["angulo"])])

                lista_obstaculos.append(ball)
                if first:
                    path_rrt.setup(inicio, final, lista_obstaculos)
                    path_rrt.planning()
                    first = False
                else:
                    path_rrt.update_obstacle(lista_obstaculos, inicio)

                path = path_rrt.path

            if len(path) > 0:
                new_frame = dibujar(frame, path)
                cv.imshow("ruta", new_frame)
                k = cv.waitKey(1) & 0xFF
                if k == 27:
                    break
        cv.destroyAllWindows()

    except Exception as e:
        print(f"error en trayectoria {e}")
    finally:
        reader.cleanup()


def dibujar(img, path):
    """Dibuja una trayectoria planificada sobre una imagen.

    Esta función toma una imagen y una lista de puntos que representan una trayectoria
    y dibuja líneas conectando estos puntos para visualizar el camino planificado.

    Args:
        img : numpy.ndarray
            Imagen base sobre la cual dibujar la trayectoria. Debe ser una imagen
            válida de OpenCV (formato BGR).
        path : list of tuple
            Lista de puntos que definen la trayectoria. Cada punto debe ser una tupla
            o lista con dos elementos (x, y) representando las coordenadas en píxeles.
            Formato: [(x1, y1), (x2, y2), ..., (xn, yn)]

    Returns:
        numpy.ndarray
            La imagen original con la trayectoria dibujada como líneas rojas conectando
            los puntos consecutivos del path.

    Notes:
        - El color de las líneas está fijo en rojo (BGR: (0, 0, 255))
        - El grosor de las líneas es de 2 píxeles
        - Las líneas se dibujan conectando puntos consecutivos en el path
        - Si el path tiene menos de 2 puntos, no se dibuja nada

    Examples:
        >>> import cv2
        >>> import numpy as np
        >>>
        >>> # Crear una imagen en blanco
        >>> imagen = np.zeros((600, 800, 3), dtype=np.uint8)
        >>>
        >>> # Definir una trayectoria simple
        >>> trayectoria = [(100, 100), (200, 150), (300, 200), (400, 250)]
        >>>
        >>> # Dibujar la trayectoria
        >>> imagen_con_ruta = dibujar(imagen, trayectoria)
        >>>
        >>> # Mostrar el resultado
        >>> cv2.imshow("Trayectoria", imagen_con_ruta)
        >>> cv2.waitKey(0)
    """
    # Color de la línea (en BGR)
    color = (0, 0, 255)  # Verde
    for i in range(1, len(path)):
        start_point = path[i-1]
        end_point = path[i]
        cv.line(img, start_point, end_point, color, 2)
    return img
