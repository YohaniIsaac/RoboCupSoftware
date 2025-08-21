import numpy as np
import cv2 as cv
from robot_soccer.perception.ball_tracking import Ball


def busqueda_ball(fr2ball_recv, ball_send):
    """Realiza la búsqueda y seguimiento continuo de la pelota en el campo de juego.

    Esta función implementa un algoritmo de detección y seguimiento de pelota
    basado en segmentación de color en espacio HSV. Utiliza detección de círculos
    en el primer frame y luego aplica seguimiento para mantener la trayectoria
    de la pelota en frames subsiguientes.

    El proceso se ejecuta en un bucle continuo hasta que se presione la tecla
    ESC o ocurra una excepción. Las coordenadas de la pelota detectada se envían
    a través de un pipe para comunicación entre procesos.

    Args:
        fr2ball_recv (multiprocessing.Pipe): Pipe receptor para recibir frames
            de video desde el proceso principal. Debe proporcionar arrays numpy
            en formato BGR.
        ballSend (multiprocessing.Pipe): Pipe emisor para enviar las coordenadas
            (x, y) de la pelota detectada al proceso de planificación de rutas.

    Returns:
        None: La función se ejecuta indefinidamente hasta recibir comando de
        parada (ESC) o excepción.

    Raises:
        Exception: Cualquier error durante el procesamiento de frames o
        comunicación entre procesos se captura y registra, pero no interrumpe
        la ejecución.

    Note:
        - Utiliza color naranja como parámetro de detección: HSV(10-30, 100-255, 20-255)
        - Requiere que la pelota sea visible en el primer frame para inicialización
        - Muestra ventana de depuración con la ROI de la pelota detectada
        - Presionar ESC termina el proceso de búsqueda

    Example:
        >>> import multiprocessing
        >>> fr2ball_env, fr2ball_recv = multiprocessing.Pipe()
        >>> ballSend, ballReceived = multiprocessing.Pipe()
        >>> proceso_busqueda = multiprocessing.Process(
        ...     target=busqueda_ball,
        ...     args=(fr2ball_recv, ballSend)
        ... )
        >>> proceso_busqueda.start()
    """
    # Color
    naranjo = ((10, 100, 20), (30, 255, 255))  # Rango de color para el naranjo

    first_frame = True
    pelota = None
    try:
        while True:
            frame = (
                fr2ball_recv.recv()
            )  # Recibir datos como bytes a través de la tubería
            img = np.copy(frame)
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

            if first_frame:
                x, y, _ = Ball.detectar_circulos_color(hsv, naranjo, img)
                pelota = Ball(naranjo, (x, y))

                first_frame = False

            else:
                x_pelota, y_pelota = pelota.seguimiento(hsv, img, frame)
                cv.imshow("pelota ", pelota.roi_hsv)

                enviar = x_pelota, y_pelota
                ball_send.send(enviar)

            k = cv.waitKey(1) & 0xFF
            if k == 27:
                break

        cv.destroyAllWindows()
    except Exception as e:
        print(f"error en la busqueda de pelota {e}")
