import logging
import traceback

import numpy as np
import cv2 as cv
from robot_soccer.perception.ball_tracking import Ball

log = logging.getLogger(__name__)


def busqueda_ball(fr2ball_recv, ball_send, enable_planning=True):
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
        enable_planning (bool): Si True, envía coordenadas al proceso de planificación.
            Default: True.

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

    pelota = None
    pelota_inicializada = False
    intentos_deteccion = 0
    max_intentos_iniciales = 100  # Intentar detectar pelota en primeros N frames

    # Configurar logging para este proceso
    logging.basicConfig(
        level=logging.INFO,
        format='%(levelname)-8s - %(filename)-15s - %(message)s'
    )

    log.info("🔍 Iniciando búsqueda de pelota...")
    log.info("   Rango de color: HSV %s", naranjo)
    log.info("Esperando frames...")

    try:
        frame_count = 0
        while True:
            # Recibir frame con timeout
            if fr2ball_recv.poll(timeout=5):  # Esperar máximo 5 segundos
                frame = fr2ball_recv.recv()
                frame_count += 1

                # Log del primer frame
                if frame_count == 1:
                    log.info("✅ Primer frame recibido")
            else:
                log.error("❌ Timeout: No se recibió frame en 5 segundos")
                log.error("   Verifica que el proceso de cámara/simulación esté enviando frames")
                continue

            img = np.copy(frame)
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

            # FASE 1: Inicialización - Detectar pelota por primera vez
            if not pelota_inicializada:
                try:
                    x, y, r = Ball.detectar_circulos_color(hsv, naranjo, img)

                    # Verificar si la detección fue exitosa
                    if x is not None and y is not None and r is not None:
                        pelota = Ball(naranjo, (x, y))
                        pelota_inicializada = True
                        log.info("✅ Pelota detectada en (%d, %d) con radio %d", x, y, r)
                        log.info("   Iniciando seguimiento continuo...")
                    else:
                        intentos_deteccion += 1

                        # Log cada 30 intentos para no saturar
                        if intentos_deteccion % 30 == 0:
                            log.warning("⚠️  Pelota no detectada (intento %d)", intentos_deteccion)

                        # Advertencia si no se detecta después de muchos intentos
                        if intentos_deteccion >= max_intentos_iniciales and intentos_deteccion % 60 == 0:
                            log.error("❌ Pelota no detectada después de %d frames", intentos_deteccion)
                            log.error("   Verifica:")
                            log.error("   - Que haya una pelota naranja visible")
                            log.error("   - La iluminación sea adecuada")
                            log.error("   - El rango de color HSV sea correcto")

                except Exception as e:
                    intentos_deteccion += 1
                    if intentos_deteccion % 30 == 0:
                        log.error("⚠️  Error detectando pelota: %s", e)

            # FASE 2: Seguimiento - Pelota ya detectada
            else:
                try:
                    x_pelota, y_pelota = pelota.seguimiento(hsv, img, frame)

                    # Mostrar ROI de seguimiento
                    if pelota.roi_hsv is not None and len(pelota.roi_hsv) > 0:
                        cv.imshow("Seguimiento Pelota", pelota.roi_hsv)

                    # Enviar coordenadas solo si el módulo de planificación está activo
                    if enable_planning:
                        enviar = (x_pelota, y_pelota)
                        ball_send.send(enviar)

                    # Log periódico del seguimiento
                    if frame_count % 120 == 0:
                        log.debug("📍 Pelota en (%d, %d)", x_pelota, y_pelota)

                except Exception as e:
                    log.error("⚠️  Error en seguimiento de pelota: %s", e)
                    log.warning("   Reintentando detección desde cero...")
                    # Reiniciar detección si falla el seguimiento
                    pelota_inicializada = False
                    pelota = None
                    intentos_deteccion = 0

            # Salir con ESC
            k = cv.waitKey(1) & 0xFF
            if k == 27:
                log.info("👋 Cerrando búsqueda de pelota...")
                break

        cv.destroyAllWindows()

    except KeyboardInterrupt:
        log.info("🛑 Búsqueda de pelota interrumpida por el usuario")
        cv.destroyAllWindows()

    except Exception as e:
        log.error("❌ Error crítico en búsqueda de pelota: %s", e)
        log.error(traceback.format_exc())
        cv.destroyAllWindows()
