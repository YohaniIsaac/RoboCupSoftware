"""Módulo para captura de video desde cámara física.

Este módulo reemplaza la simulación pygame con captura de video real,
permitiendo usar el sistema de percepción con una cámara física.
"""
import time
import logging
import cv2

log = logging.getLogger(__name__)


def camera_feed(fr2ball_env, fr2player_env, env_ruta, fr2traj_env, camera_id=2,
                enable_ball=True, enable_player=True, enable_traj=True):
    """Captura video de cámara y lo distribuye a los procesos de percepción.

    Reemplaza simulacion_principal() para usar una cámara física en lugar
    de la simulación pygame. Mantiene la misma interfaz de comunicación
    mediante pipes.

    Args:
        fr2ball_env (multiprocessing.Pipe): Pipe para enviar frames a detección de pelota.
        fr2player_env (multiprocessing.Pipe): Pipe para enviar frames a detección de jugadores.
        env_ruta (multiprocessing.Queue): Cola para recibir rutas planificadas (no usado en modo cámara).
        fr2traj_env (multiprocessing.Pipe): Pipe para enviar frames a análisis de trayectorias.
        camera_id (int): ID de la cámara (default: 2 para DroidCam).
        enable_ball (bool): Si True, envía frames al proceso de búsqueda de pelota.
        enable_player (bool): Si True, envía frames al proceso de búsqueda de jugadores.
        enable_traj (bool): Si True, envía frames al proceso de trayectorias.

    Note:
        - Requiere que droidcam-cli esté corriendo si usa DroidCam
        - Presionar ESC termina la captura
        - Muestra ventana con el feed de la cámara
    """
    # Configurar logging para este proceso
    logging.basicConfig(
        level=logging.INFO,
        format='%(levelname)-8s - %(filename)-15s - %(message)s'
    )
    log.info("=" * 60)
    log.info("INICIANDO MODO CÁMARA")
    log.info("=" * 60)
    log.info("Cámara ID: %i", camera_id)

    # Abrir cámara
    cap = cv2.VideoCapture(camera_id)

    if not cap.isOpened():
        log.error("No se pudo abrir la cámara %i", camera_id)
        log.error("Asegúrate de:")
        log.error("  1. Iniciar droidcam-cli: cd AlgortimosBasicos/ArucoTag && ./start_droidcam.sh")
        log.error("  2. Verificar que /dev/video{camera_id} existe")
        return

    # Configurar cámara
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps_cam = int(cap.get(cv2.CAP_PROP_FPS))

    log.info("Cámara abierta")
    log.info("   Resolución: %i x %i", width, height)
    log.info("   FPS: %i", fps_cam)
    log.info("=" * 60)

    frame_count = 0
    start_time = time.time()
    fps_display = 0

    try:
        while True:
            ret, frame = cap.read()

            if not ret:
                log.warning("⚠️  Error leyendo frame de la cámara")
                break

            frame_count += 1

            # Log del primer frame
            if frame_count == 1:
                log.info("✅ Primer frame capturado, comenzando envío a procesos...")

            # Calcular FPS real cada 30 frames
            if frame_count % 30 == 0:
                elapsed = time.time() - start_time
                fps_display = frame_count / elapsed if elapsed > 0 else 0
                log.debug("FPS: %.1f  | Frames: %i", fps_display, frame_count)

            # Enviar frame a procesos de percepción (solo si están habilitados)
            # Los frames ya están en formato BGR (OpenCV), no necesitan conversión
            try:
                if enable_ball:
                    fr2ball_env.send(frame)
                if enable_player:
                    fr2player_env.send(frame)
                if enable_traj:
                    fr2traj_env.send(frame)
            except Exception as e:
                log.error("Error enviando frame a procesos: %i", e)
                # break

            # Mostrar frame con información
            display_frame = frame.copy()
            cv2.putText(display_frame, "CAMERA MODE - FPS: {fps_display:.1f}",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_frame, f"Frame: {frame_count}",
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(display_frame, "ESC - Salir",
                       (10, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            cv2.imshow('Robot Soccer - Camera Feed', display_frame)

            # Salir con ESC
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                log.info("\nSaliendo del modo cámara...")
                break

    except Exception as e:
        log.error("Error en captura de cámara: %i", e)

    finally:
        # Estadísticas finales
        elapsed_total = time.time() - start_time
        avg_fps = frame_count / elapsed_total if elapsed_total > 0 else 0

        log.info("")
        log.info("=" * 60)
        log.info("ESTADÍSTICAS DE CAPTURA")
        log.info("=" * 60)
        log.info("Total de frames: %i", frame_count)
        log.info("Tiempo total: %.2f s", elapsed_total)
        log.info("FPS promedio: %.2f", avg_fps)
        log.info("=" * 60)

        # Limpiar
        cap.release()
        cv2.destroyAllWindows()
        log.info("✅ Cámara cerrada")
