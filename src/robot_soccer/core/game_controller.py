"""Controlador principal del juego de fútbol robótico.

Este módulo gestiona la ejecución multiproceso de la simulación de fútbol robótico,
coordinando la simulación principal, búsqueda de pelota, seguimiento de jugadores
y planificación de trayectorias.
"""
import logging
import multiprocessing
from .process.main_simulation import simulacion_principal
from .process.ball_search import busqueda_ball
from .process.search_for_players import busqueda_player
from .process.path import trayectoria

log = logging.getLogger(__name__)


def execute_multiprocessing(use_camera=False, camera_id=2, modules=None):
    """Ejecuta la simulación de fútbol robótico usando múltiples procesos.

    Configura y ejecuta un sistema multiproceso modular que puede incluir:
    - Simulación principal del juego / Captura de cámara
    - Búsqueda y seguimiento de la pelota
    - Detección y seguimiento de jugadores
    - Planificación de trayectorias

    Args:
        use_camera (bool): Si True, usa cámara física en lugar de simulación.
        camera_id (int): ID de la cámara a usar (default: 2 para DroidCam).
        modules (dict): Diccionario indicando qué módulos ejecutar:
            - 'perception': bool - Ejecutar detección de pelota y jugadores
            - 'path_planning': bool - Ejecutar planificación de rutas
            - 'full': bool - Ejecutar todos los módulos
            Si es None, ejecuta todos por defecto.

    La comunicación entre procesos se realiza mediante pipes y colas para
    intercambiar frames de video, coordenadas de objetos y rutas planificadas.

    Procesos creados:
        p1: Simulación principal del juego
        p2: Búsqueda y detección de la pelota
        p3: Búsqueda y seguimiento de jugadores
        p4: Planificación de trayectorias

    Returns:
        None: La función ejecuta los procesos hasta su finalización.

    Raises:
        Exception: Si ocurre un error durante la inicialización o ejecución
                  de los procesos, se captura y se muestra el mensaje de error.

    Note:
        Esta función bloquea la ejecución hasta que todos los procesos
        hayan terminado. Utiliza un máximo de 8 multiprocesos.

    Example:
        >>> controller = execute_multiprocessing()
        >>> # La simulación se ejecutará hasta completarse
    """
    try:
        # Si modules es None, ejecutar todo por defecto
        if modules is None:
            modules = {'perception': True, 'path_planning': True, 'full': True}

        # Log de configuración
        log.info("=" * 70)
        log.info("ROBOT SOCCER - CONFIGURACIÓN DE MÓDULOS")
        log.info("=" * 70)
        log.info(f"Fuente de video: {'Cámara física' if use_camera else 'Simulación'}")
        if use_camera:
            log.info(f"ID de cámara: {camera_id}")

        # Mostrar modo de ejecución
        if modules.get('video_only', False):
            log.info(f"Modo: Solo captura de video (sin procesamiento)")
        else:
            log.info(f"Módulo Percepción: {'✅' if modules['perception'] else '❌'}")
            log.info(f"Módulo Planificación: {'✅' if modules['path_planning'] else '❌'}")
        log.info("=" * 70)

        # Crear pipes de comunicación entre procesos
        fr2ball_env, fr2ball_recv = multiprocessing.Pipe()  # envia el frame para pelota
        fr2player_env, fr2player_recv = multiprocessing.Pipe()  # envia el frame para jugadores

        ball_send, ball_received = multiprocessing.Pipe()         # Enviar las coordenadas de la pelota
        player_send, player_received = multiprocessing.Pipe()     # Enviar las coordenadas de los jugadores
        fr2traj_env, fr2traj_recv = multiprocessing.Pipe()          # Para probar la ruta

        # Cola para enviar la ruta planificada
        env_ruta = multiprocessing.Queue()

        # Lista de procesos a iniciar
        processes = []

        # PROCESO 1: Fuente de video (simulación o cámara)
        # Determinar qué pipes habilitar según los módulos activos
        enable_perception = modules['perception']
        enable_planning = modules['path_planning']

        if use_camera:
            log.info("Inicializando cámara física...")
            from .process.camera_feed import camera_feed
            p1 = multiprocessing.Process(
                target=camera_feed,
                args=(fr2ball_env, fr2player_env, env_ruta, fr2traj_env, camera_id,
                      enable_perception, enable_perception, enable_planning),
                name="CameraFeed"
            )
        else:
            log.info("Inicializando simulación...")
            p1 = multiprocessing.Process(
                target=simulacion_principal,
                args=(fr2ball_env, fr2player_env, env_ruta, fr2traj_env,
                      enable_perception, enable_perception, enable_planning),
                name="Simulation"
            )
        processes.append(p1)

        # PROCESOS DE PERCEPCIÓN
        if modules['perception']:
            log.info("Inicializando módulo de percepción...")

            # PROCESO 2: Búsqueda de pelota
            p2 = multiprocessing.Process(
                target=busqueda_ball,
                args=(fr2ball_recv, ball_send, enable_planning),
                name="BallTracking"
            )
            processes.append(p2)

            # PROCESO 3: Búsqueda de jugadores
            p3 = multiprocessing.Process(
                target=busqueda_player,
                args=(fr2player_recv, player_send, use_camera, enable_planning),
                name="PlayerTracking"
            )
            processes.append(p3)

        # PROCESO DE PLANIFICACIÓN
        if modules['path_planning']:
            log.info("Inicializando módulo de planificación...")

            # PROCESO 4: Planificación de trayectorias
            p4 = multiprocessing.Process(
                target=trayectoria,
                args=(ball_received, player_received, fr2traj_recv),
                name="PathPlanning"
            )
            processes.append(p4)

        # Iniciar todos los procesos
        log.info("")
        log.info(f"▶️  Iniciando {len(processes)} proceso(s)...")
        for proc in processes:
            proc.start()
            log.info(f"   ✓ {proc.name} iniciado (PID: {proc.pid})")

        log.info("")
        log.info("=" * 70)
        log.info("Sistema en ejecución. Presiona Ctrl+C para detener.")
        log.info("=" * 70)

        # Esperar a que los procesos terminen
        for proc in processes:
            proc.join()

    except Exception as e:
        print(f"Error en main {e}")
