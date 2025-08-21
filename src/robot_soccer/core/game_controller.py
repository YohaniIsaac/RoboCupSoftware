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


def execute_multiprocessing():
    """Ejecuta la simulación de fútbol robótico usando múltiples procesos.

    Configura y ejecuta un sistema multiproceso que incluye:
    - Simulación principal del juego
    - Búsqueda y seguimiento de la pelota
    - Detección y seguimiento de jugadores
    - Planificación de trayectorias

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
        # Configurar el logger principal, para ver los todos los mensajes
        # en el proceso principal
        # logging.basicConfig(level=logging.INFO)
        # 8 multiprocesos maximospy

        # Crear una tubería para la comunicación entre procesos
        fr2ball_env, fr2ball_recv = multiprocessing.Pipe()  # envia el frame
        fr2player_env, fr2player_recv = multiprocessing.Pipe()  # envia el frame

        ball_send, ball_received = multiprocessing.Pipe()         # Enviar las coordenadas de la pelota
        player_send, player_received = multiprocessing.Pipe()     # Enviar las coordenadas de los jugadores
        fr2traj_env, fr2traj_recv = multiprocessing.Pipe()          # Para probar la ruta

        # lista para enviar la ruta planificada
        env_ruta = multiprocessing.Queue()

        # Crear los procesos
        p1 = multiprocessing.Process(target=simulacion_principal, args=(fr2ball_env, fr2player_env, env_ruta,
                                                                        fr2traj_env))
        p2 = multiprocessing.Process(target=busqueda_ball, args=(fr2ball_recv, ball_send))
        p3 = multiprocessing.Process(target=busqueda_player, args=(fr2player_recv, player_send))
        p4 = multiprocessing.Process(target=trayectoria, args=(ball_received, player_received, fr2traj_recv))
        # p5 = multiprocessing.Process(target=comandos,       args=(env_ruta,) )

        # Iniciar los procesos
        p1.start()
        p2.start()
        p3.start()
        p4.start()

        # Esperar a que los procesos terminen
        p1.join()
        p2.join()
        p3.join()
        p4.join()

    except Exception as e:
        print(f"Error en main {e}")
