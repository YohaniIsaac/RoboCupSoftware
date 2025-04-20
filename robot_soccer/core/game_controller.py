import multiprocessing
from .process.main_simulation import simulacion_principal
from .process.ball_search import busqueda_ball
from .process.search_for_players import busqueda_player
from .process.path import trayectoria

# ==========================================
# LOG
# ==========================================
from robot_soccer.utils.logger import get_logger
from robot_soccer.utils.logger import set_level, disable_module, enable_module
module_name = "core"

logger = get_logger(module_name)

# Activar depuración detallada para un módulo
set_level(module_name, "WARNING")  # DEBUG, INFO, WARNING, ERROR, CRITICAL, DISABLED
# # Desactivar registro para un módulo que está generando demasiados mensajes
# disable_module("core.physics")
# # Reactivar registro para un módulo previamente desactivado
# enable_module("core.physics", "INFO")
# ==========================================


def execute_multiprocessing():
    try:
        # Configurar el logger principal, para ver los todos los mensajes
        # en el proceso principal
        # logging.basicConfig(level=logging.INFO)
        # 8 multiprocesos maximospy

        # Crear una tubería para la comunicación entre procesos
        fr2ball_env, fr2ball_recv = multiprocessing.Pipe()  # envia el frame
        fr2player_env, fr2player_recv = multiprocessing.Pipe()  # envia el frame

        ballSend, ballReceived = multiprocessing.Pipe()         # Enviar las coordenadas de la pelota
        playerSend, playerReceived = multiprocessing.Pipe()     # Enviar las coordenadas de los jugadores
        fr2traj_env, fr2traj_recv = multiprocessing.Pipe()          # Para probar la ruta

        # lista para enviar la ruta planificada
        env_ruta = multiprocessing.Queue()

        # Crear los procesos
        p1 = multiprocessing.Process(target=simulacion_principal, args=(fr2ball_env, fr2player_env, env_ruta,
                                                                        fr2traj_env))
        p2 = multiprocessing.Process(target=busqueda_ball, args=(fr2ball_recv, ballSend))
        p3 = multiprocessing.Process(target=busqueda_player, args=(fr2player_recv, playerSend))
        p4 = multiprocessing.Process(target=trayectoria, args=(ballReceived, playerReceived, fr2traj_recv))
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
