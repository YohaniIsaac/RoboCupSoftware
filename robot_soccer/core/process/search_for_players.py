import numpy as np
import cv2 as cv
from robot_soccer.perception.player_tracking import deteccionJugadoresArucoTag

# ==========================================
# LOG
# ==========================================
from robot_soccer.utils.logger import get_logger
from robot_soccer.utils.logger import set_level, disable_module, enable_module
module_name = "core.process"

logger = get_logger(module_name)

# Activar depuración detallada para un módulo
set_level(module_name, "WARNING")  # DEBUG, INFO, WARNING, ERROR, CRITICAL, DISABLED
# # Desactivar registro para un módulo que está generando demasiados mensajes
# disable_module("core.physics")
# # Reactivar registro para un módulo previamente desactivado
# enable_module("core.physics", "INFO")
# ==========================================


def busqueda_player(fr2player_recv, playerSend):
    # first_frame = True
    # dentro = True
    try:
        while True:
            # Recibir datos como bytes a través de la tubería
            frame = fr2player_recv.recv()
            img = np.copy(frame)

            salida, datos = deteccionJugadoresArucoTag(img)
            playerSend.send(datos)

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
