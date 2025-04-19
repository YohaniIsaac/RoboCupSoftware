import numpy as np
import cv2 as cv
# import paquetes.rastreo_robots as track_rob
from robot_soccer.perception.player_tracking import deteccionJugadoresArucoTag


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
