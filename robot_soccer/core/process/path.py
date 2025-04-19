import math
import time
import cv2 as cv
from robot_soccer.ai.path_planning.rrt_star_smart import RrtStarSmart
# from config import *


def trayectoria(ballReceived, playerReceived, fr2traj_recv):
    try:
        # Inicializar el tiempo de inicio para el retraso
        start_time = time.time()
        delay_seconds = 2  # Por ejemplo, un retraso de 5 segundos
        # print("---------COORDENADAS DE PELOTA------------")
        # print("---------COORDENADAS DE JUGADORES------------")
        que_robot_mover = 1
        first = True
        path_rrt = RrtStarSmart(50, 0.50, 5, 10000,
                                None, None, None)

        while True:
            # que_robot_mover = que_robot_mover.recv()
            # final = final.recv()
            frame = fr2traj_recv.recv()
            x_ball, y_ball = ballReceived.recv()
            coords_players = playerReceived.recv()

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


def dibujar(img, path):
    # Color de la línea (en BGR)
    color = (0, 0, 255)  # Verde
    for i in range(1, len(path)):
        start_point = path[i-1]
        end_point = path[i]
        cv.line(img, start_point, end_point, color, 2)
    return img
