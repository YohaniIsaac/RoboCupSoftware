import numpy as np
import cv2 as cv
from robot_soccer.perception.ball_tracking import Ball


def busqueda_ball(fr2ball_recv, ballSend):
    # Color
    naranjo = ((10, 100, 20), (30, 255, 255))  # Rango de color para el naranjo

    first_frame = True
    pelota = None
    try:
        while True:
            frame = fr2ball_recv.recv()  # Recibir datos como bytes a través de la tubería
            img = np.copy(frame)
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

            if first_frame:
                x, y, r = Ball.detectar_circulos_color(hsv, naranjo, img)
                pelota = Ball(naranjo, (x, y))

                first_frame = False

            else:
                x_pelota, y_pelota = pelota.seguimiento(hsv, img, frame)
                cv.imshow("pelota ", pelota.roi_hsv)

                enviar = x_pelota, y_pelota
                ballSend.send(enviar)

            k = cv.waitKey(1) & 0xFF
            if k == 27:
                break

        cv.destroyAllWindows()
    except Exception as e:
        print(f"error en la busqueda de pelota {e}")
